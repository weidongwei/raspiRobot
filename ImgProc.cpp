#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>
#include <set>
#include <nlohmann/json.hpp>


#include "ImgProc.h"
#include "SeamTracker.h"
#include "detectLaser.h"
#include "yolo-seg.h"
#include "unet.h"
using json = nlohmann::json;

VisualConfig vConfig;

SeamTracker seamTracker;

static const bool PRINT_DETECT_TIMING = true;
// true: 使用激光点选择并高亮对应YOLO mask；false: 跳过选择，直接显示YOLO输出的全部mask。
static const bool USE_YOLO_LASER_SELECTION = false;


// 加载json配置文件
bool loadVisualConfig(VisualConfig& cfg, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    json j;
    file >> j;
    cfg.origin_img_path = j["origin_img_path"];
    cfg.proc_path = j["proc_path"];
    cfg.diff_path = j["diff_path"];
    cfg.csv_path = j["csv_path"];
    cfg.laser_duty = j["laser_duty"];
    cfg.exposure_time = j["exposure_time"];
    cfg.brightness = j["brightness"];
    cfg.photo_thread_mode = j["photo_thread_mode"];
    cfg.threshold_value_min = j["threshold_value_min"];
    cfg.threshold_value_rate = j["threshold_value_rate"];
    cfg.best_laser_length = j["best_laser_length"];
    cfg.best_laser_width = j["best_laser_width"];
    cfg.ratio_laser_length = j["ratio_laser_length"];
    cfg.ratio_laser_width = j["ratio_laser_width"];
    cfg.peak_suppress_win = j["peak_suppress_win"];
    cfg.patience_limit = j["patience_limit"];
    cfg.ratio_width = j["ratio_width"];
    cfg.ratio_depth = j["ratio_depth"];
    cfg.best_width = j["best_width"];
    cfg.best_depth = j["best_depth"];
    cfg.dx_between_seams_min = j["dx_between_seams_min"];
    cfg.total_score_min = j["total_score_min"];
    std::cout << "视觉配置文件加载完成：" << filename << std::endl;
    return true;
}

// 获取当前时间字符串
std::string getTimeString(){
    // 1. 获取当前系统时间（高精度，纳秒级）
    auto now = std::chrono::system_clock::now();

    // 2. 转换为时间戳（秒级，兼容传统time_t）
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

    // 3. 转为本地时间（线程安全）
    std::tm tm_time;
    localtime_r(&currentTime, &tm_time);

    std::ostringstream oss;
    oss << std::put_time(&tm_time, "%Y%m%d_%H%M%S");
    return oss.str();
}

// 读取激光点CSV文件保存到LaserData数组中
std::vector<LaserData> readLaserCSV(const std::string& filename) {
    std::vector<LaserData> dataList;
    std::ifstream file(filename);

    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return dataList; // 返回空列表
    }

    std::string line;
    // 2. 跳过表头 (laser_id,x_pixel,y_pixel,distance_cm)
    if (!std::getline(file, line)) {
        return dataList;
    }

    // 3. 逐行解析数据
    while (std::getline(file, line)) {
        // 跳过空行
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string item;
        LaserData row;

        try {
            // 解析 laser_id
            std::getline(ss, item, ',');
            row.laser_id = std::stoi(item);

            // 解析 x_pixel
            std::getline(ss, item, ',');
            row.x_pixel = std::stoi(item);

            // 解析 y_pixel
            std::getline(ss, item, ',');
            row.y_pixel = std::stoi(item);

            // 解析 distance_cm
            std::getline(ss, item, ',');
            row.distance_cm = std::stod(item);

            dataList.push_back(row);
        } catch (const std::exception& e) {
            // 报错信息：指示哪一行出了问题
            std::cerr << "Warning: Skipping malformed line: " << line << " (" << e.what() << ")" << std::endl;
        }
    }

    file.close();
    return dataList;
}


// 监控
int cctv(int camera_id){
    cv::VideoCapture cap(camera_id, cv::CAP_V4L2);
    // cv::VideoCapture cap;
    // cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << camera_id << std::endl;
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);                     // 有的驱动 1=手动，3=自动，需测试// 2. 关闭背光补偿 (OpenCV 对应宏是 CAP_PROP_BACKLIGHT)
    // cap.set(cv::CAP_PROP_BACKLIGHT, 0);                         // 关闭背光补偿
    // cap.set(cv::CAP_PROP_SHARPNESS, 100);                       // 设置锐度为 100(0 ~ 100)
    // cap.set(cv::CAP_PROP_BRIGHTNESS, vConfig.brightness);       // 设置亮度为 50(-64 ~ 64)
    // cap.set(cv::CAP_PROP_EXPOSURE, vConfig.exposure_time);      // 曝光时间整数ms(最小值50ms)
    // cap.set(cv::CAP_PROP_EXPOSURE, 3000);      // 曝光时间整数ms(最小值50ms)
    // cap.set(cv::CAP_PROP_BRIGHTNESS, 0);       // 设置亮度为 50(-64 ~ 64)

    cv::Mat origin_frame;
    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> origin_frame;
        cv::undistort(origin_frame, frame, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }
        cv::imshow("Camera Video", frame);
        cv::waitKey(100);
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

// 实时检测图像
int takeVedio(){
    cv::VideoCapture cap(2, cv::CAP_V4L2);
    // cv::VideoCapture cap;
    // cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << 0 << std::endl;
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);                     // 有的驱动 1=手动，3=自动，需测试// 2. 关闭背光补偿 (OpenCV 对应宏是 CAP_PROP_BACKLIGHT)
    cap.set(cv::CAP_PROP_BACKLIGHT, 0);                         // 关闭背光补偿
    cap.set(cv::CAP_PROP_EXPOSURE, vConfig.exposure_time);      // 曝光时间整数ms(最小值50ms)
    cap.set(cv::CAP_PROP_SHARPNESS, 100);                       // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, vConfig.brightness);       // 设置亮度为 50(-64 ~ 64)

    cv::Mat origin_frame;
    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> origin_frame;
        if (origin_frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }
        // 确认相机参数非空再去畸变
        if (!vConfig.MycameraMatrix.empty() && !vConfig.MydistCoeffs.empty()) {
            cv::undistort(origin_frame, frame, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
        } else {
            frame = origin_frame.clone();
        }
        
        cv::Mat finalMat = detectMain(frame);

        // cv::Mat displayImage;
        // std::vector<LaserData> data = detectLaserCenter(frame, &displayImage);
        // std::vector<LaserData> smoothData = smooth(data);
        // std::vector<MatchedSeamPair> results = findSeam(smoothData);
        // std::vector<MatchedSeamPair> stableResults = seamTracker.update(results);
        // cv::Mat finalMat = drawSeam(displayImage, stableResults, data);
        // std::cout << "#################################################################" << std::endl;

        cv::imshow("Camera Video", finalMat);
        cv::waitKey(300);
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

// 保存单张原始图像
int takePic(){
    std::string filename  = "origin_" + getTimeString() + ".jpg";
    std::string save_path = vConfig.origin_img_path + filename;
    cv::VideoCapture cap(2, cv::CAP_V4L2);
    // cv::VideoCapture cap;
    // cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << 0 << std::endl;
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // 有的驱动 1=手动，3=自动，需测试
    cap.set(cv::CAP_PROP_BACKLIGHT, 0);                         // 关闭背光补偿
    cap.set(cv::CAP_PROP_EXPOSURE, vConfig.exposure_time);     // 曝光时间整数us
    cap.set(cv::CAP_PROP_SHARPNESS, 100);  // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, vConfig.brightness);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat origin_frame;
    cv::Mat frame;
    cv::waitKey(1000);


    cap >> origin_frame;

    if (origin_frame.empty()) {
        std::cerr << "无法获取图像帧。" << std::endl;
        return 0;
    }
    // 确认相机参数非空再去畸变
    if (!vConfig.MycameraMatrix.empty() && !vConfig.MydistCoeffs.empty()) {
        cv::undistort(origin_frame, frame, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
    } else {
        frame = origin_frame.clone();
    }

    cv::waitKey(10);
    cv::imwrite(save_path, frame);
    std::cout << "图像已保存到 " << save_path << std::endl;
    sleep(3);

    cap.release();
    cv::destroyAllWindows();
    return 0;
}


// 检测图像中的橘子边缘
int detect_img_edge(cv::Mat src, cv::Mat &out) {
    //将原始图像 src（BGR 彩色）转换到 HSV 色彩空间
    //H (Hue, 色相)：表示颜色类型
    //S (Saturation, 饱和度)：颜色的鲜艳程度，低时颜色会发灰。
    //V (Value, 明度)：亮度，太低会变成黑色。
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);


    // 创建两个掩膜：一个用于主橙色范围，一个用于阴影/低饱和度区域
    cv::Mat mask_orange, mask_shadow, combined_mask;
    cv::inRange(hsv, cv::Scalar(10, 150, 100), cv::Scalar(25, 255, 255), mask_orange);
    cv::inRange(hsv, cv::Scalar(5, 20, 50), cv::Scalar(30, 150, 200), mask_shadow);
    combined_mask = mask_orange - mask_shadow;

    //MyCanny 函数进行边缘检测
    cv::Mat dst;
    Canny(combined_mask, dst, 50, 150, 3, false);

    //查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 绘制轮廓
    cv::Mat result = src.clone();
    cv::drawContours(result, contours, -1, cv::Scalar(0, 0, 255), 2); // 红色线, 线宽2

    out = result.clone();

    return 0;

}

int userImgProc0(cv::Mat *theMat, long beginTime, long afterTime){

    cv::Mat undistorted, out;
    cv::undistort(*theMat, undistorted, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
    detect_img_edge(undistorted, out);

    std::string filename = "image/photo_" + std::to_string(afterTime) + ".jpg";
    cv::imwrite(filename, out);

    printf("imgproc 1 处理完成\n");
    sleep(3);
    return 0;
}

int userImgProc1(cv::Mat *theMat, long beginTime, long afterTime){
    // printf("imgproc 1---------------------------开始 处理20秒\n");


    // std::string filename = "image/photo_" + std::to_string(afterTime) + ".jpg";
    // cv::imwrite(filename, *theMat);  // 保存为文件
    // //std::cout << "线程2保存照片：" << filename << std::endl;

    sleep(200);
    // printf("imgproc 1---------------------------完成 20秒\n");
    return 0;
}





// 平滑11个点
std::vector<LaserData> smooth(const std::vector<LaserData> data) {
    // std::string fname  = getTimeString() + "_points_smooth" + ".csv";
    // std::string savePath = vConfig.csv_path + fname;
    // std::ofstream ofs(savePath);
    // ofs << "laser_id,x_pixel,y_pixel,distance_cm\n";

    std::vector<LaserData> smoothedData;
    std::map<int, std::vector<LaserData>> grouped;
    for (const auto& row : data) {
        grouped[row.laser_id].push_back(row);
    }

    for (auto& item : grouped) {
        auto& rows = item.second;
        std::sort(rows.begin(), rows.end(), [](const LaserData& a, const LaserData& b) {
            return a.x_pixel < b.x_pixel;
        });

        int n = rows.size();
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            int count = 0;

            for (int j = i - 5; j <= i + 5; ++j) {
                if (j >= 0 && j < n) {
                    sum += rows[j].distance_cm;
                    count++;
                }
            }

            LaserData row = rows[i];
            row.distance_cm = sum / std::max(1, count);
            smoothedData.push_back(row);
            // ofs << row.laser_id << "," << row.x_pixel << "," << row.y_pixel << "," << row.distance_cm << "\n";
        }
    }
    std::cout << "data size: " << data.size() << ", smoothed size: " << smoothedData.size() << std::endl;
    return smoothedData;
}

//*********************************************** */
//*********************************************** */
//*********************************************** */改进橘缝检测
static double getQuantile(std::vector<double> values, double quantile) {
    if (values.empty()) return 0.0;
    if (quantile < 0.0) quantile = 0.0;
    if (quantile > 1.0) quantile = 1.0;

    std::sort(values.begin(), values.end());
    double pos = quantile * (values.size() - 1);
    int left = static_cast<int>(pos);
    int right = std::min(left + 1, static_cast<int>(values.size()) - 1);
    double ratio = pos - left;
    return values[left] * (1.0 - ratio) + values[right] * ratio;
}

/**
 * @param data 激光点数据列表；每个点需包含 laser_id、x_pixel、y_pixel 和 distance_cm。
 * @param halfWindowPixel 基线统计窗口的半宽，单位为 x 方向像素；值越大基线越平滑，但越可能抹掉局部变化。
 * @param quantile 窗口内距离的分位数，取值范围 0.0 到 1.0；例如 0.25 表示取 25% 分位距离作为局部果面基线。
 * @return std::vector<LaserBaselineData> 带有原始距离、基线距离和橘缝信号的激光点数据。
 */
// 获取两条激光的果面基线距离。baseline_distance 是每个点附近窗口内的低分位数距离曲线。
std::vector<LaserBaselineData> getLaserBaselineDistance(const std::vector<LaserData>& data, int halfWindowPixel, double quantile) {
    std::vector<LaserBaselineData> baselineData;
    std::map<int, std::vector<LaserData>> grouped;
    for (const auto& row : data) {
        grouped[row.laser_id].push_back(row);
    }

    for (auto& item : grouped) {
        auto& rows = item.second;
        std::sort(rows.begin(), rows.end(), [](const LaserData& a, const LaserData& b) {
            return a.x_pixel < b.x_pixel;
        });

        for (const auto& row : rows) {
            std::vector<double> windowDistances;
            for (const auto& neighbor : rows) {
                if (std::abs(neighbor.x_pixel - row.x_pixel) <= halfWindowPixel) {
                    windowDistances.push_back(neighbor.distance_cm);
                }
            }

            LaserBaselineData out;
            out.laser_id = row.laser_id;
            out.x_pixel = row.x_pixel;
            out.y_pixel = row.y_pixel;
            out.distance_cm = row.distance_cm;
            out.baseline_distance = getQuantile(windowDistances, quantile);
            out.seam_signal = out.distance_cm - out.baseline_distance;
            baselineData.push_back(out);
        }
    }

    return baselineData;
}

// SeamSignal数据替换原来的distance
std::vector<LaserData> buildSeamSignalData(const std::vector<LaserBaselineData>& data) {
    std::vector<LaserData> signalData;
    signalData.reserve(data.size());

    for (const auto& row : data) {
        LaserData out;
        out.laser_id = row.laser_id;
        out.x_pixel = row.x_pixel;
        out.y_pixel = row.y_pixel;
        out.distance_cm = row.seam_signal;
        signalData.push_back(out);
    }

    return signalData;
}

bool saveLaserBaselineCSV(const std::vector<LaserBaselineData>& data, const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error: Could not open baseline csv " << filename << std::endl;
        return false;
    }

    ofs << "laser_id,x_pixel,y_pixel,distance_cm,baseline_distance,seam_signal\n";
    ofs << std::fixed << std::setprecision(4);
    for (const auto& row : data) {
        ofs << row.laser_id << ","
            << row.x_pixel << ","
            << row.y_pixel << ","
            << row.distance_cm << ","
            << row.baseline_distance << ","
            << row.seam_signal << "\n";
    }

    std::cout << "baseline csv 保存完成: " << filename << std::endl;
    return true;
}
//*********************************************** */
//*********************************************** */
//*********************************************** */

// 峰值竞争(峰值索引, 激光数据)
std::vector<int> suppress_peaks(const std::vector<int>& peakIndices, const std::vector<LaserData>& data) {
    if (peakIndices.empty()) return {};

    std::vector<int> filtered;
    
    // 假设 peakIndices 已经是升序排列的
    for (int i = 0; i < peakIndices.size(); ++i) {
        int curr_idx = peakIndices[i];
        bool keep = true;

        // 向右检查win范围内的邻居
        for (int j = i + 1; j < peakIndices.size(); ++j) {
            int next_idx = peakIndices[j];
            if (std::abs(data[next_idx].x_pixel - data[curr_idx].x_pixel) > vConfig.peak_suppress_win) break;
            if (data[next_idx].distance_cm >= data[curr_idx].distance_cm) {
                keep = false;
                break;
            }
        }
        if (!keep) continue;
        
        // 向左检查win范围内的邻居
        for (int j = i - 1; j >= 0; --j) {
            int prev_idx = peakIndices[j];
            if (std::abs(data[curr_idx].x_pixel - data[prev_idx].x_pixel) > vConfig.peak_suppress_win) break;
            if (data[prev_idx].distance_cm > data[curr_idx].distance_cm) {
                keep = false;
                break;
            }
        }
        if (keep) {
            filtered.push_back(curr_idx);
        }
    
    }
    return filtered;
}

// 趋势坍塌分析函数(激光数据, 峰值索引)
SeamResult analyzeSeamStructure(const std::vector<LaserData>& data, int peakIdx) {
    SeamResult res;
    if (data.empty() || peakIdx < 0 || peakIdx >= data.size()) return res;
    res.id = data[peakIdx].laser_id;
    res.x_peak = data[peakIdx].x_pixel;
    res.dist = data[peakIdx].distance_cm;
    
    int n = data.size();

    // 向右寻找坡脚
    int right = peakIdx;
    double min_dist_right = data[peakIdx].distance_cm;
    int patience = vConfig.patience_limit;
    for (int j = peakIdx + 1; j < n; ++j) {
        double curr_dist = data[j].distance_cm;
        if (curr_dist < min_dist_right) {
            min_dist_right = curr_dist;
            right = j;
            patience = vConfig.patience_limit;
        } else {
            patience--; 
        }
        if (patience <= 0) break;
    }
    res.right_foot = data[right].x_pixel;
    // 向左寻找坡脚
    int left = peakIdx;
    double min_dist_left = data[peakIdx].distance_cm;
    patience = vConfig.patience_limit;
    for (int j = peakIdx - 1; j > 0; --j) {
        double curr_dist = data[j].distance_cm;
        if (curr_dist < min_dist_left) {
            min_dist_left = curr_dist;
            left = j;
            patience = vConfig.patience_limit;
        } else {
            patience--;
        }
        if (patience <= 0) break;
    }
    res.left_foot = data[left].x_pixel;

    // 获取凹陷宽度和深度
    res.width = std::abs(res.right_foot - res.left_foot);
    double base_dist = (min_dist_left + min_dist_right) / 2.0;
    res.depth = res.dist - base_dist;
    if (res.depth < 0) res.depth = 0;

    // 宽度评分
    double s_width = 0.0;
    double w_val = static_cast<double>(res.width);
    if (w_val <= vConfig.best_width) {
        s_width = w_val / (vConfig.best_width + 1e-5); 
    } else {
        s_width = vConfig.best_width / w_val;
    }

    // 深度评分
    double s_depth = 0.0;
    if (res.depth <= vConfig.best_depth) {
        s_depth = res.depth / (vConfig.best_depth + 1e-5);
    } else {
        s_depth = vConfig.best_depth / res.depth;
    }
    res.score = s_width * vConfig.ratio_width + s_depth * vConfig.ratio_depth;

    return res;
}

// 寻找橘缝函数
std::vector<MatchedSeamPair> findSeam(const std::vector<LaserData>& smoothedData) {
    int n = smoothedData.size();
    if (n < 15) return {};

    // 保存所有凹陷结果
    std::map<int, std::vector<SeamResult>> groupResults;

    // 每条激光线头尾容易受球面边缘、断线和基线边界影响，跳过端点附近点后再找峰。
    const int endpointSkip = 30;
    std::map<int, std::vector<LaserData>> groupedData;
    for (const auto& row : smoothedData) {
        groupedData[row.laser_id].push_back(row);
    }

    for (auto& item : groupedData) {
        std::vector<LaserData>& rows = item.second;
        std::sort(rows.begin(), rows.end(), [](const LaserData& a, const LaserData& b) {
            return a.x_pixel < b.x_pixel;
        });

        int m = rows.size();
        int beginIdx = std::max(2, endpointSkip);
        int endIdx = m - std::max(2, endpointSkip);
        if (m < 15 || beginIdx >= endIdx) continue;

        // 初选极大值
        std::vector<int> rawPeakIndices;
        for (int i = beginIdx; i < endIdx; ++i) {
            if (rows[i].distance_cm >= rows[i-1].distance_cm &&
                rows[i].distance_cm >= rows[i-2].distance_cm &&
                rows[i].distance_cm >= rows[i+1].distance_cm &&
                rows[i].distance_cm >= rows[i+2].distance_cm) {
                rawPeakIndices.push_back(i);
            }
        }

        // 调用峰值竞争函数
        std::vector<int> clearPeaks = suppress_peaks(rawPeakIndices, rows);

        for (int p_idx : clearPeaks) {
            // 调用趋势坍塌分析函数
            SeamResult current = analyzeSeamStructure(rows, p_idx);
            // 只保留中间区域的橘缝
            if(current.x_peak>120 && current.x_peak<520){
                groupResults[current.id].push_back(current);
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "检测到橘缝:id=" << current.id
                     << ", x_peak=" << current.x_peak
                     << ", dist=" << current.dist << " cm"
                     << ", left_foot=" << current.left_foot
                     << ", right_foot=" << current.right_foot
                     << ", width=" << current.width
                     << ", depth=" << current.depth << " cm"
                     << ", score=" << current.score
                     << std::endl;
            }
        }
    }

    // 找到最像橘缝的凹陷组合
    std::vector<MatchedSeamPair> tempMatchedPairs;
    std::vector<MatchedSeamPair> finalMatchedPairs;

    std::vector<SeamResult> list1 = groupResults[1];
    std::vector<SeamResult> list2 = groupResults[2];

    // 嵌套循环进行交叉匹配
    for (const auto& s1 : list1) {
        for (const auto& s2 : list2) {
            // 计算 X 轴偏差
            int dx = std::abs(s1.x_peak - s2.x_peak);
            if (dx <= vConfig.dx_between_seams_min) {
                MatchedSeamPair mp;
                mp.s1 = s1;
                mp.s2 = s2;
                // 总分为两点分数之和
                mp.total_score = s1.score + s2.score;
                tempMatchedPairs.push_back(mp);
            }
        }
    }

    // 对匹配对按总分从高到低排序
    std::sort(tempMatchedPairs.begin(), tempMatchedPairs.end(), [](const MatchedSeamPair& a, const MatchedSeamPair& b) {
        return a.total_score > b.total_score;
    });

    for(const auto& match : tempMatchedPairs){
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "暂时匹配成功的橘缝对(顺序): ID1 x_peak=" << match.s1.x_peak
             << ", ID2 x_peak=" << match.s2.x_peak
             << ", total_score=" << match.total_score << std::endl;
    }

    std::set<int> usedX1; // 记录 list1 中已使用的 x_peak
    std::set<int> usedX2; // 记录 list2 中已使用的 x_peak

    for (const auto& match : tempMatchedPairs) {
        // 检查这两点的 x_peak 是否已经在“已使用”名单里
        // count() 返回 0 表示不在名单里
        if (usedX1.count(match.s1.x_peak) == 0 && usedX2.count(match.s2.x_peak) == 0) {
            
            // 标记这两个坐标为已使用
            usedX1.insert(match.s1.x_peak);
            usedX2.insert(match.s2.x_peak);
            if(match.total_score > vConfig.total_score_min){
                // 加入最终结果
                finalMatchedPairs.push_back(match);
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "最终匹配成功的橘缝对: ID1 x_peak=" << match.s1.x_peak
                     << ", ID2 x_peak=" << match.s2.x_peak
                     << ", total_score=" << match.total_score << std::endl;
            }
        }
    }
    std::cout << "最终橘缝对数量: " << finalMatchedPairs.size() << std::endl;
    for (const auto& seam : finalMatchedPairs) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "最终橘缝对结果: id=" << seam.s1.id 
                << ", x_peak=" << seam.s1.x_peak 
                << ", dist=" << seam.s1.dist << " cm"
                << ", left_foot=" << seam.s1.left_foot 
                << ", right_foot=" << seam.s1.right_foot 
                << ", width=" << seam.s1.width 
                << ", depth=" << seam.s1.depth << " cm"
                << ", score=" << seam.total_score << std::endl;

        std::cout << "                id=" << seam.s2.id 
                << ", x_peak=" << seam.s2.x_peak 
                << ", dist=" << seam.s2.dist << " cm"
                << ", left_foot=" << seam.s2.left_foot 
                << ", right_foot=" << seam.s2.right_foot 
                << ", width=" << seam.s2.width 
                << ", depth=" << seam.s2.depth << " cm"
                << ", score=" << seam.total_score << std::endl;
    }
    std::cout << "橘缝识别完成。" << std::endl;
    return finalMatchedPairs;
}

// 将传统方法输出的峰值点改成凹陷区域中心点。
// findSeam() 仍负责判断候选和配对；这里仅把每条激光线的 x_peak
// 从局部最高点移动到 left_foot/right_foot 的中点，并吸附到真实激光点 x。
std::vector<MatchedSeamPair> moveSeamPeaksToRegionCenters(const std::vector<MatchedSeamPair>& results, const std::vector<LaserData>& data) {
    std::vector<MatchedSeamPair> centeredResults = results;

    auto snapToNearestLaserX = [&](int laserId, int targetX) {
        bool found = false;
        int bestX = targetX;
        int bestDx = 0;

        for (const auto& row : data) {
            if (row.laser_id != laserId) continue;

            int dx = std::abs(row.x_pixel - targetX);
            if (!found || dx < bestDx) {
                found = true;
                bestDx = dx;
                bestX = row.x_pixel;
            }
        }

        return bestX;
    };

    auto moveOneSeam = [&](SeamResult& seam) {
        int left = std::min(seam.left_foot, seam.right_foot);
        int right = std::max(seam.left_foot, seam.right_foot);
        if (right <= left) return;

        int centerX = (left + right) / 2;
        int oldPeak = seam.x_peak;
        seam.x_peak = snapToNearestLaserX(seam.id, centerX);

        std::cout << "橘缝点由峰值改为区域中心: id=" << seam.id
                  << ", old_x_peak=" << oldPeak
                  << ", center_x=" << centerX
                  << ", snapped_x=" << seam.x_peak
                  << ", left_foot=" << seam.left_foot
                  << ", right_foot=" << seam.right_foot
                  << std::endl;
    };

    for (auto& pair : centeredResults) {
        moveOneSeam(pair.s1);
        moveOneSeam(pair.s2);
    }

    return centeredResults;
}

// 画出激光轮廓、激光骨架线，以及两个激光橘缝点的直连线。
cv::Mat drawSeam2(cv::Mat displayImage, const std::vector<MatchedSeamPair> results, const std::vector<LaserData> data) {
    if (displayImage.empty()) return displayImage;

    if (displayImage.channels() == 1) {
        cv::cvtColor(displayImage, displayImage, cv::COLOR_GRAY2BGR);
    } else if (displayImage.channels() == 4) {
        cv::cvtColor(displayImage, displayImage, cv::COLOR_BGRA2BGR);
    } else {
        displayImage = displayImage.clone();
    }

    // 重新跑一遍轻量级激光前端，只用于调试图上标出当前检测到的轮廓。
    cv::Mat undistortedImg;
    cv::Mat diff = preprocessLaserImage(displayImage, undistortedImg);
    std::vector<std::vector<cv::Point>> contours = getLaserContours(diff);
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > displayImage.cols * displayImage.rows * 0.0002) {
            cv::drawContours(displayImage, std::vector<std::vector<cv::Point>>{contour}, -1,
                             cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
    }

    // 用传入的 LaserData 按 laser_id 连成骨架线，避免重复抽中心线带来细微差异。
    std::map<int, std::vector<cv::Point>> skeletonByLaser;
    for (const auto& row : data) {
        if (row.x_pixel < 0 || row.x_pixel >= displayImage.cols ||
            row.y_pixel < 0 || row.y_pixel >= displayImage.rows) {
            continue;
        }
        skeletonByLaser[row.laser_id].push_back(cv::Point(row.x_pixel, row.y_pixel));
    }

    const cv::Scalar skeletonColor(255, 0, 0);
    const int maxSkeletonGap = 12;

    for (auto& item : skeletonByLaser) {
        std::vector<cv::Point>& points = item.second;
        std::sort(points.begin(), points.end(), [](const cv::Point& a, const cv::Point& b) {
            if (a.x != b.x) return a.x < b.x;
            return a.y < b.y;
        });

        std::vector<cv::Point> segment;
        for (const auto& point : points) {
            if (!segment.empty() && point.x - segment.back().x > maxSkeletonGap) {
                if (segment.size() >= 2) {
                    cv::polylines(displayImage, std::vector<std::vector<cv::Point>>{segment},
                                  false, skeletonColor, 1, cv::LINE_AA);
                }
                segment.clear();
            }
            segment.push_back(point);
        }

        if (segment.size() >= 2) {
            cv::polylines(displayImage, std::vector<std::vector<cv::Point>>{segment},
                          false, skeletonColor, 1, cv::LINE_AA);
        }

        for (const auto& point : points) {
            cv::circle(displayImage, point, 1, skeletonColor, -1, cv::LINE_AA);
        }
    }

    const cv::Scalar purple(255, 0, 255);
    for (const auto& seamPair : results) {
        int result_y1 = -1;
        int result_y2 = -1;

        auto it1 = std::find_if(data.begin(), data.end(), [&](const LaserData& item) {
            return item.laser_id == seamPair.s1.id && item.x_pixel == seamPair.s1.x_peak;
        });
        if (it1 != data.end()) {
            result_y1 = it1->y_pixel;
        }

        auto it2 = std::find_if(data.begin(), data.end(), [&](const LaserData& item) {
            return item.laser_id == seamPair.s2.id && item.x_pixel == seamPair.s2.x_peak;
        });
        if (it2 != data.end()) {
            result_y2 = it2->y_pixel;
        }

        if (result_y1 < 0 || result_y2 < 0) continue;

        cv::Point p1(seamPair.s1.x_peak, result_y1);
        cv::Point p2(seamPair.s2.x_peak, result_y2);
        cv::line(displayImage, p1, p2, purple, 2, cv::LINE_AA);
    }

    return displayImage;
}

// 画出橘缝曲线
cv::Mat drawSeam(cv::Mat displayImage, const std::vector<SeamCurveResult>& curves) {
    if (curves.size() < 1) return displayImage;
    for(int i=0; i<curves.size(); i++){

        double ratio = (curves.size() > 1) ? (double)i / (curves.size() - 1) : 0.0;
        
        // 起始颜色 (纯红): (0, 0, 255)
        // 结束颜色 (浅粉): (180, 180, 255) -> 你可以调整 180 这个值，越大越白
        int b = (int)(0 + 180 * ratio); 
        int g = (int)(0 + 180 * ratio);
        int r = 255; 
        cv::Scalar currentColor(b, g, r);

        if (curves[i].curve_points.size() >= 2) {
            std::vector<std::vector<cv::Point>> polylineGroup;
            polylineGroup.push_back(curves[i].curve_points);
            cv::polylines(displayImage, polylineGroup, false, currentColor, 2, cv::LINE_AA);

            // 画出两条激光实际找到的橘缝锚点（黑色实心圆）
            if (curves[i].has_laser_points) {
                cv::circle(displayImage, curves[i].laser_point1, 4, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                cv::circle(displayImage, curves[i].laser_point2, 4, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            }
        }
    }

    return displayImage;

}

// 实时绘制类似 matplotlib 俯视散点图的激光重建视图。
cv::Mat showLaserReconstructionView(const std::vector<LaserData>& data,
                                    const std::vector<LaserPlotTarget>& targets,
                                    const std::string& windowName,
                                    bool showWindow) {
    const int canvasWidth = 900;
    const int canvasHeight = 680;
    const cv::Rect plotRect(80, 45, canvasWidth - 125, canvasHeight - 125);
    cv::Mat canvas(canvasHeight, canvasWidth, CV_8UC3, cv::Scalar(248, 248, 248));

    cv::rectangle(canvas, plotRect, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    cv::rectangle(canvas, plotRect, cv::Scalar(50, 50, 50), 1, cv::LINE_AA);
    cv::putText(canvas, "Multi-Laser Reconstruction (top view)",
                cv::Point(plotRect.x, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(30, 30, 30), 2, cv::LINE_AA);

    if (data.empty()) {
        cv::putText(canvas, "No laser data",
                    cv::Point(plotRect.x + 20, plotRect.y + plotRect.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(80, 80, 80), 2, cv::LINE_AA);
        if (showWindow) cv::imshow(windowName, canvas);
        return canvas;
    }

    double minDist = std::numeric_limits<double>::max();
    double maxDist = -std::numeric_limits<double>::max();
    std::set<int> laserIds;
    for (const auto& row : data) {
        if (!std::isfinite(row.distance_cm)) continue;
        minDist = std::min(minDist, row.distance_cm);
        maxDist = std::max(maxDist, row.distance_cm);
        laserIds.insert(row.laser_id);
    }

    if (minDist == std::numeric_limits<double>::max()) {
        cv::putText(canvas, "No valid distance data",
                    cv::Point(plotRect.x + 20, plotRect.y + plotRect.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(80, 80, 80), 2, cv::LINE_AA);
        if (showWindow) cv::imshow(windowName, canvas);
        return canvas;
    }

    double distRange = maxDist - minDist;
    if (distRange < 1e-6) distRange = 1.0;
    double distPad = std::max(0.5, distRange * 0.08);
    minDist -= distPad;
    maxDist += distPad;

    auto clamp01 = [](double value) {
        return std::max(0.0, std::min(1.0, value));
    };

    auto toPlotPoint = [&](double xPixel, double distanceCm) {
        double xRatio = clamp01(xPixel / 640.0);
        double dRatio = clamp01((distanceCm - minDist) / (maxDist - minDist));
        int x = plotRect.x + static_cast<int>(std::round(xRatio * plotRect.width));
        int y = plotRect.y + plotRect.height - static_cast<int>(std::round(dRatio * plotRect.height));
        return cv::Point(x, y);
    };

    auto laserColor = [](int laserId) {
        switch (laserId) {
            case 1: return cv::Scalar(0, 145, 255);
            case 2: return cv::Scalar(70, 185, 70);
            case 3: return cv::Scalar(220, 90, 160);
            case 4: return cv::Scalar(180, 100, 40);
            default: return cv::Scalar(110, 110, 110);
        }
    };

    auto softColor = [](const cv::Scalar& color) {
        return cv::Scalar(
            color[0] * 0.45 + 255.0 * 0.55,
            color[1] * 0.45 + 255.0 * 0.55,
            color[2] * 0.45 + 255.0 * 0.55
        );
    };

    const int gridXCount = 8;
    for (int i = 0; i <= gridXCount; ++i) {
        int x = plotRect.x + static_cast<int>(std::round(plotRect.width * i / static_cast<double>(gridXCount)));
        cv::line(canvas, cv::Point(x, plotRect.y), cv::Point(x, plotRect.y + plotRect.height),
                 cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
        int xLabel = static_cast<int>(std::round(640.0 * i / gridXCount));
        cv::putText(canvas, std::to_string(xLabel), cv::Point(x - 12, plotRect.y + plotRect.height + 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(70, 70, 70), 1, cv::LINE_AA);
    }

    const int gridYCount = 6;
    for (int i = 0; i <= gridYCount; ++i) {
        int y = plotRect.y + static_cast<int>(std::round(plotRect.height * i / static_cast<double>(gridYCount)));
        cv::line(canvas, cv::Point(plotRect.x, y), cv::Point(plotRect.x + plotRect.width, y),
                 cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
        double distLabel = maxDist - (maxDist - minDist) * i / gridYCount;
        cv::putText(canvas, cv::format("%.1f", distLabel), cv::Point(18, y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(70, 70, 70), 1, cv::LINE_AA);
    }

    cv::putText(canvas, "x_pixel", cv::Point(plotRect.x + plotRect.width / 2 - 28, canvasHeight - 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(50, 50, 50), 1, cv::LINE_AA);
    cv::putText(canvas, "distance_cm", cv::Point(15, plotRect.y - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50), 1, cv::LINE_AA);

    std::map<int, std::vector<LaserData>> grouped;
    for (const auto& row : data) {
        if (!std::isfinite(row.distance_cm)) continue;
        grouped[row.laser_id].push_back(row);
    }

    for (auto& item : grouped) {
        auto& rows = item.second;
        std::sort(rows.begin(), rows.end(), [](const LaserData& a, const LaserData& b) {
            if (a.x_pixel != b.x_pixel) return a.x_pixel < b.x_pixel;
            return a.y_pixel < b.y_pixel;
        });

        cv::Scalar pointColor = softColor(laserColor(item.first));
        cv::Scalar lineColor = softColor(laserColor(item.first));
        bool hasPrev = false;
        LaserData prevRow;
        cv::Point prevPoint;
        for (const auto& row : rows) {
            cv::Point point = toPlotPoint(row.x_pixel, row.distance_cm);
            if (hasPrev && std::abs(row.x_pixel - prevRow.x_pixel) <= 12) {
                cv::line(canvas, prevPoint, point, lineColor, 1, cv::LINE_AA);
            }
            cv::circle(canvas, point, 2, pointColor, -1, cv::LINE_AA);
            prevRow = row;
            prevPoint = point;
            hasPrev = true;
        }
    }

    int legendX = plotRect.x + plotRect.width - 130;
    int legendY = plotRect.y + 22;
    for (int laserId : laserIds) {
        cv::circle(canvas, cv::Point(legendX, legendY), 5, laserColor(laserId), -1, cv::LINE_AA);
        cv::putText(canvas, "Laser " + std::to_string(laserId), cv::Point(legendX + 12, legendY + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
        legendY += 22;
    }

    for (const auto& target : targets) {
        const LaserData* best = nullptr;
        double bestScore = std::numeric_limits<double>::max();
        for (const auto& row : data) {
            if (row.laser_id != target.laser_id || !std::isfinite(row.distance_cm)) continue;
            double score = std::abs(row.x_pixel - target.x_pixel);
            if (target.use_distance) {
                score += std::abs(row.distance_cm - target.distance_cm) * 8.0;
            }
            if (score < bestScore) {
                bestScore = score;
                best = &row;
            }
        }

        if (best == nullptr) continue;

        cv::Point markPoint = toPlotPoint(best->x_pixel, best->distance_cm);
        cv::circle(canvas, markPoint, 8, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::circle(canvas, markPoint, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        std::string label = cv::format("ID%d x%d %.1fcm y%d",
                                      best->laser_id, best->x_pixel, best->distance_cm, best->y_pixel);
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseline);
        int labelX = std::min(std::max(markPoint.x + 10, plotRect.x + 4),
                              plotRect.x + plotRect.width - textSize.width - 8);
        int labelY = markPoint.y - 10;
        if (labelY - textSize.height < plotRect.y) labelY = markPoint.y + textSize.height + 14;
        labelY = std::min(std::max(labelY, plotRect.y + textSize.height + 4),
                          plotRect.y + plotRect.height - 6);

        cv::Rect labelRect(labelX - 3, labelY - textSize.height - 4,
                           textSize.width + 6, textSize.height + baseline + 6);
        cv::rectangle(canvas, labelRect, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::rectangle(canvas, labelRect, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        cv::putText(canvas, label, cv::Point(labelX, labelY),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
    }

    if (showWindow) cv::imshow(windowName, canvas);
    return canvas;
}

// 计算
static double elapsedMs(std::chrono::steady_clock::time_point begin, std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// 检测主函数
cv::Mat detectMain(cv::Mat originImage){
    cv::Mat displayImage;
    auto s1 = std::chrono::steady_clock::now();
    std::vector<LaserData> data = detectLaserCenter(originImage, &displayImage);
    std::vector<LaserData> smoothData = smooth(data);

    // 获取基线数据，改进激光数据
    std::vector<LaserBaselineData> baselineData = getLaserBaselineDistance(smoothData, 40, 0.25);
    // saveLaserBaselineCSV(baselineData, vConfig.csv_path + getTimeString() + "_points_baseline.csv");
    std::vector<LaserData> seamSignalData = buildSeamSignalData(baselineData);

    // 传统方法找橘缝
    std::vector<MatchedSeamPair> results = findSeam(seamSignalData);
    // std::vector<MatchedSeamPair> stableResults = seamTracker.update(results);
    auto s2 = std::chrono::steady_clock::now();
    if (PRINT_DETECT_TIMING) {
        std::cout << "[Timing] 传统算法耗时: "
                  << std::fixed << std::setprecision(2)
                  << elapsedMs(s1, s2) << " ms"
                  << std::endl;
    }

    // unet找橘缝曲线
    auto unetTraceStart = std::chrono::steady_clock::now();
    std::vector<SeamCurveResult> curveResults = traceSeamCurvesByUnet(originImage, results, data);
    auto unetTraceEnd = std::chrono::steady_clock::now();
    if (PRINT_DETECT_TIMING) {
        std::cout << "[Timing] U-Net 橘缝曲线算法耗时: "
                  << std::fixed << std::setprecision(2)
                  << elapsedMs(unetTraceStart, unetTraceEnd) << " ms"
                  << ", curves=" << curveResults.size()
                  << std::endl;
    }
    cv::Mat finalMat = drawSeam(displayImage, curveResults);

    // 画可能分瓣线像素点
    cv::Mat finalMat2 = drawUnetSeamProbabilityPoints(originImage, 0.97f, 1);
    cv::imshow("finalMat2", finalMat2);

    // 画出传统线
    cv::Mat finalMat1 = drawSeam2(originImage, results, smoothData);
    cv::imshow("finalMat1", finalMat1);

    // 实时查看 x_pixel-distance_cm 俯视重建图，并把当前稳定橘缝点标成红星。
    std::vector<LaserPlotTarget> laserPlotTargets;
    for (const auto& seam : results) {
        laserPlotTargets.push_back(LaserPlotTarget(seam.s1.id, seam.s1.x_peak));
        laserPlotTargets.push_back(LaserPlotTarget(seam.s2.id, seam.s2.x_peak));
    }
    showLaserReconstructionView(smoothData, laserPlotTargets, "Laser Reconstruction", true);
    showLaserReconstructionView(seamSignalData, laserPlotTargets, "Laser Seam Signal", true);


    // yolo找橘缝曲线
    // YoloSegTiming yoloTiming;
    // finalMat = applyYoloSegToDisplay(originImage, finalMat, stableResults, data, USE_YOLO_LASER_SELECTION, &yoloTiming);

    cv::waitKey(1);


    return finalMat;
}



