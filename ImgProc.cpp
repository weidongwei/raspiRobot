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
using json = nlohmann::json;

VisualConfig vConfig;

SeamTracker seamTracker;

// YOLO-seg 默认参数。这里故意不写入 visualConfig.json，避免影响原有视觉配置。
static const bool YOLO_SEAM_ENABLE = true;
static const char* YOLO_SEAM_MODEL_PATH = "/home/dw/robot/cpp/best.onnx";
static const int YOLO_SEAM_INPUT_SIZE = 640;
static const double YOLO_SEAM_CONF_THRESHOLD = 0.25;
static const double YOLO_SEAM_NMS_THRESHOLD = 0.45;
static const bool YOLO_SEAM_DRAW_ALL = true;
static const bool YOLO_SEAM_PRINT_TIMING = true;


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
    cv::VideoCapture cap(0, cv::CAP_V4L2);
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
    cv::VideoCapture cap(0, cv::CAP_V4L2);
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
    sleep(1000);

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
    // 初选极大值
    std::vector<int> rawPeakIndices;
    for (int i = 2; i < n - 2; ++i) {
        if (smoothedData[i].distance_cm >= smoothedData[i-1].distance_cm &&
            smoothedData[i].distance_cm >= smoothedData[i-2].distance_cm &&
            smoothedData[i].distance_cm >= smoothedData[i+1].distance_cm &&
            smoothedData[i].distance_cm >= smoothedData[i+2].distance_cm) {
            rawPeakIndices.push_back(i);
        }
    }
    // 调用峰值竞争函数
    std::vector<int> clearPeaks = suppress_peaks(rawPeakIndices, smoothedData);
    
    // 保存所有凹陷结果
    std::map<int, std::vector<SeamResult>> groupResults;
    for (int p_idx : clearPeaks) {
        // 调用趋势坍塌分析函数
        SeamResult current = analyzeSeamStructure(smoothedData, p_idx);
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

// //*********************************************** */
// //*********************************************** */
// //*********************************************** */
// static int clampInt(int value, int minValue, int maxValue) {
//     if (value < minValue) return minValue;
//     if (value > maxValue) return maxValue;
//     return value;
// }

// static cv::Mat normalizeToFloat01(const cv::Mat& src) {
//     cv::Mat srcFloat;
//     src.convertTo(srcFloat, CV_32F);

//     double minVal = 0.0;
//     double maxVal = 0.0;
//     cv::minMaxLoc(srcFloat, &minVal, &maxVal);
//     if (maxVal - minVal < 1e-5) {
//         return cv::Mat::zeros(src.size(), CV_32F);
//     }

//     cv::Mat dst;
//     cv::subtract(srcFloat, cv::Scalar(minVal), dst);
//     dst.convertTo(dst, CV_32F, 1.0 / (maxVal - minVal));
//     return dst;
// }

// static bool findNearestLaserEndpoint(const std::vector<LaserData>& laserData, int laserId, int targetX, const cv::Size& imageSize, cv::Point& endpoint) {
//     bool found = false;
//     int bestGap = std::numeric_limits<int>::max();
//     int bestY = -1;

//     for (const auto& row : laserData) {
//         if (row.laser_id != laserId) continue;

//         int gap = std::abs(row.x_pixel - targetX);
//         if (gap < bestGap) {
//             bestGap = gap;
//             bestY = row.y_pixel;
//             found = true;
//         }
//     }

//     if (!found || imageSize.width <= 0 || imageSize.height <= 0) return false;

//     // SeamTracker 会平移 x_peak，平移后不一定刚好存在同 x 的激光点。
//     // 这里用“最近 x 的激光点”取 y，但保留追踪后的 targetX，让曲线端点跟随稳定轨迹。
//     endpoint.x = clampInt(targetX, 0, imageSize.width - 1);
//     endpoint.y = clampInt(bestY, 0, imageSize.height - 1);
//     return true;
// }

// static std::vector<cv::Point> buildLinePath(cv::Point p1, cv::Point p2, const cv::Size& imageSize) {
//     std::vector<cv::Point> path;
//     int steps = std::max(std::abs(p2.x - p1.x), std::abs(p2.y - p1.y));
//     steps = std::max(steps, 1);
//     path.reserve(steps + 1);

//     for (int i = 0; i <= steps; ++i) {
//         double t = static_cast<double>(i) / steps;
//         int x = static_cast<int>(std::round(p1.x * (1.0 - t) + p2.x * t));
//         int y = static_cast<int>(std::round(p1.y * (1.0 - t) + p2.y * t));
//         path.push_back(cv::Point(
//             clampInt(x, 0, imageSize.width - 1),
//             clampInt(y, 0, imageSize.height - 1)
//         ));
//     }

//     return path;
// }

// static std::vector<cv::Point> smoothCurvePath(const std::vector<cv::Point>& path, int radius, const cv::Size& imageSize) {
//     if (path.size() < 3 || radius <= 0) return path;

//     std::vector<cv::Point> smoothed = path;
//     for (int i = 1; i < static_cast<int>(path.size()) - 1; ++i) {
//         int left = std::max(0, i - radius);
//         int right = std::min(static_cast<int>(path.size()) - 1, i + radius);
//         double sumX = 0.0;
//         int count = 0;

//         for (int j = left; j <= right; ++j) {
//             sumX += path[j].x;
//             count++;
//         }

//         // 路径是逐行动态规划出来的，y 代表扫描行，平滑时只平滑 x，避免曲线行序被打乱。
//         smoothed[i].x = clampInt(static_cast<int>(std::round(sumX / std::max(1, count))), 0, imageSize.width - 1);
//         smoothed[i].y = path[i].y;
//     }

//     // 首尾是两条激光实际找到的橘缝端点，必须固定，不能被平滑移动。
//     smoothed.front() = path.front();
//     smoothed.back() = path.back();
//     return smoothed;
// }

// static cv::Mat buildSeamCostMap(const cv::Mat& roiImage) {
//     if (roiImage.empty()) return {};

//     cv::Mat bgr;
//     cv::Mat gray;
//     if (roiImage.channels() == 4) {
//         cv::cvtColor(roiImage, bgr, cv::COLOR_BGRA2BGR);
//         cv::cvtColor(roiImage, gray, cv::COLOR_BGRA2GRAY);
//     } else if (roiImage.channels() == 3) {
//         bgr = roiImage;
//         cv::cvtColor(roiImage, gray, cv::COLOR_BGR2GRAY);
//     } else {
//         gray = roiImage.clone();
//         cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
//     }

//     cv::Mat smoothGray;
//     cv::GaussianBlur(gray, smoothGray, cv::Size(5, 5), 0);

//     int minSide = std::max(3, std::min(roiImage.cols, roiImage.rows));
//     int kernelSize = std::min(17, minSide);
//     if (kernelSize % 2 == 0) kernelSize--;
//     kernelSize = std::max(3, kernelSize);
//     cv::Mat blackhatKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
//     cv::Mat blackhat;
//     cv::morphologyEx(smoothGray, blackhat, cv::MORPH_BLACKHAT, blackhatKernel);

//     cv::Mat gradX, gradY, gradMag;
//     cv::Sobel(smoothGray, gradX, CV_32F, 1, 0, 3);
//     cv::Sobel(smoothGray, gradY, CV_32F, 0, 1, 3);
//     cv::magnitude(gradX, gradY, gradMag);
//     cv::Mat absGradX;
//     cv::convertScaleAbs(gradX, absGradX);

//     std::vector<cv::Mat> channels;
//     cv::split(bgr, channels);
//     cv::Mat bFloat, gFloat, rFloat;
//     channels[0].convertTo(bFloat, CV_32F);
//     channels[1].convertTo(gFloat, CV_32F);
//     channels[2].convertTo(rFloat, CV_32F);
//     cv::Mat greenRaw = gFloat - 0.5 * (bFloat + rFloat);
//     cv::threshold(greenRaw, greenRaw, 0.0, 0.0, cv::THRESH_TOZERO);

//     cv::Mat hsv;
//     cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
//     std::vector<cv::Mat> hsvChannels;
//     cv::split(hsv, hsvChannels);

//     cv::Mat blackhatEvidence = normalizeToFloat01(blackhat);
//     cv::Mat grayFloat = normalizeToFloat01(smoothGray);
//     cv::Mat ones = cv::Mat::ones(grayFloat.size(), CV_32F);
//     cv::Mat darkEvidence = ones - grayFloat;
//     cv::Mat gradEvidence = normalizeToFloat01(gradMag);
//     cv::Mat verticalEdgeEvidence = normalizeToFloat01(absGradX);
//     cv::Mat brightEvidence = normalizeToFloat01(hsvChannels[2]);
//     cv::Mat lowSaturationEvidence = ones - normalizeToFloat01(hsvChannels[1]);
//     cv::Mat brightPithEvidence = brightEvidence.mul(lowSaturationEvidence);
//     cv::Mat greenPenalty = normalizeToFloat01(greenRaw);

//     // 代价图的含义：越像橘缝，代价越低；越不像橘缝，代价越高。
//     // 橘缝既可能是暗凹槽，也可能是白色/浅黄色橘络，所以这里同时保留两类证据。
//     cv::Mat darkGrooveEvidence = 0.65 * blackhatEvidence + 0.35 * darkEvidence;
//     // 白色橘络通常表现为“亮度高 + 饱和度低”，和橘皮橙色高饱和纹理区分开。
//     cv::Mat pithLineEvidence = 0.75 * brightPithEvidence + 0.25 * verticalEdgeEvidence;
//     // 竖向边界梯度比全方向梯度更偏向分瓣线，弱化横向激光边缘的影响。
//     cv::Mat seamEvidence = 0.35 * darkGrooveEvidence
//                           + 0.45 * pithLineEvidence
//                           + 0.20 * verticalEdgeEvidence;

//     // 绿色激光是测量工具，不是橘缝本体。这里提高绿色高亮区域的代价，避免路径沿激光横线走。
//     cv::Mat cost = ones - seamEvidence + 0.75 * greenPenalty;
//     cv::GaussianBlur(cost, cost, cv::Size(3, 3), 0);
//     return cost;
// }

// static bool traceSingleSeamCurve(const cv::Mat& originImage, cv::Point p1, cv::Point p2, std::vector<cv::Point>& curvePoints, double& meanCost) {
//     if (originImage.empty()) return false;

//     cv::Point start = p1;
//     cv::Point end = p2;
//     if (start.y > end.y) {
//         std::swap(start, end);
//     }

//     const int MIN_VERTICAL_SPAN = 8;
//     const int ROI_X_MARGIN = 70;
//     const int ROI_Y_MARGIN = 8;
//     const int MAX_X_STEP_PER_ROW = 6;
//     const double TRANSITION_WEIGHT = 0.03;
//     const double SHAPE_PRIOR_WEIGHT = 0.18;

//     if (std::abs(end.y - start.y) < MIN_VERTICAL_SPAN) return false;

//     int x0 = clampInt(std::min(start.x, end.x) - ROI_X_MARGIN, 0, originImage.cols - 1);
//     int x1 = clampInt(std::max(start.x, end.x) + ROI_X_MARGIN, 0, originImage.cols - 1);
//     int y0 = clampInt(std::min(start.y, end.y) - ROI_Y_MARGIN, 0, originImage.rows - 1);
//     int y1 = clampInt(std::max(start.y, end.y) + ROI_Y_MARGIN, 0, originImage.rows - 1);
//     if (x1 <= x0 || y1 <= y0) return false;

//     cv::Rect roiRect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
//     cv::Mat costMap = buildSeamCostMap(originImage(roiRect));
//     if (costMap.empty()) return false;

//     cv::Point localStart(start.x - roiRect.x, start.y - roiRect.y);
//     cv::Point localEnd(end.x - roiRect.x, end.y - roiRect.y);
//     localStart.x = clampInt(localStart.x, 0, costMap.cols - 1);
//     localEnd.x = clampInt(localEnd.x, 0, costMap.cols - 1);
//     localStart.y = clampInt(localStart.y, 0, costMap.rows - 1);
//     localEnd.y = clampInt(localEnd.y, 0, costMap.rows - 1);

//     const double INF = std::numeric_limits<double>::max() / 4.0;
//     std::vector<double> prevDp(costMap.cols, INF);
//     std::vector<double> currDp(costMap.cols, INF);
//     cv::Mat parent(costMap.rows, costMap.cols, CV_16S, cv::Scalar(-1));

//     prevDp[localStart.x] = costMap.at<float>(localStart.y, localStart.x);

//     // 动态规划：从上端点逐行走到下端点。
//     // 每下一行只允许 x 移动 MAX_X_STEP_PER_ROW，防止路径突然跳到无关暗纹。
//     // 两点本身不能确定曲线，这里的“图像代价 + 平滑约束”提供了中间曲线的证据。
//     for (int y = localStart.y + 1; y <= localEnd.y; ++y) {
//         std::fill(currDp.begin(), currDp.end(), INF);
//         double t = static_cast<double>(y - localStart.y) / std::max(1, localEnd.y - localStart.y);
//         double lineX = localStart.x * (1.0 - t) + localEnd.x * t;

//         for (int x = 0; x < costMap.cols; ++x) {
//             int left = std::max(0, x - MAX_X_STEP_PER_ROW);
//             int right = std::min(costMap.cols - 1, x + MAX_X_STEP_PER_ROW);
//             double bestPrev = INF;
//             int bestPrevX = -1;

//             for (int px = left; px <= right; ++px) {
//                 if (prevDp[px] >= INF / 2.0) continue;
//                 double transitionCost = TRANSITION_WEIGHT * std::abs(x - px);
//                 double candidate = prevDp[px] + transitionCost;
//                 if (candidate < bestPrev) {
//                     bestPrev = candidate;
//                     bestPrevX = px;
//                 }
//             }

//             if (bestPrevX < 0) continue;

//             // 轻微形状先验：不强迫走直线，只是避免跑到 ROI 边缘的孤立暗点。
//             double shapeCost = SHAPE_PRIOR_WEIGHT * std::min(1.0, std::abs(x - lineX) / (ROI_X_MARGIN + 1.0));
//             currDp[x] = bestPrev + costMap.at<float>(y, x) + shapeCost;
//             parent.at<short>(y, x) = static_cast<short>(bestPrevX);
//         }

//         prevDp.swap(currDp);
//     }

//     double totalCost = prevDp[localEnd.x];
//     if (totalCost >= INF / 2.0) return false;

//     std::vector<cv::Point> reversedPath;
//     int x = localEnd.x;
//     for (int y = localEnd.y; y >= localStart.y; --y) {
//         reversedPath.push_back(cv::Point(roiRect.x + x, roiRect.y + y));
//         if (y == localStart.y) break;

//         int prevX = parent.at<short>(y, x);
//         if (prevX < 0) return false;
//         x = prevX;
//     }

//     std::reverse(reversedPath.begin(), reversedPath.end());
//     curvePoints = smoothCurvePath(reversedPath, 2, originImage.size());
//     meanCost = totalCost / std::max(1, static_cast<int>(curvePoints.size()));
//     return curvePoints.size() >= 2;
// }

// std::vector<SeamCurveResult> traceSeamCurvesByImage(const cv::Mat& originImage, const std::vector<MatchedSeamPair>& seamPairs, const std::vector<LaserData>& laserData) {
//     std::vector<SeamCurveResult> curves;
//     curves.reserve(seamPairs.size());

//     for (const auto& pair : seamPairs) {
//         SeamCurveResult result;
//         result.pair = pair;
//         result.mean_cost = 0.0;
//         result.fallback_to_line = false;

//         cv::Point p1;
//         cv::Point p2;
//         bool hasP1 = findNearestLaserEndpoint(laserData, pair.s1.id, pair.s1.x_peak, originImage.size(), p1);
//         bool hasP2 = findNearestLaserEndpoint(laserData, pair.s2.id, pair.s2.x_peak, originImage.size(), p2);

//         if (!hasP1 || !hasP2) {
//             result.fallback_to_line = true;
//             std::cout << "[SeamCurve] 找不到橘缝端点对应的激光 y，跳过该条曲线。" << std::endl;
//             curves.push_back(result);
//             continue;
//         }

//         if (!traceSingleSeamCurve(originImage, p1, p2, result.curve_points, result.mean_cost)) {
//             // 图像证据不足时退回直线，保证实时显示和后续控制不会因为曲线搜索失败而完全丢点。
//             result.curve_points = buildLinePath(p1, p2, originImage.size());
//             result.fallback_to_line = true;
//         }

//         curves.push_back(result);
//     }

//     return curves;
// }

// // 画出橘缝曲线
// cv::Mat drawSeam(cv::Mat displayImage, const std::vector<SeamCurveResult>& curves) {
//     if (curves.size() < 1) return displayImage;
//     for(int i=0; i<curves.size(); i++){

//         double ratio = (curves.size() > 1) ? (double)i / (curves.size() - 1) : 0.0;
        
//         // 起始颜色 (纯红): (0, 0, 255)
//         // 结束颜色 (浅粉): (180, 180, 255) -> 你可以调整 180 这个值，越大越白
//         int b = (int)(0 + 180 * ratio); 
//         int g = (int)(0 + 180 * ratio);
//         int r = 255; 
//         cv::Scalar currentColor(b, g, r);

//         if (curves[i].curve_points.size() >= 2) {
//             std::vector<std::vector<cv::Point>> polylineGroup;
//             polylineGroup.push_back(curves[i].curve_points);
//             cv::polylines(displayImage, polylineGroup, false, currentColor, 2, cv::LINE_AA);

//             // 画出两个关键点（红色实心圆）
//             cv::circle(displayImage, curves[i].curve_points.front(), 3, cv::Scalar(0, 0, 255), -1);
//             cv::circle(displayImage, curves[i].curve_points.back(), 3, cv::Scalar(0, 0, 255), -1);
//         }
//     }

//     return displayImage;

// }
// //*********************************************** */
// //*********************************************** */
// //*********************************************** */

// 画出橘缝线
cv::Mat drawSeam(cv::Mat displayImage, const std::vector<MatchedSeamPair> results, const std::vector<LaserData> data) {
    if (results.size() < 1) return displayImage;
    int id = 1;
    for(int i=0; i<results.size(); i++){

        double ratio = (results.size() > 1) ? (double)i / (results.size() - 1) : 0.0;
        
        // 起始颜色 (纯红): (0, 0, 255)
        // 结束颜色 (浅粉): (180, 180, 255) -> 你可以调整 180 这个值，越大越白
        int b = (int)(0 + 180 * ratio); 
        int g = (int)(0 + 180 * ratio);
        int r = 255; 
        cv::Scalar currentColor(b, g, r);

        // 查找对应的 y_pixel
        int result_y1 = -1;
        int result_y2 = -1;
        // 第一组
        auto it1 = std::find_if(data.begin(), data.end(), [&](const LaserData& item) {
            return item.laser_id == id && item.x_pixel == results[i].s1.x_peak;
        });
        if (it1 != data.end()) {
            result_y1 = it1->y_pixel;
        }
        // 第二组
        auto it2 = std::find_if(data.begin(), data.end(), [&](const LaserData& item) {
            return item.laser_id == id+1 && item.x_pixel == results[i].s2.x_peak;
        });
        if (it2 != data.end()) {
            result_y2 = it2->y_pixel;
        }
        cv::Point p1(results[i].s1.x_peak, result_y1); 
        cv::Point p2(results[i].s2.x_peak, result_y2);

        // 画连接线（亮黄色）
        cv::line(displayImage, p1, p2, currentColor, 2, cv::LINE_AA);

        // 画出两个关键点（红色实心圆）
        cv::circle(displayImage, p1, 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(displayImage, p2, 3, cv::Scalar(0, 0, 255), -1);
    }

    return displayImage;

}

//**********************yolo************************* */
//*********************************************** */
//*********************************************** */
struct YoloLetterboxInfo {
    float scale = 1.0f;
    int padX = 0;
    int padY = 0;
    int newW = 0;
    int newH = 0;
};

struct YoloSegResult {
    int classId = 0;
    float score = 0.0f;
    cv::Rect box;
    cv::Mat mask;
};

static float sigmoidFloat(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

static cv::Mat yoloLetterbox(const cv::Mat& image, int inputSize, YoloLetterboxInfo& info) {
    const int w = image.cols;
    const int h = image.rows;
    info.scale = std::min(inputSize / static_cast<float>(w), inputSize / static_cast<float>(h));
    info.newW = static_cast<int>(std::round(w * info.scale));
    info.newH = static_cast<int>(std::round(h * info.scale));
    info.padX = (inputSize - info.newW) / 2;
    info.padY = (inputSize - info.newH) / 2;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(info.newW, info.newH));

    cv::Mat output(inputSize, inputSize, image.type(), cv::Scalar(114, 114, 114));
    resized.copyTo(output(cv::Rect(info.padX, info.padY, info.newW, info.newH)));
    return output;
}

static cv::Mat yoloDetectionsToRows(const cv::Mat& blob) {
    if (blob.dims == 2) return blob.clone();
    if (blob.dims != 3 || blob.size[0] != 1) {
        throw std::runtime_error("YOLO detection output shape unsupported.");
    }

    cv::Mat mat(blob.size[1], blob.size[2], CV_32F, const_cast<float*>(blob.ptr<float>()));
    if (blob.size[1] < blob.size[2]) {
        cv::Mat transposed;
        cv::transpose(mat, transposed);
        return transposed.clone();
    }
    return mat.clone();
}

static cv::Mat yoloProtoToRows(const cv::Mat& proto, int& maskDim, int& maskH, int& maskW) {
    if (proto.dims != 4 || proto.size[0] != 1) {
        throw std::runtime_error("YOLO mask prototype output shape unsupported.");
    }
    maskDim = proto.size[1];
    maskH = proto.size[2];
    maskW = proto.size[3];
    return cv::Mat(maskDim, maskH * maskW, CV_32F, const_cast<float*>(proto.ptr<float>())).clone();
}

static cv::Mat buildYoloMask(
    const std::vector<float>& coeff,
    const cv::Mat& protoRows,
    int maskH,
    int maskW,
    const YoloLetterboxInfo& info,
    cv::Size originalSize,
    int inputSize
) {
    cv::Mat coeffMat(1, static_cast<int>(coeff.size()), CV_32F, const_cast<float*>(coeff.data()));
    cv::Mat maskFlat = coeffMat * protoRows;
    cv::Mat mask(maskH, maskW, CV_32F, maskFlat.ptr<float>());

    cv::Mat prob(maskH, maskW, CV_32F);
    for (int y = 0; y < maskH; ++y) {
        const float* src = mask.ptr<float>(y);
        float* dst = prob.ptr<float>(y);
        for (int x = 0; x < maskW; ++x) {
            dst[x] = sigmoidFloat(src[x]);
        }
    }

    cv::Mat inputMask;
    cv::resize(prob, inputMask, cv::Size(inputSize, inputSize), 0, 0, cv::INTER_LINEAR);

    cv::Rect roi(info.padX, info.padY, info.newW, info.newH);
    roi &= cv::Rect(0, 0, inputMask.cols, inputMask.rows);
    cv::Mat unpadded = inputMask(roi).clone();

    cv::Mat originalProb;
    cv::resize(unpadded, originalProb, originalSize, 0, 0, cv::INTER_LINEAR);

    cv::Mat binary;
    cv::threshold(originalProb, binary, 0.5, 255.0, cv::THRESH_BINARY);
    binary.convertTo(binary, CV_8U);
    return binary;
}

static cv::dnn::Net* getYoloSegNet() {
    static cv::dnn::Net net;
    static bool triedLoad = false;
    static bool loaded = false;

    if (!YOLO_SEAM_ENABLE) return nullptr;
    if (!triedLoad) {
        triedLoad = true;
        try {
            net = cv::dnn::readNetFromONNX(YOLO_SEAM_MODEL_PATH);
            loaded = !net.empty();
            if (loaded) {
                std::cout << "[YOLO-seg] 模型加载完成: " << YOLO_SEAM_MODEL_PATH << std::endl;
            } else {
                std::cerr << "[YOLO-seg] 模型为空，使用传统橘缝曲线。" << std::endl;
            }
        } catch (const std::exception& e) {
            loaded = false;
            std::cerr << "[YOLO-seg] 模型加载失败: " << e.what() << "，使用传统橘缝曲线。" << std::endl;
        }
    }

    return loaded ? &net : nullptr;
}

static std::vector<YoloSegResult> inferYoloSeg(const cv::Mat& image) {
    std::vector<YoloSegResult> results;
    cv::dnn::Net* net = getYoloSegNet();
    if (net == nullptr || image.empty()) return results;

    try {
        YoloLetterboxInfo info;
        int inputSize = std::max(32, YOLO_SEAM_INPUT_SIZE);
        cv::Mat input = yoloLetterbox(image, inputSize, info);
        cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(inputSize, inputSize), cv::Scalar(), true, false);
        net->setInput(blob);

        std::vector<cv::Mat> outputs;
        net->forward(outputs, net->getUnconnectedOutLayersNames());
        if (outputs.size() < 2) return results;

        cv::Mat detRows = yoloDetectionsToRows(outputs[0]);
        int maskDim = 0;
        int maskH = 0;
        int maskW = 0;
        cv::Mat protoRows = yoloProtoToRows(outputs[1], maskDim, maskH, maskW);
        const int numClasses = detRows.cols - 4 - maskDim;
        if (numClasses <= 0) return results;

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> classIds;
        std::vector<std::vector<float>> coeffs;
        float confThreshold = static_cast<float>(YOLO_SEAM_CONF_THRESHOLD);

        for (int i = 0; i < detRows.rows; ++i) {
            const float* row = detRows.ptr<float>(i);
            int bestClass = 0;
            float bestScore = row[4];
            for (int c = 1; c < numClasses; ++c) {
                if (row[4 + c] > bestScore) {
                    bestScore = row[4 + c];
                    bestClass = c;
                }
            }
            if (bestScore < confThreshold) continue;

            const float cx = row[0];
            const float cy = row[1];
            const float bw = row[2];
            const float bh = row[3];
            int left = static_cast<int>(std::round((cx - bw * 0.5f - info.padX) / info.scale));
            int top = static_cast<int>(std::round((cy - bh * 0.5f - info.padY) / info.scale));
            int width = static_cast<int>(std::round(bw / info.scale));
            int height = static_cast<int>(std::round(bh / info.scale));
            cv::Rect box(left, top, width, height);
            box &= cv::Rect(0, 0, image.cols, image.rows);
            if (box.area() <= 0) continue;

            std::vector<float> coeff(maskDim);
            for (int k = 0; k < maskDim; ++k) {
                coeff[k] = row[4 + numClasses + k];
            }

            boxes.push_back(box);
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
            coeffs.push_back(std::move(coeff));
        }

        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, confThreshold, static_cast<float>(YOLO_SEAM_NMS_THRESHOLD), keep);

        for (int idx : keep) {
            YoloSegResult result;
            result.classId = classIds[idx];
            result.score = scores[idx];
            result.box = boxes[idx];
            result.mask = buildYoloMask(coeffs[idx], protoRows, maskH, maskW, info, image.size(), inputSize);

            cv::Mat boxMask = cv::Mat::zeros(result.mask.size(), CV_8U);
            cv::rectangle(boxMask, result.box, 255, cv::FILLED);
            cv::bitwise_and(result.mask, boxMask, result.mask);
            results.push_back(std::move(result));
        }

        std::sort(results.begin(), results.end(), [](const YoloSegResult& a, const YoloSegResult& b) {
            return a.score > b.score;
        });
    } catch (const std::exception& e) {
        std::cerr << "[YOLO-seg] 推理失败: " << e.what() << std::endl;
    }

    return results;
}

static void yoloThinningIteration(cv::Mat& img, int iter) {
    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);
    for (int i = 1; i < img.rows - 1; ++i) {
        for (int j = 1; j < img.cols - 1; ++j) {
            uchar p2 = img.at<uchar>(i - 1, j);
            uchar p3 = img.at<uchar>(i - 1, j + 1);
            uchar p4 = img.at<uchar>(i, j + 1);
            uchar p5 = img.at<uchar>(i + 1, j + 1);
            uchar p6 = img.at<uchar>(i + 1, j);
            uchar p7 = img.at<uchar>(i + 1, j - 1);
            uchar p8 = img.at<uchar>(i, j - 1);
            uchar p9 = img.at<uchar>(i - 1, j - 1);

            int transitions = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                              (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                              (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                              (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int neighbors = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (transitions == 1 && neighbors >= 2 && neighbors <= 6 && m1 == 0 && m2 == 0) {
                marker.at<uchar>(i, j) = 1;
            }
        }
    }
    img &= ~marker;
}

static cv::Mat skeletonizeYoloMask(const cv::Mat& binaryMask) {
    cv::Mat img;
    cv::threshold(binaryMask, img, 1, 1, cv::THRESH_BINARY);
    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        yoloThinningIteration(img, 0);
        yoloThinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);

    img *= 255;
    return img;
}

static double yoloMaskPointCost(const cv::Mat& mask, const std::vector<cv::Point>& laserPoints) {
    if (laserPoints.empty()) return 0.0;

    cv::Mat inv;
    cv::threshold(mask, inv, 1, 255, cv::THRESH_BINARY_INV);
    cv::Mat dist;
    cv::distanceTransform(inv, dist, cv::DIST_L2, 3);

    double cost = 0.0;
    for (const cv::Point& p : laserPoints) {
        if (p.x < 0 || p.y < 0 || p.x >= mask.cols || p.y >= mask.rows) {
            cost += 1000.0;
        } else {
            cost += dist.at<float>(p.y, p.x);
        }
    }
    return cost / static_cast<double>(laserPoints.size());
}

static std::set<int> selectYoloMasksByLaserPoints(
    const std::vector<YoloSegResult>& results,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData,
    const cv::Size& imageSize
) {
    std::set<int> selected;
    if (results.empty()) return selected;

    for (const auto& pair : seamPairs) {
        cv::Point p1;
        cv::Point p2;

        std::vector<cv::Point> laserPoints = {p1, p2};
        int bestIdx = -1;
        double bestCost = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(results.size()); ++i) {
            double cost = yoloMaskPointCost(results[i].mask, laserPoints) - 10.0 * results[i].score;
            if (cost < bestCost) {
                bestCost = cost;
                bestIdx = i;
            }
        }
        if (bestIdx >= 0) selected.insert(bestIdx);
    }

    return selected;
}

static cv::Mat drawYoloSegResults(
    cv::Mat displayImage,
    const std::vector<YoloSegResult>& results,
    const std::set<int>& selectedMaskIds
) {
    if (results.empty()) return displayImage;

    const std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 165, 255),
        cv::Scalar(255, 0, 255),
        cv::Scalar(255, 255, 0),
        cv::Scalar(0, 255, 255),
    };

    cv::Mat maskLayer = displayImage.clone();
    int drawCount = YOLO_SEAM_DRAW_ALL ? static_cast<int>(results.size()) : std::min(1, static_cast<int>(results.size()));
    for (int i = 0; i < drawCount; ++i) {
        cv::Mat colorImage(displayImage.size(), displayImage.type(), colors[i % colors.size()]);
        colorImage.copyTo(maskLayer, results[i].mask);
    }

    cv::Mat overlay;
    cv::addWeighted(maskLayer, 0.35, displayImage, 0.65, 0.0, overlay);

    for (int i = 0; i < drawCount; ++i) {
        bool selected = selectedMaskIds.count(i) > 0;
        cv::Scalar color = selected ? cv::Scalar(0, 255, 0) : colors[i % colors.size()];
        int width = selected ? 3 : 2;

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(results[i].mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(overlay, contours, -1, color, width);

        cv::Mat skeleton = skeletonizeYoloMask(results[i].mask);
        for (int y = 0; y < skeleton.rows; ++y) {
            for (int x = 0; x < skeleton.cols; ++x) {
                if (skeleton.at<uchar>(y, x) > 0) {
                    overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                }
            }
        }

        cv::rectangle(overlay, results[i].box, color, width);
        cv::putText(
            overlay,
            "yolo" + std::to_string(i) + " " + std::to_string(results[i].score).substr(0, 4),
            results[i].box.tl() + cv::Point(0, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv::LINE_AA
        );
    }

    return overlay;
}
//**********************yolo************************* */
//*********************************************** */
//*********************************************** */

// 检测主函数
cv::Mat detectMain(cv::Mat originImage){
    cv::Mat displayImage;
    std::vector<LaserData> data = detectLaserCenter(originImage, &displayImage);
    std::vector<LaserData> smoothData = smooth(data);

    std::vector<LaserBaselineData> baselineData = getLaserBaselineDistance(smoothData, 80, 0.25);
    // saveLaserBaselineCSV(baselineData, vConfig.csv_path + getTimeString() + "_points_baseline.csv");
    std::vector<LaserData> seamSignalData = buildSeamSignalData(baselineData);

    std::vector<MatchedSeamPair> results = findSeam(seamSignalData);
    std::vector<MatchedSeamPair> stableResults = seamTracker.update(results);
    // std::vector<SeamCurveResult> curveResults = traceSeamCurvesByImage(originImage, stableResults, data);
    // cv::Mat finalMat = drawSeam(displayImage, curveResults);
    cv::Mat finalMat = drawSeam(displayImage, stableResults, data);

    // cv::Mat finalMat = drawSeam(displayImage, results, data);


    std::vector<YoloSegResult> yoloResults = inferYoloSeg(originImage);
    if (!yoloResults.empty()) {
        std::set<int> selectedMaskIds = selectYoloMasksByLaserPoints(yoloResults, stableResults, data, originImage.size());
        finalMat = drawYoloSegResults(finalMat, yoloResults, selectedMaskIds);
        std::cout << "[YOLO-seg] 橘缝数量: " << yoloResults.size();
        if (!selectedMaskIds.empty()) {
            std::cout << ", 激光约束匹配数量: " << selectedMaskIds.size();
        }
        std::cout << std::endl;
    }

    cv::waitKey(1);


    return finalMat;
}




