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
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);                     // 有的驱动 1=手动，3=自动，需测试// 2. 关闭背光补偿 (OpenCV 对应宏是 CAP_PROP_BACKLIGHT)
    cap.set(cv::CAP_PROP_BACKLIGHT, 0);                         // 关闭背光补偿
    cap.set(cv::CAP_PROP_SHARPNESS, 100);                       // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, vConfig.brightness);       // 设置亮度为 50(-64 ~ 64)
    cap.set(cv::CAP_PROP_EXPOSURE, vConfig.exposure_time);      // 曝光时间整数ms(最小值50ms)
    // cap.set(cv::CAP_PROP_EXPOSURE, 5000);      // 曝光时间整数ms(最小值50ms)
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
        

        cv::Mat displayImage;
        std::vector<LaserData> data = detectLaserCenter(frame, &displayImage);
        std::vector<LaserData> smoothData = smooth(data);
        std::vector<MatchedSeamPair> results = findSeam(smoothData);
        std::vector<MatchedSeamPair> stableResults = seamTracker.update(results);
        cv::Mat finalMat = drawSeam(displayImage, stableResults, data);
        std::cout << "#################################################################" << std::endl;

        cv::imshow("Camera Video", finalMat);
        cv::waitKey(200);
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
// 保存多张原始图像
int saveVedio(){
    cv::VideoCapture cap(2, cv::CAP_V4L2);
    // cv::VideoCapture cap;
    // cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << 2 << std::endl;
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


    while (true) {
        cap >> origin_frame;
        cv::undistort(origin_frame, frame, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }


        std::string filename  = "origin_" + getTimeString() + ".jpg";
        std::string save_path = vConfig.origin_img_path + filename;
        // cv::imshow("Camera Video", frame);
        cv::waitKey(10);
        cv::imwrite(save_path, frame); 
        std::cout << "图像已保存到 " << save_path << std::endl;
        cv::imshow("Camera Video", frame);
        cv::waitKey(1000);
        // sleep(1);
        // detect_laser_edge(cv::imread("/home/dw/robot/image/1.jpg"));
        // detect_laser_center(cv::imread("/home/dw/robot/image/1.jpg"));
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
    cv::undistort(origin_frame, frame, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
    if (frame.empty()) {
        std::cerr << "无法获取图像帧。" << std::endl;
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
    int n = data.size();

    // 中间部分使用滑动窗口卷积
    for (int i = 0; i < n; ++i) {
        double sum = 0;
        int count = 0;
        for (int j = i - 5; j <= i + 5; ++j) {
            if (j >= 0 && j < n) {
                sum += data[j].distance_cm;
                count++;
            }
        }
        double val = sum / count;
        LaserData row = data[i];
        row.distance_cm = val;
        smoothedData.push_back(row);
        // ofs << data[i].laser_id << "," << data[i].x_pixel << "," << data[i].y_pixel << "," << val << "\n";
    }
    std::cout << "data size: " << data.size() << ", smoothed size: " << smoothedData.size() << std::endl;
    return smoothedData;
}

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

    // // --- 新增：激光断裂处检测 (检测跳变) ---
    // for (int i = 1; i < n; ++i) {
    //     // 只有同一条激光线内的相邻点才计算跳变
    //     if (smoothedData[i].laser_id == smoothedData[i-1].laser_id) {
    //         int dx = std::abs(smoothedData[i].x_pixel - smoothedData[i-1].x_pixel);
    //         int dy = std::abs(smoothedData[i].y_pixel - smoothedData[i-1].y_pixel);

    //         // 如果跳变超过阈值 (10像素)
    //         if (dx > 10 || dy > 10) {
    //             SeamResult gapSeam;
    //             gapSeam.id = smoothedData[i].laser_id;
    //             // 取中点作为橘缝坐标
    //             gapSeam.x_peak = (smoothedData[i].x_pixel + smoothedData[i-1].x_pixel) / 2;
    //             gapSeam.dist = (smoothedData[i].distance_cm + smoothedData[i-1].distance_cm) / 2.0;
                
    //             // 模拟坡脚和宽度（由于是断裂，给一个理想化的评分参数）
    //             gapSeam.left_foot = smoothedData[i-1].x_pixel;
    //             gapSeam.right_foot = smoothedData[i].x_pixel;
    //             gapSeam.width = dx;
    //             gapSeam.depth = vConfig.best_depth; // 断裂处默认深度很大
                
    //             // 给断裂处一个极高的初始评分，因为它通常是最明显的特征
    //             gapSeam.score = 1.0 * vConfig.ratio_width + 1.0 * vConfig.ratio_depth;

    //             // 同样进行区域过滤
    //             if(gapSeam.x_peak > 50 && gapSeam.x_peak < 600) {
    //                 groupResults[gapSeam.id].push_back(gapSeam);
    //                 std::cout << "[断裂检测] 发现跳变橘缝: id=" << gapSeam.id 
    //                           << ", x=" << gapSeam.x_peak << " (Gap dx:" << dx << " dy:" << dy << ")" << std::endl;
    //             }
    //         }
    //     }
    // }


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

// 检测主函数
int detectMain(cv::Mat originImage){
    cv::Mat displayImage;
    std::vector<LaserData> data = detectLaserCenter(originImage, &displayImage);
    std::vector<LaserData> smoothData = smooth(data);
    std::vector<MatchedSeamPair> results = findSeam(smoothData);
    std::vector<MatchedSeamPair> stableResults = seamTracker.update(results);
    cv::Mat finalMat = drawSeam(displayImage, stableResults, data);

    // cv::Mat finalMat = drawSeam(displayImage, results, data);




    // std::string filename  = getTimeString() + "_displayImage" + ".jpg";
    // std::string save_path = vConfig.proc_path + filename;
    // cv::imwrite(save_path, finalMat);
    // cv::imshow("Final Detection", finalMat);
    cv::waitKey(1);

    return 0;
}




