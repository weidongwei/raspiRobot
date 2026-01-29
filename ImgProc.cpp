#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>
#include <set>
#include <nlohmann/json.hpp>

#include "ImgProc.h"
using json = nlohmann::json;


VisualConfig vConfig;

// 加载json配置文件
bool loadVisualConfig(VisualConfig& cfg, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    json j;
    file >> j;
    cfg.origin_img_path = j["origin_img_path"];
    cfg.base_path = j["base_path"];
    cfg.laser_duty = j["laser_duty"];
    cfg.exposure_time = j["exposure_time"];
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
    cap.set(cv::CAP_PROP_BRIGHTNESS, 0);                        // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, vConfig.MycameraMatrix, vConfig.MydistCoeffs);

        cv::Mat displayImage;
        std::vector<LaserData> data = detectLaserCenter(undistorted, &displayImage);
        std::vector<LaserData> smoothData = smooth(data);
        std::vector<MatchedSeamPair> results = findSeam(smoothData);
        cv::Mat finalMat = drawSeam(displayImage, results, data);
        std::cout << "#################################################################" << std::endl;

        cv::imshow("Camera Video", finalMat);
        cv::waitKey(100);
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
// 保存多张原始图像
int saveVedio(){
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
    cap.set(cv::CAP_PROP_BRIGHTNESS, 0);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }

        cv::Mat undistorted;
        cv::undistort(frame, undistorted, vConfig.MycameraMatrix, vConfig.MydistCoeffs);

        std::string filename  = "origin_" + getTimeString() + ".jpg";
        std::string save_path = vConfig.origin_img_path + filename;
        // cv::imshow("Camera Video", undistorted);
        cv::waitKey(10);
        cv::imwrite(save_path, undistorted); 
        std::cout << "图像已保存到 " << save_path << std::endl;
        cv::imshow("Camera Video", undistorted);
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
    cap.set(cv::CAP_PROP_BRIGHTNESS, 0);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    cap >> frame;
    if (frame.empty()) {
        std::cerr << "无法获取图像帧。" << std::endl;
    }
    cv::Mat undistorted;
    cv::undistort(frame, undistorted, vConfig.MycameraMatrix, vConfig.MydistCoeffs);
    cv::waitKey(10);
    cv::imwrite(save_path, undistorted); 
    std::cout << "图像已保存到 " << save_path << std::endl;
    sleep(1);

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

bool calibrateCameraFromImages(
    const std::vector<cv::Mat>& images,
    cv::Size boardSize,
    float squareSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs,
    double& reprojError,
    const std::string& debugSaveDir
) {
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    // 构造棋盘世界坐标
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            objp.emplace_back(j * squareSize,
                              i * squareSize,
                              0.0f);
        }
    }

    cv::Size imageSize;
    int imgIdx = 0;

    for (const auto& img : images) {
        if (img.empty()) {
            imgIdx++;
            continue;
        }

        imageSize = img.size();

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray,
            boardSize,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE
        );

        // ---------- 保存中间图像 ----------
        cv::Mat vis = img.clone();

        if (found) {
            cv::cornerSubPix(
                gray,
                corners,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS |
                    cv::TermCriteria::COUNT,
                    30, 0.001
                )
            );

            imagePoints.push_back(corners);
            objectPoints.push_back(objp);

            cv::drawChessboardCorners(
                vis,
                boardSize,
                corners,
                found
            );
        }

        // 保存图像（无论成功还是失败）
        std::ostringstream name;
        name << debugSaveDir << "/calib_"
             << imgIdx
             << (found ? "_ok.jpg" : "_fail.jpg");

        cv::imwrite(name.str(), vis);
        imgIdx++;
    }

    if (imagePoints.size() < 10) {
        std::cerr << "Not enough valid calibration images." << std::endl;
        return false;
    }

    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs
    );

    // ---------- 计算重投影误差 ----------
    double totalError = 0.0;
    size_t totalPoints = 0;

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            objectPoints[i],
            rvecs[i],
            tvecs[i],
            cameraMatrix,
            distCoeffs,
            projected
        );

        double err = cv::norm(imagePoints[i], projected, cv::NORM_L2);
        totalError += err * err;
        totalPoints += objectPoints[i].size();
    }

    reprojError = std::sqrt(totalError / totalPoints);

    return true;
}

//相机标定
int biaoding(){
    std::vector<cv::Mat> images;
    // 读取标定图像
    for (int i = 1; i <= 22; ++i) {
        std::string path = "/home/dw/robot/image/origin_image/biaoding" + std::to_string(i) + ".jpg";
        cv::Mat img = cv::imread(path);
        if (!img.empty())
            images.push_back(img);
    }

    cv::Mat cameraMatrix, distCoeffs;
    double reprojError;

    bool ok = calibrateCameraFromImages(
        images,
        cv::Size(11, 8),
        3.0f,
        cameraMatrix,
        distCoeffs,
        reprojError,
        "/home/dw/robot/image/origin_image"
    );

    if (ok) {
        std::cout << "\n===== Camera Calibration Result =====\n";

        std::cout << "Camera Matrix (K):\n";
        std::cout << cameraMatrix << std::endl;

        std::cout << "\nDistortion Coefficients (D):\n";
        std::cout << distCoeffs << std::endl;

        std::cout << "\nReprojection Error:\n";
        std::cout << reprojError << std::endl;

        std::cout << "=====================================\n";
    }
    return 0;
}

//激光像素坐标转实际距离,平面到喷嘴的距离
double y_pixel_to_distance1(double y_pixel) {
    double a = 5.63095238;
    double b = -97.55952381;
    double c = 462.5952381;

    double A = a;
    double B = b;
    double C = c - y_pixel;

    double delta = B*B - 4*A*C;
    if (delta < 0) return -1; // 无解

    double d1 = (-B + sqrt(delta)) / (2*A);
    double d2 = (-B - sqrt(delta)) / (2*A);


    return (d1 >= 0 && d1 <= 9) ? d1 : d2;
}

double y_pixel_to_distance2(double y_pixel) {
    double a = 4.35714286;
    double b = -87.23809524;
    double c = 523.88095238;

    double A = a;
    double B = b;
    double C = c - y_pixel;

    double delta = B*B - 4*A*C;
    if (delta < 0) return -1; // 无解

    double d1 = (-B + sqrt(delta)) / (2*A);
    double d2 = (-B - sqrt(delta)) / (2*A);


    return (d1 >= 0 && d1 <= 9) ? d1 : d2;
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



// 预处理函数(去畸变、绿色通道增强及平滑处理)
cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut) {
    // 去畸变 (保存 undistortedOut 用于后续绘图)
    cv::undistort(input, undistortedOut, vConfig.MycameraMatrix, vConfig.MydistCoeffs);

    // 通道增强（绿色提取）
    std::vector<cv::Mat> bgr;
    cv::split(undistortedOut, bgr);
    cv::Mat diff = bgr[1] - 0.5 * (bgr[0] + bgr[2]);
    
    cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
    diff.convertTo(diff, CV_8U);
    cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0);
    return diff;
}

// 计算当前轮廓组合的得分
double calculateScore(const std::vector<std::vector<cv::Point>>& contours) {
    if (contours.empty()) return 0.0;
    struct LaserMetric {
        double length;
        double area;
    };

    std::vector<LaserMetric> metrics;
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < 10.0) continue; 
        double length = cv::arcLength(cnt, false) * 0.5;
        metrics.push_back({length, area});
    }

    if (metrics.empty()) return 0.0;
    
    // 按长度排序，取前两个
    std::sort(metrics.begin(), metrics.end(), [](const LaserMetric& a, const LaserMetric& b){
        return a.length > b.length;
    });

    double total_len = 0;
    double total_area = 0;
    int count = std::min((int)metrics.size(), 2); // 确保不会超过实际数量
    for (int i = 0; i < count; ++i) {
        total_len += metrics[i].length;
        total_area += metrics[i].area;
    }

    // 长度评分
    double s_len = 0.0;
    double ideal_l = vConfig.best_laser_length;
    if (total_len <= ideal_l) {
        s_len = total_len / ideal_l;
    } else {
        s_len = std::max(0.0, 1.0 - (total_len - ideal_l) / (ideal_l/2));
    }

    // 细度评分
    double s_thin = 0.0;
    double ideal_w = vConfig.best_laser_width;
    double ideal_a = ideal_l * ideal_w;

    if (total_area <= ideal_a) {
        s_thin = total_area / ideal_a;
    } else {
        s_thin = std::max(0.0, 1.0 - (total_area - ideal_a) / (ideal_a + 1e-5));
    }
    

    // 综合加权
    double final_score = (s_len * vConfig.ratio_laser_length) + (s_thin * vConfig.ratio_laser_width);

    return final_score;
}

// 轮廓检测与过滤
std::vector<std::vector<cv::Point>> getLaserContours(const cv::Mat& diff) {
    double minVal, maxVal;
    cv::minMaxLoc(diff, &minVal, &maxVal);
    
    int start_thresh = static_cast<int>(maxVal * 0.65); 
    int end_thresh = vConfig.threshold_value_min;
    int step = vConfig.threshold_value_rate;

    double max_total_score = -1.0;
    int best_thresh = start_thresh;
    std::vector<std::vector<cv::Point>> best_contours;

    for (int thresh = start_thresh; thresh >= end_thresh; thresh -= step) {
        cv::Mat mask;
        cv::threshold(diff, mask, thresh, 255, cv::THRESH_BINARY);
        
        std::vector<std::vector<cv::Point>> current_contours;
        cv::findContours(mask, current_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 调用新评分系统
        double score = calculateScore(current_contours);

        
        // 记录最优解
        if (score > max_total_score) {
            max_total_score = score;
            best_thresh = thresh;
            best_contours = current_contours;
        }
    }

    std::cout << "最佳阈值: " << best_thresh << " 综合得分: " << max_total_score << std::endl;

    // 最后的输出过滤：依然按面积保留前两个
    std::sort(best_contours.begin(), best_contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });
    
    if (best_contours.size() > 2) best_contours.resize(2);
    return best_contours;

}

// 中心线几何提取
std::vector<LaserContour> extractCenterlinePoints(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& diff) {
    std::vector<LaserContour> laser_contours;

    // 权重设置
    const double weight_top = 1;
    const double weight_center = 1.0 - weight_top;

    for (int i = 0; i < contours.size(); ++i) {
        cv::Mat mask = cv::Mat::zeros(diff.size(), CV_8U);
        
        // std::string filename  = "test.jpg";
        // std::string save_path = vConfig.base_path + filename;
        // cv::imwrite(save_path, diff);
        
        cv::drawContours(mask, contours, i, cv::Scalar(255), cv::FILLED);
        
        cv::Rect box = cv::boundingRect(contours[i]);
        LaserContour lc;
        double y_sum = 0;

        for (int x = box.x; x < box.x + box.width; ++x) {
            int first_y = -1, last_y = -1;
            for (int y = box.y; y < box.y + box.height; ++y) {
                if (mask.at<uchar>(y, x) > 0 && diff.at<uchar>(y, x) > 0) {
                    if (first_y == -1) first_y = y;
                    last_y = y;
                }
            }
            if (first_y != -1) {
                int center_y = (first_y + last_y) / 2;
                // int center_y = first_y;
                
                int top_y = first_y;
                int fused_y = std::round(center_y * weight_center + top_y * weight_top);

                lc.xs.push_back(x);
                lc.ys.push_back(fused_y);
                y_sum += fused_y;
            }
        }
        if (!lc.ys.empty()) {
            lc.y_average = y_sum / lc.ys.size();
            laser_contours.push_back(lc);
        }
    }
    
    // 判定上下激光类型
    if (laser_contours.size() == 2) {
        bool firstIsUpper = laser_contours[0].y_average < laser_contours[1].y_average;
        laser_contours[0].laser_type = firstIsUpper ? 1 : 2;
        laser_contours[1].laser_type = firstIsUpper ? 2 : 1;
    } else if (laser_contours.size() == 1) {
        laser_contours[0].laser_type = 1; // 默认
    }
    
    return laser_contours;
}

// 保存结果与可视化
cv::Mat saveAndVisualize(const std::vector<std::vector<cv::Point>>& contours, const std::vector<LaserContour>& lcs, cv::Mat& canvas, const cv::Mat& diff, std::vector<LaserData>& outData) {
    // 红色轮廓绘制
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > canvas.cols * canvas.rows * 0.0002) {
            cv::drawContours(canvas, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
        }
    }

    // std::string timeStr = getTimeString();
    // std::ofstream ofs(vConfig.base_path + timeStr + "_points.csv");
    // ofs << "laser_id,x_pixel,y_pixel,distance_cm\n";

    cv::Mat mask_center = cv::Mat::zeros(diff.size(), CV_8U);

    // 提取数据并准备画中心点
    for (const auto& lc : lcs) {
        for (size_t i = 0; i < lc.xs.size(); ++i) {
            int x = lc.xs[i], y = lc.ys[i];
            double dis = (lc.laser_type == 1) ? y_pixel_to_distance1(y) : y_pixel_to_distance2(y);
            if (dis <= 0) continue;

            outData.push_back({lc.laser_type, x, y, dis});
            // ofs << lc.laser_type << "," << x << "," << y << "," << dis << "\n";
            mask_center.at<uchar>(y, x) = 255;
        }
    }
    
    // 闭运算修复断线并画出蓝色中心点
    cv::morphologyEx(mask_center, mask_center, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1)));
    for (int y = 0; y < mask_center.rows; ++y) {
        for (int x = 0; x < mask_center.cols; ++x) {
            if (mask_center.at<uchar>(y, x) > 0)
                cv::circle(canvas, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
        }
    }

    // cv::imwrite(vConfig.base_path + timeStr + "_detected.jpg", canvas);
    // ofs.close();
    return canvas;
}

// 检测激光中心线主函数
std::vector<LaserData> detectLaserCenter(cv::Mat image, cv::Mat* imageOut) {
    cv::Mat undistortedImg;
    std::vector<LaserData> laserPoints;

    // 1. 预处理
    cv::Mat diff = preprocessLaserImage(image, undistortedImg);

    // 2. 找轮廓
    auto contours = getLaserContours(diff);
    if (contours.empty()) return {};

    // 3. 提取中心线坐标
    auto lcs = extractCenterlinePoints(contours, diff);

    // 4. 保存与可视化
    cv::Mat canvas = saveAndVisualize(contours, lcs, undistortedImg, diff, laserPoints);
    
    *imageOut = canvas;

    std::cout << "激光中心线检测完成。" << std::endl;
    return laserPoints;
}



// 平滑11个点
std::vector<LaserData> smooth(const std::vector<LaserData> data) {
    std::vector<LaserData> smoothedData;

    std::string fname  = getTimeString() + "_points_smooth" + ".csv";
    std::string savePath = vConfig.base_path + fname;
    std::ofstream ofs(savePath);
    ofs << "laser_id,x_pixel,y_pixel,distance_cm\n";
    
    int n = data.size();

    // 中间部分使用滑动窗口卷积
    for (int i = 5; i < n - 5; ++i) {
        double val = (data[i-5].distance_cm  +
                     data[i-4].distance_cm  +
                     data[i-3].distance_cm  +
                     data[i-2].distance_cm  +
                     data[i-1].distance_cm  +
                     data[i].distance_cm  +
                     data[i+1].distance_cm  +
                     data[i+2].distance_cm  +
                     data[i+3].distance_cm  +
                     data[i+4].distance_cm  +
                     data[i+5].distance_cm )/11;
        LaserData row;
        row.laser_id = data[i-5].laser_id;
        row.x_pixel = data[i-5].x_pixel;
        row.y_pixel = data[i-5].y_pixel;
        row.distance_cm = val;
        smoothedData.push_back(row);
        ofs << data[i].laser_id << "," << data[i].x_pixel << "," << data[i].y_pixel << "," << val << "\n";

    }
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

// 趋势坍塌分析函数(激光数据, 峰值索引, 耐心值, 最佳宽度, 最佳深度, 宽度权重比, 深度权重比)
SeamResult analyzeSeamStructure(const std::vector<LaserData>& data, int peakIdx) {
    SeamResult res;
    res.id = data[peakIdx].laser_id;
    res.x_peak = data[peakIdx].x_pixel;
    res.dist = data[peakIdx].distance_cm;
    
    int n = data.size();

    // 向右寻找坡脚
    int right = peakIdx;
    int patience = vConfig.patience_limit;
    for (int j = peakIdx + 1; j < n; ++j) {
        if (data[j].distance_cm <= data[j-1].distance_cm) {
            patience = vConfig.patience_limit;
        } else {patience--;}
        right = j;
        if (patience <= 0) break;
    }
    // 向左寻找坡脚
    int left = peakIdx;
    patience = vConfig.patience_limit;
    for (int j = peakIdx - 1; j > 0; --j) {
        if (data[j].distance_cm <= data[j+1].distance_cm) {
            patience = vConfig.patience_limit;
        } else {patience--;}
        left = j;
        if (patience <= 0) break;
    }
    res.left_foot = data[left].x_pixel;
    res.right_foot = data[right].x_pixel;

    // 获取凹陷宽度和深度
    res.width = std::abs(data[right].x_pixel - data[left].x_pixel);
    double base_dist = (data[left].distance_cm + data[right].distance_cm) / 2.0;
    res.depth = res.dist - base_dist;


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
    cv::Mat finalMat = drawSeam(displayImage, results, data);

    std::string filename  = getTimeString() + "_displayImage" + ".jpg";
    std::string save_path = vConfig.base_path + filename;
    cv::imwrite(save_path, finalMat);

    return 0;
}



