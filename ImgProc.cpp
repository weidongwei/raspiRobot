#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>

#include "ImgProc.h"


cv::Mat MycameraMatrix = (cv::Mat_<double>(3, 3) <<
    622.8518195651313, 0, 324.9937701989255,
    0, 622.3557983623182, 240.0932694742121,
    0, 0, 1);

cv::Mat MydistCoeffs = (cv::Mat_<double>(1, 5) <<
    -0.2417610723353469, 1.347540335379867,
    -0.001577442260184354, -0.001153209194483007,
    -3.015609162190799);

double reprojError = 0.138174;



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
// 采集图像
int takeVedio(){
    std::string base_path = "/home/dw/robot/image/origin_image/";
    std::string filename  = "origin_" + getTimeString() + ".jpg";
    std::string save_path = base_path + filename;
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
    cap.set(cv::CAP_PROP_EXPOSURE, 300);     // 曝光时间整数us
    cap.set(cv::CAP_PROP_SHARPNESS, 100);  // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, 50);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }
        cv::imshow("Camera Video", frame);
        cv::waitKey(10);
        // cv::imwrite(save_path, frame); 
        // std::cout << "图像已保存到 " << save_path << std::endl;
        // sleep(1);
        // detect_laser_edge(cv::imread("/home/dw/robot/image/1.jpg"));
        // detect_laser_center(cv::imread("/home/dw/robot/image/1.jpg"));
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

// 保存图像集
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
    cap.set(cv::CAP_PROP_EXPOSURE, 300);     // 曝光时间整数us
    cap.set(cv::CAP_PROP_SHARPNESS, 100);  // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, 50);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }
        std::string base_path = "/home/dw/robot/image/video/";
        std::string filename  = "origin_" + getTimeString() + ".jpg";
        std::string save_path = base_path + filename;
        // cv::imshow("Camera Video", frame);
        cv::waitKey(10);
        cv::imwrite(save_path, frame); 
        std::cout << "图像已保存到 " << save_path << std::endl;
        sleep(1);
        // detect_laser_edge(cv::imread("/home/dw/robot/image/1.jpg"));
        // detect_laser_center(cv::imread("/home/dw/robot/image/1.jpg"));
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

int takePic(){
    std::string base_path = "/home/dw/robot/image/origin_image/";
    std::string filename  = "origin_" + getTimeString() + ".jpg";
    std::string save_path = base_path + filename;
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
    cap.set(cv::CAP_PROP_EXPOSURE, 300);     // 曝光时间整数us
    cap.set(cv::CAP_PROP_SHARPNESS, 50);  // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, 30);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    cap >> frame;
    if (frame.empty()) {
        std::cerr << "无法获取图像帧。" << std::endl;
    }
    cv::Mat undistorted;
    cv::undistort(frame, undistorted, MycameraMatrix, MydistCoeffs);
    cv::waitKey(10);
    cv::imwrite(save_path, undistorted); 
    std::cout << "图像已保存到 " << save_path << std::endl;
    sleep(1);
    // detect_laser_edge(cv::imread("/home/dw/robot/image/1.jpg"));
    // detect_laser_center(cv::imread("/home/dw/robot/image/1.jpg"));

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
    cv::undistort(*theMat, undistorted, MycameraMatrix, MydistCoeffs);
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


// 一个简单的 SG 滤波器函数（窗口大小为 5，多项式阶数为 2）
// 权重系数推导自 SG 算法公式
int savgolFilter5(const std::vector<LaserData>& data) {
    //打开csv文件
    std::string path = "/home/dw/robot/image/proc_laser/";
    std::string fname  = getTimeString() + "_points" + ".csv";
    std::string savePath = path + fname;
    std::ofstream ofs(savePath);
    if (!ofs.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return -1;
    }
    // 写表头
    ofs << "laser_id,x_pixel,y_pixel,distance_cm\n";
    
    int n = data.size();
    if (n < 5) return 0;

    // SG 窗口为 5, 阶数为 2 的标准系数（归一化因子为 35）
    // 系数分布：[-3, 12, 17, 12, -3] / 35
    // const double coeffs[] = {-3.0/35.0, 12.0/35.0, 17.0/35.0, 12.0/35.0, -3.0/35.0};
    const double coeffs[] = {
        -21.0/231.0, 14.0/231.0, 39.0/231.0, 54.0/231.0, 59.0/231.0, 
         54.0/231.0, 39.0/231.0, 14.0/231.0, -21.0/231.0
    };
    double coe = 1/9;

    // 中间部分使用滑动窗口卷积
    for (int i = 5; i < n - 5; ++i) {
        // double val = data[i-2].distance_cm * coeffs[0] +
        //               data[i-1].distance_cm * coeffs[1] +
        //               data[i].distance_cm   * coeffs[2] +
        //               data[i+1].distance_cm * coeffs[3] +
        //               data[i+2].distance_cm * coeffs[4];
        // double val = data[i-4].distance_cm * coeffs[0] +
        //              data[i-3].distance_cm * coeffs[1] +
        //              data[i-2].distance_cm * coeffs[2] +
        //              data[i-1].distance_cm * coeffs[3] +
        //              data[i].distance_cm   * coeffs[4] +
        //              data[i+1].distance_cm * coeffs[5] +
        //              data[i+2].distance_cm * coeffs[6] +
        //              data[i+3].distance_cm * coeffs[7] +
        //              data[i+4].distance_cm * coeffs[8];
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
        ofs << data[i].laser_id << "," << data[i].x_pixel << "," << data[i].y_pixel << "," << val << "\n";
    }

    // 边界处理（简单处理：保持原样或使用半窗口平均）
    // smoothed[0] = data[0]; smoothed[1] = data[1];
    // smoothed[n-2] = data[n-2]; smoothed[n-1] = data[n-1];

    ofs.close();

    return 0;
}


// int detect_laser_edge(cv::Mat img) {
//     // 1. 检查输入图像
//     if (img.empty()) {
//         std::cerr << "Error: Image is empty." << std::endl;
//         return -1;
//     }

//     // 2. 拆分通道，增强绿色分量
//     std::vector<cv::Mat> bgr;
//     cv::split(img, bgr);

//     // 差分增强绿色激光信号：绿色 - 平均(红,蓝)
//     cv::Mat diff = bgr[1] - 0.5 * (bgr[0] + bgr[2]);
//     diff.convertTo(diff, CV_8U);

//     // 3. 平滑滤波，降低噪声
//     cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0);

//     // 4. 自动阈值调整
//     int threshold_value = 120;
//     std::vector<std::vector<cv::Point>> contours;

//     while (true) {
//         // 二值化
//         cv::Mat mask;
//         cv::threshold(diff, mask, threshold_value, 255, cv::THRESH_BINARY);

//         // 查找轮廓
//         cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         // 如果检测到的区域 <= 2 或阈值太低则停止
//         if ((contours.size() <= 5 && contours.size() > 0) || threshold_value <= 3)
//             break;

//         // 否则降低阈值
//         threshold_value -= 3;
//     }

//     std::cout << "最终阈值: " << threshold_value 
//               << "，检测到区域数: " << contours.size() << std::endl;

//     // 5. 绘制轮廓
//     cv::Mat diff_color, img_with_contours = img.clone();
//     cv::cvtColor(diff, diff_color, cv::COLOR_GRAY2BGR);

//     for (const auto& contour : contours) {
//         double area = cv::contourArea(contour);
//         if (area > 30) {
//             cv::drawContours(diff_color, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
//             cv::drawContours(img_with_contours, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 2);
//         }
//     }

//     // 6. 保存结果图
//     cv::imwrite("/home/dw/robot/image/diff_mask.jpg", diff_color);
//     cv::imwrite("/home/dw/robot/image/original.jpg", img_with_contours);

//     std::cout << "激光区域已圈出，共检测到 " << contours.size() 
//               << " 个区域（最终阈值=" << threshold_value << "）" << std::endl;

//     return 0;
// }

int detect_laser_edge(cv::Mat img) {
    if (img.empty()) {
        std::cerr << "Error: Image is empty." << std::endl;
        return -1;
    }

    // 通道增强
    std::vector<cv::Mat> bgr;
    cv::split(img, bgr);
    cv::Mat diff = bgr[1] - 0.5 * (bgr[0] + bgr[2]);

    cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
    diff.convertTo(diff, CV_8U);
    cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0);

    // 自动阈值初始化
    double minVal, maxVal;
    cv::minMaxLoc(diff, &minVal, &maxVal);
    int threshold_value = static_cast<int>(maxVal * 0.7);
    std::vector<std::vector<cv::Point>> contours;

    int iter = 0;
    while (iter++ < 40) {
        cv::Mat mask;
        cv::threshold(diff, mask, threshold_value, 255, cv::THRESH_BINARY);
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if ((contours.size() <= 2 && contours.size() > 0) || threshold_value <= 3)
            break;

        threshold_value -= 3;
    }

    cv::Mat diff_color, img_with_contours = img.clone();
    cv::cvtColor(diff, diff_color, cv::COLOR_GRAY2BGR);

    double area_thresh = img.cols * img.rows * 0.0002;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > area_thresh) {
            cv::drawContours(diff_color, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
            cv::drawContours(img_with_contours, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 2);
        }
    }

    cv::imwrite("/home/dw/robot/image/diff_mask.jpg", diff_color);
    cv::imwrite("/home/dw/robot/image/original.jpg", img_with_contours);

    std::cout << "激光区域已圈出，共检测到 " << contours.size() 
              << " 个区域（最终阈值=" << threshold_value << "）" << std::endl;

    return 0;
}

int detect_laser_center(cv::Mat image) {
    if (image.empty()) {
        std::cerr << "Error: Image is empty." << std::endl;
        return -1;
    }

    //打开csv文件
    std::string path = "/home/dw/robot/image/proc_laser/";
    std::string fname  = getTimeString() + "_points" + ".csv";
    std::string savePath = path + fname;
    std::ofstream ofs(savePath);
    if (!ofs.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return -1;
    }
    // 写表头
    ofs << "laser_id,x_pixel,y_pixel,distance_cm\n";

    // 去畸变
    cv::Mat img;
    cv::undistort(image, img, MycameraMatrix, MydistCoeffs);

    // 1️⃣ 通道增强（提取绿色激光）
    std::vector<cv::Mat> bgr;
    cv::split(img, bgr);
    cv::Mat diff = bgr[1] - 0.5 * (bgr[0] + bgr[2]);
    cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
    diff.convertTo(diff, CV_8U);
    cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0);

    // 2️⃣ 自适应阈值
    double minVal, maxVal;
    cv::minMaxLoc(diff, &minVal, &maxVal);
    int threshold_value = static_cast<int>(std::max(30.0, maxVal * 0.8));

    cv::Mat mask;
    cv::threshold(diff, mask, threshold_value, 255, cv::THRESH_BINARY);

    // 3️⃣ 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ====== 根据轮廓个数来调整阈值 ======
    int iter = 0;
    while (iter++ < 100) {
        cv::Mat mask;
        cv::threshold(diff, mask, threshold_value, 255, cv::THRESH_BINARY);
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if ((contours.size() <= 2 && contours.size() > 0) || threshold_value <= 50){
            break;
        }
        printf("调整阈值中，当前阈值: %d, 检测到轮廓数: %lu\n", threshold_value, contours.size());
        threshold_value -= 3;
    }
    // ====== 只保留最大两个轮廓（去残影） ======
    int max_idx1 = -1, max_idx2 = -1;
    double max_area1 = 0.0, max_area2 = 0.0;

    for (int i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);

        if (area > max_area1) {
            // 原第一名降为第二名
            max_area2 = max_area1;
            max_idx2  = max_idx1;

            max_area1 = area;
            max_idx1  = i;
        }
        else if (area > max_area2) {
            max_area2 = area;
            max_idx2  = i;
        }
    }

    std::vector<std::vector<cv::Point>> filtered_contours;

    if (max_idx1 >= 0)
        filtered_contours.push_back(contours[max_idx1]);

    if (max_idx2 >= 0)
        filtered_contours.push_back(contours[max_idx2]);

    contours = filtered_contours;
    // ==========================================

    

    cv::Mat img_with_contours = img.clone();
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > img.cols * img.rows * 0.0002)
            cv::drawContours(img_with_contours, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
    }
    
    printf("counters size: %lu, threshold_value: %d\n", contours.size(), threshold_value);

    

    // 4️⃣ 中心线计算
    std::vector<std::vector<int>> distance_points(diff.cols);
    cv::Mat mask_center = cv::Mat::zeros(diff.size(), CV_8U);
    // 保存每个轮廓的数据
    struct LaserContour {
        std::vector<int> xs;
        std::vector<int> ys;
        double y_average;
        int laser_type; // 1 或 2
    };
    std::vector<LaserContour> laser_contours;
    for (const auto& contour : contours) {

        cv::Rect box = cv::boundingRect(contour);
        if (box.area() < 1000) continue;

        LaserContour lc;

        for (int x = box.x; x < box.x + box.width; ++x) {
            std::vector<int> y_coords;
            for (int y = box.y; y < box.y + box.height; ++y) {
                if (diff.at<uchar>(y, x) > threshold_value)
                    y_coords.push_back(y);
            }

            if (!y_coords.empty()) {
                int y_center = (y_coords.front() + y_coords.back()) / 2;
                lc.xs.push_back(x);
                lc.ys.push_back(y_center);
            }
        }

        if (lc.ys.empty()) continue;

        double sum = 0;
        for (int y : lc.ys) sum += y;
        lc.y_average = sum / lc.ys.size();

        laser_contours.push_back(lc);
    }

    // ---------- 安全检查 ----------
    if (laser_contours.empty()) {
        std::cerr << "未检测到激光轮廓" << std::endl;
        return -1;
    }

    printf("laser_contours size: %lu\n", laser_contours.size());

    // ---------- 第二遍：按轮廓选择距离函数 ----------
    if(laser_contours[0].y_average < laser_contours[1].y_average){
        laser_contours[0].laser_type = 1;
        laser_contours[1].laser_type = 2;
    } else{
        laser_contours[0].laser_type = 2;
        laser_contours[1].laser_type = 1;
    }
    for (const auto& lc : laser_contours) {

        for (size_t i = 0; i < lc.xs.size(); ++i) {
            int x = lc.xs[i];
            int y = lc.ys[i];

            double dis = (lc.laser_type == 1) ? y_pixel_to_distance1(y) : y_pixel_to_distance2(y);

            if (dis <= 0) continue;

            ofs << lc.laser_type << "," << x << "," << y << "," << dis << "\n";
            mask_center.at<uchar>(y, x) = 255;
        }
    }


    // 5️⃣ 修复断线（形态学闭运算）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1));
    cv::morphologyEx(mask_center, mask_center, cv::MORPH_CLOSE, kernel);

    // 6️⃣ 可视化
    for (int y = 0; y < mask_center.rows; ++y) {
        for (int x = 0; x < mask_center.cols; ++x) {
            if (mask_center.at<uchar>(y, x) > 0)
                cv::circle(img_with_contours, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
        }
    }

    
    std::string base_path = "/home/dw/robot/image/proc_laser/";
    std::string filename1  = getTimeString() + "_diff" + ".jpg";
    std::string filename2  = getTimeString() + "_mask" +  ".jpg";
    std::string filename3  = getTimeString() + "_detected" + ".jpg";
    std::string save_path1 = base_path + filename1;
    std::string save_path2 = base_path + filename2;
    std::string save_path3 = base_path + filename3;
    // 7️⃣ 保存结果
    cv::imwrite(save_path1, diff);
    cv::imwrite(save_path2, mask_center);
    cv::imwrite(save_path3, img_with_contours);

    std::cout << "激光中心线检测完成。" << std::endl;
    ofs.close();
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










int findAllPeaks(const std::vector<LaserData>& smoothedData) {
    int n = smoothedData.size();
    
    // 窗口半径设为2，即检查当前点是否为连续5个点中的最大值
    for (int i = 2; i < n - 2; ++i) {
        double current = smoothedData[i].distance_cm;

        // 局部极大值判定：比左右各两个点都大（或相等）
        if (current >= smoothedData[i-1].distance_cm &&
            current >= smoothedData[i-2].distance_cm &&
            current >= smoothedData[i+1].distance_cm &&
            current >= smoothedData[i+2].distance_cm) {
            printf("极大值点 ->id:%d, X轴像素: %d, 距离: %.2f cm\n", smoothedData[i].laser_id, smoothedData[i].x_pixel, current);
        }
    }

    return 0;
}


