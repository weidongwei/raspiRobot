#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <chrono>
#include <ctime>



cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    793.0954708529097, 0, 390.0087179575285,
    0, 792.5891147885803, 300.4939653275754,
    0, 0, 1);

cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
    -0.4836918927522525, 0.3019374911984649,
    0.003686647866372401, 0.002812744443721311,
    -0.1460461058419775);

// 采集图像
int takepic(){
    std::string save_path = "/home/dw/robot/image/1.jpg";
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    // cv::VideoCapture cap;
    // cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << 0 << std::endl;
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);  // 有的驱动 1=手动，3=自动，需测试
    // cap.set(cv::CAP_PROP_EXPOSURE, 3000);     // 曝光时间整数us
    // cap.set(cv::CAP_PROP_SHARPNESS, 100);  // 设置锐度为 100(0 ~ 100)
    // cap.set(cv::CAP_PROP_BRIGHTNESS, 50);  // 设置亮度为 50(-64 ~ 64)

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
    cv::undistort(*theMat, undistorted, cameraMatrix, distCoeffs);
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

int detect_laser_center(cv::Mat img) {
    if (img.empty()) {
        std::cerr << "Error: Image is empty." << std::endl;
        return -1;
    }

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
    int threshold_value = static_cast<int>(std::max(30.0, maxVal * 0.5));

    cv::Mat mask;
    cv::threshold(diff, mask, threshold_value, 255, cv::THRESH_BINARY);

    // 3️⃣ 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat img_with_contours = img.clone();
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > img.cols * img.rows * 0.0002)
            cv::drawContours(img_with_contours, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
    }

    // 4️⃣ 中心线计算
    cv::Mat mask_center = cv::Mat::zeros(diff.size(), CV_8U);
    for (const auto& contour : contours) {
        cv::Rect box = cv::boundingRect(contour);
        if (box.area() < 100) continue;

        for (int x = box.x; x < box.x + box.width; ++x) {
            std::vector<int> y_coords;
            for (int y = box.y; y < box.y + box.height; ++y) {
                if (diff.at<uchar>(y, x) > threshold_value)
                    y_coords.push_back(y);
            }
            if (!y_coords.empty()) {
                int y_center = (y_coords.front() + y_coords.back()) / 2;
                mask_center.at<uchar>(y_center, x) = 255;
            }
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

    // 7️⃣ 保存结果
    cv::imwrite("/home/dw/robot/image/laser_center_detected.jpg", img_with_contours);
    cv::imwrite("/home/dw/robot/image/laser_center_mask.jpg", mask_center);

    std::cout << "激光中心线检测完成。" << std::endl;
    return 0;
}

int putouttime(){
    // 1. 获取当前系统时间（高精度，纳秒级）
    auto now = std::chrono::system_clock::now();

    // 2. 转换为时间戳（秒级，兼容传统time_t）
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

    // 3. 输出本地时间（基础格式）
    std::cout << "当前时间（基础格式）：" << std::ctime(&currentTime);
    
    return 0;
}









