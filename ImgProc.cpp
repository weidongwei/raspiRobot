#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <fstream>



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
double y_pixel_to_distance(double y_pixel) {
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
    ofs << "x_pixel,y_pixel,distance_cm\n";

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
    while (iter++ < 40) {
        cv::Mat mask;
        cv::threshold(diff, mask, threshold_value, 255, cv::THRESH_BINARY);
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if ((contours.size() <= 2 && contours.size() > 0) || threshold_value <= 3)
            break;

        threshold_value -= 3;
    }
    // ======================================


    // ====== 只保留最大轮廓（去残影） ======
    int max_idx = -1;
    double max_area = 0.0;

    for (int i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }

    std::vector<std::vector<cv::Point>> filtered_contours;
    if (max_idx >= 0) {
        filtered_contours.push_back(contours[max_idx]);
    }
    contours = filtered_contours;
    // ======================================

    

    cv::Mat img_with_contours = img.clone();
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > img.cols * img.rows * 0.0002)
            cv::drawContours(img_with_contours, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
    }

    // 4️⃣ 中心线计算
    std::vector<std::vector<int>> distance_points(diff.cols);
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

                double dis = y_pixel_to_distance(y_center);
                distance_points[x].push_back(dis);
                printf("x: %d, y_center: %d, distance: %.2f\n", x, y_center, dis);
                if (dis > 0) {  // 过滤非法值
                    ofs << x << ","
                        << 230 << ","
                        << dis << "\n";
                }

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













