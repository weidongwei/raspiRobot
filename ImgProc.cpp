#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include "main.h"
#include "thread.h"

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    793.0954708529097, 0, 390.0087179575285,
    0, 792.5891147885803, 300.4939653275754,
    0, 0, 1);

cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
    -0.4836918927522525, 0.3019374911984649,
    0.003686647866372401, 0.002812744443721311,
    -0.1460461058419775);


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