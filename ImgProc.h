#ifndef IMGPROC_H
#define IMGPROC_H
#include <stdio.h>
#include <opencv2/opencv.hpp>

int takePic();
int takeVedio();
int userImgProc0(cv::Mat *theMat, long beginTime, long afterTime);
int userImgProc1(cv::Mat *theMat, long beginTime, long afterTime);
int detect_img_edge(cv::Mat src, cv::Mat &out);
int detect_laser_edge(cv::Mat img);
int detect_laser_center(cv::Mat img);
std::string getTimeString();

bool calibrateCameraFromImages(
    const std::vector<cv::Mat>& images,   // 标定图像
    cv::Size boardSize,                    // 内角点数量 (9,6)
    float squareSize,                      // 格子尺寸 (mm)
    cv::Mat& cameraMatrix,                 // 输出：相机内参
    cv::Mat& distCoeffs,                   // 输出：畸变参数
    double& reprojError,                   // 输出：重投影误差
    const std::string& debugSaveDir        // 中间图像保存目录
);
int biaoding();
double y_pixel_to_distance1(double y_pixel);
double y_pixel_to_distance2(double y_pixel);


#endif // IMGPROC_H