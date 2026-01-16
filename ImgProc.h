#ifndef IMGPROC_H
#define IMGPROC_H
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

// 激光轮廓结构体
struct LaserContour {
    std::vector<int> xs, ys;
    double y_average = 0;
    int laser_type = 0;
};

// 激光线数据结构体
struct LaserData {
    int laser_id;
    int x_pixel;
    int y_pixel;
    double distance_cm;
};

// 凹陷结果结构体
struct SeamResult {
    int id;             // 激光器ID
    int x_peak;         // 峰值像素坐标
    double dist;        // 峰值距离(cm)
    double depth;       // 峰值深度(cm)
    int width;          // 坡宽(像素)
    int left_foot;      // 左坡脚像素坐标
    int right_foot;     // 右坡脚像素坐标
    double score;       // 综合评分
};

// 两个凹陷点橘缝匹配结构体
struct MatchedSeamPair {
    SeamResult s1;
    SeamResult s2;
    double total_score;
};

int takePic();
int takeVedio();
int saveVedio();
int userImgProc0(cv::Mat *theMat, long beginTime, long afterTime);
int userImgProc1(cv::Mat *theMat, long beginTime, long afterTime);
std::vector<LaserData> detect_laser_center(cv::Mat img);
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
std::vector<LaserData> readLaserCSV(const std::string& filename);

cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut);
std::vector<std::vector<cv::Point>> getLaserContours(const cv::Mat& diff);
std::vector<LaserContour> extractCenterlinePoints(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& diff);
void saveAndVisualize(const std::vector<std::vector<cv::Point>>& contours, const std::vector<LaserContour>& lcs, cv::Mat& canvas, const cv::Mat& diff, std::vector<LaserData>& outData);
std::vector<LaserData> detectLaserCenter(cv::Mat image);


std::vector<LaserData> smooth(const std::vector<LaserData> data);
SeamResult analyzeSeamStructure(const std::vector<LaserData>& data, int peakIdx, int patience_limit, double best_width, double best_depth, double ratio_width, double ratio_depth);
std::vector<int> suppress_peaks(const std::vector<int>& peakIndices, const std::vector<LaserData>& data, int win);
std::vector<MatchedSeamPair> findSeam(const std::vector<LaserData>& smoothedData);
cv::Mat drawSeam(cv::Mat displayImage, const std::vector<MatchedSeamPair> results, const std::vector<LaserData> data);



#endif // IMGPROC_H