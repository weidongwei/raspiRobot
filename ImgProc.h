#ifndef IMGPROC_H
#define IMGPROC_H
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

// 视觉检测参数
struct VisualConfig {
    // 相机矩阵和畸变参数
    cv::Mat MycameraMatrix = (cv::Mat_<double>(3, 3) <<
    622.8518195651313, 0, 324.9937701989255,
    0, 622.3557983623182, 240.0932694742121,
    0, 0, 1);
    cv::Mat MydistCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.2417610723353469, 1.347540335379867,
        -0.001577442260184354, -0.001153209194483007,
        -3.015609162190799);
    double reprojError = 0.138174;
    
    // 原始数据路径
    std::string origin_img_path;
    // 数据保存路径
    std::string base_path;
    // 激光参数
    double laser_duty;              // 激光占空比
    // 摄像头参数
    int exposure_time;              // 曝光时间(ms)
    // photo线程模式
    int photo_thread_mode;          // 1: 实时检测图像 2: 保存多张原始图像 3: 保存单张原始图像
    // 自适应阈值参数
    int counter_maxsize;            // 轮廓最大值
    int threshold_value_min;        // 最小阈值
    int threshold_value_rate;       // 每次迭代减少的阈值
    int best_laser_length;          // 理想激光长度
    int best_laser_width;           // 理想激光宽度 
    double ratio_laser_length;      // 激光长度权重比
    double ratio_laser_width;       // 激光宽度权重比
    // 峰值竞争参数
    int peak_suppress_win;          // 峰值抑制窗口
    //趋势坍塌分析函数参数
    int patience_limit;             // 耐心值
    double ratio_width;             // 宽度权重比
    double ratio_depth;             // 深度权重比
    double best_width;              // 最佳宽度
    double best_depth;              // 最佳深度
    // 匹配橘缝对
    int dx_between_seams_min;       // 相邻橘缝最小距离
    double total_score_min;         // 综合评分最低分数
};
extern VisualConfig vConfig;

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

bool loadVisualConfig(VisualConfig& cfg, const std::string& filename);
std::string getTimeString();
int takePic();
int takeVedio();
int saveVedio();
int userImgProc0(cv::Mat *theMat, long beginTime, long afterTime);
int userImgProc1(cv::Mat *theMat, long beginTime, long afterTime);

bool calibrateCameraFromImages( const std::vector<cv::Mat>& images, cv::Size boardSize, float squareSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, double& reprojError, const std::string& debugSaveDir );
int biaoding();
double y_pixel_to_distance1(double y_pixel);
double y_pixel_to_distance2(double y_pixel);
std::vector<LaserData> readLaserCSV(const std::string& filename);

cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut);
double calculateScore(const std::vector<std::vector<cv::Point>>& contours);
std::vector<std::vector<cv::Point>> getLaserContours(const cv::Mat& diff);
std::vector<LaserContour> extractCenterlinePoints(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& diff);
cv::Mat saveAndVisualize(const std::vector<std::vector<cv::Point>>& contours, const std::vector<LaserContour>& lcs, cv::Mat& canvas, const cv::Mat& diff, std::vector<LaserData>& outData);
std::vector<LaserData> detectLaserCenter(cv::Mat image, cv::Mat* imageOut);


std::vector<LaserData> smooth(const std::vector<LaserData> data);
SeamResult analyzeSeamStructure(const std::vector<LaserData>& data, int peakIdx);
std::vector<int> suppress_peaks(const std::vector<int>& peakIndices, const std::vector<LaserData>& data);
std::vector<MatchedSeamPair> findSeam(const std::vector<LaserData>& smoothedData);
cv::Mat drawSeam(cv::Mat displayImage, const std::vector<MatchedSeamPair> results, const std::vector<LaserData> data);

int detectMain(cv::Mat originImage);


#endif // IMGPROC_H