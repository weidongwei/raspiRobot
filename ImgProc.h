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
    // proc数据保存路径
    std::string proc_path;
    // diff数据保存路径
    std::string diff_path;
    // csv数据保存路径
    std::string csv_path;
    // 激光参数
    double laser_duty;              // 激光占空比
    // 摄像头参数
    int exposure_time;              // 曝光时间(ms)
    int brightness;                 // 亮度
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



// ── 单条橘缝的卡尔曼跟踪器 ─────────────────────────────────
struct SeamTrack {
    int    id;
    int    hits;          // 连续匹配帧数
    int    misses;        // 连续丢失帧数
    bool   confirmed;     // hits >= HIT_THRESH 才输出

    // 卡尔曼状态：x 位置 + x 速度
    double x;             // 当前估计 x
    double vx;            // 当前估计速度（像素/帧）

    // 卡尔曼协方差（简化为标量，各维独立）
    double p_x  = 50.0;   // 位置不确定度
    double p_vx = 10.0;   // 速度不确定度

    // 噪声参数
    static constexpr double Q_x  = 2.0;   // 过程噪声：位置
    static constexpr double Q_vx = 1.0;   // 过程噪声：速度
    static constexpr double R    = 8.0;   // 观测噪声

    static constexpr int HIT_THRESH  = 3;  // 连续命中几帧才确认
    static constexpr int MAX_MISSES  = 5;  // 连续丢失几帧后删除

    // 预测下一帧位置
    void predict() {
        x   += vx;
        p_x  += p_vx + Q_x;
        p_vx += Q_vx;
    }

    // 用观测值更新
    void update(double measured_x) {
        // 卡尔曼增益
        double K = p_x / (p_x + R);
        double innovation = measured_x - x;
        x   += K * innovation;
        vx  += 0.3 * K * innovation;   // 速度也跟随更新
        p_x  = (1.0 - K) * p_x;
        p_vx = std::max(p_vx * 0.9, 0.5);
        hits++;
        misses = 0;
        confirmed = (hits >= HIT_THRESH);
    }

    // 未匹配到观测时
    void mark_missed() {
        misses++;
        hits = std::max(0, hits - 1);
    }

    bool is_dead() const { return misses > MAX_MISSES; }

    // 预测位置（用于匹配时的门控）
    double predicted_x() const { return x + vx; }
};

// ── 多目标跟踪管理器 ──────────────────────────────────────────
struct SeamTracker {
    std::vector<SeamTrack> tracks;
    int next_id = 0;

    static constexpr double GATE = 30.0;   // 匹配门限（像素）

    // 主接口：输入本帧检测到的 x 坐标列表，返回稳定的 x 列表
    std::vector<int> update(const std::vector<int>& detections);

    // 只返回已确认轨迹的平滑 x 坐标
    std::vector<int> confirmed_xs() const {
        std::vector<int> xs;
        for (auto& t : tracks)
            if (t.confirmed) xs.push_back((int)std::round(t.x));
        return xs;
    }
};

#endif // IMGPROC_H