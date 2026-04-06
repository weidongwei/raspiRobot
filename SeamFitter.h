#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

// ── 单条分瓣线的曲线拟合器 ────────────────────────────────────
// 使用方式：
//   1. 每帧检测到橘缝后调用 addPoints() 追加两个激光交点
//   2. 追加了足够的点后调用 fit() 进行拟合
//   3. 调用 draw() 把曲线画到图像上
//   4. 橘子换下一个时调用 reset() 清空

struct SeamCurve {
    int   id       = -1;
    bool  fitted   = false;

    // 存储的是相对于参考 x_peak 的偏移，
    // 这样曲线"形状"与当前旋转位置无关
    // point.x = x_offset = x_pixel - ref_x_peak
    // point.y = y_pixel
    std::vector<cv::Point2f> points;

    // 二次多项式系数：x_offset = a0 + a1*y + a2*y^2
    double a0 = 0, a1 = 0, a2 = 0;

    // 参数
    int    min_points   = 10;    // 至少累积多少个点才拟合
    int    max_points   = 200;   // 最多保留多少个点（防止无限增长）
    double outlier_thr  = 15.0;  // 拟合后剔除残差超过此像素的离群点

    void reset() {
        points.clear();
        fitted = false;
        a0 = a1 = a2 = 0;
    }
};

// ── 全局管理器：管理多条分瓣线 ──────────────────────────────
class SeamFitter {
public:
    // 追加一条缝本帧的两个激光交点
    // seam_id    : 对应跟踪器的 track id（或 findSeam 的结果序号）
    // x1,y1      : Laser 1 检测到的 x_pixel, y_pixel
    // x2,y2      : Laser 2 检测到的 x_pixel, y_pixel
    // ref_x_peak : 本帧该缝的参考 x（用 s1.x_peak 即可）
    void addPoints(int seam_id,
                   int x1, int y1,
                   int x2, int y2,
                   int ref_x_peak);

    // 对所有曲线执行拟合（点数足够才实际拟合）
    void fitAll();

    // 把所有已拟合的曲线画到图像上
    // current_x_peaks : 本帧每条缝的 x_peak（用于把相对偏移转回绝对位置）
    //                   key = seam_id, value = 当前 x_peak
    cv::Mat draw(cv::Mat img,
                 const std::map<int,int>& current_x_peaks,
                 int img_rows) const;

    // 清空某条缝（橘子换下一个时使用）
    void reset(int seam_id);

    // 清空所有
    void resetAll();

    // 获取某条缝的曲线（用于外部自定义绘制）
    const SeamCurve* getCurve(int seam_id) const;

private:
    std::map<int, SeamCurve> curves_;

    // 对单条曲线做最小二乘拟合，带离群点剔除
    static bool fitCurve(SeamCurve& curve);
};

// 全局单例，在 ImgProc.cpp 里 extern 声明后直接使用
extern SeamFitter g_seamFitter;
