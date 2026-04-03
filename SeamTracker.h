#pragma once
#include <vector>
#include <deque>

// SeamTracker.h 不 include ImgProc.h，避免循环依赖。
// ImgProc.h 在自己的 #endif 之前 include 本文件即可：
//     #include "SeamTracker.h"
// 这样本文件能直接使用 ImgProc.h 中定义的 MatchedSeamPair 等结构体。

// ============================================================
//  二阶卡尔曼滤波器
//  状态量：[x（像素位置）, v（像素/帧速度）]
//  状态转移：x(k+1) = x(k) + v(k)
//            v(k+1) = v(k)
//  观测量：  z = x_measured
// ============================================================
struct SeamKalman {
    double x   = 0.0;    // 位置估计（像素）
    double v   = 0.0;    // 速度估计（像素/帧）
    double P00 = 100.0;  // 协方差矩阵
    double P01 = 0.0;
    double P10 = 0.0;
    double P11 = 10.0;

    // ---- 噪声参数（调参说明见 SeamTracker.cpp） ----
    double Q_x = 0.5;   // 过程噪声-位置
    double Q_v = 0.1;   // 过程噪声-速度
    double R   = 8.0;   // 测量噪声

    bool initialized = false;

    void   init(double x0);     // 初始化状态
    void   predict();           // 预测一帧（修改内部状态）
    void   update(double z);    // 用测量值更新状态
    double nextX() const;       // 预测下一帧位置（只读，不修改状态）
};


// ============================================================
//  单条分瓣线的滑动窗口投票器
// ============================================================
struct SeamCandidate {
    SeamKalman kf;

    // ---- 投票窗口参数 ----
    // WINDOW    ：观察的历史帧数
    // CONFIRM_TH：窗口内命中 >= 此值，才对外输出
    // LOST_TH   ：窗口满后命中 <= 此值，删除该轨迹
    static const int WINDOW     = 8;
    static const int CONFIRM_TH = 5;
    static const int LOST_TH    = 2;

    std::deque<bool> hits;
    int  hitCount  = 0;
    bool confirmed = false;
    bool hasPair   = false;

    // 匹配门限（像素）：测量 x 与预测 x 之差超过此值不视为同一条线
    double gate = 30.0;

    void   recordHit(bool hit);  // 记录本帧命中情况，更新投票窗口
    bool   isAlive()  const;     // 轨迹是否仍然存活
    double currentX() const;     // 当前卡尔曼估计位置
};


// ============================================================
//  多路分瓣线追踪管理器（主接口）
//
//  典型用法（在 takeVedio / detectMain 外部声明，跨帧复用）：
//
//    SeamTracker seamTracker;
//    // 每帧循环内：
//    auto stableResults = seamTracker.update(rawPairs);
//    cv::Mat finalMat   = drawSeam(displayImage, stableResults, data);
// ============================================================
class SeamTracker {
public:
    static const int MAX_TRACKS = 12;  // 最多同时追踪的分瓣线数
    double newTrackGate = 40.0;        // 新建轨迹的 x 间距门限（像素）

    std::vector<SeamCandidate>   tracks;
    std::vector<MatchedSeamPair> pairStore;  // 与 tracks 等长，存最近一次有效匹配对

    // 主更新函数：输入 findSeam() 原始结果，输出跨帧稳定后的结果
    std::vector<MatchedSeamPair> update(std::vector<MatchedSeamPair>& rawPairs);

    // 换橘子或重新开始时调用，清空所有轨迹状态
    void reset();

    // 调试：打印所有轨迹当前状态到 stdout
    void printStatus() const;

private:
    // 取 MatchedSeamPair 两条激光线的平均 x 作为代表坐标
    static double midX(const MatchedSeamPair& p);

    // 保证 pairStore 长度与 tracks 保持同步
    void syncPairStore();
};
