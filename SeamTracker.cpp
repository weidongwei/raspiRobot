#include "ImgProc.h"      // MatchedSeamPair 等结构体定义（ImgProc.h 末尾已 include SeamTracker.h）
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "SeamTracker.h"

// ============================================================
//  调参说明
//  Q_x 大 → 位置更信测量值，响应快但输出抖动
//  Q_v 大 → 速度估计变化快，适合加速度场景
//  R   大 → 更信预测值，输出平滑但有滞后
//  gate   → 匹配门限，摄像头匀速绕橘子时 30px 通常足够
// ============================================================


// ============================================================
//  SeamKalman 实现
// ============================================================

void SeamKalman::init(double x0) {
    x = x0; v = 0.0;
    P00 = 100.0; P01 = 0.0;
    P10 = 0.0;   P11 = 10.0;
    initialized = true;
}

void SeamKalman::predict() {
    // 状态转移：x' = x + v,  v' = v
    x = x + v;
    // 协方差传播：P' = F*P*F^T + Q
    double newP00 = P00 + P01 + P10 + P11 + Q_x;
    double newP01 = P01 + P11;
    double newP10 = P10 + P11;
    double newP11 = P11 + Q_v;
    P00 = newP00; P01 = newP01;
    P10 = newP10; P11 = newP11;
}

void SeamKalman::update(double z) {
    // 卡尔曼增益
    double S   = P00 + R;
    double K0  = P00 / S;   // 位置增益
    double K1  = P10 / S;   // 速度增益
    double inn = z - x;     // 新息（innovation）
    // 状态更新
    x += K0 * inn;
    v += K1 * inn;
    // 协方差更新
    double newP00 = (1.0 - K0) * P00;
    double newP01 = (1.0 - K0) * P01;
    double newP10 = P10 - K1 * P00;
    double newP11 = P11 - K1 * P01;
    P00 = newP00; P01 = newP01;
    P10 = newP10; P11 = newP11;
}

double SeamKalman::nextX() const {
    // 只读预测，不修改内部状态，用于匹配门限计算
    return x + v;
}


// ============================================================
//  SeamCandidate 实现
// ============================================================

void SeamCandidate::recordHit(bool hit) {
    hits.push_back(hit);
    if (hit) hitCount++;
    // 滑动窗口：超出 WINDOW 则弹出最老的一帧
    if ((int)hits.size() > WINDOW) {
        if (hits.front()) hitCount--;
        hits.pop_front();
    }
    // 更新确认状态
    if (hitCount >= CONFIRM_TH) confirmed = true;
    if ((int)hits.size() >= WINDOW && hitCount <= LOST_TH) confirmed = false;
}

bool SeamCandidate::isAlive() const {
    if ((int)hits.size() < WINDOW) return true;  // 观察期内保留
    return hitCount > LOST_TH;
}

double SeamCandidate::currentX() const {
    return kf.x;
}


// ============================================================
//  SeamTracker 实现
// ============================================================

std::vector<MatchedSeamPair> SeamTracker::update(std::vector<MatchedSeamPair>& rawPairs) {

    // === 步骤 1：为每条现有轨迹在 rawPairs 中找最优匹配（贪心） ===
    std::vector<bool> used(rawPairs.size(), false);

    for (int ti = 0; ti < (int)tracks.size(); ++ti) {
        auto& track = tracks[ti];

        // nextX() 只读预测位置，不修改 kf 状态
        double predX = track.kf.initialized ? track.kf.nextX() : -1.0;

        int    bestIdx  = -1;
        double bestDist = track.gate;

        for (int i = 0; i < (int)rawPairs.size(); ++i) {
            if (used[i]) continue;
            double measX = midX(rawPairs[i]);
            double d     = (predX >= 0.0) ? std::abs(measX - predX) : 0.0;
            if (d < bestDist) { bestDist = d; bestIdx = i; }
        }

        if (bestIdx >= 0) {
            // 命中：predict → update（每帧严格只调用一次，避免双重预测）
            double measX = midX(rawPairs[bestIdx]);
            if (!track.kf.initialized) {
                track.kf.init(measX);          // 首次命中，只初始化不预测
            } else {
                track.kf.predict();
                track.kf.update(measX);
            }
            track.recordHit(true);
            track.hasPair = true;
            syncPairStore();
            pairStore[ti] = rawPairs[bestIdx];
            used[bestIdx] = true;
        } else {
            // 漏检：仅预测，不更新，投一次 miss 票
            if (track.kf.initialized) track.kf.predict();
            track.recordHit(false);
        }
    }

    // === 步骤 2：未被匹配的 rawPairs → 新建轨迹 ===
    for (int i = 0; i < (int)rawPairs.size(); ++i) {
        if (used[i]) continue;
        double measX = midX(rawPairs[i]);

        // 与已有轨迹距离过近则跳过，防止重复建轨迹
        bool tooClose = false;
        for (const auto& t : tracks) {
            if (t.kf.initialized && std::abs(t.currentX() - measX) < newTrackGate) {
                tooClose = true;
                break;
            }
        }
        if (!tooClose && (int)tracks.size() < MAX_TRACKS) {
            SeamCandidate nc;
            nc.kf.init(measX);
            nc.recordHit(true);
            nc.hasPair = true;
            tracks.push_back(nc);
            pairStore.push_back(rawPairs[i]);
            std::cout << "[SeamTracker] 新建轨迹 x=" << (int)measX << std::endl;
        }
    }

    // === 步骤 3：清理死亡轨迹（从后往前删，保持索引正确） ===
    for (int i = (int)tracks.size() - 1; i >= 0; --i) {
        if (!tracks[i].isAlive()) {
            std::cout << "[SeamTracker] 删除轨迹 x="
                      << (int)tracks[i].currentX() << std::endl;
            tracks.erase(tracks.begin() + i);
            pairStore.erase(pairStore.begin() + i);
        }
    }

    // === 步骤 4：收集已确认轨迹，用卡尔曼 x 修正坐标后输出 ===
    std::vector<MatchedSeamPair> stableOutput;
    for (int i = 0; i < (int)tracks.size(); ++i) {
        const auto& track = tracks[i];
        if (!track.confirmed || !track.hasPair) continue;

        // MatchedSeamPair smoothed = pairStore[i];
        // int kfX = (int)std::round(track.currentX());

        // // 两条激光线分别按各自偏差修正，保持两线相对结构不变
        // int d1 = kfX - smoothed.s1.x_peak;
        // int d2 = kfX - smoothed.s2.x_peak;
        // smoothed.s1.x_peak     += d1;
        // smoothed.s1.left_foot  += d1;
        // smoothed.s1.right_foot += d1;
        // smoothed.s2.x_peak     += d2;
        // smoothed.s2.left_foot  += d2;
        // smoothed.s2.right_foot += d2;

        MatchedSeamPair smoothed = pairStore[i];
        int kfX = (int)std::round(track.currentX());

        // 原始中点
        int rawMidX = (smoothed.s1.x_peak + smoothed.s2.x_peak) / 2;

        // 统一平移量：卡尔曼平滑中点 - 原始中点
        int d = kfX - rawMidX;

        // s1 和 s2 用同一个 d 整体平移
        smoothed.s1.x_peak     += d;
        smoothed.s1.left_foot  += d;
        smoothed.s1.right_foot += d;
        smoothed.s2.x_peak     += d;
        smoothed.s2.left_foot  += d;
        smoothed.s2.right_foot += d;

        stableOutput.push_back(smoothed);
    }

    // 按 x 从左到右排序，保证每帧输出顺序一致
    std::sort(stableOutput.begin(), stableOutput.end(),
        [](const MatchedSeamPair& a, const MatchedSeamPair& b) {
            return a.s1.x_peak < b.s1.x_peak;
        });

    std::cout << "[SeamTracker] 轨迹=" << tracks.size()
              << " 已确认=" << stableOutput.size()
              << " 本帧原始=" << rawPairs.size() << std::endl;
    printStatus();

    return stableOutput;
}

void SeamTracker::reset() {
    tracks.clear();
    pairStore.clear();
    std::cout << "[SeamTracker] 已重置所有轨迹" << std::endl;
}

void SeamTracker::printStatus() const {
    for (int i = 0; i < (int)tracks.size(); ++i) {
        const auto& t = tracks[i];
        std::cout << std::fixed << std::setprecision(1)
                  << "  轨迹[" << i << "]"
                  << " x="    << t.currentX()
                  << " v="    << t.kf.v
                  << " 命中=" << t.hitCount << "/" << SeamCandidate::WINDOW
                  << (t.confirmed ? " [已确认]" : " [观察中]")
                  << std::endl;
    }
}

double SeamTracker::midX(const MatchedSeamPair& p) {
    return (p.s1.x_peak + p.s2.x_peak) / 2.0;
}

void SeamTracker::syncPairStore() {
    while ((int)pairStore.size() < (int)tracks.size()) {
        pairStore.emplace_back();
    }
}
