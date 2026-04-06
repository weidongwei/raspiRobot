#include "SeamFitter.h"
#include <iostream>
#include <cmath>
#include <algorithm>

SeamFitter g_seamFitter;

// ── 内部工具：最小二乘拟合 x = a0 + a1*y + a2*y² ────────────
// 输入点集 pts，输出系数 a0/a1/a2，返回是否成功
bool SeamFitter::fitCurve(SeamCurve& curve) {
    auto& pts = curve.points;
    int n = (int)pts.size();
    if (n < curve.min_points) return false;

    // ── 第一次拟合 ──────────────────────────────────────────
    auto doFit = [](const std::vector<cv::Point2f>& p,
                    double& a0, double& a1, double& a2) -> bool {
        int n = (int)p.size();
        // 构建 A 矩阵（n×3）和 b 向量（n×1）
        // 方程：x_offset = a0 + a1*y + a2*y²
        cv::Mat A(n, 3, CV_64F);
        cv::Mat b(n, 1, CV_64F);
        for (int i = 0; i < n; i++) {
            double y = p[i].y;
            A.at<double>(i,0) = 1.0;
            A.at<double>(i,1) = y;
            A.at<double>(i,2) = y * y;
            b.at<double>(i,0) = p[i].x;   // x_offset
        }
        cv::Mat coeffs;
        bool ok = cv::solve(A, b, coeffs, cv::DECOMP_SVD);
        if (!ok) return false;
        a0 = coeffs.at<double>(0);
        a1 = coeffs.at<double>(1);
        a2 = coeffs.at<double>(2);
        return true;
    };

    double a0, a1, a2;
    if (!doFit(pts, a0, a1, a2)) return false;

    // ── 离群点剔除后再拟合一次 ──────────────────────────────
    std::vector<cv::Point2f> inliers;
    for (auto& p : pts) {
        double x_pred = a0 + a1*p.y + a2*p.y*p.y;
        if (std::abs(p.x - x_pred) <= curve.outlier_thr)
            inliers.push_back(p);
    }

    if ((int)inliers.size() < curve.min_points) {
        // 剔除后点太少，用第一次的结果
        curve.a0 = a0; curve.a1 = a1; curve.a2 = a2;
        curve.fitted = true;
        return true;
    }

    // 用 inlier 点再拟合一次
    if (!doFit(inliers, a0, a1, a2)) return false;
    curve.a0 = a0; curve.a1 = a1; curve.a2 = a2;
    curve.fitted = true;

    std::cout << "[SeamFitter] seam " << curve.id
              << " fitted with " << inliers.size() << " inliers"
              << "  a0=" << a0 << " a1=" << a1 << " a2=" << a2
              << std::endl;
    return true;
}

// ── addPoints ────────────────────────────────────────────────
void SeamFitter::addPoints(int seam_id,
                           int x1, int y1,
                           int x2, int y2,
                           int ref_x_peak) {
    SeamCurve& curve = curves_[seam_id];
    curve.id = seam_id;

    // 存相对偏移：x_offset = x_pixel - ref_x_peak
    float off1 = (float)(x1 - ref_x_peak);
    float off2 = (float)(x2 - ref_x_peak);

    curve.points.push_back({off1, (float)y1});
    curve.points.push_back({off2, (float)y2});

    // 超过上限时丢弃最老的点（滑动窗口）
    if ((int)curve.points.size() > curve.max_points) {
        int excess = (int)curve.points.size() - curve.max_points;
        curve.points.erase(curve.points.begin(),
                           curve.points.begin() + excess);
    }
}

// ── fitAll ───────────────────────────────────────────────────
void SeamFitter::fitAll() {
    for (auto& [id, curve] : curves_) {
        fitCurve(curve);
    }
}

// ── draw ─────────────────────────────────────────────────────
cv::Mat SeamFitter::draw(cv::Mat img,
                         const std::map<int,int>& current_x_peaks,
                         int img_rows) const {
    for (auto& [id, curve] : curves_) {
        if (!curve.fitted) continue;

        // 找到本帧该缝的当前 x_peak
        auto it = current_x_peaks.find(id);
        if (it == current_x_peaks.end()) continue;
        int cur_x_peak = it->second;

        // 颜色：用 id 区分不同的缝
        static const cv::Scalar COLORS[] = {
            {0,0,255}, {0,180,255}, {0,255,180},
            {255,0,180}, {180,0,255}, {255,180,0}
        };
        cv::Scalar color = COLORS[id % 6];

        // 按 y 从 0 到 img_rows 逐像素画曲线
        cv::Point prev(-1, -1);
        for (int y = 0; y < img_rows; y++) {
            // x_offset = a0 + a1*y + a2*y²
            double x_offset = curve.a0 + curve.a1*y + curve.a2*y*y;
            int x = (int)std::round(cur_x_peak + x_offset);

            // 超出图像边界跳过
            if (x < 0 || x >= img.cols) { prev = {-1,-1}; continue; }

            cv::Point cur(x, y);
            if (prev.x >= 0)
                cv::line(img, prev, cur, color, 2, cv::LINE_AA);
            prev = cur;
        }
    }
    return img;
}

// ── reset ────────────────────────────────────────────────────
void SeamFitter::reset(int seam_id) {
    auto it = curves_.find(seam_id);
    if (it != curves_.end()) it->second.reset();
}

void SeamFitter::resetAll() {
    for (auto& [id, curve] : curves_) curve.reset();
    curves_.clear();
}

// ── getCurve ─────────────────────────────────────────────────
const SeamCurve* SeamFitter::getCurve(int seam_id) const {
    auto it = curves_.find(seam_id);
    return (it != curves_.end()) ? &it->second : nullptr;
}
