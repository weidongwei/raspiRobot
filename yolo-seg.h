#ifndef YOLO_SEG_H
#define YOLO_SEG_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "ImgProc.h"

struct YoloSegTiming {
    double infer_ms = 0.0;
    double draw_ms = 0.0;
    int mask_count = 0;
    int selected_count = 0;
};

// 在传统检测结果图上叠加 YOLO-seg 橘缝 mask、骨架和可选的激光约束选择结果。
cv::Mat applyYoloSegToDisplay(
    const cv::Mat& originImage,
    cv::Mat displayImage,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData,
    bool useLaserSelection = true,
    YoloSegTiming* timing = nullptr
);

#endif // YOLO_SEG_H
