#ifndef UNET_H
#define UNET_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "ImgProc.h"

// 使用 MobileNetV3-Large U-Net 的概率图，在双激光橘缝端点之间追踪橘缝曲线。
std::vector<SeamCurveResult> traceSeamCurvesByUnet(
    const cv::Mat& originImage,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData
);

#endif // UNET_H
