#ifndef UNET_H
#define UNET_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "ImgProc.h"

// 使用整图 U-Net 概率图和双激光端点追踪橘缝曲线。
// 两个激光点之间、激光点上方、激光点下方都使用同一套概率图 DP 追线。
std::vector<SeamCurveResult> traceSeamCurvesByUnet(
    const cv::Mat& originImage,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData
);

// 仅可视化 U-Net 推理出的橘缝概率点，不做激光约束和曲线搜索。
cv::Mat drawUnetSeamProbabilityPoints(
    const cv::Mat& originImage,
    float threshold = 0.5f,
    int pointStride = 2
);

bool saveUnetSeamProbabilityPoints(
    const cv::Mat& originImage,
    const std::string& outputPath,
    float threshold = 0.5f,
    int pointStride = 2
);

#endif // UNET_H
