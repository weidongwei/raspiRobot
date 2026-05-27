#ifndef UNET_H
#define UNET_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "ImgProc.h"

// 使用传统双激光橘缝点作为硬锚点，在 U-Net 概率图中向上/向下延伸并连接成橘缝曲线。
// 找不到可靠概率路径时退回两个激光点之间的端点直线。
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
