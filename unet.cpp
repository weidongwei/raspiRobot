#include "unet.h"

#include <opencv2/dnn.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

static const bool UNET_SEAM_ENABLE = true;
static const char* UNET_MODEL_PATH_PRIMARY = "/home/dw/robot/cpp/best_unet.onnx";
static const char* UNET_MODEL_PATH_FALLBACK = "cpp/best_unet.onnx";
static const int UNET_INPUT_SIZE = 320;
static const int UNET_ROI_X_MARGIN = 80;
static const int UNET_ROI_Y_MARGIN = 8;
static const int UNET_MIN_VERTICAL_SPAN = 8;
static const int UNET_MAX_X_STEP_PER_ROW = 6;
static const double UNET_TRANSITION_WEIGHT = 0.03;
static const double UNET_SHAPE_PRIOR_WEIGHT = 0.14;

static int clampIntUnet(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static bool fileExistsUnet(const std::string& path) {
    std::ifstream file(path.c_str());
    return file.good();
}

static float sigmoidUnet(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

static bool findNearestLaserEndpointUnet(
    const std::vector<LaserData>& laserData,
    int laserId,
    int targetX,
    const cv::Size& imageSize,
    cv::Point& endpoint
) {
    bool found = false;
    int bestGap = std::numeric_limits<int>::max();
    int bestY = -1;

    for (const auto& row : laserData) {
        if (row.laser_id != laserId) continue;

        int gap = std::abs(row.x_pixel - targetX);
        if (gap < bestGap) {
            bestGap = gap;
            bestY = row.y_pixel;
            found = true;
        }
    }

    if (!found || imageSize.width <= 0 || imageSize.height <= 0) return false;

    endpoint.x = clampIntUnet(targetX, 0, imageSize.width - 1);
    endpoint.y = clampIntUnet(bestY, 0, imageSize.height - 1);
    return true;
}

static std::vector<cv::Point> buildLinePathUnet(cv::Point p1, cv::Point p2, const cv::Size& imageSize) {
    std::vector<cv::Point> path;
    int steps = std::max(std::abs(p2.x - p1.x), std::abs(p2.y - p1.y));
    steps = std::max(steps, 1);
    path.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        int x = static_cast<int>(std::round(p1.x * (1.0 - t) + p2.x * t));
        int y = static_cast<int>(std::round(p1.y * (1.0 - t) + p2.y * t));
        path.push_back(cv::Point(
            clampIntUnet(x, 0, imageSize.width - 1),
            clampIntUnet(y, 0, imageSize.height - 1)
        ));
    }

    return path;
}

static std::vector<cv::Point> smoothCurvePathUnet(const std::vector<cv::Point>& path, int radius, const cv::Size& imageSize) {
    if (path.size() < 3 || radius <= 0) return path;

    std::vector<cv::Point> smoothed = path;
    for (int i = 1; i < static_cast<int>(path.size()) - 1; ++i) {
        int left = std::max(0, i - radius);
        int right = std::min(static_cast<int>(path.size()) - 1, i + radius);
        double sumX = 0.0;
        int count = 0;

        for (int j = left; j <= right; ++j) {
            sumX += path[j].x;
            count++;
        }

        smoothed[i].x = clampIntUnet(static_cast<int>(std::round(sumX / std::max(1, count))), 0, imageSize.width - 1);
        smoothed[i].y = path[i].y;
    }

    smoothed.front() = path.front();
    smoothed.back() = path.back();
    return smoothed;
}

static cv::dnn::Net* getUnetNet() {
    static cv::dnn::Net net;
    static bool triedLoad = false;
    static bool loaded = false;

    if (!UNET_SEAM_ENABLE) return nullptr;

    if (!triedLoad) {
        triedLoad = true;
        std::string modelPath;
        if (fileExistsUnet(UNET_MODEL_PATH_PRIMARY)) {
            modelPath = UNET_MODEL_PATH_PRIMARY;
        } else if (fileExistsUnet(UNET_MODEL_PATH_FALLBACK)) {
            modelPath = UNET_MODEL_PATH_FALLBACK;
        }

        if (modelPath.empty()) {
            std::cerr << "[U-Net] 找不到模型文件，使用端点直线兜底。" << std::endl;
            return nullptr;
        }

        try {
            net = cv::dnn::readNetFromONNX(modelPath);
            loaded = !net.empty();
            if (loaded) {
                std::cout << "[U-Net] 模型加载完成: " << modelPath << std::endl;
            } else {
                std::cerr << "[U-Net] 模型为空，使用端点直线兜底。" << std::endl;
            }
        } catch (const std::exception& e) {
            loaded = false;
            std::cerr << "[U-Net] 模型加载失败: " << e.what() << "，使用端点直线兜底。" << std::endl;
        }
    }

    return loaded ? &net : nullptr;
}

static cv::Mat makeUnetBlob(const cv::Mat& image) {
    cv::Mat bgr;
    if (image.channels() == 4) {
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    } else if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = image;
    }

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    cv::resize(rgb, rgb, cv::Size(UNET_INPUT_SIZE, UNET_INPUT_SIZE), 0, 0, cv::INTER_LINEAR);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> channels;
    cv::split(rgb, channels);
    const float mean[3] = {0.485f, 0.456f, 0.406f};
    const float stddev[3] = {0.229f, 0.224f, 0.225f};
    for (int c = 0; c < 3; ++c) {
        channels[c] = (channels[c] - mean[c]) / stddev[c];
    }

    cv::Mat normalized;
    cv::merge(channels, normalized);
    return cv::dnn::blobFromImage(normalized, 1.0, cv::Size(UNET_INPUT_SIZE, UNET_INPUT_SIZE), cv::Scalar(), false, false);
}

static cv::Mat inferUnetProbability(const cv::Mat& image) {
    cv::dnn::Net* net = getUnetNet();
    if (net == nullptr || image.empty()) return {};

    try {
        cv::Mat blob = makeUnetBlob(image);
        net->setInput(blob);
        cv::Mat output = net->forward();

        if (output.total() != static_cast<size_t>(UNET_INPUT_SIZE * UNET_INPUT_SIZE)) {
            std::cerr << "[U-Net] 输出尺寸不符合预期，使用端点直线兜底。" << std::endl;
            return {};
        }

        cv::Mat probSmall(UNET_INPUT_SIZE, UNET_INPUT_SIZE, CV_32F, output.ptr<float>());
        probSmall = probSmall.clone();

        double minVal = 0.0;
        double maxVal = 0.0;
        cv::minMaxLoc(probSmall, &minVal, &maxVal);
        if (minVal < -1e-4 || maxVal > 1.0001) {
            for (int y = 0; y < probSmall.rows; ++y) {
                float* row = probSmall.ptr<float>(y);
                for (int x = 0; x < probSmall.cols; ++x) {
                    row[x] = sigmoidUnet(row[x]);
                }
            }
        }

        cv::Mat probFull;
        cv::resize(probSmall, probFull, image.size(), 0, 0, cv::INTER_LINEAR);
        for (int y = 0; y < probFull.rows; ++y) {
            float* row = probFull.ptr<float>(y);
            for (int x = 0; x < probFull.cols; ++x) {
                row[x] = std::min(1.0f, std::max(0.0f, row[x]));
            }
        }
        return probFull;
    } catch (const std::exception& e) {
        std::cerr << "[U-Net] 推理失败: " << e.what() << "，使用端点直线兜底。" << std::endl;
        return {};
    }
}

static bool traceSingleCurveByProbability(
    const cv::Mat& probability,
    cv::Point p1,
    cv::Point p2,
    std::vector<cv::Point>& curvePoints,
    double& meanCost
) {
    if (probability.empty()) return false;

    cv::Point start = p1;
    cv::Point end = p2;
    if (start.y > end.y) {
        std::swap(start, end);
    }

    if (std::abs(end.y - start.y) < UNET_MIN_VERTICAL_SPAN) return false;

    int x0 = clampIntUnet(std::min(start.x, end.x) - UNET_ROI_X_MARGIN, 0, probability.cols - 1);
    int x1 = clampIntUnet(std::max(start.x, end.x) + UNET_ROI_X_MARGIN, 0, probability.cols - 1);
    int y0 = clampIntUnet(std::min(start.y, end.y) - UNET_ROI_Y_MARGIN, 0, probability.rows - 1);
    int y1 = clampIntUnet(std::max(start.y, end.y) + UNET_ROI_Y_MARGIN, 0, probability.rows - 1);
    if (x1 <= x0 || y1 <= y0) return false;

    cv::Rect roiRect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    cv::Mat probRoi = probability(roiRect);

    cv::Point localStart(start.x - roiRect.x, start.y - roiRect.y);
    cv::Point localEnd(end.x - roiRect.x, end.y - roiRect.y);
    localStart.x = clampIntUnet(localStart.x, 0, probRoi.cols - 1);
    localEnd.x = clampIntUnet(localEnd.x, 0, probRoi.cols - 1);
    localStart.y = clampIntUnet(localStart.y, 0, probRoi.rows - 1);
    localEnd.y = clampIntUnet(localEnd.y, 0, probRoi.rows - 1);

    const double INF = std::numeric_limits<double>::max() / 4.0;
    std::vector<double> prevDp(probRoi.cols, INF);
    std::vector<double> currDp(probRoi.cols, INF);
    cv::Mat parent(probRoi.rows, probRoi.cols, CV_32S, cv::Scalar(-1));

    prevDp[localStart.x] = 1.0 - probRoi.at<float>(localStart.y, localStart.x);

    for (int y = localStart.y + 1; y <= localEnd.y; ++y) {
        std::fill(currDp.begin(), currDp.end(), INF);
        double t = static_cast<double>(y - localStart.y) / std::max(1, localEnd.y - localStart.y);
        double lineX = localStart.x * (1.0 - t) + localEnd.x * t;

        for (int x = 0; x < probRoi.cols; ++x) {
            int left = std::max(0, x - UNET_MAX_X_STEP_PER_ROW);
            int right = std::min(probRoi.cols - 1, x + UNET_MAX_X_STEP_PER_ROW);
            double bestPrev = INF;
            int bestPrevX = -1;

            for (int px = left; px <= right; ++px) {
                if (prevDp[px] >= INF / 2.0) continue;
                double transitionCost = UNET_TRANSITION_WEIGHT * std::abs(x - px);
                double candidate = prevDp[px] + transitionCost;
                if (candidate < bestPrev) {
                    bestPrev = candidate;
                    bestPrevX = px;
                }
            }

            if (bestPrevX < 0) continue;

            double probabilityCost = 1.0 - static_cast<double>(probRoi.at<float>(y, x));
            double shapeCost = UNET_SHAPE_PRIOR_WEIGHT * std::min(1.0, std::abs(x - lineX) / (UNET_ROI_X_MARGIN + 1.0));
            currDp[x] = bestPrev + probabilityCost + shapeCost;
            parent.at<int>(y, x) = bestPrevX;
        }

        prevDp.swap(currDp);
    }

    double totalCost = prevDp[localEnd.x];
    if (totalCost >= INF / 2.0) return false;

    std::vector<cv::Point> reversedPath;
    int x = localEnd.x;
    for (int y = localEnd.y; y >= localStart.y; --y) {
        reversedPath.push_back(cv::Point(roiRect.x + x, roiRect.y + y));
        if (y == localStart.y) break;

        int prevX = parent.at<int>(y, x);
        if (prevX < 0) return false;
        x = prevX;
    }

    std::reverse(reversedPath.begin(), reversedPath.end());
    curvePoints = smoothCurvePathUnet(reversedPath, 2, probability.size());
    meanCost = totalCost / std::max(1, static_cast<int>(curvePoints.size()));
    return curvePoints.size() >= 2;
}

std::vector<SeamCurveResult> traceSeamCurvesByUnet(
    const cv::Mat& originImage,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData
) {
    std::vector<SeamCurveResult> curves;
    curves.reserve(seamPairs.size());

    if (originImage.empty() || seamPairs.empty()) return curves;

    cv::Mat probability = inferUnetProbability(originImage);

    for (const auto& pair : seamPairs) {
        SeamCurveResult result;
        result.pair = pair;
        result.mean_cost = 0.0;
        result.fallback_to_line = false;

        cv::Point p1;
        cv::Point p2;
        bool hasP1 = findNearestLaserEndpointUnet(laserData, pair.s1.id, pair.s1.x_peak, originImage.size(), p1);
        bool hasP2 = findNearestLaserEndpointUnet(laserData, pair.s2.id, pair.s2.x_peak, originImage.size(), p2);

        if (!hasP1 || !hasP2) {
            result.fallback_to_line = true;
            std::cout << "[U-Net] 找不到橘缝端点对应的激光 y，跳过该条曲线。" << std::endl;
            curves.push_back(result);
            continue;
        }

        if (probability.empty() || !traceSingleCurveByProbability(probability, p1, p2, result.curve_points, result.mean_cost)) {
            result.curve_points = buildLinePathUnet(p1, p2, originImage.size());
            result.fallback_to_line = true;
        }

        curves.push_back(result);
    }

    return curves;
}
