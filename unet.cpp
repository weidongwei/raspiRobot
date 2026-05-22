#include "unet.h"

#include <opencv2/dnn.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

// 总开关：false 时不加载 U-Net，曲线会退回传统端点直线兜底。
static const bool UNET_SEAM_ENABLE = true;
// 树莓派部署时优先加载的 ONNX 路径。
static const char* UNET_MODEL_PATH_PRIMARY = "/home/dw/robot/cpp/best_unet_lite_320.onnx";
// 本地/相对路径兜底，方便在项目目录中直接运行。
static const char* UNET_MODEL_PATH_FALLBACK = "cpp/best_unet_lite_320.onnx";
// 模型固定输入尺寸；必须和 train3/export_onnx.py 导出的尺寸一致。
static const int UNET_INPUT_SIZE = 320;
// DP 追线时，端点连线左右额外扩展的搜索宽度。越大越能跟弯曲线，但越容易跑偏。
static const int UNET_ROI_X_MARGIN = 80;
// DP 追线时，端点上下额外扩展的搜索高度，主要避免端点贴边导致裁剪过紧。
static const int UNET_ROI_Y_MARGIN = 8;
// 两个端点 y 差小于该值时认为跨度太短，不做 DP 追线。
static const int UNET_MIN_VERTICAL_SPAN = 8;
// DP 每向下一行允许的最大横向跳动像素。越大曲线越灵活，越小曲线越平滑。
static const int UNET_MAX_X_STEP_PER_ROW = 10;
// DP 横向跳动惩罚权重，用来抑制曲线突然左右跳。
static const double UNET_TRANSITION_WEIGHT = 0.03;
// DP 直线先验权重，用来防止路径跑到 ROI 边缘的孤立高概率点。
static const double UNET_SHAPE_PRIOR_WEIGHT = 0.14;
// 提取完整候选曲线时的基础概率阈值，不会低于这个值。
static const float UNET_FULL_CURVE_BASE_THRESHOLD = 0.50f;
// 提取完整候选曲线时使用的概率分位数；0.97 表示只保留最高约 3% 的概率区域。
static const double UNET_FULL_CURVE_PERCENTILE = 0.97;
// 连通域面积小于该值时视为噪声，不作为完整候选曲线。
static const int UNET_FULL_CURVE_MIN_AREA = 30;
// 候选曲线中心线点数小于该值时丢弃，避免短碎片干扰。
static const int UNET_FULL_CURVE_MIN_POINTS = 12;
// 最多保留的完整候选曲线数量，防止噪声过多拖慢后续匹配。
static const int UNET_FULL_CURVE_MAX_COUNT = 12;
// 两个激光锚点到候选完整曲线的平均距离门限，超过则认为该候选不匹配。
static const double UNET_FULL_CURVE_ANCHOR_GATE = 40.0;
// 用传统激光点校准完整 U-Net 曲线时，在目标 y 附近取 x 的窗口。
static const int UNET_SHIFT_Y_WINDOW = 12;
// 如果候选曲线离激光点所在 y 太远，则不参与横向平移匹配。
static const int UNET_SHIFT_MAX_Y_GAP = 24;
// 完整曲线只允许做 x 方向整体平移；超过该值认为候选线不可信。
static const double UNET_SHIFT_MAX_ABS_DX = 60.0;
// 上下两个激光点估计出的横向偏移差太大时，不做刚性平移。
static const double UNET_SHIFT_MAX_DX_DELTA = 35.0;
// 候选评分里对上下偏移不一致的惩罚权重。
static const double UNET_SHIFT_DX_DELTA_WEIGHT = 1.5;
// 候选评分里对 U-Net 曲线平均低概率的惩罚权重。
static const double UNET_SHIFT_MEAN_COST_WEIGHT = 20.0;

// 前向声明：执行 U-Net ONNX 推理并返回与原图同尺寸的 0~1 概率图。
static cv::Mat inferUnetProbability(const cv::Mat& image);

// 将整数坐标限制在指定范围内，避免访问图像边界外的像素。
static int clampIntUnet(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

struct LastUnetProbabilityCache {
    bool valid = false;
    const uchar* imageData = nullptr;
    int rows = 0;
    int cols = 0;
    int type = 0;
    cv::Rect roi;
    cv::Mat probability;
};

static LastUnetProbabilityCache& getLastUnetProbabilityCache() {
    static LastUnetProbabilityCache cache;
    return cache;
}

static void storeLastUnetProbability(const cv::Mat& image, const cv::Rect& roi, const cv::Mat& probability) {
    LastUnetProbabilityCache& cache = getLastUnetProbabilityCache();
    cache.valid = !image.empty() && !roi.empty();
    cache.imageData = image.data;
    cache.rows = image.rows;
    cache.cols = image.cols;
    cache.type = image.type();
    cache.roi = roi;
    cache.probability = probability;
}

static bool takeLastUnetProbability(const cv::Mat& image, const cv::Rect& roi, cv::Mat& probability) {
    LastUnetProbabilityCache& cache = getLastUnetProbabilityCache();
    bool matched = cache.valid &&
                   cache.imageData == image.data &&
                   cache.rows == image.rows &&
                   cache.cols == image.cols &&
                   cache.type == image.type() &&
                   cache.roi.x == roi.x &&
                   cache.roi.y == roi.y &&
                   cache.roi.width == roi.width &&
                   cache.roi.height == roi.height;
    if (matched) {
        probability = cache.probability;
    }
    cache.valid = false;
    return matched;
}

// 检查模型文件是否存在；用于优先加载树莓派部署路径，失败后尝试本地相对路径。
static bool fileExistsUnet(const std::string& path) {
    std::ifstream file(path.c_str());
    return file.good();
}

// 计算概率图的分位数阈值，用来从“全图概率”里自适应取出最高置信度区域。
static float probabilityPercentileUnet(const cv::Mat& probability, double percentile) {
    std::vector<float> values;
    values.reserve(probability.total());

    for (int y = 0; y < probability.rows; ++y) {
        const float* row = probability.ptr<float>(y);
        for (int x = 0; x < probability.cols; ++x) {
            values.push_back(row[x]);
        }
    }

    if (values.empty()) return 1.0f;
    percentile = std::min(1.0, std::max(0.0, percentile));
    size_t idx = static_cast<size_t>(std::round(percentile * static_cast<double>(values.size() - 1)));
    idx = std::min(idx, values.size() - 1);
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

// 将模型 logits 转成概率；如果导出的 ONNX 已带 sigmoid，则推理阶段会跳过这一步。
static float sigmoidUnet(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// 根据 findSeam() 给出的 x_peak，在对应 laser_id 的激光点列中找最近 y，恢复真实图像端点。
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

// 构造两个端点之间的直线路径；当模型或 DP 追线失败时作为兜底输出。
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

// 对逐行路径做轻微 x 方向平滑，保留 y 的扫描顺序，并固定首尾端点。
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

// 把任意输入图转换成 BGR 显示图，供调试函数直接在图上画概率点。
static cv::Mat makeBgrDisplayImageUnet(const cv::Mat& image) {
    cv::Mat display;
    if (image.empty()) return display;
    if (image.channels() == 4) {
        cv::cvtColor(image, display, cv::COLOR_BGRA2BGR);
    } else if (image.channels() == 1) {
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    } else {
        display = image.clone();
    }
    return display;
}

// 延迟加载 U-Net ONNX 模型。第一次调用时加载，之后复用同一个 OpenCV DNN Net。
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

// 调试可视化：只运行 U-Net 概率图推理，把超过阈值的像素用蓝色点标在原图上。
cv::Mat drawUnetSeamProbabilityPoints(
    const cv::Mat& originImage,
    float threshold,
    int pointStride
) {
    cv::Mat display = makeBgrDisplayImageUnet(originImage);
    if (display.empty()) return display;

    cv::Rect fullImageRoi(0, 0, originImage.cols, originImage.rows);
    cv::Mat probability;
    bool reusedProbability = takeLastUnetProbability(originImage, fullImageRoi, probability);
    if (!reusedProbability) {
        probability = inferUnetProbability(originImage);
    }
    if (probability.empty()) return display;

    threshold = std::min(1.0f, std::max(0.0f, threshold));
    pointStride = std::max(1, pointStride);

    double minProb = 0.0;
    double maxProb = 0.0;
    cv::minMaxLoc(probability, &minProb, &maxProb);
    int pointCount = 0;
    int radius = (pointStride <= 2) ? 1 : std::max(1, pointStride / 2);

    for (int y = 0; y < probability.rows; y += pointStride) {
        const float* probRow = probability.ptr<float>(y);
        for (int x = 0; x < probability.cols; x += pointStride) {
            if (probRow[x] >= threshold) {
                cv::circle(display, cv::Point(x, y), radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
                pointCount++;
            }
        }
    }

    std::cout << "[U-Net] prob_points threshold=" << threshold
              << ", min=" << minProb
              << ", max=" << maxProb
              << ", points=" << pointCount
              << ", reused=" << (reusedProbability ? "true" : "false")
              << std::endl;

    return display;
}

// 调试可视化的保存版本：生成蓝色概率点图并写入指定路径。
bool saveUnetSeamProbabilityPoints(
    const cv::Mat& originImage,
    const std::string& outputPath,
    float threshold,
    int pointStride
) {
    cv::Mat display = drawUnetSeamProbabilityPoints(originImage, threshold, pointStride);
    if (display.empty()) return false;
    return cv::imwrite(outputPath, display);
}

// 按训练时的预处理方式构造输入 blob：BGR->RGB、resize 到模型输入尺寸、0~1、ImageNet mean/std。
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

// 执行 U-Net 推理并把模型输出恢复到输入图尺寸。返回值是每个像素属于橘缝的概率。
static cv::Mat inferUnetProbability(const cv::Mat& image) {
    cv::dnn::Net* net = getUnetNet();
    if (net == nullptr || image.empty()) return {};

    try {
        cv::Mat blob = makeUnetBlob(image);
        net->setInput(blob);
        cv::Mat output = net->forward();

        if (output.total() != static_cast<size_t>(UNET_INPUT_SIZE * UNET_INPUT_SIZE)) {
            std::cerr << "[U-Net] 输出尺寸不符合预期，期望 "
                      << UNET_INPUT_SIZE << "x" << UNET_INPUT_SIZE
                      << "，使用端点直线兜底。" << std::endl;
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

// 在两个给定端点之间，用 U-Net 概率图做逐行动态规划，寻找最低代价橘缝路径。
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

// 从概率图中构造高置信度二值 mask，用于估计完整橘缝的上下延伸范围。
static cv::Mat buildUnetFullCurveMask(const cv::Mat& probability, float& effectiveThreshold) {
    if (probability.empty()) return {};

    float percentileThreshold = probabilityPercentileUnet(probability, UNET_FULL_CURVE_PERCENTILE);
    effectiveThreshold = std::max(UNET_FULL_CURVE_BASE_THRESHOLD, percentileThreshold);

    cv::Mat binary(probability.size(), CV_8U, cv::Scalar(0));
    for (int y = 0; y < probability.rows; ++y) {
        const float* probRow = probability.ptr<float>(y);
        uchar* dstRow = binary.ptr<uchar>(y);
        for (int x = 0; x < probability.cols; ++x) {
            if (probRow[x] >= effectiveThreshold) {
                dstRow[x] = 255;
            }
        }
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    return binary;
}

// 对单个连通域按 y 行统计概率加权 x 中心，得到该连通域的粗中心线。
static std::vector<cv::Point> centerlineFromComponentRows(
    const cv::Mat& labels,
    int componentId,
    const cv::Mat& probability,
    double& meanProbability
) {
    std::vector<cv::Point> points;
    double probSum = 0.0;
    int probCount = 0;

    for (int y = 0; y < labels.rows; ++y) {
        const int* labelRow = labels.ptr<int>(y);
        const float* probRow = probability.ptr<float>(y);
        double weightedX = 0.0;
        double weightSum = 0.0;

        for (int x = 0; x < labels.cols; ++x) {
            if (labelRow[x] != componentId) continue;

            double weight = std::max(1e-4, static_cast<double>(probRow[x]));
            weightedX += x * weight;
            weightSum += weight;
            probSum += probRow[x];
            probCount++;
        }

        if (weightSum > 0.0) {
            int centerX = clampIntUnet(static_cast<int>(std::round(weightedX / weightSum)), 0, labels.cols - 1);
            points.push_back(cv::Point(centerX, y));
        }
    }

    meanProbability = probCount > 0 ? probSum / probCount : 0.0;
    if (points.size() < 2) return points;
    return smoothCurvePathUnet(points, 3, probability.size());
}

// 计算一条曲线的平均 x，用于候选曲线从左到右排序。
static double meanXOfCurveUnet(const std::vector<cv::Point>& points) {
    if (points.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& p : points) {
        sum += p.x;
    }
    return sum / static_cast<double>(points.size());
}

// 计算一个激光锚点到某条候选曲线的最近欧氏距离。
static double pointToCurveDistanceUnet(const cv::Point& p, const std::vector<cv::Point>& curve) {
    if (curve.empty()) return std::numeric_limits<double>::max() / 4.0;

    double best = std::numeric_limits<double>::max();
    for (const auto& q : curve) {
        double dx = static_cast<double>(p.x - q.x);
        double dy = static_cast<double>(p.y - q.y);
        best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    return best;
}

// 用两个激光锚点到候选曲线的平均距离，衡量该候选曲线是否对应当前橘缝对。
static double curveAnchorCostUnet(const std::vector<cv::Point>& curve, cv::Point p1, cv::Point p2) {
    double d1 = pointToCurveDistanceUnet(p1, curve);
    double d2 = pointToCurveDistanceUnet(p2, curve);
    return 0.5 * (d1 + d2);
}

struct ShiftEstimateUnet {
    bool valid = false;
    double dx = 0.0;
    double dx1 = 0.0;
    double dx2 = 0.0;
    double cost = 0.0;
};

// 在指定 y 附近估计候选曲线的 x。只用于 x 方向校准，不修改曲线的 y。
static bool findCurveXAtYUnet(
    const std::vector<cv::Point>& curve,
    int targetY,
    double& xAtY
) {
    if (curve.empty()) return false;

    int bestDy = std::numeric_limits<int>::max();
    int bestX = 0;
    double weightedX = 0.0;
    double weightSum = 0.0;

    for (const auto& point : curve) {
        int dy = std::abs(point.y - targetY);
        if (dy < bestDy) {
            bestDy = dy;
            bestX = point.x;
        }

        if (dy <= UNET_SHIFT_Y_WINDOW) {
            double weight = 1.0 / static_cast<double>(dy + 1);
            weightedX += point.x * weight;
            weightSum += weight;
        }
    }

    if (weightSum > 0.0) {
        xAtY = weightedX / weightSum;
        return true;
    }

    if (bestDy <= UNET_SHIFT_MAX_Y_GAP) {
        xAtY = bestX;
        return true;
    }

    return false;
}

// 根据两个传统激光锚点估计完整 U-Net 候选曲线需要做的整体 x 平移。
static ShiftEstimateUnet estimateCurveShiftXUnet(
    const std::vector<cv::Point>& curve,
    cv::Point p1,
    cv::Point p2
) {
    ShiftEstimateUnet estimate;
    double curveX1 = 0.0;
    double curveX2 = 0.0;

    if (!findCurveXAtYUnet(curve, p1.y, curveX1)) return estimate;
    if (!findCurveXAtYUnet(curve, p2.y, curveX2)) return estimate;

    estimate.dx1 = static_cast<double>(p1.x) - curveX1;
    estimate.dx2 = static_cast<double>(p2.x) - curveX2;
    estimate.dx = 0.5 * (estimate.dx1 + estimate.dx2);

    double dxDelta = std::abs(estimate.dx1 - estimate.dx2);
    if (std::abs(estimate.dx) > UNET_SHIFT_MAX_ABS_DX) return estimate;
    if (dxDelta > UNET_SHIFT_MAX_DX_DELTA) return estimate;

    estimate.cost = std::abs(estimate.dx1)
                  + std::abs(estimate.dx2)
                  + dxDelta * UNET_SHIFT_DX_DELTA_WEIGHT;
    estimate.valid = true;
    return estimate;
}

// 只做 x 方向整体平移，y 坐标保持 U-Net 完整曲线原样。
static std::vector<cv::Point> shiftCurveXUnet(
    const std::vector<cv::Point>& curve,
    double dx,
    const cv::Size& imageSize
) {
    std::vector<cv::Point> shifted;
    shifted.reserve(curve.size());

    int roundedDx = static_cast<int>(std::round(dx));
    for (const auto& point : curve) {
        int x = clampIntUnet(point.x + roundedDx, 0, imageSize.width - 1);
        int y = clampIntUnet(point.y, 0, imageSize.height - 1);
        shifted.push_back(cv::Point(x, y));
    }

    return shifted;
}

// 从完整 U-Net 候选线里选出最适合两个传统橘缝点的一条，并只做 x 方向整体平移。
static bool selectShiftedFullCurveUnet(
    const std::vector<SeamCurveResult>& fullCurves,
    const std::vector<bool>& used,
    cv::Point p1,
    cv::Point p2,
    const cv::Size& imageSize,
    std::vector<cv::Point>& shiftedCurve,
    double& meanCost,
    int& selectedIdx
) {
    selectedIdx = -1;
    meanCost = 0.0;
    shiftedCurve.clear();

    double bestCost = std::numeric_limits<double>::max();
    ShiftEstimateUnet bestEstimate;

    for (int i = 0; i < static_cast<int>(fullCurves.size()); ++i) {
        if (i < static_cast<int>(used.size()) && used[i]) continue;

        ShiftEstimateUnet estimate = estimateCurveShiftXUnet(fullCurves[i].curve_points, p1, p2);
        if (!estimate.valid) continue;

        double cost = estimate.cost + fullCurves[i].mean_cost * UNET_SHIFT_MEAN_COST_WEIGHT;
        if (cost < bestCost) {
            bestCost = cost;
            bestEstimate = estimate;
            selectedIdx = i;
        }
    }

    if (selectedIdx < 0) return false;

    shiftedCurve = shiftCurveXUnet(fullCurves[selectedIdx].curve_points, bestEstimate.dx, imageSize);
    meanCost = fullCurves[selectedIdx].mean_cost;

    std::cout << "[U-Net] 使用完整候选线横向平移: idx=" << selectedIdx
              << ", dx=" << bestEstimate.dx
              << ", dx1=" << bestEstimate.dx1
              << ", dx2=" << bestEstimate.dx2
              << ", cost=" << bestCost
              << std::endl;

    return shiftedCurve.size() >= 2;
}

// 拼接上/中/下三段曲线；如果相邻段首尾相同，则跳过重复端点。
static void appendCurveSegmentUnet(std::vector<cv::Point>& dst, const std::vector<cv::Point>& segment) {
    if (segment.empty()) return;
    if (dst.empty()) {
        dst.insert(dst.end(), segment.begin(), segment.end());
        return;
    }

    size_t start = 0;
    if (dst.back() == segment.front()) {
        start = 1;
    }
    dst.insert(dst.end(), segment.begin() + start, segment.end());
}

// 以两个激光点为锚点生成整条橘缝：上段、中段、下段都用概率图 DP 追线。
static bool buildThreePartCurveByProbability(
    const cv::Mat& probability,
    cv::Point p1,
    cv::Point p2,
    const std::vector<cv::Point>* selectedFullCurve,
    cv::Size imageSize,
    std::vector<cv::Point>& curvePoints,
    double& meanCost
) {
    curvePoints.clear();
    meanCost = 0.0;
    if (probability.empty()) return false;

    cv::Point topLaser = p1;
    cv::Point bottomLaser = p2;
    if (topLaser.y > bottomLaser.y) {
        std::swap(topLaser, bottomLaser);
    }

    std::vector<cv::Point> middleSegment;
    double middleCost = 0.0;
    if (!traceSingleCurveByProbability(probability, topLaser, bottomLaser, middleSegment, middleCost)) {
        middleSegment = buildLinePathUnet(topLaser, bottomLaser, imageSize);
        middleCost = 1.0;
    }

    std::vector<cv::Point> upperSegment;
    std::vector<cv::Point> lowerSegment;
    double upperCost = 0.0;
    double lowerCost = 0.0;
    int costCount = 1;

    if (selectedFullCurve != nullptr && selectedFullCurve->size() >= 2) {
        cv::Point topEnd = selectedFullCurve->front();
        cv::Point bottomEnd = selectedFullCurve->back();
        if (topEnd.y > bottomEnd.y) {
            std::swap(topEnd, bottomEnd);
        }

        if (topLaser.y - topEnd.y >= UNET_MIN_VERTICAL_SPAN) {
            if (traceSingleCurveByProbability(probability, topEnd, topLaser, upperSegment, upperCost)) {
                costCount++;
            } else {
                upperSegment.clear();
            }
        }

        if (bottomEnd.y - bottomLaser.y >= UNET_MIN_VERTICAL_SPAN) {
            if (traceSingleCurveByProbability(probability, bottomLaser, bottomEnd, lowerSegment, lowerCost)) {
                costCount++;
            } else {
                lowerSegment.clear();
            }
        }
    }

    appendCurveSegmentUnet(curvePoints, upperSegment);
    appendCurveSegmentUnet(curvePoints, middleSegment);
    appendCurveSegmentUnet(curvePoints, lowerSegment);

    if (curvePoints.size() >= 3) {
        curvePoints = smoothCurvePathUnet(curvePoints, 2, imageSize);
    }

    meanCost = (middleCost + upperCost + lowerCost) / std::max(1, costCount);
    return curvePoints.size() >= 2;
}

// 从 U-Net 概率图中提取若干条完整候选橘缝中心线，主要用于确定上/下延伸端点。
static std::vector<SeamCurveResult> extractFullUnetCurves(const cv::Mat& probability) {
    std::vector<SeamCurveResult> curves;
    if (probability.empty()) return curves;

    float effectiveThreshold = 0.0f;
    cv::Mat binary = buildUnetFullCurveMask(probability, effectiveThreshold);
    if (binary.empty()) return curves;

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    int componentCount = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

    for (int id = 1; id < componentCount; ++id) {
        int area = stats.at<int>(id, cv::CC_STAT_AREA);
        if (area < UNET_FULL_CURVE_MIN_AREA) continue;

        double meanProbability = 0.0;
        std::vector<cv::Point> centerline = centerlineFromComponentRows(labels, id, probability, meanProbability);
        if (static_cast<int>(centerline.size()) < UNET_FULL_CURVE_MIN_POINTS) continue;

        SeamCurveResult result;
        result.curve_points = centerline;
        result.mean_cost = 1.0 - meanProbability;
        result.fallback_to_line = false;
        curves.push_back(result);
    }

    std::sort(curves.begin(), curves.end(), [](const SeamCurveResult& a, const SeamCurveResult& b) {
        if (a.curve_points.size() != b.curve_points.size()) {
            return a.curve_points.size() > b.curve_points.size();
        }
        return meanXOfCurveUnet(a.curve_points) < meanXOfCurveUnet(b.curve_points);
    });

    if (static_cast<int>(curves.size()) > UNET_FULL_CURVE_MAX_COUNT) {
        curves.resize(UNET_FULL_CURVE_MAX_COUNT);
    }

    std::sort(curves.begin(), curves.end(), [](const SeamCurveResult& a, const SeamCurveResult& b) {
        return meanXOfCurveUnet(a.curve_points) < meanXOfCurveUnet(b.curve_points);
    });

    std::cout << "[U-Net] full_curves threshold=" << effectiveThreshold
              << ", curves=" << curves.size()
              << std::endl;
    return curves;
}

// 主入口：传统双激光结果给出锚点，优先用完整 U-Net 候选线做 x 方向校准。
// 如果候选线和平移约束不可信，再退回端点 DP 追线。
std::vector<SeamCurveResult> traceSeamCurvesByUnet(
    const cv::Mat& originImage,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData
) {
    std::vector<SeamCurveResult> curves;
    curves.reserve(seamPairs.size());

    if (originImage.empty()) return curves;

    cv::Mat probability = inferUnetProbability(originImage);
    cv::Rect fullImageRoi(0, 0, originImage.cols, originImage.rows);
    storeLastUnetProbability(originImage, fullImageRoi, probability);
    std::vector<SeamCurveResult> fullCurves = extractFullUnetCurves(probability);

    if (seamPairs.empty()) {
        return fullCurves;
    }

    std::vector<bool> used(fullCurves.size(), false);
    for (const auto& pair : seamPairs) {
        SeamCurveResult result;
        result.pair = pair;
        result.mean_cost = 0.0;
        result.fallback_to_line = false;

        cv::Point p1;
        cv::Point p2;
        bool hasP1 = findNearestLaserEndpointUnet(laserData, pair.s1.id, pair.s1.x_peak, originImage.size(), p1);
        bool hasP2 = findNearestLaserEndpointUnet(laserData, pair.s2.id, pair.s2.x_peak, originImage.size(), p2);
        result.laser_point1 = p1;
        result.laser_point2 = p2;
        result.has_laser_points = hasP1 && hasP2;

        if (!hasP1 || !hasP2) {
            result.fallback_to_line = true;
            std::cout << "[U-Net] 找不到橘缝端点对应的激光 y，跳过该条曲线。" << std::endl;
            curves.push_back(result);
            continue;
        }

        int shiftedIdx = -1;
        if (selectShiftedFullCurveUnet(
                fullCurves,
                used,
                p1,
                p2,
                originImage.size(),
                result.curve_points,
                result.mean_cost,
                shiftedIdx
            )) {
            result.fallback_to_line = false;
            if (shiftedIdx >= 0 && shiftedIdx < static_cast<int>(used.size())) {
                used[shiftedIdx] = true;
            }
            curves.push_back(result);
            continue;
        }

        int bestIdx = -1;
        double bestCost = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(fullCurves.size()); ++i) {
            if (used[i]) continue;

            double cost = curveAnchorCostUnet(fullCurves[i].curve_points, p1, p2);
            if (cost < bestCost) {
                bestCost = cost;
                bestIdx = i;
            }
        }

        const std::vector<cv::Point>* selectedFullCurve = nullptr;
        if (bestIdx >= 0 && bestCost <= UNET_FULL_CURVE_ANCHOR_GATE) {
            selectedFullCurve = &fullCurves[bestIdx].curve_points;
        }

        if (!probability.empty() && buildThreePartCurveByProbability(
                probability,
                p1,
                p2,
                selectedFullCurve,
                originImage.size(),
                result.curve_points,
                result.mean_cost
            )) {
            result.fallback_to_line = false;
            if (selectedFullCurve != nullptr && bestIdx >= 0) {
                used[bestIdx] = true;
            }
        } else {
            result.curve_points = buildLinePathUnet(p1, p2, originImage.size());
            result.fallback_to_line = true;
        }

        curves.push_back(result);
    }

    return curves;
}
