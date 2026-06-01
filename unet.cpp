#include "unet.h"

#include <opencv2/dnn.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

// 模型与开关
static const bool UNET_SEAM_ENABLE = true; // U-Net 总开关；false 退回传统直线
static const char* UNET_MODEL_PATH_PRIMARY = "/home/dw/robot/cpp/best_unet_320.onnx"; // 部署模型；改错会加载失败
static const char* UNET_MODEL_PATH_FALLBACK = "cpp/best_unet_320.onnx"; // 本地兜底；改错会失去本地兜底
static const int UNET_INPUT_SIZE = 320; // 模型输入；必须匹配 ONNX 导出尺寸

// 中段 DP 基础参数
static const int UNET_ROI_X_MARGIN = 55; // 中段 ROI 半宽；调高更能弯但易串线，调低更稳但易截断
static const int UNET_MIDDLE_HARD_X_MARGIN = 34; // 默认中段硬半宽；调高更自由，调低更贴传统线
static const int UNET_ROI_Y_MARGIN = 8; // 端点 y 余量；调高容错更大，调低 ROI 更紧
static const int UNET_MIN_VERTICAL_SPAN = 8; // 最小 y 跨度；调高少追短线，调低允许短线
static const int UNET_MAX_X_STEP_PER_ROW = 7; // 默认单行横跳；调高更能弯但易漂，调低更平滑但跟弯弱
static const double UNET_TRANSITION_WEIGHT = 0.07; // 横跳惩罚；调高更平滑更保守，调低更追概率
static const double UNET_SHAPE_PRIOR_WEIGHT = 0.32; // 默认中段线先验；调高更贴传统线，调低更信 U-Net

// 全图候选线提取
static const float UNET_FULL_CURVE_BASE_THRESHOLD = 0.50f; // 全图候选阈值；调高少噪声但易漏，调低多候选但噪声多
static const double UNET_FULL_CURVE_PERCENTILE = 0.97; // 全图候选分位；调高更严格，调低更宽松
static const int UNET_FULL_CURVE_MIN_AREA = 30; // 最小连通域；调高滤小噪声，调低保留碎线
static const int UNET_FULL_CURVE_MIN_POINTS = 12; // 最短候选线；调高少短线，调低多碎片
static const int UNET_FULL_CURVE_MAX_COUNT = 12; // 最多候选线；调高保留更多但更慢，调低更快但可能漏

// 完整候选线平移匹配
static const int UNET_SHIFT_Y_WINDOW = 12; // 平移估计 y 窗口；调高更平滑，调低更局部
static const int UNET_SHIFT_MAX_Y_GAP = 24; // 平移估计最大 y 差；调高容错大，调低更严格
static const double UNET_SHIFT_MAX_ABS_DX = 60.0; // 最大整体平移；调高允许大偏移，调低拒绝偏离
static const double UNET_SHIFT_MAX_DX_DELTA = 35.0; // 上下平移差；调高允许倾斜误差，调低更要求一致

// 候选选择代价
static const double UNET_SELECT_MAX_ANCHOR_ERROR = 12.0; // 平移线最大锚点误差；调高更宽松，调低更准但易退回
static const double UNET_SELECT_ANCHOR_WEIGHT = 0.035; // 锚点误差权重；调高更信传统点，调低更信曲线形状
static const double UNET_SELECT_PROB_WEIGHT = 1.00; // 概率代价权重；调高更偏 U-Net 高概率线，调低更偏传统约束
static const double UNET_SELECT_SHAPE_WEIGHT = 0.45; // 形状偏离权重；调高更贴传统外推，调低更保留 U-Net 形状
static const double UNET_SELECT_SHAPE_NORM = 60.0; // 形状偏离归一化；调高更宽松，调低更严格
static const double UNET_SELECT_EDGE_SHIFTED_RATIO = 1.5; // 边缘平移线接受倍率；调高更偏平移线，调低更偏锚点DP
static const double UNET_SELECT_CENTER_SHIFTED_RATIO = 0.5; // 中心平移线接受倍率；调高更易选平移线，调低更严格

// 上下延伸追线
static const int UNET_EXTENSION_X_MARGIN = 44; // 默认延伸半宽；调高延伸更自由，调低更贴外推线
static const float UNET_EXTENSION_ROW_PROB_THRESHOLD = 0.45f; // 延伸行阈值；调高更早停止，调低延伸更长
static const int UNET_EXTENSION_MIN_ROWS = 8; // 最短延伸；调高少短延伸，调低允许短延伸
static const int UNET_EXTENSION_LOW_PROB_PATIENCE = 18; // 低概率耐心；调高更不易停，调低更早停
static const double UNET_EXTENSION_SHAPE_PRIOR_WEIGHT = 0.26; // 默认延伸先验；调高更贴外推线，调低更信 U-Net

// 锚点附近保护
static const int UNET_ANCHOR_LOW_PROB_RADIUS = 20; // 锚点弱概率半径；调高锚点附近更信传统，调低更早信 U-Net
static const int UNET_ANCHOR_FULL_PROB_RADIUS = 40; // 概率恢复半径；调高更慢恢复 U-Net，调低更快恢复
static const double UNET_ANCHOR_MIN_PROB_WEIGHT = 0.20; // 锚点最低概率权重；调高更信锚点附近 U-Net，调低更信传统
static const double UNET_ANCHOR_NEAR_TRANSITION_WEIGHT = 0.12; // 锚点横跳惩罚；调高锚点附近更直，调低更能弯

// 多橘缝隔离
static const int UNET_NEIGHBOR_BOUND_MARGIN = 12; // 相邻线隔离余量；调高更防串线但可用区变窄，调低更宽松

// 图像中心自适应权重
static const double UNET_CENTER_UNET_WEIGHT = 1.00; // 中心 U-Net 权重；调高中心更信 U-Net，调低中心更保守
static const double UNET_EDGE_UNET_WEIGHT = 0.18; // 边缘 U-Net 权重；调高边缘更追概率，调低边缘更信传统
static const double UNET_CENTER_TRANSITION_SCALE = 0.85; // 中心横跳倍率；调高中心更平滑，调低中心更灵活
static const double UNET_EDGE_TRANSITION_SCALE = 2.30; // 边缘横跳倍率；调高边缘更稳，调低边缘更容易漂
static const double UNET_CENTER_MIDDLE_SHAPE_WEIGHT = 0.22; // 中心中段先验；调高中心更贴直线，调低中心更随 U-Net
static const double UNET_EDGE_MIDDLE_SHAPE_WEIGHT = 1.00; // 边缘中段先验；调高边缘更贴传统，调低边缘更随 U-Net
static const double UNET_CENTER_EXTENSION_SHAPE_WEIGHT = 0.18; // 中心延伸先验；调高中心延伸更直，调低更随 U-Net
static const double UNET_EDGE_EXTENSION_SHAPE_WEIGHT = 0.95; // 边缘延伸先验；调高边缘延伸更贴外推，调低更随 U-Net
static const int UNET_CENTER_MIDDLE_X_MARGIN = 46; // 中心中段半宽；调高更能弯，调低更防跑偏
static const int UNET_EDGE_MIDDLE_X_MARGIN = 18; // 边缘中段半宽；调高边缘更自由，调低更防串线
static const int UNET_CENTER_EXTENSION_X_MARGIN = 58; // 中心延伸半宽；调高延伸更自由，调低更保守
static const int UNET_EDGE_EXTENSION_X_MARGIN = 20; // 边缘延伸半宽；调高边缘可追弯，调低更防跑偏
static const int UNET_CENTER_MAX_X_STEP = 9; // 中心单行横跳；调高中心更灵活，调低更平滑
static const int UNET_EDGE_MAX_X_STEP = 3; // 边缘单行横跳；调高边缘更灵活，调低更稳

// 前向声明：执行 U-Net ONNX 推理并返回与原图同尺寸的 0~1 概率图。
static cv::Mat inferUnetProbability(const cv::Mat& image);

// 将整数坐标限制在指定范围内，避免访问图像边界外的像素。
static int clampIntUnet(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

// 计算离激光锚点的渐变系数：0 表示靠近锚点，1 表示已经进入正常 U-Net 追线区域。
static double anchorDistanceBlendUnet(int distancePx) {
    if (distancePx <= UNET_ANCHOR_LOW_PROB_RADIUS) return 0.0;
    if (distancePx >= UNET_ANCHOR_FULL_PROB_RADIUS) return 1.0;
    return static_cast<double>(distancePx - UNET_ANCHOR_LOW_PROB_RADIUS) /
           static_cast<double>(UNET_ANCHOR_FULL_PROB_RADIUS - UNET_ANCHOR_LOW_PROB_RADIUS);
}

static double anchorProbabilityWeightUnet(int distancePx) {
    double blend = anchorDistanceBlendUnet(distancePx);
    return UNET_ANCHOR_MIN_PROB_WEIGHT * (1.0 - blend) + blend;
}

static double anchorTransitionWeightUnet(int distancePx) {
    double blend = anchorDistanceBlendUnet(distancePx);
    return UNET_ANCHOR_NEAR_TRANSITION_WEIGHT * (1.0 - blend) + UNET_TRANSITION_WEIGHT * blend;
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

struct UnetTraceXBounds {
    bool enabled = false;
    int minX = 0;
    int maxX = 0;
};

struct UnetTraceTuning {
    double centerTrust = 1.0;
    double unetWeight = 1.0;
    double transitionWeightScale = 1.0;
    double middleShapePriorWeight = UNET_SHAPE_PRIOR_WEIGHT;
    double extensionShapePriorWeight = UNET_EXTENSION_SHAPE_PRIOR_WEIGHT;
    int middleXMargin = UNET_MIDDLE_HARD_X_MARGIN;
    int extensionXMargin = UNET_EXTENSION_X_MARGIN;
    int maxXStepPerRow = UNET_MAX_X_STEP_PER_ROW;
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

static bool clampXRangeToTraceBoundsUnet(int& x0, int& x1, const UnetTraceXBounds* bounds) {
    if (bounds == nullptr || !bounds->enabled) return x0 <= x1;
    x0 = std::max(x0, bounds->minX);
    x1 = std::min(x1, bounds->maxX);
    return x0 <= x1;
}

static double clampDoubleUnet(double value, double minValue, double maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static double lerpUnet(double edgeValue, double centerValue, double centerTrust) {
    centerTrust = clampDoubleUnet(centerTrust, 0.0, 1.0);
    return edgeValue * (1.0 - centerTrust) + centerValue * centerTrust;
}

static int lerpIntUnet(int edgeValue, int centerValue, double centerTrust) {
    return static_cast<int>(std::round(lerpUnet(edgeValue, centerValue, centerTrust)));
}

static UnetTraceTuning buildTraceTuningForAnchorsUnet(
    cv::Point p1,
    cv::Point p2,
    const cv::Size& imageSize
) {
    UnetTraceTuning tuning;
    if (imageSize.width <= 1) return tuning;

    double seamMidX = 0.5 * (static_cast<double>(p1.x) + static_cast<double>(p2.x));
    double imageCenterX = 0.5 * static_cast<double>(imageSize.width - 1);
    double halfWidth = std::max(1.0, imageCenterX);
    double rawTrust = clampDoubleUnet(1.0 - std::abs(seamMidX - imageCenterX) / halfWidth, 0.0, 1.0);
    tuning.centerTrust = rawTrust * rawTrust;

    tuning.unetWeight = lerpUnet(UNET_EDGE_UNET_WEIGHT, UNET_CENTER_UNET_WEIGHT, tuning.centerTrust);
    tuning.transitionWeightScale = lerpUnet(UNET_EDGE_TRANSITION_SCALE, UNET_CENTER_TRANSITION_SCALE, tuning.centerTrust);
    tuning.middleShapePriorWeight = lerpUnet(
        UNET_EDGE_MIDDLE_SHAPE_WEIGHT, UNET_CENTER_MIDDLE_SHAPE_WEIGHT, tuning.centerTrust);
    tuning.extensionShapePriorWeight = lerpUnet(
        UNET_EDGE_EXTENSION_SHAPE_WEIGHT, UNET_CENTER_EXTENSION_SHAPE_WEIGHT, tuning.centerTrust);
    tuning.middleXMargin = std::max(1, lerpIntUnet(
        UNET_EDGE_MIDDLE_X_MARGIN, UNET_CENTER_MIDDLE_X_MARGIN, tuning.centerTrust));
    tuning.extensionXMargin = std::max(1, lerpIntUnet(
        UNET_EDGE_EXTENSION_X_MARGIN, UNET_CENTER_EXTENSION_X_MARGIN, tuning.centerTrust));
    tuning.maxXStepPerRow = std::max(1, lerpIntUnet(
        UNET_EDGE_MAX_X_STEP, UNET_CENTER_MAX_X_STEP, tuning.centerTrust));

    std::cout << "[U-Net] center tuning: seam_mid_x=" << seamMidX
              << ", image_center_x=" << imageCenterX
              << ", raw_trust=" << rawTrust
              << ", trust=" << tuning.centerTrust
              << ", unet_w=" << tuning.unetWeight
              << ", line_w=" << tuning.middleShapePriorWeight
              << ", mid_margin=" << tuning.middleXMargin
              << ", ext_margin=" << tuning.extensionXMargin
              << ", max_step=" << tuning.maxXStepPerRow
              << std::endl;

    return tuning;
}

static double seamPairAnchorMidXUnet(const MatchedSeamPair& pair) {
    return 0.5 * (static_cast<double>(pair.s1.x_peak) + static_cast<double>(pair.s2.x_peak));
}

static std::vector<UnetTraceXBounds> buildTraceBoundsForPairsUnet(
    const std::vector<MatchedSeamPair>& seamPairs,
    const cv::Size& imageSize
) {
    std::vector<UnetTraceXBounds> bounds(seamPairs.size());
    if (seamPairs.size() <= 1 || imageSize.width <= 0) return bounds;

    struct PairCenter {
        int index = 0;
        double centerX = 0.0;
        int minAnchorX = 0;
        int maxAnchorX = 0;
    };

    std::vector<PairCenter> centers;
    centers.reserve(seamPairs.size());
    for (int i = 0; i < static_cast<int>(seamPairs.size()); ++i) {
        PairCenter item;
        item.index = i;
        item.centerX = seamPairAnchorMidXUnet(seamPairs[i]);
        item.minAnchorX = std::min(seamPairs[i].s1.x_peak, seamPairs[i].s2.x_peak);
        item.maxAnchorX = std::max(seamPairs[i].s1.x_peak, seamPairs[i].s2.x_peak);
        centers.push_back(item);
    }

    std::sort(centers.begin(), centers.end(), [](const PairCenter& a, const PairCenter& b) {
        return a.centerX < b.centerX;
    });

    for (int pos = 0; pos < static_cast<int>(centers.size()); ++pos) {
        const PairCenter& cur = centers[pos];
        int leftBound = 0;
        int rightBound = imageSize.width - 1;

        if (pos > 0) {
            double splitX = 0.5 * (centers[pos - 1].centerX + cur.centerX);
            leftBound = static_cast<int>(std::round(splitX)) + UNET_NEIGHBOR_BOUND_MARGIN;
        }
        if (pos + 1 < static_cast<int>(centers.size())) {
            double splitX = 0.5 * (cur.centerX + centers[pos + 1].centerX);
            rightBound = static_cast<int>(std::round(splitX)) - UNET_NEIGHBOR_BOUND_MARGIN;
        }

        leftBound = std::min(leftBound, cur.minAnchorX - UNET_NEIGHBOR_BOUND_MARGIN);
        rightBound = std::max(rightBound, cur.maxAnchorX + UNET_NEIGHBOR_BOUND_MARGIN);
        leftBound = clampIntUnet(leftBound, 0, imageSize.width - 1);
        rightBound = clampIntUnet(rightBound, 0, imageSize.width - 1);

        if (leftBound > rightBound) {
            int center = clampIntUnet(static_cast<int>(std::round(cur.centerX)), 0, imageSize.width - 1);
            leftBound = clampIntUnet(center - UNET_MIDDLE_HARD_X_MARGIN, 0, imageSize.width - 1);
            rightBound = clampIntUnet(center + UNET_MIDDLE_HARD_X_MARGIN, 0, imageSize.width - 1);
        }

        bounds[cur.index].enabled = true;
        bounds[cur.index].minX = leftBound;
        bounds[cur.index].maxX = rightBound;

        std::cout << "[U-Net] trace bounds pair=" << cur.index
                  << ", x=[" << leftBound << "," << rightBound << "]"
                  << ", center=" << cur.centerX
                  << std::endl;
    }

    return bounds;
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
    double& meanCost,
    const UnetTraceXBounds* xBounds = nullptr,
    const UnetTraceTuning* tuning = nullptr
) {
    if (probability.empty()) return false;

    cv::Point start = p1;
    cv::Point end = p2;
    if (start.y > end.y) {
        std::swap(start, end);
    }

    if (std::abs(end.y - start.y) < UNET_MIN_VERTICAL_SPAN) return false;

    int middleXMargin = tuning != nullptr ? tuning->middleXMargin : UNET_MIDDLE_HARD_X_MARGIN;
    int maxXStep = tuning != nullptr ? tuning->maxXStepPerRow : UNET_MAX_X_STEP_PER_ROW;
    double unetWeight = tuning != nullptr ? tuning->unetWeight : 1.0;
    double transitionScale = tuning != nullptr ? tuning->transitionWeightScale : 1.0;
    double shapeWeight = tuning != nullptr ? tuning->middleShapePriorWeight : UNET_SHAPE_PRIOR_WEIGHT;
    int roiXMargin = std::max(UNET_ROI_X_MARGIN, middleXMargin + maxXStep + 4);

    int x0 = clampIntUnet(std::min(start.x, end.x) - roiXMargin, 0, probability.cols - 1);
    int x1 = clampIntUnet(std::max(start.x, end.x) + roiXMargin, 0, probability.cols - 1);
    if (!clampXRangeToTraceBoundsUnet(x0, x1, xBounds)) return false;
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
        int distanceToAnchor = std::min(y - localStart.y, localEnd.y - y);
        double probabilityWeight = anchorProbabilityWeightUnet(distanceToAnchor) * unetWeight;
        double transitionWeight = anchorTransitionWeightUnet(distanceToAnchor) * transitionScale;

        int rowX0 = clampIntUnet(static_cast<int>(std::round(lineX)) - middleXMargin, 0, probRoi.cols - 1);
        int rowX1 = clampIntUnet(static_cast<int>(std::round(lineX)) + middleXMargin, 0, probRoi.cols - 1);
        int globalRowX0 = roiRect.x + rowX0;
        int globalRowX1 = roiRect.x + rowX1;
        if (!clampXRangeToTraceBoundsUnet(globalRowX0, globalRowX1, xBounds)) {
            continue;
        }
        rowX0 = globalRowX0 - roiRect.x;
        rowX1 = globalRowX1 - roiRect.x;

        for (int x = rowX0; x <= rowX1; ++x) {
            int left = std::max(0, x - maxXStep);
            int right = std::min(probRoi.cols - 1, x + maxXStep);
            double bestPrev = INF;
            int bestPrevX = -1;

            for (int px = left; px <= right; ++px) {
                if (prevDp[px] >= INF / 2.0) continue;
                double transitionCost = transitionWeight * std::abs(x - px);
                double candidate = prevDp[px] + transitionCost;
                if (candidate < bestPrev) {
                    bestPrev = candidate;
                    bestPrevX = px;
                }
            }

            if (bestPrevX < 0) continue;

            double probabilityCost = probabilityWeight * (1.0 - static_cast<double>(probRoi.at<float>(y, x)));
            double offsetRatio = std::min(1.0, std::abs(x - lineX) / (middleXMargin + 1.0));
            double shapeCost = shapeWeight * offsetRatio * offsetRatio;
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

// 从一个传统激光锚点出发，沿 y 方向逐行在 U-Net 概率图里追高概率路径。
// direction=-1 表示向上延伸，direction=1 表示向下延伸；返回点序为“锚点 -> 外侧端点”。
static bool traceAnchoredExtensionByProbability(
    const cv::Mat& probability,
    cv::Point anchor,
    int direction,
    double slopeDxPerDy,
    std::vector<cv::Point>& extensionPoints,
    double& meanCost,
    const UnetTraceXBounds* xBounds = nullptr,
    const UnetTraceTuning* tuning = nullptr
) {
    extensionPoints.clear();
    meanCost = 0.0;

    if (probability.empty()) return false;
    if (direction != -1 && direction != 1) return false;
    if (anchor.x < 0 || anchor.x >= probability.cols ||
        anchor.y < 0 || anchor.y >= probability.rows) {
        return false;
    }

    int maxStep = direction < 0 ? anchor.y : (probability.rows - 1 - anchor.y);
    if (maxStep < UNET_EXTENSION_MIN_ROWS) return false;

    int extensionXMargin = tuning != nullptr ? tuning->extensionXMargin : UNET_EXTENSION_X_MARGIN;
    int maxXStep = tuning != nullptr ? tuning->maxXStepPerRow : UNET_MAX_X_STEP_PER_ROW;
    double unetWeight = tuning != nullptr ? tuning->unetWeight : 1.0;
    double transitionScale = tuning != nullptr ? tuning->transitionWeightScale : 1.0;
    double shapeWeight = tuning != nullptr ? tuning->extensionShapePriorWeight : UNET_EXTENSION_SHAPE_PRIOR_WEIGHT;

    const double INF = std::numeric_limits<double>::max() / 4.0;
    cv::Mat parent(maxStep + 1, probability.cols, CV_32S, cv::Scalar(-1));
    std::vector<double> prevDp(probability.cols, INF);
    std::vector<double> currDp(probability.cols, INF);
    std::vector<double> bestCostAtStep(maxStep + 1, INF);
    std::vector<int> bestXAtStep(maxStep + 1, -1);

    prevDp[anchor.x] = 0.0;
    bestCostAtStep[0] = 0.0;
    bestXAtStep[0] = anchor.x;

    int lastGoodStep = 0;
    int lowProbStreak = 0;

    for (int step = 1; step <= maxStep; ++step) {
        int y = anchor.y + direction * step;
        std::fill(currDp.begin(), currDp.end(), INF);

        double predictedX = static_cast<double>(anchor.x) + slopeDxPerDy * static_cast<double>(y - anchor.y);
        double probabilityWeight = anchorProbabilityWeightUnet(step) * unetWeight;
        double transitionWeight = anchorTransitionWeightUnet(step) * transitionScale;
        int x0 = clampIntUnet(static_cast<int>(std::round(predictedX)) - extensionXMargin, 0, probability.cols - 1);
        int x1 = clampIntUnet(static_cast<int>(std::round(predictedX)) + extensionXMargin, 0, probability.cols - 1);
        if (x1 < x0) std::swap(x0, x1);
        if (!clampXRangeToTraceBoundsUnet(x0, x1, xBounds)) break;

        const float* probRow = probability.ptr<float>(y);
        float rowBestProb = 0.0f;
        for (int x = x0; x <= x1; ++x) {
            rowBestProb = std::max(rowBestProb, probRow[x]);
        }

        for (int x = x0; x <= x1; ++x) {
            int left = std::max(0, x - maxXStep);
            int right = std::min(probability.cols - 1, x + maxXStep);
            double bestPrev = INF;
            int bestPrevX = -1;

            for (int px = left; px <= right; ++px) {
                if (prevDp[px] >= INF / 2.0) continue;
                double transitionCost = transitionWeight * std::abs(x - px);
                double candidate = prevDp[px] + transitionCost;
                if (candidate < bestPrev) {
                    bestPrev = candidate;
                    bestPrevX = px;
                }
            }

            if (bestPrevX < 0) continue;

            double probabilityCost = probabilityWeight * (1.0 - static_cast<double>(probRow[x]));
            double offsetRatio = std::min(1.0, std::abs(static_cast<double>(x) - predictedX) /
                                                (extensionXMargin + 1.0));
            double shapeCost = shapeWeight * offsetRatio * offsetRatio;
            currDp[x] = bestPrev + probabilityCost + shapeCost;
            parent.at<int>(step, x) = bestPrevX;
        }

        double bestCost = INF;
        int bestX = -1;
        for (int x = x0; x <= x1; ++x) {
            if (currDp[x] < bestCost) {
                bestCost = currDp[x];
                bestX = x;
            }
        }
        if (bestX < 0) break;

        bestCostAtStep[step] = bestCost;
        bestXAtStep[step] = bestX;

        if (step <= UNET_ANCHOR_LOW_PROB_RADIUS) {
            lowProbStreak = 0;
        } else if (rowBestProb >= UNET_EXTENSION_ROW_PROB_THRESHOLD) {
            lastGoodStep = step;
            lowProbStreak = 0;
        } else {
            lowProbStreak++;
        }

        prevDp.swap(currDp);

        if (step >= UNET_EXTENSION_MIN_ROWS &&
            lowProbStreak >= UNET_EXTENSION_LOW_PROB_PATIENCE) {
            break;
        }
    }

    if (lastGoodStep < UNET_EXTENSION_MIN_ROWS) return false;
    int x = bestXAtStep[lastGoodStep];
    if (x < 0) return false;

    std::vector<cv::Point> reversedPath;
    reversedPath.reserve(lastGoodStep + 1);
    for (int step = lastGoodStep; step >= 0; --step) {
        int y = anchor.y + direction * step;
        reversedPath.push_back(cv::Point(x, y));
        if (step == 0) break;

        int prevX = parent.at<int>(step, x);
        if (prevX < 0) return false;
        x = prevX;
    }

    std::reverse(reversedPath.begin(), reversedPath.end());
    extensionPoints = smoothCurvePathUnet(reversedPath, 2, probability.size());
    meanCost = bestCostAtStep[lastGoodStep] / std::max(1, lastGoodStep);
    return extensionPoints.size() >= 2;
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

struct ShiftEstimateUnet {
    bool valid = false;
    double dx = 0.0;
    double dx1 = 0.0;
    double dx2 = 0.0;
};

struct CurveCandidateScoreUnet {
    bool valid = false;
    double total = std::numeric_limits<double>::max();
    double anchorCost = 0.0;
    double probabilityCost = 1.0;
    double shapeCost = 0.0;
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

static double probabilityCostOnCurveUnet(
    const cv::Mat& probability,
    const std::vector<cv::Point>& curve,
    double fallbackCost
) {
    if (probability.empty() || curve.empty()) return fallbackCost;

    double total = 0.0;
    int count = 0;
    for (const auto& point : curve) {
        if (point.x < 0 || point.x >= probability.cols ||
            point.y < 0 || point.y >= probability.rows) {
            continue;
        }

        total += 1.0 - static_cast<double>(probability.at<float>(point.y, point.x));
        count++;
    }

    return count > 0 ? total / static_cast<double>(count) : fallbackCost;
}

static double shapeCostToTraditionalLineUnet(
    const std::vector<cv::Point>& curve,
    cv::Point p1,
    cv::Point p2
) {
    if (curve.empty()) return 1.0;

    cv::Point top = p1;
    cv::Point bottom = p2;
    if (top.y > bottom.y) std::swap(top, bottom);

    double slopeDxPerDy = 0.0;
    int dy = bottom.y - top.y;
    if (std::abs(dy) > 1) {
        slopeDxPerDy = static_cast<double>(bottom.x - top.x) / static_cast<double>(dy);
    }

    double total = 0.0;
    int count = 0;
    for (const auto& point : curve) {
        double predictedX = static_cast<double>(top.x) +
                            slopeDxPerDy * static_cast<double>(point.y - top.y);
        total += std::min(1.0, std::abs(static_cast<double>(point.x) - predictedX) / UNET_SELECT_SHAPE_NORM);
        count++;
    }

    return count > 0 ? total / static_cast<double>(count) : 1.0;
}

static bool curveInsideTraceBoundsUnet(
    const std::vector<cv::Point>& curve,
    const UnetTraceXBounds* bounds
) {
    if (bounds == nullptr || !bounds->enabled) return true;
    for (const auto& point : curve) {
        if (point.x < bounds->minX || point.x > bounds->maxX) {
            return false;
        }
    }
    return true;
}

static CurveCandidateScoreUnet scoreCurveCandidateUnet(
    const std::vector<cv::Point>& curve,
    const cv::Mat& probability,
    cv::Point p1,
    cv::Point p2,
    const UnetTraceXBounds* xBounds,
    double probabilityCostOverride,
    bool requireTightAnchors
) {
    CurveCandidateScoreUnet score;
    if (curve.size() < 2) return score;
    if (!curveInsideTraceBoundsUnet(curve, xBounds)) return score;

    double curveX1 = 0.0;
    double curveX2 = 0.0;
    if (!findCurveXAtYUnet(curve, p1.y, curveX1)) return score;
    if (!findCurveXAtYUnet(curve, p2.y, curveX2)) return score;

    double anchorDx1 = std::abs(curveX1 - static_cast<double>(p1.x));
    double anchorDx2 = std::abs(curveX2 - static_cast<double>(p2.x));
    score.anchorCost = 0.5 * (anchorDx1 + anchorDx2);
    if (requireTightAnchors && score.anchorCost > UNET_SELECT_MAX_ANCHOR_ERROR) return score;

    score.probabilityCost = probabilityCostOverride >= 0.0
        ? probabilityCostOverride
        : probabilityCostOnCurveUnet(probability, curve, 1.0);
    score.shapeCost = shapeCostToTraditionalLineUnet(curve, p1, p2);
    score.total = UNET_SELECT_ANCHOR_WEIGHT * score.anchorCost
                + UNET_SELECT_PROB_WEIGHT * score.probabilityCost
                + UNET_SELECT_SHAPE_WEIGHT * score.shapeCost;
    score.valid = true;
    return score;
}

// 从完整 U-Net 候选线里选出最适合两个传统橘缝点的一条，并只做 x 方向整体平移。
static bool selectShiftedFullCurveUnet(
    const std::vector<SeamCurveResult>& fullCurves,
    const std::vector<bool>& used,
    const cv::Mat& probability,
    cv::Point p1,
    cv::Point p2,
    const cv::Size& imageSize,
    const UnetTraceXBounds* xBounds,
    std::vector<cv::Point>& shiftedCurve,
    CurveCandidateScoreUnet& selectedScore,
    int& selectedIdx
) {
    selectedIdx = -1;
    selectedScore = CurveCandidateScoreUnet();
    shiftedCurve.clear();

    double bestCost = std::numeric_limits<double>::max();
    ShiftEstimateUnet bestEstimate;

    for (int i = 0; i < static_cast<int>(fullCurves.size()); ++i) {
        if (i < static_cast<int>(used.size()) && used[i]) continue;

        ShiftEstimateUnet estimate = estimateCurveShiftXUnet(fullCurves[i].curve_points, p1, p2);
        if (!estimate.valid) continue;

        std::vector<cv::Point> candidate = shiftCurveXUnet(fullCurves[i].curve_points, estimate.dx, imageSize);
        CurveCandidateScoreUnet score = scoreCurveCandidateUnet(
            candidate, probability, p1, p2, xBounds, fullCurves[i].mean_cost, true);
        if (!score.valid) continue;

        if (score.total < bestCost) {
            bestCost = score.total;
            bestEstimate = estimate;
            shiftedCurve = candidate;
            selectedScore = score;
            selectedIdx = i;
        }
    }

    if (selectedIdx < 0) return false;

    std::cout << "[U-Net] 使用完整候选线横向平移: idx=" << selectedIdx
              << ", dx=" << bestEstimate.dx
              << ", dx1=" << bestEstimate.dx1
              << ", dx2=" << bestEstimate.dx2
              << ", cost=" << selectedScore.total
              << ", anchor=" << selectedScore.anchorCost
              << ", prob=" << selectedScore.probabilityCost
              << ", shape=" << selectedScore.shapeCost
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

// 新的融合策略：传统双激光点作为硬锚点，U-Net 概率图负责从锚点向外追高概率路径。
// 输出顺序为整条橘缝从上到下：上延伸段 -> 两激光点之间 -> 下延伸段。
static bool buildAnchoredCurveByProbability(
    const cv::Mat& probability,
    cv::Point p1,
    cv::Point p2,
    cv::Size imageSize,
    std::vector<cv::Point>& curvePoints,
    double& meanCost,
    const UnetTraceXBounds* xBounds = nullptr,
    const UnetTraceTuning* tuning = nullptr
) {
    curvePoints.clear();
    meanCost = 0.0;
    if (probability.empty()) return false;

    cv::Point topLaser = p1;
    cv::Point bottomLaser = p2;
    if (topLaser.y > bottomLaser.y) {
        std::swap(topLaser, bottomLaser);
    }

    double slopeDxPerDy = 0.0;
    int dy = bottomLaser.y - topLaser.y;
    if (std::abs(dy) > 1) {
        slopeDxPerDy = static_cast<double>(bottomLaser.x - topLaser.x) / static_cast<double>(dy);
    }

    std::vector<cv::Point> upperFromAnchor;
    std::vector<cv::Point> lowerFromAnchor;
    std::vector<cv::Point> middleSegment;
    double upperCost = 0.0;
    double lowerCost = 0.0;
    double middleCost = 0.0;
    int costCount = 0;

    bool hasUpper = traceAnchoredExtensionByProbability(
        probability, topLaser, -1, slopeDxPerDy, upperFromAnchor, upperCost, xBounds, tuning);
    bool hasLower = traceAnchoredExtensionByProbability(
        probability, bottomLaser, 1, slopeDxPerDy, lowerFromAnchor, lowerCost, xBounds, tuning);

    bool hasMiddleProbability = traceSingleCurveByProbability(
        probability, topLaser, bottomLaser, middleSegment, middleCost, xBounds, tuning);
    if (!hasMiddleProbability) {
        middleSegment = buildLinePathUnet(topLaser, bottomLaser, imageSize);
        middleCost = 1.0;
    }

    if (hasUpper) {
        std::reverse(upperFromAnchor.begin(), upperFromAnchor.end());
        appendCurveSegmentUnet(curvePoints, upperFromAnchor);
        meanCost += upperCost;
        costCount++;
    }

    appendCurveSegmentUnet(curvePoints, middleSegment);
    meanCost += middleCost;
    costCount++;

    if (hasLower) {
        appendCurveSegmentUnet(curvePoints, lowerFromAnchor);
        meanCost += lowerCost;
        costCount++;
    }

    meanCost = meanCost / std::max(1, costCount);
    std::cout << "[U-Net] 锚点追线: upper=" << (hasUpper ? "true" : "false")
              << ", middle=" << (hasMiddleProbability ? "probability" : "line")
              << ", lower=" << (hasLower ? "true" : "false")
              << ", points=" << curvePoints.size()
              << ", mean_cost=" << meanCost
              << std::endl;

    return (hasUpper || hasMiddleProbability || hasLower) && curvePoints.size() >= 2;
}

// 无传统激光锚点时的调试兜底：从 U-Net 概率图中提取若干条完整候选橘缝中心线。
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

// 主入口：传统双激光点负责定位，U-Net 完整候选线负责形状；候选按统一代价选择。
// 如果模型不可用或候选不可信，则退回两个激光点之间的端点直线。
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

    if (seamPairs.empty()) {
        return extractFullUnetCurves(probability);
    }

    std::vector<UnetTraceXBounds> traceBounds = buildTraceBoundsForPairsUnet(seamPairs, originImage.size());
    std::vector<SeamCurveResult> fullCurves = extractFullUnetCurves(probability);
    std::vector<bool> usedFullCurve(fullCurves.size(), false);

    for (int pairIdx = 0; pairIdx < static_cast<int>(seamPairs.size()); ++pairIdx) {
        const auto& pair = seamPairs[pairIdx];
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

        UnetTraceTuning tuning = buildTraceTuningForAnchorsUnet(p1, p2, originImage.size());
        const UnetTraceXBounds* pairBounds =
            pairIdx < static_cast<int>(traceBounds.size()) ? &traceBounds[pairIdx] : nullptr;

        std::vector<cv::Point> anchoredCurve;
        double anchoredMeanCost = 0.0;
        CurveCandidateScoreUnet anchoredScore;
        bool hasAnchored = !probability.empty() && buildAnchoredCurveByProbability(
            probability,
            p1,
            p2,
            originImage.size(),
            anchoredCurve,
            anchoredMeanCost,
            pairBounds,
            &tuning
        );
        if (hasAnchored) {
            anchoredScore = scoreCurveCandidateUnet(
                anchoredCurve, probability, p1, p2, pairBounds, -1.0, false);
        }

        std::vector<cv::Point> shiftedCurve;
        CurveCandidateScoreUnet shiftedScore;
        int shiftedIdx = -1;
        if (!probability.empty()) {
            selectShiftedFullCurveUnet(
                fullCurves,
                usedFullCurve,
                probability,
                p1,
                p2,
                originImage.size(),
                pairBounds,
                shiftedCurve,
                shiftedScore,
                shiftedIdx
            );
        }

        if (anchoredScore.valid) {
            std::cout << "[U-Net] 锚点DP候选: cost=" << anchoredScore.total
                      << ", anchor=" << anchoredScore.anchorCost
                      << ", prob=" << anchoredScore.probabilityCost
                      << ", shape=" << anchoredScore.shapeCost
                      << std::endl;
        }
        if (shiftedScore.valid) {
            std::cout << "[U-Net] 平移候选: cost=" << shiftedScore.total
                      << ", anchor=" << shiftedScore.anchorCost
                      << ", prob=" << shiftedScore.probabilityCost
                      << ", shape=" << shiftedScore.shapeCost
                      << std::endl;
        }

        double shiftedAcceptRatio = lerpUnet(
            UNET_SELECT_EDGE_SHIFTED_RATIO,
            UNET_SELECT_CENTER_SHIFTED_RATIO,
            tuning.centerTrust
        );
        bool useShifted = false;
        if (shiftedScore.valid && !anchoredScore.valid) {
            useShifted = true;
        } else if (shiftedScore.valid && anchoredScore.valid) {
            useShifted = shiftedScore.total <= anchoredScore.total * shiftedAcceptRatio;
        }

        std::cout << "[U-Net] 选择倍率: pair=" << pairIdx
                  << ", center_trust=" << tuning.centerTrust
                  << ", shifted_ratio=" << shiftedAcceptRatio
                  << std::endl;

        if (useShifted) {
            result.curve_points = shiftedCurve;
            result.mean_cost = shiftedScore.total;
            result.fallback_to_line = false;
            if (shiftedIdx >= 0 && shiftedIdx < static_cast<int>(usedFullCurve.size())) {
                usedFullCurve[shiftedIdx] = true;
            }
            std::cout << "[U-Net] 选择平移完整线: pair=" << pairIdx
                      << ", idx=" << shiftedIdx
                      << ", cost=" << shiftedScore.total
                      << std::endl;
        } else if (anchoredScore.valid) {
            result.curve_points = anchoredCurve;
            result.mean_cost = anchoredScore.total;
            result.fallback_to_line = false;
            std::cout << "[U-Net] 选择锚点DP线: pair=" << pairIdx
                      << ", cost=" << anchoredScore.total
                      << std::endl;
        } else {
            result.curve_points = buildLinePathUnet(p1, p2, originImage.size());
            result.fallback_to_line = true;
            result.mean_cost = 1.0;
            std::cout << "[U-Net] 候选都不可用，使用端点直线: pair=" << pairIdx << std::endl;
        }

        curves.push_back(result);
    }

    return curves;
}
