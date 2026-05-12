#include "yolo-seg.h"

#include <opencv2/dnn.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

// YOLO-seg 默认参数。这里故意不写入 visualConfig.json，避免影响原有视觉配置。
static const bool YOLO_SEAM_ENABLE = true;
static const char* YOLO_SEAM_MODEL_PATH = "/home/dw/robot/cpp/best.onnx";
static const int YOLO_SEAM_INPUT_SIZE = 640;
static const double YOLO_SEAM_CONF_THRESHOLD = 0.25;
static const double YOLO_SEAM_NMS_THRESHOLD = 0.45;
static const bool YOLO_SEAM_DRAW_ALL = true;

struct YoloLetterboxInfo {
    float scale = 1.0f;
    int padX = 0;
    int padY = 0;
    int newW = 0;
    int newH = 0;
};

struct YoloSegResult {
    int classId = 0;
    float score = 0.0f;
    cv::Rect box;
    cv::Mat mask;
};

// 将整数限制在给定范围内，避免坐标越界。
static int clampIntLocal(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

// 根据橘缝峰值 x 坐标，在指定激光线数据中寻找最近的真实激光像素点。
// findSeam() 只保存 x_peak，这里补回对应的 y 像素，作为 YOLO mask 选择的几何锚点。
static bool findNearestLaserEndpointLocal(
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

    endpoint.x = clampIntLocal(targetX, 0, imageSize.width - 1);
    endpoint.y = clampIntLocal(bestY, 0, imageSize.height - 1);
    return true;
}

// YOLO mask 原型输出是 logit，这里转成 0~1 的概率。
static float sigmoidFloat(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// 按 YOLO 常用 letterbox 方式缩放输入图，保持宽高比并用灰色填充到正方形输入尺寸。
// 同时记录缩放比例和 padding，后面需要把检测框和 mask 映射回原图。
static cv::Mat yoloLetterbox(const cv::Mat& image, int inputSize, YoloLetterboxInfo& info) {
    const int w = image.cols;
    const int h = image.rows;
    info.scale = std::min(inputSize / static_cast<float>(w), inputSize / static_cast<float>(h));
    info.newW = static_cast<int>(std::round(w * info.scale));
    info.newH = static_cast<int>(std::round(h * info.scale));
    info.padX = (inputSize - info.newW) / 2;
    info.padY = (inputSize - info.newH) / 2;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(info.newW, info.newH));

    cv::Mat output(inputSize, inputSize, image.type(), cv::Scalar(114, 114, 114));
    resized.copyTo(output(cv::Rect(info.padX, info.padY, info.newW, info.newH)));
    return output;
}

// 将 OpenCV DNN 返回的 YOLO detection 输出统一整理成“每一行一个候选框”的二维矩阵。
// 不同 YOLO/ONNX 导出版本可能是 [1, attrs, N] 或 [1, N, attrs]，这里做兼容处理。
static cv::Mat yoloDetectionsToRows(const cv::Mat& blob) {
    if (blob.dims == 2) return blob.clone();
    if (blob.dims != 3 || blob.size[0] != 1) {
        throw std::runtime_error("YOLO detection output shape unsupported.");
    }

    cv::Mat mat(blob.size[1], blob.size[2], CV_32F, const_cast<float*>(blob.ptr<float>()));
    if (blob.size[1] < blob.size[2]) {
        cv::Mat transposed;
        cv::transpose(mat, transposed);
        return transposed.clone();
    }
    return mat.clone();
}

// 将 YOLO-seg 的 mask prototype 输出整理成二维矩阵，便于用每个检测框的 mask 系数线性组合出实例 mask。
static cv::Mat yoloProtoToRows(const cv::Mat& proto, int& maskDim, int& maskH, int& maskW) {
    if (proto.dims != 4 || proto.size[0] != 1) {
        throw std::runtime_error("YOLO mask prototype output shape unsupported.");
    }
    maskDim = proto.size[1];
    maskH = proto.size[2];
    maskW = proto.size[3];
    return cv::Mat(maskDim, maskH * maskW, CV_32F, const_cast<float*>(proto.ptr<float>())).clone();
}

// 根据单个候选框的 mask 系数和 prototype 生成该实例的二值 mask，并映射回原图尺寸。
// 当前返回的是 0/255 二值 mask，用于可视化、距离变换和骨架提取。
static cv::Mat buildYoloMask(
    const std::vector<float>& coeff,
    const cv::Mat& protoRows,
    int maskH,
    int maskW,
    const YoloLetterboxInfo& info,
    cv::Size originalSize,
    int inputSize
) {
    cv::Mat coeffMat(1, static_cast<int>(coeff.size()), CV_32F, const_cast<float*>(coeff.data()));
    cv::Mat maskFlat = coeffMat * protoRows;
    cv::Mat mask(maskH, maskW, CV_32F, maskFlat.ptr<float>());

    cv::Mat prob(maskH, maskW, CV_32F);
    for (int y = 0; y < maskH; ++y) {
        const float* src = mask.ptr<float>(y);
        float* dst = prob.ptr<float>(y);
        for (int x = 0; x < maskW; ++x) {
            dst[x] = sigmoidFloat(src[x]);
        }
    }

    cv::Mat inputMask;
    cv::resize(prob, inputMask, cv::Size(inputSize, inputSize), 0, 0, cv::INTER_LINEAR);

    cv::Rect roi(info.padX, info.padY, info.newW, info.newH);
    roi &= cv::Rect(0, 0, inputMask.cols, inputMask.rows);
    cv::Mat unpadded = inputMask(roi).clone();

    cv::Mat originalProb;
    cv::resize(unpadded, originalProb, originalSize, 0, 0, cv::INTER_LINEAR);

    cv::Mat binary;
    cv::threshold(originalProb, binary, 0.5, 255.0, cv::THRESH_BINARY);
    binary.convertTo(binary, CV_8U);
    return binary;
}

// 延迟加载 YOLO-seg ONNX 模型。第一次调用时加载，后续复用同一个静态 net。
// 加载失败时返回 nullptr，主流程会自动保持传统检测结果显示。
static cv::dnn::Net* getYoloSegNet() {
    static cv::dnn::Net net;
    static bool triedLoad = false;
    static bool loaded = false;

    if (!YOLO_SEAM_ENABLE) return nullptr;
    if (!triedLoad) {
        triedLoad = true;
        try {
            net = cv::dnn::readNetFromONNX(YOLO_SEAM_MODEL_PATH);
            loaded = !net.empty();
            if (loaded) {
                std::cout << "[YOLO-seg] 模型加载完成: " << YOLO_SEAM_MODEL_PATH << std::endl;
            } else {
                std::cerr << "[YOLO-seg] 模型为空，使用传统橘缝曲线。" << std::endl;
            }
        } catch (const std::exception& e) {
            loaded = false;
            std::cerr << "[YOLO-seg] 模型加载失败: " << e.what() << "，使用传统橘缝曲线。" << std::endl;
        }
    }

    return loaded ? &net : nullptr;
}

// 对整张图执行 YOLO-seg 推理，输出经过置信度阈值和 NMS 后的橘缝分割结果。
// 每个结果包含类别、置信度、检测框和映射回原图的二值 mask。
static std::vector<YoloSegResult> inferYoloSeg(const cv::Mat& image) {
    std::vector<YoloSegResult> results;
    cv::dnn::Net* net = getYoloSegNet();
    if (net == nullptr || image.empty()) return results;

    try {
        YoloLetterboxInfo info;
        int inputSize = std::max(32, YOLO_SEAM_INPUT_SIZE);
        cv::Mat input = yoloLetterbox(image, inputSize, info);
        cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(inputSize, inputSize), cv::Scalar(), true, false);
        net->setInput(blob);

        std::vector<cv::Mat> outputs;
        net->forward(outputs, net->getUnconnectedOutLayersNames());
        if (outputs.size() < 2) return results;

        cv::Mat detRows = yoloDetectionsToRows(outputs[0]);
        int maskDim = 0;
        int maskH = 0;
        int maskW = 0;
        cv::Mat protoRows = yoloProtoToRows(outputs[1], maskDim, maskH, maskW);
        const int numClasses = detRows.cols - 4 - maskDim;
        if (numClasses <= 0) return results;

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> classIds;
        std::vector<std::vector<float>> coeffs;
        float confThreshold = static_cast<float>(YOLO_SEAM_CONF_THRESHOLD);

        for (int i = 0; i < detRows.rows; ++i) {
            const float* row = detRows.ptr<float>(i);
            int bestClass = 0;
            float bestScore = row[4];
            for (int c = 1; c < numClasses; ++c) {
                if (row[4 + c] > bestScore) {
                    bestScore = row[4 + c];
                    bestClass = c;
                }
            }
            if (bestScore < confThreshold) continue;

            const float cx = row[0];
            const float cy = row[1];
            const float bw = row[2];
            const float bh = row[3];
            int left = static_cast<int>(std::round((cx - bw * 0.5f - info.padX) / info.scale));
            int top = static_cast<int>(std::round((cy - bh * 0.5f - info.padY) / info.scale));
            int width = static_cast<int>(std::round(bw / info.scale));
            int height = static_cast<int>(std::round(bh / info.scale));
            cv::Rect box(left, top, width, height);
            box &= cv::Rect(0, 0, image.cols, image.rows);
            if (box.area() <= 0) continue;

            std::vector<float> coeff(maskDim);
            for (int k = 0; k < maskDim; ++k) {
                coeff[k] = row[4 + numClasses + k];
            }

            boxes.push_back(box);
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
            coeffs.push_back(std::move(coeff));
        }

        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, confThreshold, static_cast<float>(YOLO_SEAM_NMS_THRESHOLD), keep);

        for (int idx : keep) {
            YoloSegResult result;
            result.classId = classIds[idx];
            result.score = scores[idx];
            result.box = boxes[idx];
            result.mask = buildYoloMask(coeffs[idx], protoRows, maskH, maskW, info, image.size(), inputSize);

            cv::Mat boxMask = cv::Mat::zeros(result.mask.size(), CV_8U);
            cv::rectangle(boxMask, result.box, 255, cv::FILLED);
            cv::bitwise_and(result.mask, boxMask, result.mask);
            results.push_back(std::move(result));
        }

        std::sort(results.begin(), results.end(), [](const YoloSegResult& a, const YoloSegResult& b) {
            return a.score > b.score;
        });
    } catch (const std::exception& e) {
        std::cerr << "[YOLO-seg] 推理失败: " << e.what() << std::endl;
    }

    return results;
}

// Zhang-Suen 细化算法的一轮迭代，用于把二值 mask 逐步细化成单像素骨架。
static void yoloThinningIteration(cv::Mat& img, int iter) {
    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);
    for (int i = 1; i < img.rows - 1; ++i) {
        for (int j = 1; j < img.cols - 1; ++j) {
            uchar p2 = img.at<uchar>(i - 1, j);
            uchar p3 = img.at<uchar>(i - 1, j + 1);
            uchar p4 = img.at<uchar>(i, j + 1);
            uchar p5 = img.at<uchar>(i + 1, j + 1);
            uchar p6 = img.at<uchar>(i + 1, j);
            uchar p7 = img.at<uchar>(i + 1, j - 1);
            uchar p8 = img.at<uchar>(i, j - 1);
            uchar p9 = img.at<uchar>(i - 1, j - 1);

            int transitions = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                              (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                              (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                              (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int neighbors = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (transitions == 1 && neighbors >= 2 && neighbors <= 6 && m1 == 0 && m2 == 0) {
                marker.at<uchar>(i, j) = 1;
            }
        }
    }
    img &= ~marker;
}

// 对 YOLO 输出的二值橘缝 mask 做骨架化，得到用于显示的中心线。
// 注意：这里的骨架只是可视化辅助，不保证是从两个激光点连通的最优路径。
static cv::Mat skeletonizeYoloMask(const cv::Mat& binaryMask) {
    cv::Mat img;
    cv::threshold(binaryMask, img, 1, 1, cv::THRESH_BINARY);
    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        yoloThinningIteration(img, 0);
        yoloThinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);

    img *= 255;
    return img;
}

// 计算一组激光锚点到某个 YOLO mask 的平均距离。
// 距离越小，说明这个 mask 越可能对应当前双激光检测到的那条橘缝。
static double yoloMaskPointCost(const cv::Mat& mask, const std::vector<cv::Point>& laserPoints) {
    if (laserPoints.empty()) return 0.0;

    cv::Mat inv;
    cv::threshold(mask, inv, 1, 255, cv::THRESH_BINARY_INV);
    cv::Mat dist;
    cv::distanceTransform(inv, dist, cv::DIST_L2, 3);

    double cost = 0.0;
    for (const cv::Point& p : laserPoints) {
        if (p.x < 0 || p.y < 0 || p.x >= mask.cols || p.y >= mask.rows) {
            cost += 1000.0;
        } else {
            cost += dist.at<float>(p.y, p.x);
        }
    }
    return cost / static_cast<double>(laserPoints.size());
}

// 对每一组传统算法输出的 MatchedSeamPair，利用两个激光锚点从 YOLO masks 中选择最匹配的实例。
// 当前选择标准是“锚点到 mask 的距离”减去一部分 YOLO 置信度奖励。
static std::set<int> selectYoloMasksByLaserPoints(
    const std::vector<YoloSegResult>& results,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData,
    const cv::Size& imageSize
) {
    std::set<int> selected;
    if (results.empty()) return selected;

    for (const auto& pair : seamPairs) {
        cv::Point p1;
        cv::Point p2;
        if (!findNearestLaserEndpointLocal(laserData, pair.s1.id, pair.s1.x_peak, imageSize, p1)) continue;
        if (!findNearestLaserEndpointLocal(laserData, pair.s2.id, pair.s2.x_peak, imageSize, p2)) continue;

        std::vector<cv::Point> laserPoints = {p1, p2};
        int bestIdx = -1;
        double bestCost = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(results.size()); ++i) {
            double cost = yoloMaskPointCost(results[i].mask, laserPoints) - 10.0 * results[i].score;
            if (cost < bestCost) {
                bestCost = cost;
                bestIdx = i;
            }
        }
        if (bestIdx >= 0) selected.insert(bestIdx);
    }

    return selected;
}

// 将 YOLO-seg 结果叠加到显示图上：彩色 mask、轮廓、骨架、检测框和置信度。
// 被激光锚点选中的 mask 会用绿色强调。
static cv::Mat drawYoloSegResults(
    cv::Mat displayImage,
    const std::vector<YoloSegResult>& results,
    const std::set<int>& selectedMaskIds
) {
    if (results.empty()) return displayImage;

    const std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 165, 255),
        cv::Scalar(255, 0, 255),
        cv::Scalar(255, 255, 0),
        cv::Scalar(0, 255, 255),
    };

    int drawCount = YOLO_SEAM_DRAW_ALL ? static_cast<int>(results.size()) : std::min(1, static_cast<int>(results.size()));
    cv::Mat overlay = displayImage.clone();

    for (int i = 0; i < drawCount; ++i) {
        bool selected = selectedMaskIds.count(i) > 0;
        cv::Scalar color = selected ? cv::Scalar(0, 255, 0) : colors[i % colors.size()];
        int width = selected ? 3 : 2;

        cv::Mat skeleton = skeletonizeYoloMask(results[i].mask);
        std::vector<std::vector<cv::Point>> lineContours;
        cv::findContours(skeleton, lineContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::drawContours(overlay, lineContours, -1, color, width, cv::LINE_AA);
    }

    return overlay;
}

// 主程序调用入口：在现有传统检测显示图上叠加 YOLO-seg 结果。
// originImage 用于模型推理，displayImage 是传统激光/橘缝检测已经画好的图。
// useLaserSelection 控制是否用双激光橘缝点高亮最匹配的 YOLO mask。
// timing 可选返回推理耗时、绘制耗时、mask 数量和激光约束选中数量。
cv::Mat applyYoloSegToDisplay(
    const cv::Mat& originImage,
    cv::Mat displayImage,
    const std::vector<MatchedSeamPair>& seamPairs,
    const std::vector<LaserData>& laserData,
    bool useLaserSelection,
    YoloSegTiming* timing
) {
    auto inferStart = std::chrono::steady_clock::now();
    std::vector<YoloSegResult> yoloResults = inferYoloSeg(originImage);
    auto inferEnd = std::chrono::steady_clock::now();

    std::set<int> selectedMaskIds;
    if (!yoloResults.empty()) {
        if (useLaserSelection) {
            selectedMaskIds = selectYoloMasksByLaserPoints(yoloResults, seamPairs, laserData, originImage.size());
        }
        displayImage = drawYoloSegResults(displayImage, yoloResults, selectedMaskIds);
        std::cout << "[YOLO-seg] 橘缝数量: " << yoloResults.size();
        if (useLaserSelection && !selectedMaskIds.empty()) {
            std::cout << ", 激光约束匹配数量: " << selectedMaskIds.size();
        } else if (!useLaserSelection) {
            std::cout << ", 直接显示全部mask";
        }
        std::cout << std::endl;
    }
    auto drawEnd = std::chrono::steady_clock::now();

    if (timing != nullptr) {
        timing->infer_ms = std::chrono::duration<double, std::milli>(inferEnd - inferStart).count();
        timing->draw_ms = std::chrono::duration<double, std::milli>(drawEnd - inferEnd).count();
        timing->mask_count = static_cast<int>(yoloResults.size());
        timing->selected_count = static_cast<int>(selectedMaskIds.size());
    }

    return displayImage;
}
