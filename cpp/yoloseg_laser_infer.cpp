#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

struct LetterboxInfo {
    float scale = 1.0f;
    int padX = 0;
    int padY = 0;
    int newW = 0;
    int newH = 0;
};

struct SegResult {
    int classId = 0;
    float score = 0.0f;
    cv::Rect box;
    cv::Mat mask;
    std::vector<float> coeff;
};

static float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

static cv::Mat letterbox(const cv::Mat& image, int inputSize, LetterboxInfo& info) {
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

static cv::Mat detectionsToRows(const cv::Mat& blob) {
    if (blob.dims == 2) {
        return blob;
    }
    if (blob.dims != 3 || blob.size[0] != 1) {
        throw std::runtime_error("Unsupported detection output shape.");
    }

    const int dim1 = blob.size[1];
    const int dim2 = blob.size[2];
    cv::Mat mat(dim1, dim2, CV_32F, const_cast<float*>(blob.ptr<float>()));

    if (dim1 < dim2) {
        cv::Mat transposed;
        cv::transpose(mat, transposed);
        return transposed.clone();
    }
    return mat.clone();
}

static cv::Mat protoToRows(const cv::Mat& proto, int& maskDim, int& maskH, int& maskW) {
    if (proto.dims != 4 || proto.size[0] != 1) {
        throw std::runtime_error("Unsupported prototype output shape.");
    }
    maskDim = proto.size[1];
    maskH = proto.size[2];
    maskW = proto.size[3];
    return cv::Mat(maskDim, maskH * maskW, CV_32F, const_cast<float*>(proto.ptr<float>())).clone();
}

static cv::Mat buildMask(
    const std::vector<float>& coeff,
    const cv::Mat& protoRows,
    int maskH,
    int maskW,
    const LetterboxInfo& info,
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
            dst[x] = sigmoid(src[x]);
        }
    }

    cv::Mat fullInputMask;
    cv::resize(prob, fullInputMask, cv::Size(inputSize, inputSize), 0, 0, cv::INTER_LINEAR);

    cv::Rect roi(info.padX, info.padY, info.newW, info.newH);
    roi &= cv::Rect(0, 0, fullInputMask.cols, fullInputMask.rows);
    cv::Mat unpadded = fullInputMask(roi).clone();

    cv::Mat originalProb;
    cv::resize(unpadded, originalProb, originalSize, 0, 0, cv::INTER_LINEAR);

    cv::Mat binary;
    cv::threshold(originalProb, binary, 0.5, 255.0, cv::THRESH_BINARY);
    binary.convertTo(binary, CV_8U);
    return binary;
}

static std::vector<SegResult> inferYoloSeg(
    cv::dnn::Net& net,
    const cv::Mat& image,
    int inputSize,
    float confThreshold,
    float nmsThreshold
) {
    LetterboxInfo info;
    cv::Mat input = letterbox(image, inputSize, info);
    cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(inputSize, inputSize), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.size() < 2) {
        throw std::runtime_error("YOLO segmentation ONNX should return detections and mask prototypes.");
    }

    cv::Mat detRows = detectionsToRows(outputs[0]);
    int maskDim = 0;
    int maskH = 0;
    int maskW = 0;
    cv::Mat protoRows = protoToRows(outputs[1], maskDim, maskH, maskW);
    const int numClasses = detRows.cols - 4 - maskDim;
    if (numClasses <= 0) {
        throw std::runtime_error("Could not infer class count from ONNX output.");
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classIds;
    std::vector<std::vector<float>> coeffs;

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
        if (bestScore < confThreshold) {
            continue;
        }

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
        if (box.area() <= 0) {
            continue;
        }

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
    cv::dnn::NMSBoxes(boxes, scores, confThreshold, nmsThreshold, keep);

    std::vector<SegResult> results;
    for (int idx : keep) {
        SegResult result;
        result.classId = classIds[idx];
        result.score = scores[idx];
        result.box = boxes[idx];
        result.coeff = coeffs[idx];
        result.mask = buildMask(result.coeff, protoRows, maskH, maskW, info, image.size(), inputSize);

        cv::Mat boxMask = cv::Mat::zeros(result.mask.size(), CV_8U);
        cv::rectangle(boxMask, result.box, 255, cv::FILLED);
        cv::bitwise_and(result.mask, boxMask, result.mask);
        results.push_back(std::move(result));
    }

    std::sort(results.begin(), results.end(), [](const SegResult& a, const SegResult& b) {
        return a.score > b.score;
    });
    return results;
}

static double maskPointCost(const cv::Mat& mask, const std::vector<cv::Point>& laserPoints) {
    if (laserPoints.empty()) {
        return 0.0;
    }
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

static int chooseByLaserConstraint(const std::vector<SegResult>& results, const std::vector<cv::Point>& laserPoints) {
    if (results.empty()) {
        return -1;
    }
    if (laserPoints.empty()) {
        return 0;
    }

    int bestIdx = 0;
    double bestCost = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(results.size()); ++i) {
        double cost = maskPointCost(results[i].mask, laserPoints);
        cost -= 10.0 * static_cast<double>(results[i].score);
        if (cost < bestCost) {
            bestCost = cost;
            bestIdx = i;
        }
    }
    return bestIdx;
}

static void thinningIteration(cv::Mat& img, int iter) {
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

static cv::Mat skeletonize(const cv::Mat& binaryMask) {
    cv::Mat img;
    cv::threshold(binaryMask, img, 1, 1, cv::THRESH_BINARY);
    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(img, 0);
        thinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);

    img *= 255;
    return img;
}

static std::vector<cv::Point> centerlinePointsByColumn(const cv::Mat& skeleton) {
    std::vector<cv::Point> points;
    for (int x = 0; x < skeleton.cols; ++x) {
        int sumY = 0;
        int count = 0;
        for (int y = 0; y < skeleton.rows; ++y) {
            if (skeleton.at<uchar>(y, x) > 0) {
                sumY += y;
                ++count;
            }
        }
        if (count > 0) {
            points.emplace_back(x, static_cast<int>(std::round(sumY / static_cast<double>(count))));
        }
    }
    return points;
}

static void saveCenterlineCsv(
    const std::string& path,
    const std::vector<std::vector<cv::Point>>& allPoints,
    const std::vector<SegResult>& results,
    int selected
) {
    std::ofstream out(path);
    out << "seam_id,x,y,score,selected\n";
    for (int i = 0; i < static_cast<int>(allPoints.size()); ++i) {
        for (const auto& p : allPoints[i]) {
            out << i << "," << p.x << "," << p.y << "," << results[i].score << "," << (i == selected ? 1 : 0) << "\n";
        }
    }
}

static cv::Mat drawAllResults(
    const cv::Mat& image,
    const std::vector<SegResult>& results,
    const std::vector<cv::Mat>& skeletons,
    const std::vector<cv::Point>& laserPoints,
    int selected
) {
    cv::Mat maskLayer = image.clone();
    const std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 165, 255),
        cv::Scalar(255, 0, 255),
        cv::Scalar(255, 255, 0),
        cv::Scalar(0, 255, 255),
    };

    // Draw masks first. Boxes and labels are drawn later so they stay crisp.
    for (int i = 0; i < static_cast<int>(results.size()); ++i) {
        const cv::Scalar color = colors[i % colors.size()];
        cv::Mat colorImage(image.size(), image.type(), color);
        colorImage.copyTo(maskLayer, results[i].mask);
    }

    cv::Mat overlay;
    cv::addWeighted(maskLayer, 0.35, image, 0.65, 0.0, overlay);

    for (int i = 0; i < static_cast<int>(results.size()); ++i) {
        const cv::Scalar color = colors[i % colors.size()];
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(results[i].mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        const int contourWidth = (i == selected) ? 3 : 2;
        cv::drawContours(overlay, contours, -1, color, contourWidth);
    }

    for (int i = 0; i < static_cast<int>(results.size()); ++i) {
        const cv::Mat& skeleton = skeletons[i];
        for (int y = 0; y < skeleton.rows; ++y) {
            for (int x = 0; x < skeleton.cols; ++x) {
                if (skeleton.at<uchar>(y, x) > 0) {
                    overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                }
            }
        }
    }

    for (int i = 0; i < static_cast<int>(results.size()); ++i) {
        const cv::Scalar color = colors[i % colors.size()];
        cv::Scalar boxColor = (i == selected) ? cv::Scalar(0, 255, 0) : color;
        cv::rectangle(overlay, results[i].box, boxColor, (i == selected) ? 3 : 2);
        cv::putText(
            overlay,
            "seam" + std::to_string(i) + " " + std::to_string(results[i].score).substr(0, 4),
            results[i].box.tl() + cv::Point(0, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            boxColor,
            2,
            cv::LINE_AA
        );
    }

    for (const auto& p : laserPoints) {
        cv::circle(overlay, p, 6, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
    return overlay;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " model.onnx image.jpg output.png [x1 y1 x2 y2] [input_size] [conf] [nms]\n\n"
                  << "Example:\n"
                  << "  " << argv[0] << " best.onnx laser_on.jpg result.png 210 180 255 330\n";
        return 1;
    }

    const std::string modelPath = argv[1];
    const std::string imagePath = argv[2];
    const std::string outPath = argv[3];

    std::vector<cv::Point> laserPoints;
    int argi = 4;
    if (argc >= 8) {
        laserPoints.emplace_back(std::stoi(argv[4]), std::stoi(argv[5]));
        laserPoints.emplace_back(std::stoi(argv[6]), std::stoi(argv[7]));
        argi = 8;
    }

    int inputSize = 640;
    float conf = 0.25f;
    float nms = 0.45f;
    if (argc > argi) inputSize = std::stoi(argv[argi]);
    if (argc > argi + 1) conf = std::stof(argv[argi + 1]);
    if (argc > argi + 2) nms = std::stof(argv[argi + 2]);

    cv::Mat image = cv::imread(imagePath);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << imagePath << "\n";
        return 2;
    }

    cv::dnn::Net net = cv::dnn::readNetFromONNX(modelPath);
    std::vector<SegResult> results = inferYoloSeg(net, image, inputSize, conf, nms);
    if (results.empty()) {
        std::cerr << "No seam mask found.\n";
        return 3;
    }

    const int selected = laserPoints.empty() ? -1 : chooseByLaserConstraint(results, laserPoints);
    std::vector<cv::Mat> skeletons;
    std::vector<std::vector<cv::Point>> centerlines;
    for (const SegResult& result : results) {
        cv::Mat skeleton = skeletonize(result.mask);
        skeletons.push_back(skeleton);
        centerlines.push_back(centerlinePointsByColumn(skeleton));
    }

    cv::Mat overlay = drawAllResults(image, results, skeletons, laserPoints, selected);
    if (!cv::imwrite(outPath, overlay)) {
        std::cerr << "Failed to write output image: " << outPath << "\n";
        return 4;
    }

    const std::string csvPath = outPath + ".centerline.csv";
    saveCenterlineCsv(csvPath, centerlines, results, selected);
    std::cout << "Saved: " << outPath << "\n";
    std::cout << "Saved centerline: " << csvPath << "\n";
    std::cout << "Masks: " << results.size();
    if (selected >= 0) {
        std::cout << ", selected: " << selected << ", selected score: " << results[selected].score
                  << ", laser cost: " << maskPointCost(results[selected].mask, laserPoints);
    }
    std::cout << "\n";
    return 0;
}
