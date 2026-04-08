#include <opencv2/opencv.hpp>
#include <fstream>

#include "detectLaser.h"
#include "calibrateCamera.h"


// 预处理函数(去畸变、绿色通道增强及平滑处理)
cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut) {
    // 去畸变 (保存 undistortedOut 用于后续绘图)
    cv::undistort(input, undistortedOut, vConfig.MycameraMatrix, vConfig.MydistCoeffs);

    // 通道增强（绿色提取）
    std::vector<cv::Mat> bgr;
    cv::split(undistortedOut, bgr);
    cv::Mat diff = bgr[1] - 0.5 * (bgr[0] + bgr[2]);
    
    cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
    diff.convertTo(diff, CV_8U);
    cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0);
    return diff;
}

// 计算当前轮廓组合的得分
double calculateScore(const std::vector<std::vector<cv::Point>>& contours) {
    if (contours.empty()) return 0.0;
    struct LaserMetric {
        double length;
        double area;
    };

    std::vector<LaserMetric> metrics;
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < 10.0) continue; 
        double length = cv::arcLength(cnt, false) * 0.5;
        metrics.push_back({length, area});
    }

    if (metrics.empty()) return 0.0;
    
    // 按长度排序，取前两个
    std::sort(metrics.begin(), metrics.end(), [](const LaserMetric& a, const LaserMetric& b){
        return a.length > b.length;
    });

    double total_len = 0;
    double total_area = 0;
    int count = std::min((int)metrics.size(), 2); // 确保不会超过实际数量
    for (int i = 0; i < count; ++i) {
        total_len += metrics[i].length;
        total_area += metrics[i].area;
    }

    // 长度评分
    double s_len = 0.0;
    double ideal_l = vConfig.best_laser_length;
    if (total_len <= ideal_l) {
        s_len = total_len / ideal_l;
    } else {
        s_len = std::max(0.0, 1.0 - (total_len - ideal_l) / (ideal_l/2));
    }

    // 细度评分
    double s_thin = 0.0;
    double ideal_w = vConfig.best_laser_width;
    double ideal_a = ideal_l * ideal_w;

    if (total_area <= ideal_a) {
        s_thin = total_area / ideal_a;
    } else {
        s_thin = std::max(0.0, 1.0 - (total_area - ideal_a) / (ideal_a + 1e-5));
    }
    

    // 综合加权
    double final_score = (s_len * vConfig.ratio_laser_length) + (s_thin * vConfig.ratio_laser_width);

    return final_score;
}

// 轮廓检测与过滤
std::vector<std::vector<cv::Point>> getLaserContours(const cv::Mat& diff) {
    double minVal, maxVal;
    cv::minMaxLoc(diff, &minVal, &maxVal);
    
    int start_thresh = static_cast<int>(maxVal * 0.65); 
    int end_thresh = vConfig.threshold_value_min;
    int step = vConfig.threshold_value_rate;

    double max_total_score = -1.0;
    int best_thresh = start_thresh;
    std::vector<std::vector<cv::Point>> best_contours;

    for (int thresh = start_thresh; thresh >= end_thresh; thresh -= step) {
        cv::Mat mask;
        cv::threshold(diff, mask, thresh, 255, cv::THRESH_BINARY);
        
        std::vector<std::vector<cv::Point>> current_contours;
        cv::findContours(mask, current_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        //
        // std::string filename  = getTimeString() + "_diff" + ".jpg";
        // std::string save_path = vConfig.diff_path + filename;
        // cv::imwrite(save_path, diff);
        //

        // 调用新评分系统
        double score = calculateScore(current_contours);

        
        // 记录最优解
        if (score > max_total_score) {
            max_total_score = score;
            best_thresh = thresh;
            best_contours = current_contours;
        }
    }

    std::cout << "最佳阈值: " << best_thresh << " 综合得分: " << max_total_score << std::endl;

    //最后的输出过滤：依然按面积保留前两个
    std::sort(best_contours.begin(), best_contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });
    
    if (best_contours.size() > 5) best_contours.resize(5);
    return best_contours;

}



// 中心线几何提取
std::vector<LaserContour> extractCenterlinePoints(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& diff) {
    std::vector<LaserContour> laser_contours;

    // 权重设置
    const double weight_top = 1;
    const double weight_center = 1.0 - weight_top;

    for (int i = 0; i < contours.size(); ++i) {
        cv::Mat mask = cv::Mat::zeros(diff.size(), CV_8U);
        
        // std::string filename  = "test.jpg";
        // std::string save_path = vConfig.proc_path + filename;
        // cv::imwrite(save_path, diff);
        
        cv::drawContours(mask, contours, i, cv::Scalar(255), cv::FILLED);
        
        cv::Rect box = cv::boundingRect(contours[i]);
        LaserContour lc;
        double y_sum = 0;

        for (int x = box.x; x < box.x + box.width; ++x) {
            int first_y = -1, last_y = -1;
            for (int y = box.y; y < box.y + box.height; ++y) {
                if (mask.at<uchar>(y, x) > 0 && diff.at<uchar>(y, x) > 0) {
                    if (first_y == -1) first_y = y;
                    last_y = y;
                }
            }
            if (first_y != -1) {
                int center_y = (first_y + last_y) / 2;
                // int center_y = first_y;
                
                int top_y = first_y;
                int fused_y = std::round(center_y * weight_center + top_y * weight_top);

                lc.xs.push_back(x);
                lc.ys.push_back(fused_y);
                y_sum += fused_y;
            }
        }
        if (!lc.ys.empty()) {
            lc.y_average = y_sum / lc.ys.size();
            laser_contours.push_back(lc);
        }
    }
    
    // // 判定上下激光类型
    // if (laser_contours.size() == 2) {
    //     bool firstIsUpper = laser_contours[0].y_average < laser_contours[1].y_average;
    //     laser_contours[0].laser_type = firstIsUpper ? 1 : 2;
    //     laser_contours[1].laser_type = firstIsUpper ? 2 : 1;
    // } else if (laser_contours.size() == 1) {
    //     laser_contours[0].laser_type = 1; // 默认
    // }

    // 3. 多轮廓身份判定逻辑
    int num_found = laser_contours.size();
    std::cout<< "num_found = " << num_found <<std::endl;
    if (num_found >= 2) {
        // 由于输入 contours 是按面积降序排列的，
        // 因此 laser_contours[0] 和 [1] 就是最大的两个“领头羊”。
        
        // 首先确定这两个领头羊谁是上（1号），谁是下（2号）
        int idxA = 0;
        int idxB = 1;
        if (laser_contours[idxA].y_average > laser_contours[idxB].y_average) {
            std::swap(idxA, idxB);
        }
        
        // 记录基准高度
        double refY1 = laser_contours[idxA].y_average; // 较小的Y（靠上）
        double refY2 = laser_contours[idxB].y_average; // 较大的Y（靠下）

        // 为所有轮廓（包括领头羊和碎碎的小轮廓）分配编号
        for (int i = 0; i < num_found; ++i) {
            double currentY = laser_contours[i].y_average;
            double distTo1 = std::abs(currentY - refY1);
            double distTo2 = std::abs(currentY - refY2);

            if (distTo1 < distTo2) {
                laser_contours[i].laser_type = 1; // 离1号近，归为1号
            } else {
                laser_contours[i].laser_type = 2; // 离2号近，归为2号
            }
        }
    } 
    else if (num_found == 1) {
        // 只有一个轮廓时，无法对比，默认标为 1
        laser_contours[0].laser_type = 1;
    }
    
    return laser_contours;
}

// 保存结果与可视化
cv::Mat saveAndVisualize(const std::vector<std::vector<cv::Point>>& contours, std::vector<LaserContour>& lcs, cv::Mat& canvas, const cv::Mat& diff, std::vector<LaserData>& outData) {
    // 红色轮廓绘制
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > canvas.cols * canvas.rows * 0.0002) {
            cv::drawContours(canvas, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 1);
        }
    }

    // std::string timeStr = getTimeString();
    // std::ofstream ofs(vConfig.csv_path + timeStr + "_points.csv");
    // ofs << "laser_id,x_pixel,y_pixel,distance_cm\n";

    cv::Mat mask_center = cv::Mat::zeros(diff.size(), CV_8U);

    // --- 多级排序 ---
    std::sort(lcs.begin(), lcs.end(), [](const LaserContour& a, const LaserContour& b) {
        // 1. 先按 laser_type 排序
        if (a.laser_type != b.laser_type) {
            return a.laser_type < b.laser_type;
        }
        
        // 2. 编号相同时，按 X 坐标起始位置排序
        // 增加判空保护，并确保比较的是每一段的“最左端”
        if (!a.xs.empty() && !b.xs.empty()) {
            return a.xs[0] < b.xs[0]; 
        }
        
        // 3. 如果万一有一段是空的（理论上不应该），排在后面
        return a.xs.empty() < b.xs.empty();
    });

    // 提取数据并准备画中心点
    for (const auto& lc : lcs) {
        for (size_t i = 0; i < lc.xs.size(); ++i) {
            int x = lc.xs[i], y = lc.ys[i];
            double dis = (lc.laser_type == 1) ? y_pixel_to_distance1(y) : y_pixel_to_distance2(y);
            if (dis <= 0) continue;

            outData.push_back({lc.laser_type, x, y, dis});
            // ofs << lc.laser_type << "," << x << "," << y << "," << dis << "\n";
            mask_center.at<uchar>(y, x) = 255;
        }
    }
    
    // 闭运算修复断线并画出蓝色中心点
    cv::morphologyEx(mask_center, mask_center, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1)));
    for (int y = 0; y < mask_center.rows; ++y) {
        for (int x = 0; x < mask_center.cols; ++x) {
            if (mask_center.at<uchar>(y, x) > 0)
                cv::circle(canvas, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
        }
    }

    // cv::imwrite(vConfig.proc_path + timeStr + "_detected.jpg", canvas);
    // ofs.close();
    return canvas;
}

// 检测激光中心线主函数
std::vector<LaserData> detectLaserCenter(cv::Mat image, cv::Mat* imageOut) {
    cv::Mat undistortedImg;
    std::vector<LaserData> laserPoints;

    // 1. 预处理
    cv::Mat diff = preprocessLaserImage(image, undistortedImg);

    // 2. 找轮廓
    auto contours = getLaserContours(diff);
    if (contours.empty()) return {};

    // 3. 提取中心线坐标
    auto lcs = extractCenterlinePoints(contours, diff);

    // 4. 保存与可视化
    cv::Mat canvas = saveAndVisualize(contours, lcs, undistortedImg, diff, laserPoints);
    
    *imageOut = canvas;

    std::cout << "激光中心线检测完成。" << std::endl;
    return laserPoints;
}

