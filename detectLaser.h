#ifndef DETECTLASER_H
#define DETECTLASER_H

#include "ImgProc.h"


cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut);
double calculateScore(const std::vector<std::vector<cv::Point>>& contours);
std::vector<std::vector<cv::Point>> getLaserContours(const cv::Mat& diff);
std::vector<LaserContour> extractCenterlinePoints(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& diff);
cv::Mat saveAndVisualize(const std::vector<std::vector<cv::Point>>& contours, const std::vector<LaserContour>& lcs, cv::Mat& canvas, const cv::Mat& diff, std::vector<LaserData>& outData);
std::vector<LaserData> detectLaserCenter(cv::Mat image, cv::Mat* imageOut);


struct LaserResultContour {
    double topY = -1.0;
    double bottomY = -1.0;
    bool found = false;
};
LaserResultContour detectDoubleLaserByContours(const cv::Mat& processed, int thresholdVal);
void processFrame(cv::Mat frame);



#endif // DETECTLASER_H