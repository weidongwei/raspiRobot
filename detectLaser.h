#ifndef DETECTLASER_H
#define DETECTLASER_H

#include "ImgProc.h"


cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut);
double calculateScore(const std::vector<std::vector<cv::Point>>& contours);
std::vector<std::vector<cv::Point>> getLaserContours(const cv::Mat& diff);
std::vector<LaserContour> extractCenterlinePoints(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& diff);
cv::Mat saveAndVisualize(const std::vector<std::vector<cv::Point>>& contours, const std::vector<LaserContour>& lcs, cv::Mat& canvas, const cv::Mat& diff, std::vector<LaserData>& outData);
std::vector<LaserData> detectLaserCenter(cv::Mat image, cv::Mat* imageOut);


#endif // DETECTLASER_H