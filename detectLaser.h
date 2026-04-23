#ifndef DETECTLASER_H
#define DETECTLASER_H

#include "ImgProc.h"


cv::Mat preprocessLaserImage(const cv::Mat& input, cv::Mat& undistortedOut);
// double calculateScore(const std::vector<std::vector<cv::Point>>& contours);
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
LaserResultContour processFrame(cv::Mat frame);



struct LaserCoeffs {
    double a, b, c, e;
    double d_min, d_max;
};
// 四个激光线的标定系数
// y_pixel = a*d^3 + b*d^2 + c*d + e
const LaserCoeffs LASER[4] = {
    // a,           b,            c,             e,            d_min, d_max
    {-6.15999637e-04, 1.71978289e-01, -1.86942539e+01, 8.85322388e+02, 40.0, 90.0}, // 终端1上激光
    {-5.1141296975e-04, 1.5092110510e-01, -1.7759635948e+01, 9.5313166783e+02,  40.0, 90.0}, // 终端1下激光
    {-5.6669858413e-04, 1.3568318759e-01, -1.6139127088e+01, 6.3690639872e+02,  14.0, 64.0}, // 终端2上激光
    {-4.2589554770e-04, 1.0396442855e-01, -1.3386642311e+01, 6.2994168021e+02,  14.0, 64.0} // 终端2下激光
};
double y_pixel_to_distance(double y_pixel, int laser_id);



#endif // DETECTLASER_H