#ifndef CALIBRATECAMERA_H
#define CALIBRATECAMERA_H

#include "ImgProc.h"

bool calibrateCameraFromImages( const std::vector<cv::Mat>& images, cv::Size boardSize, float squareSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, double& reprojError, const std::string& debugSaveDir );
int biaoding();
double y_pixel_to_distance1(double y_pixel);
double y_pixel_to_distance2(double y_pixel);


#endif // CALIBRATECAMERA_H