#ifndef IMGPROC_H
#define IMGPROC_H

#include <opencv2/opencv.hpp>

int takepic();
int userImgProc0(cv::Mat *theMat, long beginTime, long afterTime);
int userImgProc1(cv::Mat *theMat, long beginTime, long afterTime);
int detect_img_edge(cv::Mat src, cv::Mat &out);
int detect_laser_edge(cv::Mat img);
int detect_laser_center(cv::Mat img);


#endif // IMGPROC_H