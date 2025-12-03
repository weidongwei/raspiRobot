#ifndef MAIN_H
#define MAIN_H

#include <opencv2/opencv.hpp>


#define IMGQCAPACITY 10

struct IMGMAT {
	cv::Mat mat; 
	long int beginCapUS, capUS;
};

class IMGQUEUE {
public:
	int qID;
	IMGMAT mats[IMGQCAPACITY];
	int capIdx;
	int procIdx;
	pthread_mutex_t procLock;
	
	void initImgQueue(int qID);
};

extern cv::VideoCapture cam;
extern IMGQUEUE imgQueue;
extern long int globalUS;
extern int camNum;
extern int imgWidth, imgHeight;
long int getGlobalUS();

#endif // MAIN_H