#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <sys/time.h>

#include "thread.h"
#include "motor.h"
#include "ImgProc.h"
#include "main.h"

#define MOTOR1 1
#define MOTOR2 2

int totalCPUnum;
ThreadInfo threadInfoList[TOTAL_THREAD_NUM] = {
    {THR_CAM0, "CAM0thread", 1, 80, THR_HUNGUP}, 
    {THR_IMGPROC0, "IMGPROC0thread", 2, 80, THR_HUNGUP}, 
    {THR_IMGPROC1, "IMGPROC1thread", 3, 80, THR_HUNGUP}, 
};
pthread_mutex_t threadLock[TOTAL_THREAD_NUM];
pthread_cond_t threadCond[TOTAL_THREAD_NUM];

cv::VideoCapture cam;

IMGQUEUE imgQueue;

long int secBase, globalUS;


int main(int argc, char* argv[])
{
    can_init();
    printf("---------- CAN通讯设置成功 ----------\n");

    // u_set_zero(MOTOR1);
    // u_set_zero(MOTOR2);
    

    //打开摄像头
    cam.open(0, cv::CAP_V4L2);
    if (cam.isOpened()) {
        printf("---------- 摄像头打开 ----------\n");
    }else{
        printf("---------- 摄像头未打开 ----------\n");
        return -1;
    }
    cam.set(cv::CAP_PROP_FRAME_WIDTH, 800);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, 600);


    //创建图像队列
    imgQueue.initImgQueue(0);

    //创建锁
    for(int i=0; i<TOTAL_THREAD_NUM; i++) {
        threadLock[i] = PTHREAD_MUTEX_INITIALIZER;
        threadCond[i] = PTHREAD_COND_INITIALIZER;
    }
    totalCPUnum = sysconf(_SC_NPROCESSORS_CONF);  //获取核数
    //创建线程
    pthread_t thread[TOTAL_THREAD_NUM];
    pthread_attr_t attr;
    for(int i=0; i<TOTAL_THREAD_NUM; i++){
        makeThreadAttr(&attr, threadInfoList[i].priority, threadInfoList[i].cpu_affinity);
        int ret;
        if(i == THR_CAM0){
            ret = pthread_create(&thread[i], &attr, camThreadFunc, (void*)&threadInfoList[i]);
        }else if(i == THR_IMGPROC0 || i == THR_IMGPROC1){
            ret = pthread_create(&thread[i], &attr, imgProcThreadFunc, (void*)&threadInfoList[i]);
        }

        printf("---------- 线程创建%s", threadInfoList[i].thread_name);
        if(ret==0)  printf("成功 ----------\n");
        else printf("失败 ----------%d\n", ret);
    }
    //获取启动线程基准的时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    secBase = tv.tv_sec;
    //等待线程运行
    sleep(1);
    //唤醒cam线程
    wakeupThreadWait(THR_CAM0);


    //唤醒所有线程，使其结束。并等待线程结束
    for(int i=0; i<TOTAL_THREAD_NUM; i++)  {
        pthread_join(thread[i],NULL);
        printf("Thread %s(%d) Jion end.\n", threadInfoList[i].thread_name, threadInfoList[i].thread_id);
    }
    

    int rs = pthread_attr_destroy(&attr);  assert(rs == 0);

    return 0;
}


void IMGQUEUE::initImgQueue(int inqID) {
    printf("---------- imgQueue%d初始化", inqID);
	qID = inqID;
	capIdx = 0;
	procIdx = 0;
	procLock = PTHREAD_MUTEX_INITIALIZER;
	for(int i=0; i<IMGQCAPACITY; i++) {
        mats[i].mat = cv::Mat(300, 400, CV_8UC3, cv::Scalar(0,0,0));
        if(mats[i].mat.empty()) {printf("malloc img memory error\n"); exit(0);}
    }
    printf("成功 ----------\n");
}

long int getGlobalUS() {
    static struct timeval tv;     
    gettimeofday(&tv, NULL);
    globalUS = tv.tv_usec + (tv.tv_sec - secBase)*1000000;
    return globalUS;
}

