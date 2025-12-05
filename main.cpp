#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>


#include "motor.h"
#include "communicate.h"
#include "ImgProc.h"
#include "thread.h"


pthread_mutex_t threadLock[TOTALTHREADNUM];
pthread_cond_t threadCond[TOTALTHREADNUM];

THREADINFO threadInfo[TOTALTHREADNUM] = { 
    {THR_CAN_SEND, "CANSEND", THR_HUNGUP}, 
    {THR_CAN_RECEIVE, "CANRECEIVE", THR_HUNGUP} 
};

int main(int argc, char *argv[]){
    // 初始化线程信息
    CANDevice canDevice;
    canDevice.init_socket();
    pthread_t thread[TOTALTHREADNUM];  
    beginExit = false;
    for(int i=0; i<TOTALTHREADNUM; i++) {
        threadLock[i] = PTHREAD_MUTEX_INITIALIZER;
        threadCond[i] = PTHREAD_COND_INITIALIZER;
        threadInfo[i].device = &canDevice;
    }

    // 创建线程
    for(int i = 0; i<TOTALTHREADNUM; i++){
        int ret;
        if(i == THR_CAN_SEND){
            ret = pthread_create(&thread[i], NULL, canSendThreadFunc, (void*)(threadInfo+i));
        }else if(i == THR_CAN_RECEIVE){
            ret = pthread_create(&thread[i], NULL, canReceiveThreadFunc, (void*)(threadInfo+i));
        }
        printf("创建线程%s", (threadInfo+i)->name);
        if(ret==0){
            printf("        done\n");
        }else{
            printf("        error code %d\n", ret);
        }   
    }

    wakeupThreadWait(THR_CAN_RECEIVE);

    // 主线程等待退出信号
    for(int i=0; i<TOTALTHREADNUM; i++)  {
        pthread_join(thread[i],NULL);
        printf("Thread %s(%d) Jion End.\n", threadInfo[i].name, threadInfo[i].ID);
    }
    return 0;
}