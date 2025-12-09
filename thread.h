#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>
#include <stdio.h>
#include "communicate.h"


#define TOTALTHREADNUM 2   //总的线程数量

// ---------- 线程方面的定义 ----------
enum THREADSTATE {THR_HUNGUP, THR_RUN};
enum THRID {THR_CAN_RECEIVE, THR_LASER_CONTROL};

struct THREADINFO {
    THRID ID;
    char name[128];
    volatile THREADSTATE runningState;

};
extern THREADINFO threadInfo[TOTALTHREADNUM];


extern pthread_mutex_t threadLock[TOTALTHREADNUM];
extern pthread_cond_t threadCond[TOTALTHREADNUM];
extern pthread_mutex_t laserMutex;
extern pthread_mutex_t canReceiveMutex;
extern bool beginExit;
extern bool laserStatus;
extern bool canReceiveStatus;


void hungupTheThread(int theID);
bool threadIsHungup(int theID);
void wakeupThreadWait(int ID);

void setLaserStatus(bool status);
bool getLaserStatus();
void setcanReceiveStatus(bool status);
bool getcanReceiveStatus();


// ---------- 线程函数 ----------
void *canReceiveThreadFunc(void* arg);
void *laserControlThreadFunc(void* arg);

#endif // THREAD_H