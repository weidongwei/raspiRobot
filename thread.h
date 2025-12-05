#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>
#include <stdio.h>
#include "communicate.h"


#define TOTALTHREADNUM 2   //总的线程数量

// ---------- 线程方面的定义 ----------
enum THREADSTATE {THR_HUNGUP, THR_RUN};
enum THRID {THR_CAN_SEND, THR_CAN_RECEIVE};

struct THREADINFO {
    THRID ID;
    char name[128];
    volatile THREADSTATE runningState;
    CANDevice* device;

};
extern THREADINFO threadInfo[TOTALTHREADNUM];


extern pthread_mutex_t threadLock[TOTALTHREADNUM];
extern pthread_cond_t threadCond[TOTALTHREADNUM];
extern bool beginExit;


void hungupTheThread(int theID);
bool threadIsHungup(int theID);
void wakeupThreadWait(int ID);


// ---------- 线程函数 ----------
void *canSendThreadFunc(void* arg);
void *canReceiveThreadFunc(void* arg);


#endif // THREAD_H