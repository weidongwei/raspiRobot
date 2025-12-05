#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#include "thread.h"
#include "ImgProc.h"
#include "communicate.h"
#include "motor.h"

bool beginExit = false;

// 挂起线程
void hungupTheThread(int theID) {
    (threadInfo+theID)->runningState = THR_HUNGUP;
}

// 检查线程是否挂起
bool threadIsHungup(int theID) {
    if((threadInfo+theID)->runningState==THR_HUNGUP)  return true;
    return false;
}


// 唤醒线程等待
void wakeupThreadWait(int ID) {
	if((threadInfo+ID)->runningState==THR_RUN) return;
	pthread_mutex_lock(threadLock+ID);
	pthread_cond_signal(threadCond+ID);
    pthread_mutex_unlock(threadLock+ID);
}

// can发送线程函数
void *canSendThreadFunc(void* arg) {
    THREADINFO *thisThreadInfo = (THREADINFO *)arg; 
    int id = thisThreadInfo->ID;
    CANDevice* canDevice = thisThreadInfo->device;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            pthread_cond_wait(&threadCond[id], &threadLock[id]);
			pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfo[id].runningState = THR_RUN;
    //----------------------------------------------
        printf("~~~~发送线程开启~~~~\n");
        canDevice->enable_motor(8,true);
        printf("信号发送完毕~~~~\n");
        sleep(10);
        hungupTheThread(id);
    }


    //终止线程
    pthread_exit(0);

}

// can接收线程函数
void *canReceiveThreadFunc(void* arg) {
    //----------------------------------------------
    THREADINFO *thisThreadInfo = (THREADINFO *)arg; 
    int id = thisThreadInfo->ID;
    CANDevice* canDevice = thisThreadInfo->device;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            pthread_cond_wait(&threadCond[id], &threadLock[id]);
			pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfo[id].runningState = THR_RUN;
    //----------------------------------------------
        while(!beginExit){
            printf("~~~~接收线程开启~~~~\n");
            canDevice->handle_can_receive();
        }
        hungupTheThread(id);
    }


    //终止线程
    pthread_exit(0);

}