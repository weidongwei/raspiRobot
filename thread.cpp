#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>

#include "thread.h"
#include "ImgProc.h"
#include "communicate.h"
#include "motor.h"

bool beginExit = false;
bool laserStatus = false;
bool canReceiveStatus = false;

// 挂起线程
void hungupTheThread(int theID) {
    pthread_mutex_lock(&threadLock[theID]);
    (threadInfo+theID)->runningState = THR_HUNGUP;
     pthread_mutex_unlock(&threadLock[theID]);
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
    threadInfo[ID].runningState = THR_RUN;   // 先改状态
	pthread_cond_signal(threadCond+ID);
    pthread_mutex_unlock(threadLock+ID);
}

// 修改激光状态访问
void setLaserStatus(bool status) {
    pthread_mutex_lock(&laserMutex);
    laserStatus = status;
    pthread_mutex_unlock(&laserMutex);
}

bool getLaserStatus() {
    pthread_mutex_lock(&laserMutex);
    bool status = laserStatus;
    pthread_mutex_unlock(&laserMutex);
    return status;
}

// 修改can接收状态访问
void setcanReceiveStatus(bool status) {
    pthread_mutex_lock(&canReceiveMutex);
    canReceiveStatus = status;
    pthread_mutex_unlock(&canReceiveMutex);
}

bool getcanReceiveStatus() {
    pthread_mutex_lock(&canReceiveMutex);
    bool status = canReceiveStatus;
    pthread_mutex_unlock(&canReceiveMutex);
    return status;
}




// can接收线程
void *canReceiveThreadFunc(void* arg) {
    THREADINFO *thisThreadInfo = (THREADINFO *)arg; 
    int id = thisThreadInfo->ID;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            while (threadInfo[id].runningState == THR_HUNGUP && !beginExit) {
                pthread_cond_wait(&threadCond[id], &threadLock[id]);
            }
            pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfo[id].runningState = THR_RUN;
        while(getcanReceiveStatus() == true && !beginExit) {
            handle_can_receive();
        }
        hungupTheThread(id);
    }
    //终止线程
    pthread_exit(0);
}

// 电机状态更新线程
void *motorStatusThreadFunc(void* arg) {
    THREADINFO *thisThreadInfo = (THREADINFO *)arg; 
    int id = thisThreadInfo->ID;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            while (threadInfo[id].runningState == THR_HUNGUP && !beginExit) {
                pthread_cond_wait(&threadCond[id], &threadLock[id]);
            }
            pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfo[id].runningState = THR_RUN;
        while(!beginExit) {
            mMotor[0].update_status();
            mMotor[1].update_status();
            mMotor[2].update_status();
            mMotor[3].update_status();
            mMotor[4].update_status();
            mMotor[5].update_status();
            mMotor[6].update_status();
            mMotor[7].update_status();
        }
        hungupTheThread(id);
    }
    //终止线程
    pthread_exit(0);
}

// 激光控制线程
void *laserControlThreadFunc(void* arg){
    THREADINFO *thisThreadInfo = (THREADINFO *)arg; 
    int id = thisThreadInfo->ID;

    const int pin = 12;
    const int period_us = 2000;     // 500Hz
    const double duty = vConfig.laser_duty;  // 激光占空比

    wiringPiSetupGpio();
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    int high_time = period_us * duty;
    int low_time  = period_us - high_time;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            while (threadInfo[id].runningState == THR_HUNGUP && !beginExit) {
                pthread_cond_wait(&threadCond[id], &threadLock[id]);
            }
			pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfo[id].runningState = THR_RUN;

        while (getLaserStatus() == true && !beginExit) {
            digitalWrite(pin, HIGH);
            std::this_thread::sleep_for(std::chrono::microseconds(high_time));
            digitalWrite(pin, LOW);
            std::this_thread::sleep_for(std::chrono::microseconds(low_time));
        }
        while (getLaserStatus() == false && !beginExit) {
            digitalWrite(pin, LOW);
        }
        hungupTheThread(id);
    }
    //终止线程
    pthread_exit(0);
}

// 摄像头拍照线程
void *photoControlThreadFunc(void* arg){
    THREADINFO *thisThreadInfo = (THREADINFO *)arg; 
    int id = thisThreadInfo->ID;

    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            while (threadInfo[id].runningState == THR_HUNGUP && !beginExit) {
                pthread_cond_wait(&threadCond[id], &threadLock[id]);
            }
            pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfo[id].runningState = THR_RUN;
        while ( !beginExit ) {
            if(vConfig.photo_thread_mode == 1)
                takeVedio();
            else if(vConfig.photo_thread_mode == 2)
                saveVedio();
            else if(vConfig.photo_thread_mode == 3)
                takePic();
        }
        hungupTheThread(id);
    }
    //终止线程
    pthread_exit(0);
}