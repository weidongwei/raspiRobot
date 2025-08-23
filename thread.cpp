#include <pthread.h>
#include <stdio.h>

#include "thread.h"
#include "ImgProc.h"
#include "main.h"
#include "motor.h"
//线程是否终止循环
bool beginExit = false;

//设置线程相关属性
int makeThreadAttr(pthread_attr_t* attr, int priority, int cpuCoreId) {
    if (attr == nullptr) return -1;

    int ret = pthread_attr_init(attr);
    if (ret != 0) {
        perror("pthread_attr_init 失败");
        return ret;
    }

    // ---------- 设置优先级 ----------
    // if (priority > 0) {
    //     struct sched_param schedParam;
    //     memset(&schedParam, 0, sizeof(schedParam));
    //     schedParam.sched_priority = priority;

    //     ret = pthread_attr_setschedpolicy(attr, SCHED_FIFO);  // 实时策略 FIFO
    //     if (ret != 0) {
    //         perror("setschedpolicy failed");
    //         return ret;
    //     }

    //     ret = pthread_attr_setschedparam(attr, &schedParam);
    //     if (ret != 0) {
    //         perror("setschedparam failed");
    //         return ret;
    //     }

    //     ret = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    //     if (ret != 0) {
    //         perror("setinheritsched failed");
    //         return ret;
    //     }
    // }

    // ---------- 设置 CPU 亲和力 ----------
    if (cpuCoreId >= 0) {
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        CPU_SET(cpuCoreId, &cpuSet);

        ret = pthread_attr_setaffinity_np(attr, sizeof(cpuSet), &cpuSet);
        if (ret != 0) {
            perror("setaffinity_np 失败");
            return ret;
        }
    }

    return 0;
}

//尝试唤醒
void wakeupThreadTry(int thread_id){
    if(threadInfoList[thread_id].running_state == THR_RUN) return;
    printf("唤醒线程 %s\n", threadInfoList[thread_id].thread_name);
    if(pthread_mutex_trylock(&threadLock[thread_id])==0) {
		pthread_cond_signal(&threadCond[thread_id]);
		pthread_mutex_unlock(&threadLock[thread_id]);
	}
}

//唤醒
void wakeupThreadWait(int thread_id){
    if(threadInfoList[thread_id].running_state == THR_RUN) {
        printf("唤醒失败！\n");
        return;
    }
    printf("唤醒线程 %s\n", threadInfoList[thread_id].thread_name);
    pthread_mutex_lock(&threadLock[thread_id]);
	pthread_cond_signal(&threadCond[thread_id]);
	pthread_mutex_unlock(&threadLock[thread_id]);
}

//挂起
void hungupThread(int thread_id){
    threadInfoList[thread_id].running_state = THR_HUNGUP;
    printf("挂起线程 %s\n", threadInfoList[thread_id].thread_name);
}

//是否被挂起
bool threadIsHungup(int thread_id) {
    if(threadInfoList[thread_id].running_state == THR_HUNGUP)  return true;
    return false;
}

//线程终止循环
void stopThreadAndExit() {
    beginExit = true;
}

// ---------- 线程函数 ----------
//相机线程函数
void *camThreadFunc(void* arg){
    //----------------------------------------------
    ThreadInfo *thisThreadInfo = (ThreadInfo *)arg; 
    int id = thisThreadInfo->thread_id;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            pthread_cond_wait(&threadCond[id], &threadLock[id]);
			pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfoList[id].running_state = THR_RUN; 
    //----------------------------------------------
        //获取摄像头图像并保存到imgQueue队列
        int matIdx = imgQueue.capIdx;
        imgQueue.mats[matIdx].beginCapUS = getGlobalUS();
        cam >> imgQueue.mats[matIdx].mat;
        imgQueue.mats[matIdx].capUS = getGlobalUS();


        for(int i=THR_IMGPROC0; i<=THR_IMGPROC1; i++) {
            if(beginExit) break;
          	if(pthread_mutex_trylock(&threadLock[i])==0) {//&threadLock[id]
                if(threadIsHungup(i)) {
                    //传送信息
                    threadInfoList[i].theMat = &(imgQueue.mats[matIdx].mat);
                    threadInfoList[i].beginTime = imgQueue.mats[matIdx].beginCapUS;
                    threadInfoList[i].afterTime = imgQueue.mats[matIdx].capUS;

                    pthread_mutex_unlock(&threadLock[i]);
                    wakeupThreadWait(i);
                }
                else pthread_mutex_unlock(&threadLock[i]);
            }
        }
        imgQueue.capIdx++;
        if(imgQueue.capIdx>=IMGQCAPACITY) imgQueue.capIdx = 0;

        //hungupThread(id);
    }
    
    printf("终止线程cam\n");
    //终止线程
    pthread_exit(0);
}

//图像处理线程函数
void *imgProcThreadFunc(void* arg) {

    //----------------------------------------------
    ThreadInfo *thisThreadInfo = (ThreadInfo *)arg; 
    int id = thisThreadInfo->thread_id;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            pthread_cond_wait(&threadCond[id], &threadLock[id]);
			pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfoList[id].running_state = THR_RUN;
    //----------------------------------------------

        
        if(id == THR_IMGPROC0){
            userImgProc0((cv::Mat *)thisThreadInfo->theMat, thisThreadInfo->beginTime, thisThreadInfo->afterTime);
        }else if(id == THR_IMGPROC1){
            userImgProc1((cv::Mat *)thisThreadInfo->theMat, thisThreadInfo->beginTime, thisThreadInfo->afterTime);
        }
        wakeupThreadWait(THR_MOTION);
        
        
        hungupThread(id);
    }

    //终止线程
    pthread_exit(0);
}

//电机运动线程函数
void *motionThreadFunc(void* arg) {
    //----------------------------------------------
    ThreadInfo *thisThreadInfo = (ThreadInfo *)arg; 
    int id = thisThreadInfo->thread_id;
    while(!beginExit)  { 
        if(threadIsHungup(id)){
            pthread_mutex_lock(&threadLock[id]);
            pthread_cond_wait(&threadCond[id], &threadLock[id]);
			pthread_mutex_unlock(&threadLock[id]);
        }
        threadInfoList[id].running_state = THR_RUN;
    //----------------------------------------------
        printf("~~~~电机运动~~~~\n");
        //engineMotion();
        hungupThread(id);
    }


    //终止线程
    pthread_exit(0);

}

