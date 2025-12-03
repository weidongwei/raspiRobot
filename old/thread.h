#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>
#include <stdio.h>

// ---------- 线程方面的定义 ----------
#define CAM_NUM 1
#define IMGPROC_NUM 2
#define TOTAL_THREAD_NUM CAM_NUM+IMGPROC_NUM+1

enum ThreadState {THR_HUNGUP, THR_RUN};
enum ThreadId {THR_CAM0, THR_IMGPROC0, THR_IMGPROC1, THR_MOTION};

struct ThreadInfo {
    ThreadId thread_id;
    char thread_name[128];
    int cpu_affinity;
    int priority;
    volatile ThreadState running_state;

    //图像信息
    volatile void *theMat;
    volatile long beginTime, afterTime;
};
extern ThreadInfo threadInfoList[TOTAL_THREAD_NUM];
extern pthread_mutex_t threadLock[TOTAL_THREAD_NUM];
extern pthread_cond_t threadCond[TOTAL_THREAD_NUM];
extern bool beginExit;



// ---------- 线程设置相关函数 ----------
int makeThreadAttr(pthread_attr_t* attr, int priority, int cpuCoreId);
void wakeupThreadTry(int thread_id);
void wakeupThreadWait(int thread_id);
void hungupThread(int thread_id);
bool threadIsHungup(int thread_id);

// ---------- 线程函数 ----------
void *camThreadFunc(void* arg);
void *imgProcThreadFunc(void* arg);
void *motionThreadFunc(void* arg);




#endif // THREAD_H