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
pthread_mutex_t laserMutex;
pthread_mutex_t canReceiveMutex;

THREADINFO threadInfo[TOTALTHREADNUM] = { 
    {THR_CAN_RECEIVE, "CANRECEIVE", THR_HUNGUP}, 
    {THR_LASER_CONTROL, "LASERCONTROL", THR_HUNGUP},
    {THR_PHOTO_CONTROL, "PHOTOCONTROL", THR_HUNGUP},
    {THR_MOTOR_STATUS, "MOTORSTATUS", THR_HUNGUP}
};

int main(int argc, char *argv[]){
    // 初始化线程信息
    init_socket();

    pthread_t thread[TOTALTHREADNUM];  
    beginExit = false;
    laserStatus = false;
    canReceiveStatus = false;
    for(int i=0; i<TOTALTHREADNUM; i++) {
        threadLock[i] = PTHREAD_MUTEX_INITIALIZER;
        threadCond[i] = PTHREAD_COND_INITIALIZER;
    }
    // 添加全局互斥锁
    laserMutex = PTHREAD_MUTEX_INITIALIZER;
    canReceiveMutex = PTHREAD_MUTEX_INITIALIZER;

    // 创建线程
    for(int i = 0; i<TOTALTHREADNUM; i++){
        int ret;
        if(i == THR_CAN_RECEIVE){
            ret = pthread_create(&thread[i], NULL, canReceiveThreadFunc, (void*)(threadInfo+i));
        }else if(i == THR_LASER_CONTROL){
            ret = pthread_create(&thread[i], NULL, laserControlThreadFunc, (void*)(threadInfo+i));
        }else if(i == THR_PHOTO_CONTROL){
            ret = pthread_create(&thread[i], NULL, photoControlThreadFunc, (void*)(threadInfo+i));
        }else if(i == THR_MOTOR_STATUS){
            ret = pthread_create(&thread[i], NULL, motorStatusThreadFunc, (void*)(threadInfo+i));
        }
        printf("创建线程%s", (threadInfo+i)->name);
        if(ret==0){
            printf("成功\n");
        }else{
            printf("失败%d\n", ret);
        }   
    }
    canReceiveStatus = true;
    sleep(1);
    wakeupThreadWait(THR_CAN_RECEIVE);
    wakeupThreadWait(THR_MOTOR_STATUS);
    sleep(1);
    if(argc>1) {
        if(strcmp(argv[1], "emm")==0){
            int motor_id = atoi(argv[2]);
            float angle = atof(argv[3]);
            bool dir = (angle >=0) ? true : false;
            position_control_emm(motor_id, 5, 0, dir, abs(angle), true, false);
        }

        else if(strcmp(argv[1], "x")==0){
            int motor_id = atoi(argv[2]);
            float angle = atof(argv[3]);
            bool dir = (angle >=0) ? true : false;
            position_control_t_x(motor_id, dir, 1500, 500, 3000, abs(angle), 0, false);
        }

        else if(strcmp(argv[1], "stop")==0)              { int motor_id = atoi(argv[2]); stop_motor(motor_id); }
        else if(strcmp(argv[1], "disable")==0)           { int motor_id = atoi(argv[2]); enable_motor(motor_id,false); }
        else if(strcmp(argv[1], "enable")==0)            { int motor_id = atoi(argv[2]); enable_motor(motor_id,true); }
        else if(strcmp(argv[1], "read_pos")==0)          { int motor_id = atoi(argv[2]); printf("%d号电机 pos=%.2f\n",motor_id, mMotor[motor_id - 1].get_position());}
        else if(strcmp(argv[1], "read_rpm")==0)          { int motor_id = atoi(argv[2]); printf("%d号电机 rpm=%d\n",motor_id, mMotor[motor_id - 1].get_rpm());}
        else if(strcmp(argv[1], "read_ma")==0)           { int motor_id = atoi(argv[2]); printf("%d号电机 ma=%d\n",motor_id, mMotor[motor_id - 1].get_ma());}

        else if(strcmp(argv[1], "test1")==0){ 
            setLaserStatus(true); 
            wakeupThreadWait(THR_LASER_CONTROL);
            wakeupThreadWait(THR_PHOTO_CONTROL);
            for(int i=0; i<5; i++){
                sleep(3);
                position_control_emm(1, atoi(argv[2]), atoi(argv[4]), true, atoi(argv[3]), true, false);
                // position_control_t_x(2, false, 1500, 500, 3000, 360, 0, false);
                sleep(3);
                position_control_emm(1, atoi(argv[2]), atoi(argv[4]), false, atoi(argv[3]), true, false);
                // position_control_t_x(2, true, 1500, 500, 3000, 360, 0, false);
            }
        }

        else if(strcmp(argv[1], "test2")==0){
            setLaserStatus(true); 
            wakeupThreadWait(THR_LASER_CONTROL);
            wakeupThreadWait(THR_PHOTO_CONTROL);
            for(int i=0; i<5; i++){
                sleep(3);
                position_control_t_x(2, true, 1500, 500, 3000, atoi(argv[2]), 0, false);
                sleep(3);
                position_control_t_x(2, false, 1500, 500, 3000, atoi(argv[2]), 0, false);
            }
        }

        else if(strcmp(argv[1], "zero")==0){
            initmotor(atoi(argv[2]));
        }

        else if(strcmp(argv[1], "screw_move")==0){
            int motor_id = atoi(argv[2]);
            float distance = atof(argv[3]);
            screw_motor_move(motor_id, distance);
        }

        sleep(600);
        return 0;
    }


    // 主线程等待退出信号
    for(int i=0; i<TOTALTHREADNUM; i++)  {
        pthread_join(thread[i],NULL);
        printf("Thread %s(%d) Jion End.\n", threadInfo[i].name, threadInfo[i].ID);
    }
    return 0;
}