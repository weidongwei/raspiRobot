#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <thread>

#include "motor.h"
#include "communicate.h"
#include "ImgProc.h"


#define MOTOR1 1
#define MOTOR2 2
#define MAXMA1 200          //电机1碰撞最大电流
#define MAXMA2 200          //电机2碰撞最大电流

int main(int argc, char* argv[])
{
    //can_init();
    CANDevice canDevice;
    canDevice.init_socket();
    canDevice.start_receive_thread();
    

    char control_str[15];
    for(int i=1; i<argc; i++){
        strcpy(control_str,argv[i]);
        if (strcmp(control_str, "enable")==0)           {canDevice.enable_motor(MOTOR1,true);canDevice.enable_motor(MOTOR2,true);}
        else if(strcmp(control_str, "disable")==0)      {canDevice.enable_motor(MOTOR1,false);canDevice.enable_motor(MOTOR2,false);}
        else if(strcmp(control_str, "stop")==0)         {canDevice.stop_motor(MOTOR1);}
        else if(strcmp(control_str, "read_rpm")==0)      {canDevice.read_rpm(MOTOR1);canDevice.read_rpm(MOTOR2);}
        else if(strcmp(control_str, "read_pos")==0)      {canDevice.read_position(MOTOR1, 1);canDevice.read_position(MOTOR2, 1);}
        else if(strcmp(control_str, "read_ma")==0)      {canDevice.read_ma(MOTOR1);canDevice.read_ma(MOTOR2);}
        else if(strcmp(control_str, "set_motor")==0)      {
            canDevice.set_motor_parameter(MOTOR1, 380, 600, 60);
            canDevice.set_motor_parameter(MOTOR2, 380, 600, 60);
        }
        else if(strcmp(control_str, "set_zero")==0)      {
            canDevice.set_zero_parameter(MOTOR1, 350, 10000, 380, 150, 60);
            canDevice.set_zero_parameter(MOTOR2, 350, 10000, 380, 100, 60);
        }

        // //终端抓手
        // //开机确定零点
        // else if(strcmp(control_str, "set_zero1")==0)          u_set_zero(MOTOR1);
        // else if(strcmp(control_str, "set_zero2")==0)          u_set_zero(MOTOR2);

        // else if(strcmp(control_str, "go_zero1")==0)           u_go(MOTOR1, 0, MAXMA1, MAXMA2);
        // else if(strcmp(control_str, "go_zero2")==0)           u_go(MOTOR2, 0, MAXMA1, MAXMA2);
        // else if(strcmp(control_str, "go1")==0)                u_go(MOTOR1, 90, MAXMA1, MAXMA2);
        // else if(strcmp(control_str, "go2")==0)                u_go(MOTOR2, 90, MAXMA1, MAXMA2);
        // else if(strcmp(control_str, "pickup")==0)             u_pickup();
        // else if(strcmp(control_str, "sync_go1")==0)           u_sync_go(100);
        // else if(strcmp(control_str, "sync_go2")==0)           u_sync_go(0);



        // else if(strcmp(control_str, "laser1")==0)             {
        //     std::thread laser_thread(run_laser, true);  // 激光常亮
        //     std::thread camera_thread(takepic);         // 每5秒拍一张

        //     laser_thread.join();
        //     camera_thread.join();
        // }
        else if(strcmp(control_str, "laser1")==0)             run_laser(true);
        else if(strcmp(control_str, "laser2")==0)             run_laser(false);



        /////电机测试
        else if(strcmp(control_str, "e0")==0)             {canDevice.enable_motor(1,false);canDevice.enable_motor(2,false);}
        else if(strcmp(control_str, "e1")==0)             {canDevice.enable_motor(1,true);canDevice.enable_motor(2,true);}
        else if(strcmp(control_str, "1cw")==0)             canDevice.position_control_x(1, true, 100, 2000, 0, false);
        else if(strcmp(control_str, "1ccw")==0)            canDevice.position_control_x(1, true, 3000, 5000, 0, false);
        else if(strcmp(control_str, "2cw")==0)             canDevice.position_control_x(2, 50, 0, 180, true, false);
        else if(strcmp(control_str, "2ccw")==0)            canDevice.position_control_x(2, 50, 0, -180, true, false);
        else if(strcmp(control_str, "s")==0)             {canDevice.stop_motor(1);canDevice.stop_motor(2);}
        else if(strcmp(control_str, "x")==0)             {canDevice.position_control_x(1, true, 1500, 720, 0, false);}
        else if(strcmp(control_str, "tx")==0)             {canDevice.position_control_t_x(1, true, 100, 100, 2000, 2000, 0, false);}
        else if(strcmp(control_str, "read")==0)      {canDevice.read_position(MOTOR1, 1);}
        
        else if(strcmp(control_str, "emm")==0)             canDevice.position_control_emm(1, 100, 0, true, 2000, true, false);
        else if(strcmp(control_str, "emm1")==0)            canDevice.position_control_emm(1, 500, 0, true, 2000, true, false);

        
        else if(strcmp(control_str, "te")==0)             {
            // canDevice.position_control_x(4, true, 100, 2000, 0, false);
            canDevice.position_control_x(8, true, 100, 2000, 0, false);
            // canDevice.position_control_x(7, true, 100, 2000, 0, false);
        }


        // //摄像头测试
        // else if(strcmp(control_str, "takepic")==0)      takepic();

        // //图像处理测试
        // else if(strcmp(control_str, "imgproc")==0)      detect_laser_edge(cv::imread("/home/dw/robot/image/image1.jpg"));

        //气泵测试
        // else if(strcmp(control_str, "pump")==0){
        //     run_pump(true);   // 打开电磁阀
        //     sleep(3);         // 保持1秒
        //     run_pump(false);  // 关闭电磁阀
        // }

    }

    return 0; 

}
