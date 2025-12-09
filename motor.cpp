#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>

#include "motor.h"
#include "communicate.h"
#include "ImgProc.h"
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4
#define MOTOR5 5
#define MOTOR6 6
#define MOTOR7 7
#define MOTOR8 8
#define MStep 16            //细分数
#define GEAR_RATIO 1        //减速机减速比1
#define MAXMA1 200          //电机1碰撞最大电流
#define MAXMA2 200          //电机2碰撞最大电流
#define MAXANGLE 208        //两电机的最大角度范围为208度
#define ERRORNUM 0.1        //0.1误差



// ------------------测试电机控制函数------------------

int user_57motor(){
    position_control_emm(MOTOR1, 5, 0, true, 20, true, true);
    position_control_emm(MOTOR1, 5, 0, true, 90, true, false);
    return 0;
}

// ------------------机械抓手控制函数------------------
// 检测电机是否完成运动
int u_detect_motor(int addr, int maxI1, int maxI2){
    int times = 200, currentMa = 10;
    times = read_rpm(addr);
    while(times != 0){
        usleep(100000);
        times = read_rpm(addr);
        currentMa = read_ma(addr);
        if(currentMa > (addr == MOTOR1 ? maxI1 : maxI2)){
             stop_motor(addr);
            printf("%d号电机  碰撞\n", addr);
            return -1;
        }
    }
    printf("%d号电机  运动结束\n", addr);
    return 0;
}

// 检测多电机同步是否完成运动
int u_detect_multi_motor(int maxI1,int maxI2){
    int times1 = 200, times2 = 200, currentMa1 = 10, currentMa2 = 10;
    times1 = read_rpm(MOTOR1);
    times2 = read_rpm(MOTOR2);
    while(times1 != 0 || times2 != 0){
        usleep(100000);
        times1 = read_rpm(MOTOR1);
        times2 = read_rpm(MOTOR2);
        currentMa1 = read_ma(MOTOR1);
        currentMa2 = read_ma(MOTOR2);
        if(currentMa1 > maxI1 || currentMa2 > maxI2){
            stop_motor(MOTOR1);
            stop_motor(MOTOR2);
            printf("双电机  碰撞\n");
            return -1;
        }
    }
    printf("双电机  运动结束\n");
    return 0;
}

// 电机走到零点并设置零点
int u_set_zero(int addr){
    run_zero(addr, 2, false);
    sleep(1);
    //检测电机是否运动结束
    int times = 200;
    times = read_rpm(addr);
    while(times != 0){
        usleep(300000);
        times = read_rpm(addr);
    }
    printf("%d号电机  碰撞回零\n", addr);
    close_stall(addr);
    position_control_x(addr, true, 350, 10, 0, false);
    sleep(2);
    clear_all(addr);
    set_zero(addr, true);
    return 0;
}

// 电机走到某点
int u_go(int addr, float angle, int maxI1, int maxI2){
    float mainPos = 0, secondaryPos = 0, go_angle = 0;
    mainPos = read_position(addr, 1);
    secondaryPos = read_position(addr == MOTOR1 ? MOTOR2 : MOTOR1, 1);
    if(angle < 0 || angle > MAXANGLE + ERRORNUM || angle + secondaryPos > MAXANGLE + ERRORNUM){
        printf("%d号电机  无法到达指定位置\n", addr);
        return -2;
    }
    go_angle = angle - mainPos;
    position_control_x(addr, true, 350, go_angle, 0, false);
    usleep(100000);
    if(u_detect_motor(addr, maxI1, maxI2) == -1){
        return -1;
    }
    printf("%d号电机  到达指定位置\n", addr);
    return 0;
}

// 电机抓取物品
int u_pickup(){
    float secondaryPos = 0, go_angle = 0;
    secondaryPos = read_position(MOTOR2, 1);
    go_angle = MAXANGLE - secondaryPos;
    printf("go_angle = %0.4f\n", go_angle);
    if(u_go(MOTOR1, go_angle, 250, 250) == -1){
        //夹住后松开2度
        sleep(1);
        float pos = read_position(MOTOR1, 1);
        u_go(MOTOR1, pos - 2, 250, 250);
    }
    return 0;
}

// 抓取物品同步运动(angle是相对于带摄像头夹子的角度)
int u_sync_go(float angle){
    float pos1 = 0, pos2 = 0, goodSize = 0, goAngle1 = 0, goAngle2 = 0, MaxAngle = 0;
    pos1 = read_position(MOTOR1, 1);
    pos2 = read_position(MOTOR2, 1);
    goodSize = MAXANGLE - pos1 - pos2;
    MaxAngle = MAXANGLE - goodSize;
    if(angle < 0 || angle > MaxAngle){
        printf("无法同步运行到指定位置\n");
        return -1;
    }
    goAngle1 = MAXANGLE - angle - goodSize - pos1;
    goAngle2 = angle - pos2;
    printf("goodSize = %0.2f, goAngle1 = %0.2f, goAngle2 = %0.2f\n", goodSize, goAngle1, goAngle2);
    position_control_x(MOTOR1, true, 350, goAngle1, 0, true);
    position_control_x(MOTOR2, true, 350, goAngle2, 0, true);
    sync_run();
    usleep(100000);
    if(u_detect_multi_motor(400, 400) == -1){
        //对夹住物品的力度变化进行修正
        pos1 = read_position(MOTOR1, 1);
        pos2 = read_position(MOTOR2, 1);
        if(goodSize != MAXANGLE - pos1 - pos2){
            float motor1goAngle = (MAXANGLE - pos1 - pos2) - goodSize;
            u_go(MOTOR1, pos1 + motor1goAngle, 400, 400);
        }
        //------------------------------
            sleep(1);
            pos1 = read_position(MOTOR1, 1);
            pos2 = read_position(MOTOR2, 1);
            printf("                      xx2 = %0.2f\n", MAXANGLE - pos1 - pos2);
        //------------------------------
        return -1;
    }
    //到达指定位置后对夹住物品的力度变化进行修正
    pos1 = read_position(MOTOR1, 1);
    pos2 = read_position(MOTOR2, 1);
    if(goodSize != MAXANGLE - pos1 - pos2){
        float motor1goAngle = (MAXANGLE - pos1 - pos2) - goodSize;
        u_go(MOTOR1, pos1 + motor1goAngle, 400, 400);
    }
    //------------------------------
        sleep(1);
        pos1 = read_position(MOTOR1, 1);
        pos2 = read_position(MOTOR2, 1);
        printf("                      xx2 = %0.2f\n", MAXANGLE - pos1 - pos2);
    //------------------------------
    printf("同步运行到指定位置\n");
    return 0;
}



// 激光控制
int run_laser(bool isOpen){ 
    int pin = 13;              // BCM13
    const int period_us = 2000; // 2ms = 500Hz
    const float duty_cycle = 0.25;

    wiringPiSetupGpio(); // BCM 模式
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    if (isOpen) {
        std::cout << "开始 500Hz 手动 PWM 控制激光...\n";

        int high_time = period_us * duty_cycle;
        int low_time  = period_us - high_time;

        while (true) {
            digitalWrite(pin, HIGH);
            std::this_thread::sleep_for(std::chrono::microseconds(high_time));

            digitalWrite(pin, LOW);
            std::this_thread::sleep_for(std::chrono::microseconds(low_time));
        }
    } else {
        std::cout << "激光关闭\n";
        digitalWrite(pin, LOW);
    }

    return 0;
}

// 电磁阀控制
int run_pump(bool isOpen){
     const int pin = 26;

    // 初始化 wiringPi（使用 BCM GPIO 编号）
    if (wiringPiSetupGpio() < 0) {
        std::cerr << "无法初始化 wiringPi" << std::endl;
        return -1;
    }

    // 设置为输出模式
    pinMode(pin, OUTPUT);

    // 根据参数控制电平
    int value = isOpen ? HIGH : LOW;
    digitalWrite(pin, value);

    std::cout << "[INFO] GPIO " << pin << " 已设置为 "
              << (isOpen ? "高电平（打开电磁泵）" : "低电平（关闭电磁泵）")
              << std::endl;

    return 0;
}




// 采集图像
int takepic(){
    std::string save_path = "/home/dw/robot/image/1.jpg";
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    // cv::VideoCapture cap;
    // cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << 0 << std::endl;
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // 有的驱动 1=手动，3=自动，需测试
    cap.set(cv::CAP_PROP_EXPOSURE, 300);     // 曝光时间整数us
    cap.set(cv::CAP_PROP_SHARPNESS, 100);  // 设置锐度为 100(0 ~ 100)
    cap.set(cv::CAP_PROP_BRIGHTNESS, 50);  // 设置亮度为 50(-64 ~ 64)

    cv::Mat frame;
    cv::waitKey(1000);


    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法获取图像帧。" << std::endl;
            break;
        }

        cv::imwrite(save_path, frame); 
        std::cout << "图像已保存到 " << save_path << std::endl;
        sleep(1);
        // detect_laser_edge(cv::imread("/home/dw/robot/image/1.jpg"));
        detect_laser_center(cv::imread("/home/dw/robot/image/1.jpg"));
    }
    cap.release();
    return 0;
}