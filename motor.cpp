#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>

#include "motor.h"
#include "communicate.h"
#include "ImgProc.h"

// ----------------------电机运动控制----------------------
// 电机初始化碰撞回零
int initmotor(int addr){
    run_zero(addr, 2, false);
    sleep(1);
    while(mMotor[addr - 1].get_rpm() != 0){sleep(1);}
    position_control_t_x(addr, false, 1500, 500, 3000, 1440, 0, false);
    sleep(1);
    while(mMotor[addr - 1].get_rpm() != 0){sleep(1);}
    clear_all(addr);
    return 0;
}

// 丝杆电机运动函数
int screw_motor_move(int addr, int rpm, float distance){
    float angle = distanceToAngleConvert(addr, distance);
    bool dir = (angle >=0) ? true : false;
    position_control_t_x(addr, dir, 1500, 500, rpm, angle, 0, false);
    return 0;
}

// 丝杆电机导程转换(角度->前进距离)
float angleToDistanceConvert(int motor_id, float angle){
    int pitch = mMotor[motor_id - 1].get_screwPitch();
    return static_cast<float>((angle / 360.0f) * 8.0f);
}

// 丝杆电机导程转换(前进距离->角度)
float distanceToAngleConvert(int motor_id, float distance){
    int pitch = mMotor[motor_id - 1].get_screwPitch();
    return static_cast<float>((distance / 8.0f) * 360.0f);
}





// ----------------------电机状态更新----------------------
// 初始化电机对象数组
Motor mMotor[TOTALMOTORNUM] = {
    Motor{0}, Motor{1}, Motor{2}, Motor{3}, Motor{4}, Motor{5}, Motor{6}, Motor{7}
};

void Motor::update_status(){
    read_position(this->motor_id + 1, 1);
    read_rpm(this->motor_id + 1);
    read_ma(this->motor_id + 1);
    usleep(300000);  // 300ms 间隔
}

void Motor::set_position(float pos){ this->current_pos = pos;}
void Motor::set_rpm(int rpm){ this->current_rpm = rpm;}
void Motor::set_ma(int ma){ this->current_ma = ma;}
float Motor::get_position(){ return this->current_pos; }
int Motor::get_rpm(){ return this->current_rpm; }
int Motor::get_ma(){ return this->current_ma; }
int Motor::get_screwPitch(){ return this->screwPitch; }


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





