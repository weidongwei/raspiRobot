#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>

#include "motor.h"
#include "communicate.h"
#include "ImgProc.h"


// 初始化电机对象数组
Motor mMotor[TOTALMOTORNUM] = {
    Motor{0}, Motor{1}, Motor{2}, Motor{3}, Motor{4}, Motor{5}, Motor{6}, Motor{7}
};


void Motor::update_status(){
    read_position(this->motor_id + 1, 1);
    read_rpm(this->motor_id + 1);
    read_ma(this->motor_id + 1);
    sleep(1);
}
void Motor::set_position(float pos){ this->current_pos = pos;}
void Motor::set_rpm(int rpm){ this->current_rpm = rpm;}
void Motor::set_ma(int ma){ this->current_ma = ma;}
float Motor::get_position(){ return this->current_pos; }
int Motor::get_rpm(){ return this->current_rpm; }
int Motor::get_ma(){ return this->current_ma; }




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




