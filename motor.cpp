#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>

#include "motor.h"
#include "communicate.h"
#include "ImgProc.h"

// ----------------------电机运动控制----------------------

// 丝杆电机导程转换(角度->前进距离)
float angleToDistanceConvert(int motor_id, float angle){
    int screwPitch = mMotor[motor_id - 1].get_screwPitch();
    return static_cast<float>((angle / 360.0f) * screwPitch);
}

// 丝杆电机导程转换(前进距离->角度)
float distanceToAngleConvert(int motor_id, float distance){
    int screwPitch = mMotor[motor_id - 1].get_screwPitch();
    printf("screwPitch = %d\n", screwPitch);
    return static_cast<float>((distance / screwPitch) * 360.0f);
}

// 丝杆电机初始化碰撞回零
int initmotor(int addr){
    run_zero(addr, 2, false);
    sleep(1);
    while(mMotor[addr - 1].get_rpm() != 0){sleep(1);}
    position_control_t_x(addr, true, 1500, 500, 3000, distanceToAngleConvert(addr, 80), 0, false);
    sleep(1);
    while(mMotor[addr - 1].get_rpm() != 0){sleep(1);}
    clear_all(addr);
    return 0;
}

// 丝杆电机运动函数
int screw_motor_move(int addr, float distance){
    float cur_pos = mMotor[addr - 1].get_position();
    float up_limit = mMotor[addr - 1].get_up_limit_pos();
    float low_limit = mMotor[addr - 1].get_low_limit_pos();
    int rpm = mMotor[addr - 1].get_max_rpm();
    float  target_pos = cur_pos + distance;
    printf("当前位置 = %.2f, 目标位置 = %.2f, up_limit = %.2f, low_limit = %.2f\n", cur_pos, target_pos, up_limit, low_limit);
    if(target_pos > up_limit || target_pos < low_limit){
        printf("%d 号电机 目标位置超出限位范围！\n", addr);
        return -1;
    }
    float angle = distanceToAngleConvert(addr, distance);
    bool dir = (angle >=0) ? true : false;
    position_control_t_x(addr, dir, 1500, 500, rpm, abs(angle), 0, false);
    return 0;
}

// 57电机运动函数
int emm_motor_move(int addr, float angle){
    float cur_pos = mMotor[addr - 1].get_position();
    float up_limit = mMotor[addr - 1].get_up_limit_pos();
    float low_limit = mMotor[addr - 1].get_low_limit_pos();
    int rpm = mMotor[addr - 1].get_max_rpm();
    float  target_pos = cur_pos + angle;
    printf("当前位置 = %.2f, 目标位置 = %.2f, up_limit = %.2f, low_limit = %.2f\n", cur_pos, target_pos, up_limit, low_limit);
    if(target_pos > up_limit || target_pos < low_limit){
        printf("%d 号电机 目标位置超出限位范围！\n", addr);
        return -1;
    }
    bool dir = (angle >=0) ? true : false;
    position_control_emm(addr, rpm, 0, dir, abs(angle), true, false);
    return 0;
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

void Motor::set_motor_id(int id){ this->motor_id = id; }
void Motor::set_max_rpm(int rpm){ this->max_rpm = rpm; }
void Motor::set_up_limit_pos(float pos){ this->up_limit_pos = pos; }
void Motor::set_low_limit_pos(float pos){ this->low_limit_pos = pos; }
void Motor::set_screwPitch(int pitch){ this->screwPitch = pitch; }
void Motor::set_position(float pos){ this->current_pos = pos;}
void Motor::set_rpm(int rpm){ this->current_rpm = rpm;}
void Motor::set_ma(int ma){ this->current_ma = ma;}

int Motor::get_motor_id(){ return this->motor_id; }
int Motor::get_max_rpm(){ return this->max_rpm; }
float Motor::get_up_limit_pos(){ return this->up_limit_pos; }
float Motor::get_low_limit_pos(){ return this->low_limit_pos; }
int Motor::get_screwPitch(){ return this->screwPitch; }
float Motor::get_position(){ return this->current_pos; }
int Motor::get_rpm(){ return this->current_rpm; }
int Motor::get_ma(){ return this->current_ma; }


// 电磁阀控制
int run_pump(float time){
    const int pin = 13;
    int time_int = static_cast<int>(time * 1000000); // 转换为微秒

    // 初始化 wiringPi（使用 BCM GPIO 编号）
    if (wiringPiSetupGpio() < 0) {
        std::cerr << "无法初始化 wiringPi" << std::endl;
        return -1;
    }

    // 设置为输出模式
    pinMode(pin, OUTPUT); 

    digitalWrite(pin, HIGH);
    usleep(time_int);
    digitalWrite(pin, LOW);

 
    std::cout << "[INFO] GPIO " << pin << " 已打开电磁阀 " << time_int << " 微秒"
              << std::endl;

    return 0;
}





