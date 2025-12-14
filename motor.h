#ifndef motor_H
#define motor_H

#include <iostream>

#include "communicate.h"

#define TOTALMOTORNUM 8   //总的电机数量

int initmotor(int addr);
float angleToDistanceConvert(int motor_id, float angle);
float distanceToAngleConvert(int motor_id, float distance);
int screw_motor_move(int addr, float distance);

class Motor{
private:
    int motor_id;               // 电机ID，从0开始
    int max_rpm;                // 最大速度
    float up_limit_pos;         // 上限位置
    float low_limit_pos;        // 下限位置
    int screwPitch;             // 丝杆导程

    float current_pos;          // 当前角度位置
    int current_rpm;            // 当前转速
    int current_ma;             // 当前电流
public:
    Motor(int id): motor_id(id), current_pos(0), current_rpm(0), current_ma(0), up_limit_pos(0), low_limit_pos(0), max_rpm(0), screwPitch(0) {}
    void update_status();
    void set_motor_id(int id);
    void set_max_rpm(int rpm);
    void set_up_limit_pos(float pos);
    void set_low_limit_pos(float pos);
    void set_screwPitch(int pitch);
    void set_position(float pos);
    void set_rpm(int rpm);
    void set_ma(int ma);

    int get_motor_id();
    int get_max_rpm();
    float get_up_limit_pos();
    float get_low_limit_pos();
    float get_position();
    int get_rpm();
    int get_ma();
    int get_screwPitch();
};
extern Motor mMotor[TOTALMOTORNUM];


// 电磁阀控制
int run_pump(bool isOpen);



#endif // motor_H