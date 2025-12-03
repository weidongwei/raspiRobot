#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <linux/can.h>
#include <iostream>

canid_t get_base_id(int addr);
int can_init();
int can_send(can_frame frame);
int can_receive(struct can_frame * r_frame,unsigned int filter_id);
int recive_dayin(uint8_t id, int addr, std::string controlName);
int enable_motor(int addr, bool enable);
int speed_control(int addr, bool direction, int rpm, int acceleration, bool multiMachine);
int position_control(int addr, int rpm, int acceleration, bool dir, int pulses, bool absolute, bool multiMachine);
int position_control_x(int addr, bool dir, int rpm, float angle, int mode, bool multiMachine, int maxma);
int position_control_t_x(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine, int maxma);
int position_control_t_x2(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine);
int stop_motor(int addr);
int clear_all(int addr);
int set_zero(int addr, bool save);
int run_zero(int addr,int mode, bool multiMachine);
int set_division(int addr,bool save, int division);
int set_motor_parameter(int addr, int dangerRpm, int dangerMa, int dangerTime);
int set_zero_parameter(int addr, int acceleration, int timeout , int dangerRpm, int dangerMa, int dangerTime);
int close_stall(int addr);
int sync_run();


int read_rpm(int addr);
float read_position(int addr, float gear_ratio);
float read_position_x(int addr, float gear_ratio);
int read_ma(int addr);

#endif // COMMUNICATE_H