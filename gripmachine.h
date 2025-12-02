#ifndef GRIPMACHINE_H
#define GRIPMACHINE_H


#include <linux/can.h>
#include <iostream>

canid_t get_base_id(int addr);
int can_init();
int can_send(can_frame frame);
int can_receive(struct can_frame * r_frame,unsigned int filter_id);
int recive_dayin(uint8_t id, int addr, std::string controlName);
int enable_motor(int addr, bool enable);
int speed_control(int addr, bool direction, int rpm, int acceleration, bool multiMachine);
int position_control(int addr, int rpm, int acceleration, float angle, bool absolute, bool multiMachine);
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
float read_position(int addr);
int read_ma(int addr);


int u_detect_motor(int addr, int maxMa1, int maxMa2);
int u_detect_multi_motor(int maxI1,int maxI2);
int u_set_zero(int addr);
int u_go(int addr, float angle, int maxI1, int maxI2);
int u_pickup();
int u_sync_go(float angle);
// int run_laser(bool isOpen);
int run_pump(bool isOpen);
int takepic();

#endif // GRIPMACHINE_H