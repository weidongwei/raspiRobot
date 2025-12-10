#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <linux/can.h>
#include <iostream>
#include <unistd.h>
#include <cstdint>

#include "motor.h"

extern int can_socket_fd;   // 全局 CAN 套接字
extern std::string can_ifname; // CAN 接口名，比如 "can0"


bool loadJson(const std::string& filename);
std::string getJson(int id);
int can_init();
canid_t get_base_id(int addr);

// can通讯相关函数
int init_socket();
int can_send(const struct can_frame& frame);
void send_packet(canid_t base_id, uint8_t dlc, const uint8_t* data);
int can_receive(struct can_frame* r_frame);
void handle_can_receive();
// 控制动作命令
int enable_motor(int addr, bool enable);
int speed_control(int addr, bool direction, int acc, int rpm, bool multiMachine);
int position_control_emm(int addr, int rpm, int acceleration, bool dir, float pulses, bool absolute, bool multiMachine);
int position_control_x(int addr, bool dir, int rpm, float angle, int mode, bool multiMachine);
int position_control_t_x(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine);
int stop_motor(int addr);
int clear_all(int addr);
int set_zero(int addr, bool save);
int run_zero(int addr,int mode, bool multiMachine);
int set_division(int addr,bool save, int division);
int set_motor_parameter(int addr, int dangerRpm, int dangerMa, int dangerTime);
int set_zero_parameter(int addr, int acceleration, int timeout , int dangerRpm, int dangerMa, int dangerTime);
int close_stall(int addr);
int sync_run();
// 读取状态命令
int read_rpm(int addr);
int read_position(int addr, float gear_ratio);
int read_ma(int addr);

#endif // COMMUNICATE_H