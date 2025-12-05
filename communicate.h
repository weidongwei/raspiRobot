#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <linux/can.h>
#include <iostream>
#include <atomic>
#include <thread>
#include <unistd.h>

int can_init();
bool loadJson(const std::string& filename);
std::string getDescription(int id);

class CANDevice {
public:
    static uint8_t FIXED_CHECKSUM;
    CANDevice(const std::string& name = "can0") : ifname(name), sock_fd(-1) {}
    ~CANDevice(){
        if (sock_fd >= 0){
            close(sock_fd);
            sock_fd = -1;
        }
    }
    // can通讯相关函数
    int init_socket();
    int can_send(const struct can_frame& frame);
    void send_packet(canid_t base_id, uint8_t dlc, const uint8_t* data);
    int can_receive(struct can_frame* r_frame, int filter_id);
    int recive_dayin(uint8_t id, int addr, const std::string& controlName);
    canid_t get_base_id(int addr);
    void handle_can_receive();
    // 控制动作命令
    int enable_motor(int addr, bool enable);
    int speed_control(int addr, bool direction, int acc, int rpm, bool multiMachine);
    int position_control_emm(int addr, int rpm, int acceleration, bool dir, int pulses, bool absolute, bool multiMachine);
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
    float read_position(int addr, float gear_ratio);
    int read_ma(int addr);
private:
    std::string ifname;
    int sock_fd;
    std::thread recv_thread;
    std::atomic<bool> running{false};
};

#endif // COMMUNICATE_H