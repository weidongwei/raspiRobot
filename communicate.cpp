#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdint>
#include <unistd.h>
#include <cmath>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <fstream>
using json = nlohmann::json;

#include "communicate.h"
#include "motor.h"

#define com "sudo ip link set can0 type can bitrate 1000000"
#define up "sudo ifconfig can0 up"
#define down "sudo ifconfig can0 down"

uint8_t FIXED_CHECKSUM = 0x6B;
int can_socket_fd = -1;
std::string can_ifname = "can0";     // 默认 can0
int running = 1;                     // CAN 接收线程运行标志

std::unordered_map<int, std::string> canDescriptions;

// json文件读取
bool loadJson(const std::string& filename){
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "无法打开 JSON 文件: " << filename << std::endl;
        return false;
    }

    json j;
    f >> j;
    // 获取json中candata内容
    for (auto& [key, value] : j["candata"].items()) {
        int id = std::stoi(key, nullptr, 0);  // 自动识别 0x 前缀
        canDescriptions[id] = value.get<std::string>();
    }
    // 获取json中motordata内容
    for (size_t i = 0; i < j["motordata"].size(); ++i) {
        const json& m = j["motordata"][i];

        mMotor[i].set_motor_id(m.value("电机id", 0) - 1);
        mMotor[i].set_max_rpm(m.value("最大速度", 0));
        mMotor[i].set_up_limit_pos(m.value("上限位置", 0.0f));
        mMotor[i].set_low_limit_pos(m.value("下限位置", 0.0f));
        mMotor[i].set_screwPitch(m.value("丝杠导程", 0));
    }

    return true;
}

std::string getJson(int id){
    auto it = canDescriptions.find(id);
    if (it != canDescriptions.end()) {
        return it->second;
    }
    return "未知指令";
}

// CAN通讯初始化
int can_init(){
    system(down);
    system(com);
    system(up);
    return 0;
} 
// 获取基础CAN ID
canid_t get_base_id(int addr) {
    return static_cast<canid_t>(addr) << 8;
}
// 初始化CAN套接字
int init_socket(){
    loadJson("data.json");

    struct ifreq ifr {};
    struct sockaddr_can addr{};

    can_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_fd < 0) {
        perror("socket create failed");
        return -1;
    }

    strcpy(ifr.ifr_name, can_ifname.c_str());
    if (ioctl(can_socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 关闭回环
    int loopback = 0;
    setsockopt(can_socket_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // 不接收自己的消息
    int recvOwn = 0;
    setsockopt(can_socket_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwn, sizeof(recvOwn));

    if (bind(can_socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        return -1;
    }

    printf("CAN 初始化成功：%s\n", can_ifname.c_str());
    return 0;
}
// 发送CAN数据帧
int can_send(const struct can_frame& frame){
    if (can_socket_fd < 0) {
        printf("Socket not initialized!\n");
        return -1;
    }

    int nbytes = write(can_socket_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        printf("CAN Send Failed!\n");
        return -1;
    }
    return 0;
}
// 发送数据包
void send_packet(canid_t base_id, uint8_t dlc, const uint8_t* data){
    struct can_frame frame{};
    frame.can_id = base_id | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    can_send(frame);
}
// 接收CAN数据帧
int can_receive(struct can_frame* r_frame){
    if (can_socket_fd < 0) return -1;

    int nbytes = read(can_socket_fd, r_frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("CAN read error");
        return -1;
    }

    return 0;
}
// 处理接收到的CAN数据帧
void handle_can_receive() {
    struct pollfd fds;
    fds.fd = can_socket_fd;
    fds.events = POLLIN;
    while (running) {
        int ret = poll(&fds, 1, 200);   // 200ms 可中断等待
        if (!running) break;

        if (ret > 0 && (fds.revents & POLLIN)) {
            struct can_frame response{};
            if (can_receive(&response) == 0) {
                uint8_t motorID = (response.can_id >> 8) & 0xFF;
                int cmd = response.data[0];
                if (response.can_dlc == 3) {
                    uint8_t cmd_status = response.data[1];
                    uint8_t checksum = response.data[2];
                    if (checksum == FIXED_CHECKSUM) {
                        if (cmd_status == 0x02)
                            printf("%d号电机 %s 启动成功\n", motorID, getJson(cmd).c_str());
                        else if (cmd_status == 0xE2)
                            printf("%d号电机 %s 条件不满足\n", motorID, getJson(cmd).c_str());
                        else if (cmd_status == 0xEE)
                            printf("%d号电机 %s 命令错误\n", motorID, getJson(cmd).c_str());
                    }
                }else if(response.can_dlc == 7 && cmd == 0x36){ //电机实时位置
                    uint8_t dir = response.data[1];
                    uint32_t pos = (response.data[2] << 24) | (response.data[3] << 16) | (response.data[4] << 8) | response.data[5];
                    uint8_t checksum = response.data[6];
                    float pos_val;
                    if(motorID == 1 || motorID == 4 || motorID == 8){
                        pos_val = (pos * 360.0f) / 65536.0f;
                    }else{
                        pos_val = pos / 10.0f;
                        pos_val = angleToDistanceConvert(motorID, pos_val);
                    }
                    if (checksum == FIXED_CHECKSUM) {
                        mMotor[motorID-1].set_position((dir == 0) ? pos_val : -pos_val);
                        // printf("%d号电机 实时位置：%.2f°\n", motorID, (dir == 0) ? pos_val : -pos_val);
                    }
                }else if(response.can_dlc == 5 && cmd == 0x35){ //电机实时转速
                    uint8_t dir = response.data[1];
                    uint32_t rpm = (response.data[2] << 8) | response.data[3];
                    uint8_t checksum = response.data[4];
                    float rpm_val;
                    if(motorID == 1 || motorID == 4 || motorID == 8){
                        rpm_val = rpm;
                    }else{
                        rpm_val = rpm / 10.0f;
                    }
                    if (checksum == FIXED_CHECKSUM) {
                        mMotor[motorID-1].set_rpm((dir == 0) ? rpm_val : -rpm_val);
                        // printf("%d号电机 实时转速：%.2f rpm\n", motorID, (dir == 0) ? rpm_val : -rpm_val);
                    }
                }else if(response.can_dlc == 4 && cmd == 0x27){ //电机实时电流
                    uint32_t ma = (response.data[1] << 8) | response.data[2];
                    uint8_t checksum = response.data[3];
                    if (checksum == FIXED_CHECKSUM) {
                        mMotor[motorID-1].set_ma(ma);
                        // printf("%d号电机 实时电流：%d Ma\n", motorID, ma);
                    }
                }
            }
        }
    }
}

//----------------------控制动作命令----------------------
// 电机使能控制
int enable_motor(int addr, bool enable){   
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 5;
    uint8_t enable_val = static_cast<uint8_t>(enable ? 0x01 : 0x00);
    uint8_t data[] = {
        0xF3,               // 命令
        0xAB,               // 固定标识
        enable_val,         // 使能状态
        0x00,               // 多机同步标志
        FIXED_CHECKSUM      // 校验
    };
    printf("----\n");
    send_packet(base_id, dlc, data);
    return 0;
}

// 速度模式控制
// speed_control(MOTOR1, true, 350, 10, false);
int speed_control(int addr, bool direction, int acc, int rpm, bool multiMachine){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 7;
    uint8_t dir = static_cast<uint8_t>(direction ? 0x01 : 0x00);
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data[] = {
        0xF6,                               // 命令
        dir,                                // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(acc), // 加速度档位   --0A表示加速度档位为0x0A = 10
        static_cast<uint8_t>(rpm >> 8),     // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF),   // 速度低字节   --05 DC表示速度为0x05DC = 1500(RPM)
        mMachine,                           // 多机同步标志 --00表示不启用多机同步（01表示启用）
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 位置模式控制(emm)
// position_control_emm(MOTOR1, 350, 0, true, 90, true, false);
int position_control_emm(int addr, int rpm, int acceleration, bool dir, float pulses, bool absolute, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x00 : 0x01);
    int pulses_val = abs(pulses) / 360 * 200 * 16 * 1;
    uint8_t data1[8] = {
        0xFD,                                           // 功能码
        dir_val,                                        // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(rpm >> 8),                 // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF),               // 速度低字节    --05 DC表示速度为0x05DC = 1500(RPM)
        static_cast<uint8_t>(acceleration),             // 加速度档位    --0A表示加速度档位为0x0A = 10
        static_cast<uint8_t>((pulses_val >> 24) & 0xFF),    // 脉冲3         --00 00 7D 00表示脉冲数为0x00007D00 = 32000个
        static_cast<uint8_t>((pulses_val >> 16) & 0xFF),    // 脉冲2
        static_cast<uint8_t>((pulses_val >> 8) & 0xFF)      // 脉冲1
    };

    
    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 5;
    uint8_t absolute_val = static_cast<uint8_t>(absolute ? 0x00 : 0x01);
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data2[5] = {
        0xFD,                                       // 功能码（第二帧重复）
        static_cast<uint8_t>(pulses_val & 0xFF),        // 脉冲0
        absolute_val,                               // 相对/绝对模式    --00表示相对位置模式（01表示绝对位置模式）
        mMachine,                                   // 多机同步标志     --00表示不启用多机同步（01表示启用）
        FIXED_CHECKSUM,                             // 校验
    };

    send_packet(base_id1, dlc1, data1);
    send_packet(base_id2, dlc2, data2);

    return 0;
}

// 位置模式控制(X)
int position_control_x(int addr, bool dir, int rpm, float angle, int mode, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
    int angle_val = static_cast<int>(angle * 10);
    int rpm_val = rpm * 10;
    uint8_t data1[8] = {
        0xFB,                                               // 功能码
        dir_val,                                            // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(rpm_val >> 8),                     // 速度高字节
        static_cast<uint8_t>(rpm_val & 0xFF),                   // 速度低字节    --05 DC表示速度为0x05DC = 1500(RPM)
        static_cast<uint8_t>((angle_val >> 24) & 0xFF),     // 脉冲3         --00 00 7D 00表示脉冲数为0x00007D00 = 32000个
        static_cast<uint8_t>((angle_val >> 16) & 0xFF),     // 脉冲2
        static_cast<uint8_t>((angle_val >> 8) & 0xFF),      // 脉冲1
        static_cast<uint8_t>(angle_val & 0xFF)              // 脉冲0
    };

    
    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 4;
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data2[4] = {
        0xFB,                                               // 功能码
        static_cast<uint8_t>(mode),                         // 相对/绝对模式    --00表示相对位置模式（01表示绝对位置模式）
        mMachine,                                           // 多机同步标志     --00表示不启用多机同步（01表示启用）
        FIXED_CHECKSUM,                                     // 校验
    };


    send_packet(base_id1, dlc1, data1);
    send_packet(base_id2, dlc2, data2);

    return 0;
}

// 梯形曲线加减速(X)
int position_control_t_x(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x00 : 0x01);
    int angle_val = static_cast<int>(angle * 10);
    uint8_t data1[8] = {
        0xFD,                                               // 功能码
        dir_val,                                            // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(accup >> 8),                   // 加速加速度高字节
        static_cast<uint8_t>(accup & 0xFF),                 // 加速加速度低字节
        static_cast<uint8_t>(accdown >> 8),                 // 减速加速度高字节
        static_cast<uint8_t>(accdown & 0xFF),               // 减速加速度低字节
        static_cast<uint8_t>(rpm >> 8),                     // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF)                    // 速度低字节    --05 DC表示速度为0x05DC = 1500(RPM)
    };

    
    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 8;
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data2[8] = {
        0xFD,                                               // 功能码
        static_cast<uint8_t>((angle_val >> 24) & 0xFF),     // 脉冲3         --00 00 7D 00表示脉冲数为0x00007D00 = 32000个
        static_cast<uint8_t>((angle_val >> 16) & 0xFF),     // 脉冲2
        static_cast<uint8_t>((angle_val >> 8) & 0xFF),      // 脉冲1
        static_cast<uint8_t>(angle_val & 0xFF),             // 脉冲0
        static_cast<uint8_t>(mode),                         // 相对/绝对模式    --00表示相对位置模式（01表示绝对位置模式）
        mMachine,                                           // 多机同步标志
        FIXED_CHECKSUM                                      // 校验
    };



    send_packet(base_id1, dlc1, data1);
    send_packet(base_id2, dlc2, data2);

    return 0;
}

// 电机停止
int stop_motor(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 4;
    uint8_t data[] = {
        0xFE,         // 命令
        0x98,         // 固定标识
        0x00,         // 多机同步标志
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 当前的位置角度清零
int clear_all(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 3;
    uint8_t data[] = {
        0x0A,                               // 命令
        0x6D,                               // 命令
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 设置单圈回零的零点位置
int set_zero(int addr, bool save){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 4;
    uint8_t save_val = static_cast<uint8_t>(save ? 0x01 : 0x00);
    uint8_t data[] = {
        0x93,                               // 命令
        0x88,                               // 命令
        save_val,                           // 是否存储：0x01 表示保存
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 触发回零
int run_zero(int addr,int mode, bool multiMachine){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 4;
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data[] = {
        0x9A,                               // 命令
        static_cast<uint8_t>(mode),         // 回零模式         
                                            //  --00表示触发单圈就近回零
                                            //  --01表示触发单圈方向回零
                                            //  --02表示触发多圈无限位碰撞回零
                                            //  --03表示触发多圈有限位开关回零
        mMachine,                           // 多机同步标志     --00表示不启用多机同步（01表示启用）
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 修改任意细分
int set_division(int addr,bool save, int division){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 5;
    uint8_t save_val = static_cast<uint8_t>(save ? 0x01 : 0x00);
    uint8_t data[] = {
        0x84,                               // 命令
        0x8A,                               // 命令
        save_val,                           // 是否存储：0x01 表示保存
        static_cast<uint8_t>(division),     // 细分     --0x07 = 7细分(00表示256细分)
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 修改驱动配置参数
int set_motor_parameter(int addr, int dangerRpm, int dangerMa, int dangerTime){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t data1[8] = {
        0x48,                                           // 功能码
        0xD1,                                           // 命令
        0x01,                                           // 保存本次修改的配置参数
        0x19,                                           // 修改电机类型为1.8°电机
        0x02,                                           // 修改脉冲端口控制模式为PUL_FOC，即FOC矢量闭环模式
        0x03,                                           // 修改通讯端口复用模式为CAN，即串口/RS232/RS485通讯/CAN通讯
        0x02,                                           // 修改En引脚的有效电平为Hold，即一直有效
        0x00                                            // 修改Dir引脚的有效方向为CW，即顺时针方向;
    };

    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 8;
    uint8_t data2[8] = {
        0x48,                                           // 功能码
        0x01,                                           // 修改细分为1细分（注意：256细分用00表示）         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        0x01,                                           // 修改细分插补功能为Enable，即使能细分插补
        0x00,                                           // 修改自动熄屏功能为Disable，即不使能自动熄屏功能
        0x03,                                           // 修改开环模式工作电流为1000Ma（03EB）
        0xE8,
        0x0B,                                           // 修改闭环模式堵转时的最大电流为3000Ma(0x0BB8)
        0xB8
    };

    canid_t base_id3 = get_base_id(addr) + 2;
    uint8_t dlc3 = 8;
    uint8_t data3[8] = {
        0x48,                                           // 功能码
        0x0F,                                           // 修改闭环模式最大输出压为4000mV(0x0FA0)       --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        0xA0,
        0x05,                                           // 修改串口波特率为115200（对应屏幕选项顺序：0x00为9600，...）
        0x09,                                           // 修改CAN通讯速率为1000000（对应屏幕选项顺序：0x00为10000，...）
        0x01,                                           // 该值原为修改ID地址为1，现取消，使得可用广播地址批量修改驱动参数
        0x00,                                           // 修改通讯校验方式为0x6B
        0x01                                            // 修改控制命令应答为Receive，即只回复确认收到命令
    };

    canid_t base_id4 = get_base_id(addr) + 3;
    uint8_t dlc4 = 8;
    uint8_t data4[8] = {
        0x48,                                           // 功能码
        0x01,                                           // 修改堵转保护功能为Enable，即使能堵转保护
        static_cast<uint8_t>((dangerRpm >> 8) & 0xFF),  // 修改堵转保护转速阈值为40RPM（转/每分钟）
        static_cast<uint8_t>(dangerRpm & 0xFF), 
        static_cast<uint8_t>((dangerMa >> 8) & 0xFF),   // 修改堵转保护电流阈值为2400Ma
        static_cast<uint8_t>(dangerMa & 0xFF),
        static_cast<uint8_t>((dangerTime >> 8) & 0xFF), // 修改堵转保护检测时间阈值为4000ms
        static_cast<uint8_t>(dangerTime & 0xFF)
    };

    
    canid_t base_id5 = get_base_id(addr) + 4;
    uint8_t dlc5 = 4;
    uint8_t data5[4] = {
        0x48,                                       // 功能码（第二帧重复）
        0x00,                                       // 修改位置到达窗口为0.1°
        0x01,
        FIXED_CHECKSUM                              // 校验
    };


    send_packet(base_id1, dlc1, data1);
    send_packet(base_id2, dlc2, data2);
    send_packet(base_id3, dlc3, data3);
    send_packet(base_id4, dlc4, data4);
    send_packet(base_id5, dlc5, data5);

    return 0;
}

// 修改原点回零参数
int set_zero_parameter(int addr, int acceleration, int timeout , int dangerRpm, int dangerMa, int dangerTime){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t data1[8] = {
        0x4C,                                           // 功能码
        0xAE,                                           // 命令
        0x01,                                           // 保存本次修改的配置参数
        0x02,                                           // 多圈无限位碰撞回零
        0x00,                                           // 回零方向为CW
        static_cast<uint8_t>((acceleration >> 8) & 0xFF),//回零速度为30(RPM)
        static_cast<uint8_t>(acceleration & 0xFF),
        static_cast<uint8_t>((timeout >> 24) & 0xFF)    //回零超时时间为10000(ms)
    };

    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 8;
    uint8_t data2[8] = {
        0x4C,                                           // 功能码
        static_cast<uint8_t>((timeout >> 16) & 0xFF),
        static_cast<uint8_t>((timeout >> 8) & 0xFF),
        static_cast<uint8_t>(timeout & 0xFF),
        static_cast<uint8_t>((dangerRpm >> 8) & 0xFF),  //无限位碰撞回零检测转速为300(RPM)
        static_cast<uint8_t>(dangerRpm & 0xFF),
        static_cast<uint8_t>((dangerMa >> 8) & 0xFF),   //无限位碰撞回零检测电流为800(Ma)
        static_cast<uint8_t>(dangerMa & 0xFF)
    };

    canid_t base_id3 = get_base_id(addr) + 2;
    uint8_t dlc3 = 5;
    uint8_t data3[5] = {
        0x4C,                                          // 功能码（第二帧重复）
        static_cast<uint8_t>((dangerTime >> 8) & 0xFF),//无限位碰撞回零检测电流为800(Ma)
        static_cast<uint8_t>(dangerTime & 0xFF),
        0x00,                                          //不使能上电自动触发回零功能
        FIXED_CHECKSUM                                 // 校验
    };


    send_packet(base_id1, dlc1, data1);
    send_packet(base_id2, dlc2, data2);
    send_packet(base_id3, dlc3, data3);

    return 0;
}

// 解除堵转保护
int close_stall(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 3;
    uint8_t data[] = {
        0x0E,                               // 命令
        0x52,                               // 命令
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 多机同步
int sync_run(){
    canid_t base_id = get_base_id(0);
    uint8_t dlc = 3;
    uint8_t data[3] = {
        0xFF,         // 命令
        0x66,         // 固定标识
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

//----------------------读取参数命令----------------------
// 读取电机实时转速
int read_rpm(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x35,          // 命令
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 读取电机实时位置角度
int read_position(int addr, float gear_ratio) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x36,          // 命令
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}

// 读取电机实时电流
int read_ma(int addr) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x27,          // 命令
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    return 0;
}