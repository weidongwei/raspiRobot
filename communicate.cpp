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

#include "communicate.h"

#define com "sudo ip link set can0 type can bitrate 1000000"
#define up "sudo ifconfig can0 up"
#define down "sudo ifconfig can0 down"

uint8_t CANDevice::FIXED_CHECKSUM = 0x6B;



// CAN通讯初始化
int can_init(){
    system(down);
    system(com);
    system(up);
    return 0;
} 
// 获取基础CAN ID
canid_t CANDevice::get_base_id(int addr) {
    return static_cast<canid_t>(addr) << 8;
}
// 初始化CAN套接字
int CANDevice::init_socket(){
    sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd < 0) {
        perror("socket create failed");
        return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 关闭回环
    int loopback = 0;
    setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // 不接收自己的消息
    int recvOwn = 0;
    setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwn, sizeof(recvOwn));

    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        return false;
    }

    return 0;
}
// 发送CAN数据帧
int CANDevice::can_send(const struct can_frame& frame){
    if (sock_fd < 0) {
        printf("Socket not initialized!\n");
        return -1;
    }

    int nbytes = write(sock_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        printf("CAN Send Failed!\n");
        return -1;
    }
    return 0;
}
// 发送数据包
void CANDevice::send_packet(canid_t base_id, uint8_t dlc, const uint8_t* data){
    struct can_frame frame{};
    frame.can_id = base_id | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    can_send(frame);
}
// 接收CAN数据帧
int CANDevice::can_receive(struct can_frame* r_frame, int filter_id){
    if (sock_fd < 0) return -1;

    struct can_filter rfilter;
    rfilter.can_id = get_base_id(filter_id);
    rfilter.can_mask = CAN_SFF_MASK;

    setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock_fd, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    int ret = select(sock_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret <= 0)
        return -1;

    int nbytes = read(sock_fd, r_frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("CAN read error");
        return -1;
    }

    return 0;
}
// 接收并处理电机响应
int CANDevice::recive_dayin(uint8_t id, int addr, const std::string& controlName){
    struct can_frame response{};
    int flag = 1;

    for (int i = 0; i < 4; i++) {
        flag = can_receive(&response, addr);
        if (flag == 0) break;
    }

    if (flag != 0) {
        printf("%s：接收超时或读取失败\n", controlName.c_str());
        return -1;
    }

    if (response.can_dlc == 3 && response.data[0] == id) {
        if (response.data[1] == 0x02 && response.data[2] == FIXED_CHECKSUM)
            printf("%s 启动成功\n", controlName.c_str());
        else if (response.data[1] == 0xE2 && response.data[2] == FIXED_CHECKSUM)
            printf("%s 条件不满足\n", controlName.c_str());
    }
    else if (response.data[0] == 0x00 &&
             response.data[1] == 0xEE &&
             response.data[2] == FIXED_CHECKSUM){
        printf("%s 命令错误\n", controlName.c_str());
    }

    return 0;
}



void CANDevice::start_receive_thread() {
    if (sock_fd < 0) {
        std::cout << "sock_fd invalid, cannot start receiver thread!" << std::endl;
        return;
    }
    if (running) return;
    running = true;
    recv_thread = std::thread(&CANDevice::receive_loop, this);
}

void CANDevice::stop_receive_thread() {
    if (running) {
        running = false;
    }

    if (recv_thread.joinable()) {
        recv_thread.join();
    }
}

void CANDevice::receive_loop() {
    printf("CAN receive thread started.\n");
    
    struct can_frame response{};
    struct pollfd fds;

    fds.fd = sock_fd;
    fds.events = POLLIN | POLLERR | POLLHUP;

    const int TIMEOUT_MS = 60000;  // 60秒

    if (sock_fd < 0) {
        printf("sock_fd invalid, exit thread.\n");
        return;
    }
    while (running) {

        int ret = poll(&fds, 1, TIMEOUT_MS);

        if (ret == 0) {  
            printf("CAN receive timeout (60s), exiting thread.\n");
            running = false;
            break;
        } else if (ret < 0) {
            perror("poll error");
            running = false;
            break;
        }

        if (fds.revents & (POLLERR | POLLHUP)) {
            printf("CAN socket closed or error.\n");
            break;
        }

        int nbytes = read(sock_fd, &response, sizeof(response));
        
        if (nbytes <= 0) {
            if (nbytes < 0) perror("CAN read error");
            continue;   // 不退出线程，继续轮询
        }

        // -------- 业务逻辑部分 --------
        if (nbytes > 0 && response.can_dlc == 3 && (response.data[0] == 0xFB || response.data[0] == 0xFD)) {

            if (response.data[1] == 0x9F && response.data[2] == FIXED_CHECKSUM) {
                printf("%s 运动完毕 位置模式\n");
                running = false;
                break;
            }
            else if (response.data[1] == 0xE2 && response.data[2] == FIXED_CHECKSUM) {
                printf("%s 条件不满足 位置模式\n");
            }
        }else if (nbytes > 0 && response.data[0] == 0x00 && response.data[1] == 0xEE && response.data[2] == FIXED_CHECKSUM){
            printf("%s 命令错误 位置模式\n");
        }
    }

    printf("CAN receive thread stopped.\n");
}

//----------------------控制动作命令----------------------
// 电机使能控制
int CANDevice::enable_motor(int addr, bool enable){   
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
    send_packet(base_id, dlc, data);
    recive_dayin(0xF3, addr, "电机使能");
    return 0;
}

// 速度模式控制
// speed_control(MOTOR1, true, 350, 10, false);
int CANDevice::speed_control(int addr, bool direction, int acc, int rpm, bool multiMachine){
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
    recive_dayin(0xF6, addr, "速度模式");
    return 0;
}

// 位置模式控制(emm)
// position_control_emm(MOTOR1, 350, 0, true, 90, true, false);
int CANDevice::position_control_emm(int addr, int rpm, int acceleration, bool dir, int pulses, bool absolute, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
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


    struct can_frame frame1, frame2;
    memset(&frame1, 0, sizeof(can_frame));
    memset(&frame2, 0, sizeof(can_frame));
    frame1.can_id = base_id1 | CAN_EFF_FLAG;
    frame1.can_dlc = dlc1;
    memcpy(frame1.data, data1, dlc1);
    frame2.can_id = base_id2 | CAN_EFF_FLAG;
    frame2.can_dlc = dlc2;
    memcpy(frame2.data, data2, dlc2);

    can_send(frame1);
    can_send(frame2);

    recive_dayin(0xFD, addr, "位置模式");
    return 0;
}

// 位置模式控制(X)
int CANDevice::position_control_x(int addr, bool dir, int rpm, float angle, int mode, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
    int angle_val = static_cast<int>(angle * 10);
    uint8_t data1[8] = {
        0xFB,                                               // 功能码
        dir_val,                                            // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(rpm >> 8),                     // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF),                   // 速度低字节    --05 DC表示速度为0x05DC = 1500(RPM)
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

    // recive_dayin(0xFB, addr, "x位置模式");
    return 0;
}

// 梯形曲线加减速(X)
int CANDevice::position_control_t_x(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
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

    recive_dayin(0xFD, addr, "x梯形加速位置模式");
    return 0;
}

// 电机停止
int CANDevice::stop_motor(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 4;
    uint8_t data[] = {
        0xFE,         // 命令
        0x98,         // 固定标识
        0x00,         // 多机同步标志
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    recive_dayin(0xFE, addr, "停止");
    return 0;
}

// 当前的位置角度清零
int CANDevice::clear_all(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 3;
    uint8_t data[] = {
        0x0A,                               // 命令
        0x6D,                               // 命令
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    recive_dayin(0x0A, addr, "位置角度清零");
    return 0;
}

// 设置单圈回零的零点位置
int CANDevice::set_zero(int addr, bool save){
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
    recive_dayin(0x93, addr, "设置单圈回零的零点位置");
    return 0;
}

// 触发回零
int CANDevice::run_zero(int addr,int mode, bool multiMachine){
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
    recive_dayin(0x9A, addr, "触发回零");
    return 0;
}

// 修改任意细分
int CANDevice::set_division(int addr,bool save, int division){
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
    recive_dayin(0x84, addr, "修改细分");
    return 0;
}

// 修改驱动配置参数
int CANDevice::set_motor_parameter(int addr, int dangerRpm, int dangerMa, int dangerTime){
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

    recive_dayin(0x48, addr, "修改驱动配置参数");
    return 0;
}

// 修改原点回零参数
int CANDevice::set_zero_parameter(int addr, int acceleration, int timeout , int dangerRpm, int dangerMa, int dangerTime){
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

    recive_dayin(0x4C, addr, "修改原点回零参数");
    return 0;
}

// 解除堵转保护
int CANDevice::close_stall(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 3;
    uint8_t data[] = {
        0x0E,                               // 命令
        0x52,                               // 命令
        FIXED_CHECKSUM                      // 校验
    };
    send_packet(base_id, dlc, data);
    recive_dayin(0x0E, addr, "解除堵转保护");
    return 0;
}

// 多机同步
int CANDevice::sync_run(){
    canid_t base_id = get_base_id(0);
    uint8_t dlc = 3;
    uint8_t data[3] = {
        0xFF,         // 命令
        0x66,         // 固定标识
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);
    recive_dayin(0xFF, 1, "多机同步");
    return 0;
}

//----------------------读取参数命令----------------------
// 读取电机实时转速
int CANDevice::read_rpm(int addr){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x35,          // 命令
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);

    int value = 1;
    can_frame response;
    memset(&response, 0, sizeof(can_frame));
    if (can_receive(&response, addr) == 0) {
        if (response.can_dlc == 5 && response.data[0] == 0x35) {
            value = (response.data[2] << 8) | response.data[3];
            uint8_t sign = response.data[1]; // 00：正；01：负
            if (sign == 0x00) value = -value;
            printf("%d号电机实时转速：%d rpm\n", addr, value/10);
        } else if(response.data[0] == 0x00 && response.data[1] == 0xEE && response.data[2] == 0x6B){
            printf("读取电机实时转速错误\n");
        }
    }else {
        printf("超时或读取失败\n");
    }
    return value;
}

// 读取电机实时位置角度
float CANDevice::read_position(int addr, float gear_ratio) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x36,          // 命令
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);

    can_frame response;
    memset(&response, 0, sizeof(can_frame));
    if (can_receive(&response, addr) == 0) {
        if (response.can_dlc == 7 && response.data[0] == 0x36) {
            uint8_t sign = response.data[1]; // 00：正；01：负
            uint32_t pos_raw = 
                (response.data[2] << 24) |
                (response.data[3] << 16) |
                (response.data[4] << 8)  |
                response.data[5];
            float angle = pos_raw / 10;
            angle = angle / gear_ratio;
            if (sign == 0x00) angle = -angle;
            printf("pos_raw = %.2f\n", angle);
            angle = fmodf(angle, 360.0f);
            printf("%d号电机实时位置：%.2f°\n", addr, angle);
            return angle;
        } 
        else if (response.data[0] == 0x00 && response.data[1] == 0xEE && response.data[2] == 0x6B) {
            printf("读取电机实时位置错误\n");
        }
    } else {
        printf("超时或读取失败\n");
    }
    return 0;
}


// 读取电机实时电流
int CANDevice::read_ma(int addr) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x27,          // 命令
        FIXED_CHECKSUM // 校验
    };
    send_packet(base_id, dlc, data);

    int value = 1;
    can_frame response;
    memset(&response, 0, sizeof(can_frame));
    if (can_receive(&response, addr) == 0) {
        if (response.can_dlc == 4 && response.data[0] == 0x27) {
            value = (response.data[1] << 8) | response.data[2];
            printf("%d号电机实时电流：             %d Ma\n", addr, value);
        } else if(response.data[0] == 0x00 && response.data[1] == 0xEE && response.data[2] == 0x6B){
            printf("读取电机实时电流错误\n");
        }
    }else {
        printf("超时或读取失败\n");
    }
    return value;
}