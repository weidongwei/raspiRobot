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

#include "communicate.h"

#define com "sudo ip link set can0 type can bitrate 1000000"
#define up "sudo ifconfig can0 up"
#define down "sudo ifconfig can0 down"
#define SEND_CAN                            \
    struct can_frame frame;                 \
    memset(&frame, 0, sizeof(can_frame));   \
    frame.can_id = base_id | CAN_EFF_FLAG;  \
    frame.can_dlc = dlc;                    \
    memcpy(frame.data, data, dlc);          \
    can_send(frame);                        \


const uint8_t FIXED_CHECKSUM = 0x6B;

// 计算电机基ID (addr << 8)
canid_t get_base_id(int addr) {
    return static_cast<canid_t>(addr) << 8;
}

//CAN通讯初始化
int can_init(){
    system(down);
    system(com);
    system(up);
    return 0;
}

//发送can帧
int can_send(can_frame frame){
    int s, nbytes;                  //s:socket文件描述符,nbytes：接收到的的数据长度
    struct sockaddr_can addr;       //addr：CANsocket地址结构
    struct ifreq ifr;               //ifr：用于指定CAN接口(如can0)
    //创建套接字
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0" );
    //指定can0设备
    ioctl(s, SIOCGIFINDEX, &ifr); 
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    //关闭回环模式(Loopback)
    int loopback = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    //将套接字与can0绑定
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    //发送frame[0]
    nbytes = write(s, &frame, sizeof(frame));
    if(nbytes != sizeof(frame))
    {
        printf("Send Error frame[0]\n!");
    }
    close(s);
    return 0;
}

//接收CAN帧
int can_receive(struct can_frame *r_frame, int filter_id){
    int s, nbytes = 0;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;         //用于零时保存接收到的一帧数据
    struct can_filter rfilter;      //CAN过滤器结构

    memset(&frame,0,sizeof(can_frame));     //初始化接收帧数据
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr); 
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int loopback = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    int recv_own_msgs = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

    rfilter.can_id = get_base_id(filter_id);
    rfilter.can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    // 设置超时机制
    fd_set read_fds;
    struct timeval timeout;
    FD_ZERO(&read_fds);
    FD_SET(s, &read_fds);
    timeout.tv_sec = 1; // 1 秒
    timeout.tv_usec = 0;

    int ret = select(s + 1, &read_fds, NULL, NULL, &timeout);
    if (ret <= 0) {
        close(s);
        return -1;
    }

    nbytes = read(s, &frame, sizeof(frame));
    if (nbytes < 0) {
        perror("CAN没有读到\n");
        close(s);
        return -1;
    }

    *r_frame = frame;
    close(s);
    return 0;

}

//打印回应帧
int recive_dayin(uint8_t id, int addr, std::string controlName){
    can_frame response;
    memset(&response, 0, sizeof(can_frame));
    int flag = 1;
    for(int i=0; i<4; i++){
        flag = can_receive(&response, addr);
        if (flag == 0) break;
    }
    if (response.can_dlc == 3 && response.data[0] == id) {
        if(response.data[1] == 0x02 && response.data[2] == 0x6B){
            printf("%s启动成功\n",controlName.c_str());
        }else if(response.data[1] == 0xE2 && response.data[2] == 0x6B){
            printf("%s条件不满足\n",controlName.c_str());
        }
    } else if(response.data[0] == 0x00 && response.data[1] == 0xEE && response.data[2] == 0x6B){
        printf("%s命令错误\n",controlName.c_str());
    }
    if(flag != 0) printf("超时或读取失败\n");
    return 0;
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
    SEND_CAN
    recive_dayin(0xF3, addr, "电机使能");
    return 0;
}

// 速度模式控制
// speed_control(MOTOR1, true, 350, 10, false);
int speed_control(int addr, bool direction, int rpm, int acceleration, bool multiMachine){
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 7;
    uint8_t dir = static_cast<uint8_t>(direction ? 0x01 : 0x00);
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data[] = {
        0xF6,                               // 命令
        dir,                                // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(rpm >> 8),     // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF),   // 速度低字节   --05 DC表示速度为0x05DC = 1500(RPM)
        static_cast<uint8_t>(acceleration), // 加速度档位   --0A表示加速度档位为0x0A = 10
        mMachine,                           // 多机同步标志 --00表示不启用多机同步（01表示启用）
        FIXED_CHECKSUM                      // 校验
    };
    SEND_CAN
    recive_dayin(0xF6, addr, "速度模式");
    return 0;
}

// 位置模式控制
// position_control(MOTOR1, 350, 0, true, 90, true, false);
int position_control(int addr, int rpm, int acceleration, bool dir, int pulses, bool absolute, bool multiMachine){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
    uint8_t data1[8] = {
        0xFD,                                           // 功能码
        dir_val,                                        // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(rpm >> 8),                 // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF),               // 速度低字节    --05 DC表示速度为0x05DC = 1500(RPM)
        static_cast<uint8_t>(acceleration),             // 加速度档位    --0A表示加速度档位为0x0A = 10
        static_cast<uint8_t>((pulses >> 24) & 0xFF),    // 脉冲3         --00 00 7D 00表示脉冲数为0x00007D00 = 32000个
        static_cast<uint8_t>((pulses >> 16) & 0xFF),    // 脉冲2
        static_cast<uint8_t>((pulses >> 8) & 0xFF)      // 脉冲1
    };

    
    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 5;
    uint8_t absolute_val = static_cast<uint8_t>(absolute ? 0x00 : 0x01);
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data2[5] = {
        0xFD,                                       // 功能码（第二帧重复）
        static_cast<uint8_t>(pulses & 0xFF),        // 脉冲0
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
int position_control_x(int addr, bool dir, int rpm, float angle, int mode, bool multiMachine, int maxma){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
    int angle_val = static_cast<int>(angle * 10);
    uint8_t data1[8] = {
        0xCB,                                               // 功能码
        dir_val,                                            // 方向         --01表示旋转方向为CCW逆时针（00表示CW顺时针）
        static_cast<uint8_t>(rpm >> 8),                     // 速度高字节
        static_cast<uint8_t>(rpm & 0xFF),                   // 速度低字节    --05 DC表示速度为0x05DC = 1500(RPM)
        static_cast<uint8_t>((angle_val >> 24) & 0xFF),     // 脉冲3         --00 00 7D 00表示脉冲数为0x00007D00 = 32000个
        static_cast<uint8_t>((angle_val >> 16) & 0xFF),     // 脉冲2
        static_cast<uint8_t>((angle_val >> 8) & 0xFF),      // 脉冲1
        static_cast<uint8_t>(angle_val & 0xFF)              // 脉冲0
    };

    
    canid_t base_id2 = get_base_id(addr) + 1;
    uint8_t dlc2 = 6;
    uint8_t mMachine = static_cast<uint8_t>(multiMachine ? 0x01 : 0x00);
    uint8_t data2[6] = {
        0xCB,                                               // 功能码
        static_cast<uint8_t>(mode),                // 相对/绝对模式    --00表示相对位置模式（01表示绝对位置模式）
        mMachine,                                   // 多机同步标志     --00表示不启用多机同步（01表示启用）
        static_cast<uint8_t>(maxma >> 8),           
        static_cast<uint8_t>(maxma & 0xFF),         // 最大电流
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

    recive_dayin(0xCB, addr, "x位置模式");
    return 0;
}

// 位置模式梯形曲线加减速(X)
int position_control_t_x(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine, int maxma){
    canid_t base_id1 = get_base_id(addr);
    uint8_t dlc1 = 8;
    uint8_t dir_val = static_cast<uint8_t>(dir == true ? 0x01 : 0x00);
    int angle_val = static_cast<int>(angle * 10);
    uint8_t data1[8] = {
        0xCD,                                               // 功能码
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
        0xCD,                                               // 功能码
        static_cast<uint8_t>((angle_val >> 24) & 0xFF),     // 脉冲3         --00 00 7D 00表示脉冲数为0x00007D00 = 32000个
        static_cast<uint8_t>((angle_val >> 16) & 0xFF),     // 脉冲2
        static_cast<uint8_t>((angle_val >> 8) & 0xFF),      // 脉冲1
        static_cast<uint8_t>(angle_val & 0xFF),             // 脉冲0
        static_cast<uint8_t>(mode),                         // 相对/绝对模式    --00表示相对位置模式（01表示绝对位置模式）
        mMachine,                                           // 多机同步标志     --00表示不启用多机同步（01表示启用）
        static_cast<uint8_t>(maxma >> 8)
    };

    canid_t base_id3 = get_base_id(addr) + 2;
    uint8_t dlc3 = 3;
    uint8_t data3[3] = {
        0xCD,                                               // 功能码
        static_cast<uint8_t>(maxma & 0xFF),                 // 最大电流
        FIXED_CHECKSUM                                      // 校验
    };


    struct can_frame frame1, frame2,frame3;
    memset(&frame1, 0, sizeof(can_frame));
    memset(&frame2, 0, sizeof(can_frame));
    memset(&frame3, 0, sizeof(can_frame));
    frame1.can_id = base_id1 | CAN_EFF_FLAG;
    frame1.can_dlc = dlc1;
    memcpy(frame1.data, data1, dlc1);
    frame2.can_id = base_id2 | CAN_EFF_FLAG;
    frame2.can_dlc = dlc2;
    memcpy(frame2.data, data2, dlc2);
    frame3.can_id = base_id3 | CAN_EFF_FLAG;
    frame3.can_dlc = dlc3;
    memcpy(frame3.data, data3, dlc3);

    can_send(frame1);
    can_send(frame2);
    can_send(frame3);

    recive_dayin(0xCD, addr, "x梯形加速位置模式");
    return 0;
}

// 位置模式梯形曲线加减速(X)
int position_control_t_x2(int addr, bool dir,int accup, int accdown, int rpm, float angle, int mode, bool multiMachine){
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

    recive_dayin(0xFD, addr, "x梯形加速位置模式");
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
    SEND_CAN
    recive_dayin(0xFE, addr, "停止");
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
    SEND_CAN
    recive_dayin(0x0A, addr, "位置角度清零");
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
    SEND_CAN
    recive_dayin(0x93, addr, "设置单圈回零的零点位置");
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
    SEND_CAN
    recive_dayin(0x9A, addr, "触发回零");
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
    SEND_CAN
    recive_dayin(0x84, addr, "修改细分");
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


    struct can_frame frame1, frame2, frame3, frame4, frame5;
    memset(&frame1, 0, sizeof(can_frame));
    memset(&frame2, 0, sizeof(can_frame));
    memset(&frame3, 0, sizeof(can_frame));
    memset(&frame4, 0, sizeof(can_frame));
    memset(&frame5, 0, sizeof(can_frame));
    frame1.can_id = base_id1 | CAN_EFF_FLAG;
    frame1.can_dlc = dlc1;
    memcpy(frame1.data, data1, dlc1);
    frame2.can_id = base_id2 | CAN_EFF_FLAG;
    frame2.can_dlc = dlc2;
    memcpy(frame2.data, data2, dlc2);
    frame3.can_id = base_id3 | CAN_EFF_FLAG;
    frame3.can_dlc = dlc3;
    memcpy(frame3.data, data3, dlc3);
    frame4.can_id = base_id4 | CAN_EFF_FLAG;
    frame4.can_dlc = dlc4;
    memcpy(frame4.data, data4, dlc4);
    frame5.can_id = base_id5 | CAN_EFF_FLAG;
    frame5.can_dlc = dlc5;
    memcpy(frame5.data, data5, dlc5);

    can_send(frame1);
    can_send(frame2);
    can_send(frame3);
    can_send(frame4);
    can_send(frame5);

    recive_dayin(0x48, addr, "修改驱动配置参数");
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


    struct can_frame frame1, frame2, frame3;
    memset(&frame1, 0, sizeof(can_frame));
    memset(&frame2, 0, sizeof(can_frame));
    memset(&frame3, 0, sizeof(can_frame));
    frame1.can_id = base_id1 | CAN_EFF_FLAG;
    frame1.can_dlc = dlc1;
    memcpy(frame1.data, data1, dlc1);
    frame2.can_id = base_id2 | CAN_EFF_FLAG;
    frame2.can_dlc = dlc2;
    memcpy(frame2.data, data2, dlc2);
    frame3.can_id = base_id3 | CAN_EFF_FLAG;
    frame3.can_dlc = dlc3;
    memcpy(frame3.data, data3, dlc3);
    

    can_send(frame1);
    can_send(frame2);
    can_send(frame3);

    recive_dayin(0x4C, addr, "修改原点回零参数");
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
    SEND_CAN
    recive_dayin(0x0E, addr, "解除堵转保护");
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
    SEND_CAN
    recive_dayin(0xFF, 1, "多机同步");
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
    SEND_CAN

    int value = 1;
    can_frame response;
    memset(&response, 0, sizeof(can_frame));
    if (can_receive(&response, addr) == 0) {
        if (response.can_dlc == 5 && response.data[0] == 0x35) {
            value = (response.data[2] << 8) | response.data[3];
            uint8_t sign = response.data[1]; // 00：负；01：正
            if (sign == 0x00) value = -value;
            printf("%d号电机实时转速：%d rpm\n", addr, value);
        } else if(response.data[0] == 0x00 && response.data[1] == 0xEE && response.data[2] == 0x6B){
            printf("读取电机实时转速错误\n");
        }
    }else {
        printf("超时或读取失败\n");
    }
    return value;
}

// 读取电机实时位置角度
float read_position(int addr, float gear_ratio) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x36,          // 命令
        FIXED_CHECKSUM // 校验
    };
    SEND_CAN

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
            float angle = (pos_raw * 360.0f) / 65536.0f;
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

float read_position_x(int addr, float gear_ratio) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x36,          // 命令
        FIXED_CHECKSUM // 校验
    };
    SEND_CAN

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
int read_ma(int addr) {
    canid_t base_id = get_base_id(addr);
    uint8_t dlc = 2;
    uint8_t data[] = {
        0x27,          // 命令
        FIXED_CHECKSUM // 校验
    };
    SEND_CAN

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