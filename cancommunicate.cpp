#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <limits.h>


#include "cancommunicate.h"
#include "machine.h"

int fd_send, fd_recv;
struct can_frame sframe;
struct can_frame rFrame;


struct CODEROW{
    char funName[64];
    union {
	    unsigned short int idx;
	    unsigned char idxc[2];
    };
    unsigned char subIdx;
    unsigned char byteCode;   //只能是0x08,0x10,0x20
    union {
	    unsigned int cs;
	    unsigned char csc[4];
    };
};

#define FUNCCAPACITY 18
//功能表(Canopen地址)
CODEROW funcTab[FUNCCAPACITY] = {
    //		     idx		subIdx	byteCode
	{"当前位置", 	0x6064, 	0x00, 	0x20},
	{"当前速度", 	0x606c, 	0x00, 	0x20},

	{"控制字", 	0x6040, 	0x00, 	0x10},
	{"状态字", 	0x6041, 	0x00, 	0x10},
	{"工作模式", 	0x6060, 	0x00, 	0x08},

    {"最大加速度",0x6083, 	0x00, 	0x20},
    {"最大T速度", 0x6081, 	0x00, 	0x20},
    {"PDO1周期", 0x1800, 	0x05, 	0x10},
    {"PDO3周期", 0x1802, 	0x05, 	0x10},

	{"特殊功能", 	0x2619, 	0x00, 	0x10},
    {"走到位置", 	0x607a, 	0x00, 	0x20},

    {"电子齿轮分子", 	0x260a, 	0x00, 	0x10}
};

//将功能表中的数据(Canopen地址)转换为CAN帧格式
void changeFuncTab() {
    CODEROW *rowP;
    for(int i=0; i<FUNCCAPACITY; i++) {
        rowP = funcTab + i;
        if(strcmp(rowP->funName,"")==0) break;

        rowP->cs = 0;
        if(rowP->byteCode==0x08) rowP->csc[0] = 0x2f;
        else if(rowP->byteCode==0x10) rowP->csc[0] = 0x2b;
        else if(rowP->byteCode==0x20) rowP->csc[0] = 0x23;
        else printf("%s byteCode=%x错误\n", rowP->funName, rowP->byteCode);

        rowP->csc[1] = rowP->idxc[0];
        rowP->csc[2] = rowP->idxc[1];
        rowP->csc[3] = rowP->subIdx;
    }
}

//初始化CAN总线的发送和接收Socket,绑定到can0设备
void initCanSocket() {
    struct sockaddr_can addr_send, addr_recv;
    struct ifreq ifr_send, ifr_recv;

    //初始化fd_send
    //fd_send == -1 error 检查没有做
    fd_send = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);//创建套接字
    strcpy(ifr_send.ifr_name, "can0" );
    //ioctl系统调用未判断是否成功,获取接口索引
    ioctl(fd_send, SIOCGIFINDEX, &ifr_send); //指定 can0 设备
    addr_send.can_family = AF_CAN;
    addr_send.can_ifindex = ifr_send.ifr_ifindex;
    bind(fd_send, (struct sockaddr *)&addr_send, sizeof(addr_send));//将套接字与 can0 绑定
    //setsockopt(fd_send, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    //初始化fd_recv
    //fd_recv = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);//创建套接字
    fd_recv = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr_recv.ifr_name, "can0");
    ioctl(fd_recv, SIOCGIFINDEX, &ifr_recv);
    addr_recv.can_family = AF_CAN;
    addr_recv.can_ifindex = ifr_recv.ifr_ifindex;
    bind(fd_recv, (struct sockaddr *)&addr_recv, sizeof(addr_recv));
    //setsockopt(fd_recv, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    changeFuncTab();
}



void checkPackage() {
    static short prif, mIdx;
    static int itemFound;
    static CODEROW *rowP;

    #pragma pack (1)
    union RECDATA{
        struct {
            char c0;
            short idx;
            char subIdx;
            int val;
        }data58;
        struct {
            int currentPos;
            short stateWord;
        }pdo124Data;
        char c[8];
    };
    #pragma pack ()
    static RECDATA recDate;

    read(fd_recv, &rFrame, sizeof(can_frame));
    
    if(machine.nowPDOMode == 4) return;
    if(rFrame.can_id<=0) return;


    //CRC
    prif = rFrame.can_id >> 4;
    mIdx = (rFrame.can_id & 0x0f);
    memcpy(&recDate, rFrame.data, rFrame.can_dlc);

    itemFound = false;
    switch(prif) {
    case 0x70:
        //printf("电机%d收到心跳\n", mIdx);
        break;
    case 0x58:
        itemFound = false;
        for(int i=0; i<FUNCCAPACITY; i++) {
            if(itemFound) break;
            rowP = funcTab + i;
            if(strcmp(rowP->funName,"")==0) break;//循环找到接收can帧的funcTab
            if(recDate.data58.idx!=rowP->idx) continue;
            if(recDate.data58.subIdx!=rowP->subIdx) continue;
            machine.recFunID = i;

            switch(recDate.data58.c0) {
                case 0x60:
                    printf("电机%d%s设置成功\n", mIdx, rowP->funName);
                    itemFound = true;
                    break;
                case 0x80:
                    printf("电机%d%s设置失败\n", mIdx, rowP->funName);
                    itemFound = true;
                    break;
                case 0x4f:
                case 0x4b:
                case 0x43:
                    printf("电机%d%s=0x%x(%d)\n", mIdx, rowP->funName,
                    recDate.data58.val, recDate.data58.val);
                    itemFound = true;
                    if(i<5) {
                        machine.motor[mIdx-1].setRecDataToMotor(i, recDate.data58.val);
                    }
                    break;
                default :
                    printf("电机%d首字0x%x无法解释\n", mIdx, recDate.data58.c0);
                break;
            }
        }
        break;
    case 0x18:
    case 0x28:
    case 0x48:
        machine.motor[mIdx-1].setRecDataToMotor(MID_GETPOS, recDate.pdo124Data.currentPos);
        machine.motor[mIdx-1].setRecDataToMotor(MID_STATEWORD, recDate.pdo124Data.stateWord);
        //printf("电机%d收到位置=%d，状态字=0x%x\n", mIdx,
        //       recDate.pdo124Data.currentPos,
        //       recDate.pdo124Data.stateWord);
        break;
    }
    
    machine.recCanID = rFrame.can_id;
    //printf("canid=%x\n", machine.recCanID);
    rFrame.can_id = 0;

    return;
}

//发送can帧
int writeCANframe() {
    static ssize_t bytes_written;
    static int times;
    times = 1;
    while(1) {
    	bytes_written = write(fd_send, &sframe, sizeof(can_frame));
        if(bytes_written != -1) {  //succeed, 如果失败-1，
            printf("send times %d\n", times);
            break;
        }
        times++;
        usleep(1000);
    }
    return times;
}

//发SDO帧
bool sendSDO(unsigned int funID, unsigned int mIdx, int para) {
    struct FRAMEDATA{
        union {
            int csI;
            unsigned char csc[4];
        };
        union {
            int iVal;
            unsigned char vchar[4];
        };
    }frameData;
    sframe.can_id = 0x600 + mIdx;

    frameData.csI = funcTab[funID].cs;
    if(para==INT_MIN) {
        frameData.csc[0] = 0x40;
        sframe.can_dlc = 4;
        printf("读取电机%d%s.....",mIdx,funcTab[funID].funName);
    }
    else {
        frameData.iVal = para;
        sframe.can_dlc = 8;
        printf("设置电机%d%s=0x%x(%d).....",mIdx,funcTab[funID].funName,para,para);
    }

    memcpy(sframe.data, &frameData, sizeof(FRAMEDATA));

    writeCANframe();

    int prevCANID = -1;
    int recCount = 0;
    while(1) {
        if(machine.recCanID!=prevCANID) {
            prevCANID = machine.recCanID; recCount++; }
            
        if(machine.recCanID==0x580 + mIdx &&  machine.recFunID==funID) {
            machine.recCanID = 0;
            machine.recFunID = -1;
            break;
        }
        
        if(recCount>=5)  {writeCANframe();  recCount = 0;}
    }

    return true;
}


//发PDO1帧
int sendPDO1(unsigned int mIdx, int para) {
#pragma pack (1)
struct PDO1DATA{
    short ctrlWord;
    char workMode;
    int objPos;
}frameData;
#pragma pack ()

    sframe.can_id = 0x200 + mIdx;
    sframe.can_dlc = 7;

    frameData.ctrlWord = machine.motor[mIdx - 1].ctrlWord;
    frameData.workMode = machine.motor[mIdx - 1].workMode;
    frameData.objPos = para;
    memcpy(sframe.data, &frameData, sizeof(PDO1DATA));
    printf("电机%d发送PDO1=0x%x(%d)......",mIdx, para, para);
    writeCANframe();
    while(1) {
        if(machine.recCanID==0x180 + mIdx) {
            printf("成功\n");
            break;
        }
    }

    return true;
}


//发PDO2帧
int sendPDO2(unsigned int mIdx, int objPos, int tSpeed) {
struct PDO2DATA{
    int objPos;
    int tSpeed;
}frameData;
    sframe.can_id = 0x300 + mIdx;
    sframe.can_dlc = 8;

    frameData.objPos = objPos;
    frameData.tSpeed = tSpeed;

    memcpy(sframe.data, &frameData, sizeof(PDO2DATA));
    printf("电机%d发送PDO2=0x%x(%d)  =0x%x(%d)......",mIdx, objPos, objPos, tSpeed, tSpeed);
    writeCANframe();
    while(1) {
        if(machine.recCanID==0x280 + mIdx) {
            printf("成功\n");
            break;
        }
    }
    return true;
}

//发PDO4帧
int sendPDO4(unsigned int mIdx, int objPos) {
    sframe.can_id = 0x500 + mIdx;
    sframe.can_dlc = 4;

    memcpy(sframe.data, &objPos, sizeof(int));
    printf("电机%d发送PDO4=0x%x(%d)......",mIdx, objPos, objPos);
    writeCANframe();
/*    while(1) {
        if(machine.recCanID==0x480 + mIdx) {
            printf("成功\n");
            break;
        }
    }*/
    return true;
}

//发PDO4同步帧
bool sendSync() {
    sframe.can_id = 0x80;
    sframe.can_dlc = 0;

    printf("发送PDO4同步帧......");
    writeCANframe();
    printf("成功\n");
    return true;
}

