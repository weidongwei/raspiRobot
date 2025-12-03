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
#include <math.h>
#include <limits.h>
#include <opencv2/opencv.hpp>

#include "machine.h"
#include "cancommunicate.h"
#include "main.h"

extern "C" {
    #include <cjson/cJSON.h>
}

#define pi  3.1415926

MACHINE machine;


//-------------------MOTOR----------------------
//电机回零操作
void MOTOR::motorSetZero() {
    printf("%d  set 0\n",selfID);
    sendSDO(MID_DZCLFZ, selfID, 60006);
    sendSDO(MID_DZCLFZ, selfID, 60016);
    sendSDO(MID_GETPOS, selfID, INT_MIN);
}

//读取JSON中的整数值
int readJointValueI(cJSON *joint, const char *name) {
    if(joint==NULL) { printf("关节指针为NULL"); return -1;}
    cJSON *item = cJSON_GetObjectItem(joint, name);
    if(item==NULL) { printf("没有关键词%s", name); return -1;}
    printf("%s%d\t", name, item->valueint);
    return item->valueint;
}

//读取JSON中的浮点数
int readJointValueF(cJSON *joint, const char *name) {
    if(joint==NULL) { printf("关节指针为NULL"); return -1;}
    cJSON *item = cJSON_GetObjectItem(joint, name);
    if(item==NULL) { printf("没有关键词%s", name); return -1;}
    printf("%s%0.2lf\t", name, item->valuedouble);
    return item->valuedouble;
}

//从JSON配置文件中读取某个电机（关节）的参数
void MOTOR::setPara(int inID, cJSON *joint1) {
    selfID = inID + 1;
    printf("关节%d设置------",selfID);
    zeroPoint = readJointValueI(joint1, "零位");
    maxAcc = readJointValueI(joint1, "最大加速度");
    maxSpeed = readJointValueI(joint1, "最大速度");
    decodePC = readJointValueF(joint1, "每圈读数");
    mReduce = readJointValueF(joint1, "减速比");
    decodeUpLimit = readJointValueI(joint1, "关节角上限");
    decodeLowLimit = readJointValueI(joint1, "关节角下限");
    screwPitch = readJointValueF(joint1, "丝杠导程");
    printf("\n");
}

//设置电机PDO1和PDO3的发送周期和最大速度和最大加速度
void MOTOR::initMotorCommuniation() {
    tarDecodeReached = false;
    motorStationary = false;
    reachReported = false;

    setPDO1Time(100);
    sendSDO(MID_SETPDO1DUR, selfID, INT_MIN);   //读取PDO1周期
    sendSDO(MID_SETPDO3DUR, selfID, 0);         //PDO3返回周期0毫秒

    sendSDO(MID_SETACC, selfID, maxAcc);	    //设置最大加速度
    sendSDO(MID_SETACC, selfID, INT_MIN);       //读取最大加速度
    sendSDO(MID_SETSPEED, selfID, maxSpeed);	//设置最大速度
    sendSDO(MID_SETSPEED, selfID, INT_MIN);     //读取最大速度
}

//设置PDO1的发送周期
void MOTOR::setPDO1Time(int inTime) {
    sendSDO(MID_SETPDO1DUR, selfID, inTime);
}

//设置电机工作模式、控制字
void MOTOR::setMotorWorkCondition() {
    sendSDO(MID_WORKMODE, selfID, 0x07); 	//工作模式0x07为插值模式
    sendSDO(MID_WORKMODE, selfID, INT_MIN); //读取工作模式
    sendSDO(MID_CTRLWORD, selfID, 0x2f);	//控制字0x2f为绝对模式
    sendSDO(MID_CTRLWORD, selfID, INT_MIN); //读取控制字
    sendSDO(MID_SPECIAL, 1, 7);			    //特殊功能0x07为插值模式，这个也要设
    sendSDO(MID_SPECIAL, 1, 0);             //读取特殊功能

    //读取控制字、状态字、工作模式
    sendSDO(MID_CTRLWORD, selfID, INT_MIN); 
    sendSDO(MID_STATEWORD, selfID, INT_MIN);
    sendSDO(MID_WORKMODE, selfID, INT_MIN);

    sendSDO(MID_GETPOS, selfID, INT_MIN);   //读取当前位置

    initDone = true;
}

//设置电机要到达的位置并发送pdo帧,如果工作模式为1(单机运动)发送PDO1,工作模式为4(插值运动)发送PDO4
void MOTOR::setTarDecode(int newPos, int moveMode) {
    tarDecode = newPos;
    
    tarDecodeReached = false;
    motorStationary = false;
    reachReported = false;
    
    machine.allPosReached = false;
    machine.allStationary = false;

    if(moveMode==1) sendPDO1(selfID, tarDecode);
    else if(moveMode==4) sendPDO4(selfID, tarDecode);
}

//通过返回的帧数据来更新运动状态的参数
void MOTOR::setRecDataToMotor(int dIdx, int val) {
    static long int absDalt, prevDalt;

    if(dIdx==MID_GETPOS) {
        prevDecode = currentDecode;
        currentDecode = val;
        
        //判断是否到达目标位置
        absDalt = tarDecode - currentDecode;
        if(absDalt*absDalt<2500) tarDecodeReached = true;
        else tarDecodeReached = false;

        //判断是否静止
        prevDalt = prevDecode - currentDecode;
        if(prevDalt*prevDalt<900) motorStationary = true;
        else motorStationary = false;

        //更新整机状态
        if(tarDecodeReached && motorStationary) {
            machine.tPoint[0].decodeVal[selfID-1] = currentDecode;
            if(!reachReported) {
                reachReported = true;
                machine.checkAllReached();
            }
        }
        else reachReported = false;

        //电机回传频率很高，计算和显示比较消耗cpu
        //if(machine.showRoutePoint) machine.forwardSolve(); 
    }
    //一般数据，直接存储
    else dataVal[dIdx] = val;
}

//将电机编码器读数转成关节角度
void MOTOR::decode2q() {
    q = (double)(currentDecode - zeroPoint)/decodePC/mReduce*2*pi;
}

//q关节角度转化为编码器位置, 参数的单位为度数
int MOTOR::q2decode(double inq) {
    return inq/2/pi * decodePC * mReduce + zeroPoint;
}



//-------------------MACHINE----------------------
//设置PDO模式
void MACHINE::setPDOMode(int inMode) {
    /*if (inMode==1 && nowPDOMode != 1) {
        forAllMotor(mIdx) 
            machine.motor[mIdx].setPDO1Time(50);
    } 
    else if (inMode==4 && nowPDOMode != 4) {
        forAllMotor(mIdx) 
            machine.motor[mIdx].setPDO1Time(50);
    }*/
    nowPDOMode = inMode;
}

//读取JSON文件中机器人系统的参数并初始化，调用motor的setPara
int MACHINE::readPara() {
    // 打开JSON文件
    FILE *file = fopen("robot.json", "r");
    if (file == NULL) {     // 处理文件打开失败的情况
        printf("找不到robot设置文件\n");
        return -1;
    }

    // 获取JSON文件大小
    fseek(file, 0, SEEK_END);
    //缺少fileSize == -1L判断条件
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    // 分配内存并读取JSON内容
    char *jsonString = (char *)malloc(fileSize + 1);
    fread(jsonString, 1, fileSize, file);
    jsonString[fileSize] = '\0';
    fclose(file);  // 关闭文件

    // 解析JSON字符串
    cJSON *root = cJSON_Parse(jsonString);
    if (root == NULL) {  // 处理解析失败的情况
        printf("无法解释robot设置文件\n");
        free(jsonString);
        return -1;
    }

    // 从JSON对象中获取需要的数据
    //使用GetObjectItem后最好使用cJSON_IsNumber检查对象内容是否为数字，是否为空，以排除cjson文件受损
    camNum = cJSON_GetObjectItem(root, "摄像头数目")->valueint;
    imgWidth = cJSON_GetObjectItem(root, "图像宽度")->valueint;
    imgHeight = cJSON_GetObjectItem(root, "图像高度")->valueint;
    
    printf("摄像头数目=%d  图像宽度=%d  图像高度=%d \n",
           camNum,  imgWidth, imgHeight);
    
    L1 = cJSON_GetObjectItem(root, "大臂长度")->valuedouble;
    L2 = cJSON_GetObjectItem(root, "小臂长度")->valuedouble;
    L3 = cJSON_GetObjectItem(root, "腕指长度")->valuedouble;
    motorNum = cJSON_GetObjectItem(root, "关节数")->valueint;
    showRoutePoint = cJSON_GetObjectItem(root, "显示路径点")->valueint;
    
    printf("大臂长度=%0.2lfmm  小臂长度=%0.2lfmm  腕指长度=%0.2lfmm  关节数=%d\n",
           L1,  L2, L3, motorNum);

    cJSON *joints = cJSON_GetObjectItem(root, "关节");
    int count = cJSON_GetArraySize(joints);
    if(count < machine.motorNum) {
        printf("数组数量小于关节数\n");
        free(jsonString);
        return -1;
    }
    forAllMotor(mIdx) {
        cJSON *joint1 = cJSON_GetArrayItem(joints, mIdx);
        motor[mIdx].setPara(mIdx, joint1);
    }

    // 释放资源
    cJSON_Delete(root);
    free(jsonString);

    allPosReached = false;
    allStationary = false;
    prevAllPosReached = false;
    prevAllStationary = false;
    tPointCount = 0;
    doTPidx = 0;

    return 1;
}

//电机是否到达目标且完全静止
bool MACHINE::checkAllReached() {
    prevAllPosReached = allPosReached;
    allPosReached = false;
    forAllMotor(mIdx) 
        if(!motor[mIdx].tarDecodeReached) return false;
    allPosReached = true;
    
    prevAllStationary = allStationary;
    allStationary = false;
    forAllMotor(mIdx) 
        if(!motor[mIdx].motorStationary) return false;
    allStationary = true;
        
    //if(!prevAllPosReached || !prevAllStationary) forwardSolve();
    return true;
}

//电机求正解,求出终端当前x, y, z, a(角度)并存放到tPoint[0]
void MACHINE::forwardSolve() {
    static double tmpx, tmpy, sumTheta;
    forAllMotor(mIdx) motor[mIdx].decode2q(); //当前编码器位置转化为q(弧度)

    sumTheta = motor[0].q;
    tmpx = L1*cos(sumTheta);
    tmpy = L1*sin(sumTheta);
    
    sumTheta += motor[2].q;
    tmpx += L2*cos(sumTheta);
    tmpy += L2*sin(sumTheta);

    sumTheta += motor[3].q;
    tmpx += L3*cos(sumTheta);
    tmpy += L3*sin(sumTheta);

    tPoint[0].xyza[0] = tmpx;
    tPoint[0].xyza[1] = tmpy;
    tPoint[0].xyza[2] = motor[1].screwPitch * motor[1].q / 2 /pi;
    tPoint[0].xyza[3] = sumTheta*180/pi;
    
    printf("时间(us)：%ld, 当前编码器读数=[", globalUS);
    forAllMotor(mIdx) printf("%d ", motor[mIdx].currentDecode);
    printf("] 正解为:\n  q=[");
    forAllMotor(mIdx) printf("%0.2lf ", motor[mIdx].q/2/pi*360);
    printf("][x=%0.2lf y=%0.2lf z=%0.2lf Ax=%0.2lf]  ", tPoint[0].xyza[0],
        tPoint[0].xyza[1], tPoint[0].xyza[2], tPoint[0].xyza[3]);
    printf("\n");

}

//电机求逆解,求出每个电机需要转动的角度q和编码器读数decodeVal
int MACHINE::inverseSolve(int tIdx) {
    static double xw, yw, Lw;    //腕关节坐标， 长度
    static double thetaLw, angL1Lw, angL1L2;
    static double qUP, qDN, daltUP, daltDN, tmpDalt, tmpF;
    static double  x, y, z, Ax, Az;
    //for(int mIdx=0; mIdx<3; mIdx++) printf("%0.2lfmm ", tPoint[tIdx].xyza[mIdx]);
    //for(int mIdx=3; mIdx<machine.motorNum; mIdx++) printf("%0.2lf度 ", tPoint[tIdx].xyza[mIdx]);
    //printf("   逆解为\n");

    x = tPoint[tIdx].xyza[0];
    y = tPoint[tIdx].xyza[1];
    z = tPoint[tIdx].xyza[2];
    Ax = tPoint[tIdx].xyza[3]/180*pi;
    Az = tPoint[tIdx].xyza[4]/180*pi;

    xw = x - L3*cos(Ax);
    yw = y - L3*sin(Ax);
    Lw = sqrt(xw*xw + yw*yw);
    thetaLw = acos(xw/Lw);   //返回值0-pi
    if(yw<0) thetaLw *= -1;

    //求解L1和LW的夹角angL1Lw
    tmpF = (L1*L1 + Lw*Lw - L2*L2)/2/L1/Lw;
    if(tmpF>1 || tmpF<-1) {/*printf("---无解---\n");*/ return -2;}
    angL1Lw = acos(tmpF);  //返回值0-pi

    qUP = thetaLw + angL1Lw;
    tmpDalt = qUP - tPoint[tIdx-1].qArray[0];
    daltUP = tmpDalt * tmpDalt;

    qDN = thetaLw - angL1Lw;
    tmpDalt = qDN - tPoint[tIdx-1].qArray[0];
    daltDN = tmpDalt * tmpDalt;

    //求解L1和L2的夹角angL1L2
    tmpF = (L1*L1 + L2*L2 - Lw*Lw)/2.0/L1/L2;
    if(tmpF>1 || tmpF<-1) {/*printf("---无解---\n"); */return -2;}
    angL1L2 = acos(tmpF);  //返回值0-pi

    //两个解中选择一个
    int selectSolution = tPoint[tIdx].forceSolution;
    if(selectSolution==0) {
        if(daltUP<=daltDN) selectSolution = 1;
        else selectSolution = -1;
    }

    if(selectSolution==1) {
        tPoint[tIdx].qArray[0] = qUP;
        tPoint[tIdx].qArray[2] = angL1L2 - pi;
    }
    else if(selectSolution==-1){
        tPoint[tIdx].qArray[0] = qDN;
        tPoint[tIdx].qArray[2] = pi - angL1L2;
    }
    else printf("wrong selectSolution\n");

    tPoint[tIdx].qArray[1] = z/motor[1].screwPitch*pi*2;
    tPoint[tIdx].qArray[3] = - (tPoint[tIdx].qArray[0] +
         tPoint[tIdx].qArray[2]) + Ax;
    tPoint[tIdx].qArray[4] = Az;

    //将算好的解转换成编码器读数存到tPoint的decodeVal中
    forAllMotor(mIdx) {
        tPoint[tIdx].decodeVal[mIdx] = motor[mIdx].q2decode(tPoint[tIdx].qArray[mIdx]);
    }

    /*
    printf("q=[");
    forAllMotor(mIdx) printf("%0.2lf度 ", tPoint[tIdx].qArray[mIdx]*180/pi);
    printf("]  编码器读数=[");
    forAllMotor(mIdx) printf("%d ", tPoint[tIdx].decodeVal[mIdx]);
    printf("]\n");
    */
    bool checkLimit = true;
    forAllMotor(mIdx) {
        if(tPoint[tIdx].decodeVal[mIdx] > motor[mIdx].decodeUpLimit) {
            printf("关节%d超上限%d \n", mIdx, motor[mIdx].decodeUpLimit);
            checkLimit = false;
        } 
        if(tPoint[tIdx].decodeVal[mIdx] < motor[mIdx].decodeLowLimit) {
            printf("关节%d超下限%d \n", mIdx, motor[mIdx].decodeLowLimit);
            checkLimit = false;
        } 
    }
    if(!checkLimit) return -1;

    return 0;
}

//线性插补(按照直线路径，分成若干小步,计算每个点并存入轨迹点数组tPoint[]中)
bool MACHINE::addTpointLinear(double *xyza, int stepmm, int userForceSolution) {
    static double x0, y0, z0, daltx, dalty, daltz, L;
    x0 = tPoint[tPointCount].xyza[0];
    daltx = (xyza[0] - x0);
    y0 = tPoint[tPointCount].xyza[1];
    dalty= (xyza[1] - y0);
    z0 = tPoint[tPointCount].xyza[2];
    daltz= (xyza[2] - z0);
    L = sqrt(daltx*daltx + dalty*dalty + daltz*daltz);

    //根据比例插值计算出当前轨迹点的xyz
    for(int i=1; i<int(L)+1; i+=stepmm) {
        tPointCount++;
        if(i<int(L)) {
            tPoint[tPointCount].xyza[0] = daltx*i/L + x0;
            tPoint[tPointCount].xyza[1] = dalty*i/L + y0;
            tPoint[tPointCount].xyza[2] = daltz*i/L + z0;
            tPoint[tPointCount].moveMode = 4;
        }
        else {
            tPoint[tPointCount].xyza[0] = xyza[0];
            tPoint[tPointCount].xyza[1] = xyza[1];
            tPoint[tPointCount].xyza[2] = xyza[2];
            tPoint[tPointCount].moveMode = 1;
        }
        
        //printf("%f  %f  %f  %f\n",tPoint[tPointCount].xyza[0],  tPoint[tPointCount].xyza[1],
        //tPoint[tPointCount].xyza[2], tPoint[tPointCount].xyza[3]);
        
        tPoint[tPointCount].forceSolution = userForceSolution;
        tPoint[tPointCount].sendUS = -1;
        if(inverseSolve(tPointCount)!=0) {
            tPointCount = 0;
            return false;
        }
    }
    return true;
}

//不插补中间点,直接加一个想要到达的点保存到tPoint[]中
bool MACHINE::addTpointFree(double *xyza, int userForceSolution) {
    printf("添加pdo1目标：");
    tPointCount++;
    memcpy(tPoint[tPointCount].xyza, xyza, MOTORCAPACITY*sizeof(double));
    tPoint[tPointCount].moveMode = 1;
    tPoint[tPointCount].forceSolution = userForceSolution;
    tPoint[tPointCount].sendUS = -1;
    if(inverseSolve(tPointCount)!=0) {
        tPointCount = 0;
        return false;
    }
    return true;
}

//按顺序发送tPoint[]中的每个轨迹点给电机(插补模式,逐点到位模式)
int MACHINE::doTrace() {
    static int tpMode;
    static long elapseUS;
    static long int nowUS;

    if(doTPidx<=0) return 0;

    if(doTPidx>tPointCount) {
        setPDOMode(1);
        if(!allStationary || !allPosReached) return 0;
        printf("路径完成\n");

        tPointCount = 0; doTPidx = 0;
        printf("dotpidx=%d\n",doTPidx);
        forAllMotor(mIdx)
            tPoint[doTPidx].decodeVal[mIdx] = motor[mIdx].currentDecode;
        machine.forwardSolve();
        return 0;
    }
    
    tpMode = tPoint[doTPidx].moveMode;
    setPDOMode(tpMode);
    nowUS = getGlobalUS();
    elapseUS = nowUS - tPoint[doTPidx].sendUS;
    if(tPoint[doTPidx].sendUS<0) {      //指令未发送
        tPoint[doTPidx].sendUS = getGlobalUS();
        printf("----time(u)=%ld-----doTPidx=%d------\n",nowUS, doTPidx);
        forAllMotor(mIdx) {
            motor[mIdx].setTarDecode(
                tPoint[doTPidx].decodeVal[mIdx],
                tpMode);
        }
        
        if(tpMode==4) {
            sendSync();
            long deltaT = nowUS - tPoint[doTPidx-1].sendUS;
            printf("deltaT=%ld\n", deltaT);
        }
    }
    else if(tpMode==4)  {   //tpMode==4指令已发送
        usleep(16000);
        doTPidx++;
        return 1;
    }
    else if(tpMode==1)  {   //tpMode==1,  指令已发送
        if(allStationary) {    //如果静止不动了
            if(allPosReached) {    //如果已经到位，则下一条
                doTPidx++;
                return 1;
            }
            else if(elapseUS > 500000){   //如果500ms仍未运动，则重发
                printf("resend instruction after elapseUS %ld(us)\n", elapseUS);
                tPoint[doTPidx].sendUS = -1;
                return 1;
            }
        }
        else  tPoint[doTPidx].sendUS = nowUS; //更新下发送时间
    }

    return 0;
}










