#ifndef MACHINE_H
#define MACHINE_H

extern "C" {
    #include <cjson/cJSON.h>
    #include <pthread.h>
}

class MOTOR {
public:
    int selfID;                 //电机编号
    //char bSet, bReset;
    bool initDone;              //是否完成初始化

    union {
        int dataVal[5];
        struct {
            int currentDecode;  //当前编码器位置
            int currentSpeed;   //当前速度
            int ctrlWord;       //控制字
            int stateWord;      //状态字
            int workMode;       //工作模式
        };
    };
    
    //电机预先设置的参数
    int zeroPoint;              //零点
    int maxAcc;                 //最大加速度
    int maxSpeed;               //最大速度
    double decodePC;            //编码器脉冲数
    double mReduce;             //减速比
    int decodeUpLimit;          //位置上限
    int decodeLowLimit;         //位置下限
    double screwPitch;          //丝杆螺距

    double q;                   //单位弧度
    void decode2q();            //编码器读数转关节角度
    int q2decode(double inq);   //关节角度q转化为编码器位置

    int tarDecode;              //目标位置,单位电机编码器读数
    int prevDecode;             //上一次编码器位置,单位电机编码器读数
    bool tarDecodeReached;      //是否到达目标
    bool motorStationary;       //电机是否静止
    bool reachReported;         //是否上报过到达状态

    void initMotorCommuniation();                 //初始化通信
    void setPDO1Time(int inTime);                 //设置 PDO1 的发送周期
    void motorSetZero();                          //设置电机零点
    void setPara(int inID, cJSON *joint1);        //用JSON参数初始化电机参数
    void setMotorWorkCondition();                 //设置电机运行状态
    void setTarDecode(int newPos, int moveMode);  //设置电机目标位置
    void setRecDataToMotor(int dIdx, int val);    //接收报文时更新电机数据
    void sendIns();                               //发送指令
};
#pragma pack ()


#define MOTORCAPACITY 6
class MACHINE {
public:
    int showRoutePoint;
    int motorNum;                               //电机数目
    MOTOR motor[MOTORCAPACITY];
    bool prevAllPosReached, prevAllStationary;  //上一次所有电机是否到位和静止   
    bool allPosReached, allStationary;          //所有电机是否到位和静止
    volatile int recCanID;                      //can接收ID
    volatile int recFunID;                      //can接收功能ID

    double L1, L2, L3;                          //三段连杆长度，运动学正解和逆解

    struct {
        double xyza[MOTORCAPACITY];     //XYZ单位mm, a单位为度数
        double qArray[MOTORCAPACITY];   //关节向量，单位都是弧度
        int decodeVal[MOTORCAPACITY];   //电机编码器读数
        short int moveMode;             //运动方式 1 或者 4
        short int forceSolution;        //强制逆解方式 1关节1前角度 -1关节1后角度 0接近前状态 
        long sendUS;                    //<0为指令未发送
    }tPoint[1000];                      //[0]为当前点

    volatile int tPointCount;           //总轨迹点数量,[0]为当前点，所以tPointCount点是有效的
    volatile int doTPidx;               //当前执行点索引,==0时不动，>=1时开始执行
    int nowPDOMode;                     //当前PDO模式

    void setPDOMode(int inMode);        //切换 PDO 模式
    int readPara();                     //读取配置文件参数
    bool checkAllReached();             //检查所有电机到位
    void forwardSolve();                //正运动学解算
    int inverseSolve(int tIdx);         //逆运动学解算
    //添加直线轨迹点
    bool addTpointLinear(double *xyza, int stepmm, int userForceSolution);
    //添加自由轨迹点
    bool addTpointFree(double *xyza, int userForceSolution);
    int doTrace();                      //执行轨迹
};

#define forAllMotor(mIdx)   \
    for(int mIdx = 0; mIdx<machine.motorNum; mIdx++)

extern MACHINE machine;

void initMotorCommuniation2();
void setMotorWorkCondition2();


#endif // MACHINE_H
