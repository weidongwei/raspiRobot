#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor.h"


#define MOTOR1 1
#define MOTOR2 2
#define MAXMA1 200          //电机1碰撞最大电流
#define MAXMA2 200          //电机2碰撞最大电流

int main(int argc, char* argv[])
{
    //can_init();

    char control_str[15];
    for(int i=1; i<argc; i++){
        strcpy(control_str,argv[i]);
        if (strcmp(control_str, "enable")==0)           {enable_motor(MOTOR1,true);enable_motor(MOTOR2,true);}
        else if(strcmp(control_str, "disable")==0)      {enable_motor(MOTOR1,false);enable_motor(MOTOR2,false);}
        else if(strcmp(control_str, "stop")==0)         {stop_motor(MOTOR1);stop_motor(MOTOR2);}
        else if(strcmp(control_str, "read_rpm")==0)      {read_rpm(MOTOR1);read_rpm(MOTOR2);}
        else if(strcmp(control_str, "read_pos")==0)      {read_position(MOTOR1);read_position(MOTOR2);}
        else if(strcmp(control_str, "read_ma")==0)      {read_ma(MOTOR1);read_ma(MOTOR2);}
        else if(strcmp(control_str, "set_motor")==0)      {
            set_motor_parameter(MOTOR1, 380, 600, 60);
            set_motor_parameter(MOTOR2, 380, 600, 60);
        }
        else if(strcmp(control_str, "set_zero")==0)      {
            set_zero_parameter(MOTOR1, 350, 10000, 380, 150, 60);
            set_zero_parameter(MOTOR2, 350, 10000, 380, 100, 60);
        }


        //开机确定零点
        else if(strcmp(control_str, "set_zero1")==0)          u_set_zero(MOTOR1);
        else if(strcmp(control_str, "set_zero2")==0)          u_set_zero(MOTOR2);


        //测试
        else if(strcmp(control_str, "go_zero1")==0)           u_go(MOTOR1, 0, MAXMA1, MAXMA2);
        else if(strcmp(control_str, "go_zero2")==0)           u_go(MOTOR2, 0, MAXMA1, MAXMA2);
        else if(strcmp(control_str, "go1")==0)                u_go(MOTOR1, 90, MAXMA1, MAXMA2);
        else if(strcmp(control_str, "go2")==0)                u_go(MOTOR2, 90, MAXMA1, MAXMA2);
        else if(strcmp(control_str, "pickup")==0)             u_pickup();
        else if(strcmp(control_str, "sync_go1")==0)           u_sync_go(100);
        else if(strcmp(control_str, "sync_go2")==0)           u_sync_go(0);



        else if(strcmp(control_str, "laser1")==0)             run_laser(true);
        else if(strcmp(control_str, "laser2")==0)             run_laser(false);



    }

    return 0;
    
}