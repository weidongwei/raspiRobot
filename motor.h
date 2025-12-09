#ifndef motor_H
#define motor_H

#include <iostream>


int user_57motor();

int u_detect_motor(int addr, int maxMa1, int maxMa2);
int u_detect_multi_motor(int maxI1,int maxI2);
int u_set_zero(int addr);
int u_go(int addr, float angle, int maxI1, int maxI2);
int u_pickup();
int u_sync_go(float angle);
int run_laser(bool isOpen);
int run_pump(bool isOpen);
int takepic();

#endif // motor_H