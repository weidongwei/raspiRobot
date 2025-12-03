#ifndef CANCOMMUNICATE_H
#define CANCOMMUNICATE_H

enum MID {  MID_GETPOS,	MID_GETSPEED,
        MID_CTRLWORD, MID_STATEWORD, MID_WORKMODE,
        MID_SETACC, MID_SETSPEED, MID_SETPDO1DUR, MID_SETPDO3DUR,
        MID_SPECIAL, MID_GOTOPOS,
        MID_DZCLFZ
        };

void initCanSocket();
void checkPackage();
bool sendSDO(unsigned int funID, unsigned int mIdx, int para);
int sendPDO1(unsigned int mIdx, int para);
int sendPDO2(unsigned int mIdx, int objPos, int tSpeed);
int sendPDO4(unsigned int mIdx, int objPos);
bool sendSync();
void changeFuncTab();


#endif // CANCOMMUNICATE_H
