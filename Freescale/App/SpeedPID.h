#ifndef __SPEEDPID_H__
#define __SPEEDPID_H__

#include  "PID.h"               //PID 结构体

#define SPEED_CTRL_PERIOD       10		//速度平滑控制周期 50 ms

/***********************函数声明********************/
void    Speed_Ctl_Init(double point_value, double dKpp, double dKii, double dKdd, double dImax);
signed int Speed_Ctrl(float speed);
signed int Speed_Smooth_Ctrl(signed int speed_new);
void SetSpeedSetPoint(unsigned int uiSetPoint);
double GetSpeedSetPoint(void);
PID GetSpeedPID(void);
void SetSpeedPID(PID stPID);
void SetSpeedIMax(unsigned int uiMax);
double GetSpeedIMax(void);
void ResetSpeedControl(void);
void ResetSpeedControlI(void);
#endif