#ifndef _SYSPARAM_H_
#define _SYSPARAM_H_

#include "PID.h"

#define DIRECTION_UP    1
#define DIRECTION_DOWN  -1
#define SPEED_P_DELTA	        	1
#define SPEED_I_DELTA	        	0.05
#define SPEED_D_DELTA	        	3
#define SPEED_TARGETSPEED_DELTA 	10
#define SPEED_SPEEDUP_DELTA     	1
#define ANGLE_P_DELTA	        	50
#define ANGLE_D_DELTA	        	0.5
#define DIRECTION_P_DELTA		5
#define DIRECTION_D_DELTA		5
#define SPEED_IMAX_DELTA                100

#define FAIL                    	0
#define SUCCESS                 	1
#define COMPLETE                        1

typedef struct
{
  PID stDirectionPID;
  PID stSpeedPID;
  PID stAnglePID;
  float fBalanceAngle;                //平衡角度  
  unsigned int uiElecADetecter;       //磁场强度
  unsigned int uiElecBDetecter;       //磁场强度
  unsigned int uiElecCDetecter;       //磁场强度
  unsigned int uiElecDDetecter;       //磁场强度
  unsigned int uiElecEDetecter;       //磁场强度
  double  fTargetSpeed;               //目标速度
  unsigned int  uiSpeedUp;            //加速度
  float fVol;                         //电压值
  unsigned int uiENC03X;               //陀螺仪的静止值
}SYSPARAM;


void InitSysParam();						  //³õÊ¼»¯Êý¾Ý£¬´ÓEEPROM¶ÁÊý¾Ý
unsigned char SaveParam(SYSPARAM stParam);			  //±£´æÊý¾Ýµ½EEPROM
unsigned char UpdateParam(SYSPARAM stParam);			  //±£´æÊý¾Ýµ½¸÷¸öÄ£¿é
signed char Finishing(signed char scDirection);		  //±£´æÊý¾Ý
signed char TestParam(signed char scDirection);                 //²âÊÔ²ÎÊý
signed char LoadDefaultParam(signed char scDirection);          //»Ö¸´Ä¬ÈÏÉèÖÃ

float  CalcSysBalanceAngle();							//自动检测板子平衡角度
double CalcMax();									//计算磁场范围
float  ReadSysVol();                                                    //读电压值
signed char SetSysTargetSpeed(signed char scDirection);	//设置目标速度
signed char SetSysSpeedUp(signed char scDirection);		//设置速度增量

signed char SetSysSpeedP(signed char scDirection);
signed char SetSysSpeedI(signed char scDirection);
signed char SetSysSpeedD(signed char scDirection);

signed char SetSysAngleP(signed char scDirection);
signed char SetSysAngleD(signed char scDirection);

signed char SetSysDirectionP(signed char scDirection);
signed char SetSysDirectionD(signed char scDirection);

signed char SetSysBalanceAngle(float fBalanceAngle);
signed char SetSysENC03X(unsigned int uiENC03X);
signed char SetSysENC03Z(unsigned int uiENC03Z);
signed char SetSysSpeedIMax(signed char scDirection);

signed char SetSysElecADetecter(unsigned int uiElecA);
signed char SetSysElecBDetecter(unsigned int uiElecB);
signed char SetSysElecCDetecter(unsigned int uiElecA);
signed char SetSysElecDDetecter(unsigned int uiElecB);
signed char SetSysElecEDetecter(unsigned int uiElecB);
/*******************************************************/
double GetSysTargetSpeed(void);
double GetSysSpeedUp(void);

double GetSysSpeedP(void);
double GetSysSpeedI(void);
double GetSysSpeedD(void);

double GetSysAngleP(void);
double GetSysAngleD(void);

double GetSysDirectionP(void);
double GetSysDirectionD(void);

double GetSysBalanceAngle(void);
double GetSysENC03X(void);
double GetSysENC03Z(void);

double GetSysElecADetecter(void);
double GetSysElecBDetecter(void);
double GetSysElecCDetecter(void);
double GetSysElecDDetecter(void);
double GetSysElecEDetecter(void);

double GetSysSpeedIMax(void);

double GetSysVol(void);

void Delay_m(int count);
#endif