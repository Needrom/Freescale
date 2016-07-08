#include "EEPROM.h"
#include "SpeedPID.h"
#include "AnglePID.h"
#include "DirectionPID.h"
#include "Control.h"
#include "OLED.h"
#include  "ADCFilter.h"
#include "SysParam.h"
#include "Angle.h"
#include "ElecDetecter.h"
#include "ENC03.h"
#include "Menu.h"
#include "Timer.h"
#include "MK60_adc.h"
#include "BUZZ.h"
#include "BatteryVoltage.h"

static SYSPARAM stSysParam={0};

void InitSysParam()
{
  if(EEPROM_Read_String(0,(uint8 *)(&stSysParam),sizeof(stSysParam))==FAIL)
  { 
    LoadDefaultParam(DIRECTION_UP);
    printf("fail");
  } 
  printf("success");
  UpdateParam(stSysParam);
}

unsigned char SaveParam(SYSPARAM stParam)
{
  unsigned char result = FAIL;

  if(EEPROM_Write_String(0,(uint8 *)(&stSysParam),sizeof(stSysParam))==SUCCESS)
  {
    result = SUCCESS;
  }
  return result;
}

unsigned char UpdateParam(SYSPARAM stParam)
{
  SetAnglePID(stParam.stAnglePID);
  SetSpeedPID(stParam.stSpeedPID);
  SetDirectionPID(stParam.stDirectionPID);
  
  SetDefaultAngle(stParam.fBalanceAngle);
  SetBalanceAngle(stParam.fBalanceAngle);
  SetTargetSpeed(stSysParam.fTargetSpeed);
  SetSpeedUp(stSysParam.uiSpeedUp);

  SetElecProportion(stSysParam.uiElecADetecter, stSysParam.uiElecBDetecter,stSysParam.uiElecCDetecter,stSysParam.uiElecDDetecter,stSysParam.uiElecEDetecter);
 
  SetENC03Val(stParam.uiENC03X);
  
  return 0;
}

signed char Finishing(signed char scDirection)
{ 
  //写入数据到各模块
  UpdateParam(stSysParam);
  
  //保存数据到eeprom
  SaveParam(stSysParam);
  
  return COMPLETE;
}

signed char TestParam(signed char scDirection)
{
  //Ð´ÈëÊý¾Ýµ½¸÷Ä£¿é
  UpdateParam(stSysParam);
  BuzzOff();
  Test();
  return 0;
}
signed char LoadDefaultParam(signed char scDirection)
{
  signed int i = 0;
  //7.4电压下的参数
  Speed_Ctl_Init(0, 38,4.55, 0, 5100);        //速度 PID初始化
  Dir_Ctl_Init(0, 30.85, 461.589);              //方向 PID初始化
  Angle_Ctrl_Init(-21.323, 622.5, 0, 7);    //角度 PID初始化  
  stSysParam.stDirectionPID=GetDirectionPID();
  stSysParam.stSpeedPID=GetSpeedPID();
  stSysParam.stAnglePID=GetAnglePID();
  
  stSysParam.fBalanceAngle=-21.323;
  stSysParam.fTargetSpeed=300;
  stSysParam.uiSpeedUp=2;
  stSysParam.uiENC03X = 1967;
  stSysParam.uiElecADetecter = 1447;
  stSysParam.uiElecBDetecter = 1664;
  stSysParam.uiElecCDetecter = 1765;
  stSysParam.uiElecDDetecter = 1622;
  
  UpdateParam(stSysParam);
  
  if(SaveParam(stSysParam) == SUCCESS)
  {
    OLED_Print(ITEM_LOCATION, 4, "Load Success", SELECT_STATE);
  }
  else
  {
    OLED_Print(ITEM_LOCATION, 4, "Load Fail", SELECT_STATE);
  }
  
  while(i <= 20)
  {
    if(Is50msReady() == 1)	i++;
  }
  
  return 0;
}


double CalcMax()		//计算磁场最大
{
  signed int i;
  unsigned int MaxA=0,MaxB=0,MaxC=0,MaxD=0;
  
  for(i=0;i<(1000);)
  {
    if(Is5msReady())
    {
	 if(MaxA<GetDChannelData()){MaxA=GetDChannelData();}
	 if(MaxB<GetEChannelData()){MaxB=GetEChannelData();}
	 if(MaxC<GetFChannelData()){MaxC=GetFChannelData();}
	 if(MaxD<GetGChannelData()){MaxD=GetGChannelData();}
	 i++;
    }
  }
  return (double)MaxA;
}
float ReadSysVol()
{
    float Vol=0; 
    Vol = GetBatteryVoltage();
    printf("%d\n",(int)Vol);
    stSysParam.fVol = Vol;
  return (float)Vol;
}

signed char SetSysTargetSpeed(signed char scDirection)
{
  stSysParam.fTargetSpeed += (SPEED_TARGETSPEED_DELTA * scDirection);
  return 0;
}

signed char SetSysSpeedUp(signed char scDirection)
{
  stSysParam.uiSpeedUp += (SPEED_SPEEDUP_DELTA * scDirection);
  return 0;
}

signed char SetSysSpeedP(signed char scDirection)
{
  stSysParam.stSpeedPID.Proportion += (SPEED_P_DELTA * scDirection);
  return 0;
}
signed char SetSysSpeedI(signed char scDirection)
{
  stSysParam.stSpeedPID.Integral += (SPEED_I_DELTA * scDirection);
  return 0;
}
signed char SetSysSpeedD(signed char scDirection)
{
  stSysParam.stSpeedPID.Derivative += (SPEED_D_DELTA * scDirection);
  return 0;
}

signed char SetSysAngleP(signed char scDirection)
{
  stSysParam.stAnglePID.Proportion += (ANGLE_P_DELTA * scDirection);
  return 0;
}
signed char SetSysAngleD(signed char scDirection)
{
  stSysParam.stAnglePID.Derivative += (ANGLE_D_DELTA * scDirection);
  return 0;
}

signed char SetSysDirectionP(signed char scDirection)
{
  stSysParam.stDirectionPID.Proportion += (DIRECTION_P_DELTA * scDirection);
  return 0;
}
signed char SetSysDirectionD(signed char scDirection)
{
  stSysParam.stDirectionPID.Derivative += (DIRECTION_D_DELTA * scDirection);
  return 0;
}

signed char SetSysBalanceAngle(float fBalanceAngle)
{
  stSysParam.fBalanceAngle = fBalanceAngle ;
  return 0;
}

signed char SetSysENC03X(unsigned int uiENC03X)
{
  stSysParam.uiENC03X = uiENC03X ;
  return 0;
}



signed char SetSysSpeedIMax(signed char scDirection)
{
  stSysParam.stSpeedPID.Integral_Max += (double)(SPEED_IMAX_DELTA * scDirection);
  return 0;
}

signed char SetSysElecADetecter(unsigned int uiElecA)
{
  stSysParam.uiElecADetecter = uiElecA ;
  return 0;
}
signed char SetSysElecBDetecter(unsigned int uiElecB)
{
  stSysParam.uiElecBDetecter = uiElecB;
  return 0;
}
signed char SetSysElecCDetecter(unsigned int uiElecC)
{
  stSysParam.uiElecCDetecter = uiElecC;
  return 0;
}
signed char SetSysElecDDetecter(unsigned int uiElecD)
{
  stSysParam.uiElecDDetecter = uiElecD;
  return 0;
}
signed char SetSysElecEDetecter(unsigned int uiElecE)
{
  stSysParam.uiElecEDetecter = uiElecE;
  return 0;
}
/*****************************************************/

double GetSysTargetSpeed(void)
{
  return stSysParam.fTargetSpeed;
}

double GetSysSpeedUp(void)
{
  return stSysParam.uiSpeedUp;
}

double GetSysSpeedP(void)
{
  return stSysParam.stSpeedPID.Proportion;
}

double GetSysSpeedI(void)
{
   return stSysParam.stSpeedPID.Integral;
}

double GetSysSpeedD(void)
{
  return stSysParam.stSpeedPID.Derivative;
}

double GetSysAngleP(void)
{
    return stSysParam.stAnglePID.Proportion;
}

double GetSysAngleD(void)
{
 return stSysParam.stAnglePID.Derivative;
}

double GetSysDirectionP(void)
{
  return stSysParam.stDirectionPID.Proportion;
}

double GetSysDirectionD(void)
{
  return stSysParam.stDirectionPID.Derivative;
}

double GetSysBalanceAngle(void)
{
  return stSysParam.fBalanceAngle;
}
double GetSysENC03X(void)
{
  return stSysParam.uiENC03X;
}

double GetSysElecADetecter(void)
{
  return stSysParam.uiElecADetecter;
}
double GetSysElecBDetecter(void)
{
  return stSysParam.uiElecBDetecter;
}
double GetSysElecCDetecter(void)
{
  return stSysParam.uiElecCDetecter;
}
double GetSysElecDDetecter(void)
{
  return stSysParam.uiElecDDetecter;
}
double GetSysElecEDetecter(void)
{
  return stSysParam.uiElecEDetecter;
}
double GetSysSpeedIMax(void)
{
  return stSysParam.stSpeedPID.Integral_Max;
}
double GetSysVol(void)
{
 return stSysParam.fVol;
}

