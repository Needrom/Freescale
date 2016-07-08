#ifndef __PID_H_
#define __PID_H_



typedef struct
{
	float  SetPoint;	//设定值
	float  Proportion;	//比例系数
	float  Integral;	//积分系数
	float  Derivative;	//微分系数
	float  LastErr;		//前一次误差
	float  PrevErr;		//前两次误差
	float  pErr;		//比例误差
	float  iErr;		//积分误差
	float  dErr;		//微分误差
	float  Pid_OutPut;	//pid输出
	float  Integral_Max;	//积分最大值
}PID;



#endif