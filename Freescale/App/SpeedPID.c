#include "PID.h"
#include "SpeedPID.h"
			
static PID stSpeedPID;	                //速度控制 pid
static int speedPID_old=0;            //上一次的速度 PID
static int MotorSpeedCtrlScale = 1;      //电机输出周期 （第几次平滑输出）

/* -----------------------------------------------
 * 函数名称：Speed_Ctl_Init
 *
 * 函数功能：初始化速度控制
 *
 * 参    数:            point_value：设定的速度
 * 			dKpp:比例
 * 			dKii：积分
 *			dKdd:微分
 *			Integral_side_max：变速积分边界最大值//IMax
 *			Integral_side_min：变速积分边界最小值//IMin
 *
 * 输出:无
 -------------------------------------------------*/
void Speed_Ctl_Init(double point_value, double dKpp, double dKii, double dKdd, double dIMax)
{
	stSpeedPID.SetPoint              = point_value;	//设定值
	stSpeedPID.Proportion            = dKpp;	//比例系数
	stSpeedPID.Integral              = dKii;	//积分系数
	stSpeedPID.Derivative            = dKdd;	//微分系数 
	stSpeedPID.iErr			 = 0;	        //积分误差
	stSpeedPID.Integral_Max	         = dIMax;//积分误差最大值
	stSpeedPID.LastErr		 = 0;
	stSpeedPID.PrevErr		 = 0;
}
/* -----------------------------------------------
* 函数名称:：Speed_Ctrl
*
* 函数功能：根据速度反馈来计算速度控制量
*
* 参   数：speed： 速度反馈
* 		
* 输   出：siSpeedPidOut： 速度控制量
------------------- ------------------------------*/
signed int Speed_Ctrl(float speed)
{
  float fSpeedErr;
  signed int siSpeedPidOut;
  
  fSpeedErr = stSpeedPID.SetPoint - speed;         //速度误差
  stSpeedPID.iErr += fSpeedErr;                    //速度误差累加 += -> =
  if((stSpeedPID.iErr > stSpeedPID.Integral_Max))
  {
    stSpeedPID.iErr = stSpeedPID.Integral_Max;
  }
  else if(stSpeedPID.iErr < -stSpeedPID.Integral_Max)
  {
    stSpeedPID.iErr = -stSpeedPID.Integral_Max;
  }
  siSpeedPidOut = (int)(stSpeedPID.Proportion*fSpeedErr + stSpeedPID.iErr*stSpeedPID.Integral + stSpeedPID.Derivative*(fSpeedErr-stSpeedPID.LastErr));
  stSpeedPID.LastErr = fSpeedErr;                  //设定上一次误差
  
  return siSpeedPidOut;
}

/* -----------------------------------------------
 * 函数名称：Speed_Smooth_Ctrl
 *
 * 函数功能：速度平滑控制，在每一个平滑控制周期中累加速度pid控制量，
 *					 目的是减小速度控制对角度控制的影响
 *
 * 参    数:speed_new：速度pid控制量
 *
 * 输    出:smooth_ctl_out
 -------------------------------------------------*/
signed int Speed_Smooth_Ctrl(signed int speedPID_new)
{
  int smooth_ctl_out;
  
  smooth_ctl_out = (speedPID_new - speedPID_old)*MotorSpeedCtrlScale / SPEED_CTRL_PERIOD + speedPID_old;
  
  if (++MotorSpeedCtrlScale > SPEED_CTRL_PERIOD)
  {
    MotorSpeedCtrlScale = 1;
    speedPID_old = speedPID_new;
  }
  
  return smooth_ctl_out;
}

PID GetSpeedPID(void)
{
  return  stSpeedPID;
}
void SetSpeedPID(PID stPID)
{
  stSpeedPID = stPID;
}


double GetSpeedSetPoint(void)
{
  return stSpeedPID.SetPoint;
}

void SetSpeed_speed(double point_value)
{
  stSpeedPID.SetPoint = point_value;
}

void SetSpeedIMax(unsigned int uiMax)
{
  stSpeedPID.Integral_Max = (double)uiMax;
}

double GetSpeedIMax(void)
{
  return stSpeedPID.Integral_Max;
}


void ResetSpeedControl(void)
{
  stSpeedPID.SetPoint = 0;
  stSpeedPID.pErr = 0;
  stSpeedPID.iErr = 0;
  stSpeedPID.dErr = 0;
  stSpeedPID.LastErr = 0;
  stSpeedPID.PrevErr = 0;
  stSpeedPID.Pid_OutPut = 0;
}

void ResetSpeedControlI(void)
{
  stSpeedPID.iErr = 0;
}