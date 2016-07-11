#include "PID.h"
#include "SpeedPID.h"
			
static PID stSpeedPID;	                //�ٶȿ��� pid
static int speedPID_old=0;            //��һ�ε��ٶ� PID
static int MotorSpeedCtrlScale = 1;      //���������� ���ڼ���ƽ�������

/* -----------------------------------------------
 * �������ƣ�Speed_Ctl_Init
 *
 * �������ܣ���ʼ���ٶȿ���
 *
 * ��    ��:            point_value���趨���ٶ�
 * 			dKpp:����
 * 			dKii������
 *			dKdd:΢��
 *			Integral_side_max�����ٻ��ֱ߽����ֵ//IMax
 *			Integral_side_min�����ٻ��ֱ߽���Сֵ//IMin
 *
 * ���:��
 -------------------------------------------------*/
void Speed_Ctl_Init(double point_value, double dKpp, double dKii, double dKdd, double dIMax)
{
	stSpeedPID.SetPoint              = point_value;	//�趨ֵ
	stSpeedPID.Proportion            = dKpp;	//����ϵ��
	stSpeedPID.Integral              = dKii;	//����ϵ��
	stSpeedPID.Derivative            = dKdd;	//΢��ϵ�� 
	stSpeedPID.iErr			 = 0;	        //�������
	stSpeedPID.Integral_Max	         = dIMax;//����������ֵ
	stSpeedPID.LastErr		 = 0;
	stSpeedPID.PrevErr		 = 0;
}
/* -----------------------------------------------
* ��������:��Speed_Ctrl
*
* �������ܣ������ٶȷ����������ٶȿ�����
*
* ��   ����speed�� �ٶȷ���
* 		
* ��   ����siSpeedPidOut�� �ٶȿ�����
------------------- ------------------------------*/
signed int Speed_Ctrl(float speed)
{
  float fSpeedErr;
  signed int siSpeedPidOut;
  
  fSpeedErr = stSpeedPID.SetPoint - speed;         //�ٶ����
  stSpeedPID.iErr += fSpeedErr;                    //�ٶ�����ۼ� += -> =
  if((stSpeedPID.iErr > stSpeedPID.Integral_Max))
  {
    stSpeedPID.iErr = stSpeedPID.Integral_Max;
  }
  else if(stSpeedPID.iErr < -stSpeedPID.Integral_Max)
  {
    stSpeedPID.iErr = -stSpeedPID.Integral_Max;
  }
  siSpeedPidOut = (int)(stSpeedPID.Proportion*fSpeedErr + stSpeedPID.iErr*stSpeedPID.Integral + stSpeedPID.Derivative*(fSpeedErr-stSpeedPID.LastErr));
  stSpeedPID.LastErr = fSpeedErr;                  //�趨��һ�����
  
  return siSpeedPidOut;
}

/* -----------------------------------------------
 * �������ƣ�Speed_Smooth_Ctrl
 *
 * �������ܣ��ٶ�ƽ�����ƣ���ÿһ��ƽ�������������ۼ��ٶ�pid��������
 *					 Ŀ���Ǽ�С�ٶȿ��ƶԽǶȿ��Ƶ�Ӱ��
 *
 * ��    ��:speed_new���ٶ�pid������
 *
 * ��    ��:smooth_ctl_out
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