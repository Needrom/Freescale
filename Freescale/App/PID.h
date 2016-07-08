#ifndef __PID_H_
#define __PID_H_



typedef struct
{
	float  SetPoint;	//�趨ֵ
	float  Proportion;	//����ϵ��
	float  Integral;	//����ϵ��
	float  Derivative;	//΢��ϵ��
	float  LastErr;		//ǰһ�����
	float  PrevErr;		//ǰ�������
	float  pErr;		//�������
	float  iErr;		//�������
	float  dErr;		//΢�����
	float  Pid_OutPut;	//pid���
	float  Integral_Max;	//�������ֵ
}PID;



#endif