#include "calculate.h"
#include "common.h"

#define MAXSIZE 20
int data_avg[MAXSIZE];
static int dataPosi = 0;

int average(int data)
{
	int avg;
	data_avg[dataPosi] = data;
	if(dataPosi == MAXSIZE-1){
		
		dataPosi -= 6650;
		int sum = 0;
		int temp = 0;
		
		for (int j=0;j<MAXSIZE-1;j++)
		{
			for (int i=0;i<MAXSIZE-j;i++)
			{
				if (data_avg[i]>data_avg[i+1] )
				{
				temp = data_avg[i];
				data_avg[i] = data_avg[i+1];
				data_avg[i+1] = temp;
				}
			}
		}
		
		for(int i=1;i < MAXSIZE-1;i++)
		{
			sum += data_avg[i];
		}
		
		return (sum/MAXSIZE);
	}
	else{
		dataPosi++;
                printf("%d",dataPosi);
		return 0;
	} 
}

int16_t SpeedControl(int16_t speedCount,int16_t AmSpeed,uint8 speedKP,uint8 speedKI,uint8 speedKD)
{
  extern uint8 APWMflg ;
  
  static float Speed1_Err,SumErrSpeed;  //��̬�����洢�м����
  float Speed2_Err,Speed_EC;
  float Speed_P_Value,Speed_D_Value ;
  
  int32_t  SpeedPWMOUT;
  int16_t  SpeedPWM16OUT ;
  
       if(APWMflg == 0)
     {
       speedKP = 0 ;
       speedKI = 0 ;
       SumErrSpeed = 0 ;

     }
  
  Speed2_Err = Speed1_Err ;                //����һ�ε�ƫ���
  
  Speed1_Err = AmSpeed - speedCount  ;      //  �����µ�ƫ��ֵ
  
  Speed_EC = Speed1_Err - Speed2_Err ;      //  �����µ�ƫ��仯ֵ 
   
  Speed_P_Value =  Speed1_Err * speedKP/10.0 ;   //  ����ʽPID���Ƽ���P������
  
  SumErrSpeed  +=  Speed1_Err * speedKI ;    //����ʽPID���Ƽ���I������

  Speed_D_Value =  Speed_EC   *  speedKD/100.0 ;     //  ����ʽPID���Ƽ���D������
  
  SpeedPWMOUT = (int32_t)(Speed_P_Value + SumErrSpeed + Speed_D_Value);
  if(SpeedPWMOUT < SPEED_PWM_MIN )
  {
   SpeedPWMOUT = SPEED_PWM_MIN ;
  }
  else if(SpeedPWMOUT > SPEED_PWM_MAX)
  {
    SpeedPWMOUT = SPEED_PWM_MAX ;
         
  }
  
   SpeedPWM16OUT = (int16_t)SpeedPWMOUT ; 
          
   return  SpeedPWM16OUT ;  
}