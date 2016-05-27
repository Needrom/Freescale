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
  
  static float Speed1_Err,SumErrSpeed;  //静态变量存储中间变量
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
  
  Speed2_Err = Speed1_Err ;                //将上一次的偏差保存
  
  Speed1_Err = AmSpeed - speedCount  ;      //  计算新的偏差值
  
  Speed_EC = Speed1_Err - Speed2_Err ;      //  计算新的偏差变化值 
   
  Speed_P_Value =  Speed1_Err * speedKP/10.0 ;   //  增量式PID控制计算P调节量
  
  SumErrSpeed  +=  Speed1_Err * speedKI ;    //增量式PID控制计算I调节量

  Speed_D_Value =  Speed_EC   *  speedKD/100.0 ;     //  增量式PID控制计算D调节量
  
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