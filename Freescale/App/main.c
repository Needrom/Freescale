
#include "common.h"
#include "include.h"
#include "MK60_uart.h"
#include "MK60_port.h"
#include "MK60_FTM.h"
#include "MK60_PIT.h"
#include "LDC1000.H"
#include "calculate.h"

/////////////////////////变量定义//////////////////////////
//FLAG
volatile int PIT_2msFlag;
int PIT_2msCount = 0;
int PIT_2MS_CONSTON = 2;
volatile int PIT_5msFlag;
int PIT_5msCount = 0;
int PIT_5MS_CONSTON = 5;
volatile int PIT_10msFlag;
int PIT_10msCount = 0;
int PIT_10MS_CONSTON = 10;
volatile int PIT_20msFlag;
int PIT_20msCount = 0;
int PIT_20MS_CONSTON = 20;
volatile int PIT_50msFlag;
int PIT_50msCount = 0;
int PIT_50MS_CONSTON = 2;
volatile int PIT_1sFlag;
int PIT_1sCount = 0;
int PIT_1S_CONSTON = 1000;
volatile int PIT_2sFlag;
int PIT_2sCount = 0;
int PIT_2S_CONSTON = 2000;
volatile int PIT_motorFlag;
int PIT_motorCount = 0;
volatile int PIT_steeringFlag;
int PIT_steeringCount = 0;


//LDC_val采样值
extern uint8 orgVal[12];
extern int  LDC_val;
int  LDC_SPI0_val;
int  LDC_SPI1_val;
float  LDC_result;
float frist_steering_error;
float p_steering_value;
float expectation_steering_value = 0;
float kp_steering = 0.9;   //0.25
int d_flag;
float d_steering_value;
float second_steering_error;
float kd_steering_value;
float caculate_steering_value;
float middle_steering_value;
float kd_steering = 0.15;
int LDC_ca;
int adjustment[10] = {0,0,0,0,0,0,0,0,0,0};
int flag = 0,flag1 = 0;
int LDC_val_pre;
int LDC_val1_pre;

//Motor_pid初始值
int p = 0.3,i = 0,d = 0.0001;
int AimSpeed = 50000,LastErr,PreErr,next;
  
//拨码开关
  uint8 MODE;
  uint8 MODE2;
  
  extern uint8 orgVal[12];
  extern LDC_ParameterPtr LDC_buff[50];
  //uint8 thr_data[2];
  //volatile uint32 irqflag=0;

  
//ENABLE_SET
#define STEERING_CHANGE_TEST_EN 0
#define LDC_INTB_EN 0                           //INTB模式
#define LDC_EVM_TEST_EN 1                       //是否开启test
#define PIT_EN 1
#define TESTSPACE_EN 1 							//是否开启测试区域，开启则工作区域关闭
#define SteeringTest_EN 0
#define MotorTest_EN 0
#define EncodeTest_EN 0

//STEERING
#define STEERING_MID -22000
#define STEERING_MAX 25000
#define STEERING_MIN 1
#define STEERING_ANGLE_MAX 22
#define STEERING_ANGLE_MIN -26
  
//MOTOR
#define MOTOR_FTM_N FTM0
#define MOTOR_FTM_CHN FTM_CH2
#define MOTOR_FREQUENCY 10000
#define MOTOR_DUTY 50000
#define SET_MAX 50000
  
////////////////////////////////////////////////////////////
void FTM0_INPUT_IRQHandler(void);
void FLOAT_delay(unsigned int ms);
void Pwm_set(uint8 pwm_val);
void PIT0_IRQHandler();
void Device_init();
void Steering_init();
void Motor_init();
void Encode_init();
void PIT_init();
uint32 angle_to_period(int8 deg);
void delay(uint16 n);
void Steering_Change();
void Motor_stop();
void Motor_PID_init();
void Motor_PID();
void PWM_output();


int Encode_get()
{
  int Encode_count;
  Encode_count = FTM_QUAD_get(FTM2);
  return Encode_count;
}


void Motor_PID(void)
{
  int tiffer,iErr;
  next = Encode_get();
  //if(iErr - LastErr <= -1 || iErr - PreErr >= 1) return next;
  iErr = AimSpeed - next;
  tiffer = p * iErr - i * LastErr + d * PreErr;
  PreErr = LastErr;
  LastErr = iErr;
  if(tiffer >= SET_MAX) tiffer = SET_MAX;
  FTM_PWM_Duty(FTM0,FTM_CH2,tiffer);
}

void Motor_stop()
{
	FTM_PWM_Duty(FTM0,FTM_CH2,0);
	FTM_PWM_Duty(FTM0,FTM_CH1,0);
}

void FLOAT_delay_us(int ms)//为防止time_delay_ms();与lpt冲突编写的延时
{
  int j1,k_1;int i1;
  i1=ms;
  for(j1=0;j1<i1;j1++)   
    for(k_1=0;k_1<8;k_1++);
}

void PIT_init()											
{
    
#if PIT_EN

  pit_init_ms(PIT0, 1);                          //定时 1000 个bus时钟 后中断

  set_vector_handler(PIT0_VECTORn,PIT0_IRQHandler);   // 设置中断复位函数到中断向量表里
    
  enable_irq(PIT0_IRQn);
#endif
}

void Steering_Change()
{
//  int i;
//  
//  printf("%d   \t\r\n",LDC_SPI1_val);
//  
//  if(flag)//右比左边大
//  {
//   LDC_SPI0_val=LDC_SPI0_val+adjustment[0];//左
//   if( LDC_SPI0_val<=((adjustment[8]+adjustment[0])-5)&&(LDC_val_pre-LDC_SPI1_val>8))//在左边丢线
//  //gpio_set (PTA17, 0);
//   {
//   printf("左边比右边大的时候：%d",LDC_SPI0_val);
//   LDC_result = STEERING_ANGLE_MIN;
//   FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result));
//   } 
//  else
//    
// // gpio_set (PTA17, 1);  
//    ;
//  
//   if( LDC_SPI1_val<=(adjustment[7]-5)&&(LDC_val1_pre-LDC_SPI0_val>8))//在右边丢线
//   {  LDC_result = STEERING_ANGLE_MAX;
//  FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result));
// // gpio_set (PTA17, 0);
//    
//   }
//   else
//   
//  //gpio_set (PTA17, 1);
//    ;
//  
// }
// else if(!flag)//左比右边大 flag1
//{
//  LDC_SPI1_val=LDC_SPI1_val+adjustment[0];//右
//  
//  if( LDC_SPI1_val<=((adjustment[7]+adjustment[0])-5)&&(LDC_val1_pre-LDC_SPI0_val>8))//在右边丢线
//  {//gpio_set (PTA17, 0);
//    printf("右边比左边大的时候：%d",LDC_SPI1_val);
//    LDC_result = STEERING_ANGLE_MAX;
//    FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result));
//  }
//  else
// // gpio_set (PTA17, 1);
//    ;
//  
//  if( LDC_SPI0_val<=(adjustment[8]-5)&&(LDC_val_pre-LDC_SPI1_val>8))//在左边丢线
//  { 
//
//    LDC_result = STEERING_ANGLE_MIN;
//  FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result));
//  }
//  else
//  //gpio_set (PTA17, 1); 
//    ;
// }
// LCD_BL(85,6,(uint16)LDC_SPI0_val);
// LCD_BL(3,6,(uint16)LDC_SPI1_val); 
//      
//   if(LDC_SPI0_val<adjustment[5]+5)LDC_SPI0_val=adjustment[5]+5;//右
//   else if(LDC_SPI0_val>adjustment[6]-5)LDC_SPI0_val=adjustment[6]-5;//
   
//   if(LDC_SPI1_val<adjustment[5]+5)LDC_SPI1_val=adjustment[5]+5;//左
//   else if(LDC_SPI0_val>adjustment[6]-5)LDC_SPI1_val=adjustment[6]-5;
#if STEERING_CHANGE_TEST_EN
        
//         FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(45));
	for(i=STEERING_ANGLE_MIN;i<STEERING_ANGLE_MAX;i+=5)
	{		
		FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(i));
		delay(100);
	}
        for(i=STEERING_ANGLE_MAX;i>STEERING_ANGLE_MIN;i-=5)
	{		
		FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(i));
		delay(100);
	}
#else
    LDC_result =LDC_SPI0_val - LDC_SPI1_val;
//     printf("%d   \t\r\n",LDC_SPI0_val);
//      printf("%d   \t\r\n",LDC_SPI1_val);
    frist_steering_error = expectation_steering_value + (float)LDC_result;	

    p_steering_value = kp_steering * frist_steering_error;

    if(d_flag == 0)                                                                     //预留D值开关
    {
      d_steering_value = (frist_steering_error - second_steering_error) * kd_steering;
      printf("%d\n\r",d_steering_value);
    }
    else
    {
      d_flag = 0;
      d_steering_value = 0;
    }
    
    caculate_steering_value = (p_steering_value + d_steering_value);
    second_steering_error = frist_steering_error;
     
    LDC_result = caculate_steering_value/16 - 5;
    if(LDC_result >= STEERING_ANGLE_MAX)
     {
                    LDC_result = STEERING_ANGLE_MAX;
     }
     if(LDC_result <= STEERING_ANGLE_MIN )
     {
                    LDC_result = STEERING_ANGLE_MIN;
     } 
    
    FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result));
//    printf("angle :%d\r\n",LDC_result);
//    printf("angle :%d\r\n",caculate_steering_value);
//    printf("d_steering_value:%d",d_steering_value);
//    printf("p_steering_value:%d",p_steering_value);
    
    LDC_val_pre=LDC_SPI0_val;
    LDC_val1_pre=LDC_SPI1_val;

#endif	
}

void Device_init()
{       
        gpio_init(PTC15,GPI,1);                                         //拨码开关第二位
        gpio_init(PTD2,GPI,1);                                          //拨码开关第一位
        gpio_init(PTD1,GPO,1);						//三个指示灯依次亮起
        gpio_init(PTD3,GPO,1);
        gpio_init(PTD4,GPO,1);						//每进入一次初始化PTD4灯灭，初始化完成亮起
        gpio_init(PTC5,GPO,1);

	FTM_PWM_init(FTM1,FTM_CH0,50,0);				//舵机初始化，从百分之十占空比到百分之七十占空比测试
	
	FTM_PWM_init(FTM0,FTM_CH1,10000,80000);			//电机pwm初始化
	FTM_PWM_init(FTM0,FTM_CH2,10000,80000);
	
	FTM_PWM_init(FTM2,FTM_CH0,3000000,75000);		//编码器初始化
	FTM_PWM_init(FTM2,FTM_CH1,3000000,75000);
    FTM_QUAD_Init(FTM2);							//初始化为正交解码
	
	PIT_init();
    gpio_set(PTD4,1);
}

uint32 angle_to_period(int8 deg)
{
  uint32 pulse_width = 5000+20000*(90+deg)/180;
  return (uint32)(100000*pulse_width/200000);
}

void TEST_mode()
{  
 
  MODE = gpio_get(PTD2);
  MODE2 = gpio_get(PTC15);

}

void main(void)
{
	int i1=0;
	
	Device_init();
        FLOAT_LDC_init(SPI1);
        systick_delay_ms(100);
        FLOAT_LDC_init(SPI0);
        systick_delay_ms(4000);
        uart_init(UART3,115200);
#if LDC_EVM_TEST_EN
     
     evm_test(SPI1); 
     
//     Set_reg(SPI1);
     
//     Reset_buff();
     
     evm_test(SPI0);  
     
//     Set_reg(SPI0);
#endif     
	i1 = 100;
     while(i1--)
     {
      adjustment[7]=filter(SPI0)/10;//右
      adjustment[8]=filter(SPI1)/10;//左
     }
	 
	  if(adjustment[7]>=adjustment[8])
     { flag=1;
     adjustment[0]=adjustment[7]-adjustment[8];
     }
     else
     {
       flag1=1;
     adjustment[0]=adjustment[8]-adjustment[7]; 
     }
     
     
      
      LCD_P6x8Str(0,2,"Rmax Lmin");
      printf("Rmax Rmin\r\n");
      systick_delay_ms(3000);
      i1=100;
      while(i1--)
     {
         adjustment[2]=filter(SPI1)/10;//右max
         printf("%d\r\n",adjustment[2]);
     }
      i1=100;
      while(i1--)
     {
        adjustment[1]=filter(SPI0)/10;//左min
        printf("%d\r\n",adjustment[1]);
     }
     LCD_BL(55,2,(uint16)adjustment[1]);
     LCD_BL(90,2,(uint16)adjustment[2]);
  

  
      LCD_P6x8Str(0,4,"Lmax Rmin");
      printf("Lmax Rmin\r\n");
      systick_delay_ms(3000);
      i1=100;
      while(i1--)
     {
      adjustment[3]=filter(SPI1)/10;//右min
      printf("%d\r\n",adjustment[3]);
     }
      i1=100;
      while(i1--)
     {
      adjustment[4]=filter(SPI0)/10;//左max
      printf("%d\r\n",adjustment[4]);
     }
     LCD_BL(55,4,(uint16)adjustment[4]);
     LCD_BL(90,4,(uint16)adjustment[3]);
     
     if(adjustment[1]<=adjustment[3])
       adjustment[5]=adjustment[1];//次小值
     else
       adjustment[5]=adjustment[3];//次小值
     
     if(adjustment[2]>=adjustment[4])
       adjustment[6]=adjustment[2];//次大值
     else
       adjustment[6]=adjustment[4];//次大值
     

     for(int i=0;i<10;i++)
     {
       printf("%d\r\n",adjustment[i]);
     }
     systick_delay_ms(3000);
	while(1)
	{
             TEST_mode();
             if(MODE)
             {
                LDC_SPI0_val = filter(SPI0)/10;
                systick_delay_ms(0);
                LDC_SPI1_val = filter(SPI1)/10;
                printf("ldc0_val");
                printf("%d   \t",LDC_SPI0_val);
                printf("ldc1_val");
                printf("%d   \t",LDC_SPI1_val);
                LDC_result = LDC_SPI0_val-LDC_SPI1_val;
                printf("%d\r\n",LDC_result);
                Steering_Change();
                systick_delay_ms(0);
             }
             else
             {
                 FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPI0);//orgVal[]对应上面写入的值说明初始化正常
               
                 for(int i=0;i<3;i++)
                {
                  printf("%c0",orgVal[i]);
                }
                systick_delay_ms(150);
                
                  FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPI1);//orgVal[]对应上面写入的值说明初始化正常
               
                 for(int i=0;i<3;i++)
                {
                  printf("%c1",orgVal[i]);
                }
                systick_delay_ms(150);
             }
                
                if(PIT_1sFlag == 1)
		{
                      gpio_turn(PTD4);
                      PIT_1sFlag = 0;
		}
     
	}
}




/* 
PIT计时
2ms，5ms,10ms,20ms,50ms,1s
 */

void pit0_isr()
{
  if(PIT_TFLG(PIT0) == 1)
  {
	if(++PIT_2msCount >= PIT_2MS_CONSTON)
	{             
		PIT_2msCount = 0;
		PIT_2msFlag = 1;
	}
	if(++PIT_5msCount >= PIT_5MS_CONSTON)
	{
		PIT_5msCount = 0;
		PIT_5msFlag = 1;
		PIT_steeringFlag = 1;
		PIT_motorFlag = 1;
	}
	if(++PIT_10msCount >= PIT_10MS_CONSTON)
	{
		PIT_10msCount = 0;
		PIT_10msFlag = 1;
	}
	if(++PIT_20msCount >= PIT_20MS_CONSTON)
	{
		PIT_20msCount = 0;
		PIT_20msFlag = 1;
	}
	if(++PIT_50msCount >= PIT_50MS_CONSTON)
	{
		PIT_50msCount = 0;
		PIT_50msFlag = 1;
	}
	if(++PIT_1sCount >= PIT_1S_CONSTON)
	{
							//每秒闪烁一下表示，PIT正常工作
		PIT_1sCount = 0;
		PIT_1sFlag = 1;
	}
	
    
  }
}

void PIT0_IRQHandler()									
{
  pit0_isr();
  PIT_Flag_Clear(PIT0);
}


void delay(uint16 n)
{
  uint16 i;
  while(n--)
  {
    for(i=0; i<5000; i++)
    {
      asm("nop");
    }
  }
}