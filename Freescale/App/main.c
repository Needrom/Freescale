
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


//PIN_DEFINE
#define LED_PIN PTD4							//led引脚
#define STOP_CATCH_PIN 							//停止flag捕获

//LDC_val采样值
extern uint8 orgVal[12];
extern int  LDC_val;
int  LDC_SPI0_val;
int  LDC_SPI1_val;
int  LDC_result;
int frist_steering_error;
int p_steering_value;
int expectation_steering_value = 1041;
int p_steering_value;
int kp_steering = 1;
int d_flag;
int d_steering_value;
int second_steering_error;
int kd_steering_value;
int caculate_steering_value;
int middle_steering_value;
int fg;
int kd_steering;
int Data_average;

  extern uint8  proximtyData[2];
  uint8  freqData[2];
  uint16 freqData_Sum;
  uint8  LDC_Val_Compar_For_Mid;
  
//拨码开关
  uint8 MODE;
  uint8 MODE2;
  
  extern uint8 orgVal[12];
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
#define LDC100_Data_RECALL_TEST_EN 0 
#define LDC100_Reg_RECALL_TEST_EN 1 
  
//STEERING
#define STEERING_MID -22000
#define STEERING_MAX 25000
#define STEERING_MIN 1
#define STEERING_FTM_N  NULL
#define STEERING_FTM_CHN    NULL
#define STEERING_FREQUENCY      NULL
#define STEERING_DUTY   NULL
#define STEERING_ANGLE_MAX 22
#define STEERING_ANGLE_MIN -26
  
//MOTOR
#define MOTOR_FTM_N FTM0
#define MOTOR_FTM_CHN FTM_CH2
#define MOTOR_FREQUENCY 10000
#define MOTOR_DUTY 50000
  
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

/* 
舵机初始化，从百分之十占空比到百分之七十占空比测试
 */
void Steering_init()							
{
        gpio_set(PTD4,0);
	FTM_PWM_init(FTM1,FTM_CH0,50,0);

#if SteeringTest_EN
	
	int duty;
	int dutyMax = 100;
	
	printf("Steering TEST start in 3 second");
	FLOAT_delay_us(3000);
	
	for (int duty = 10; duty < dutyMax; ++duty)
	{
		FTM_PWM_Duty(FTM1,FTM_CH0,duty*1000);
		printf("%d \r\n", duty);
		FLOAT_delay_us(30);
	}
#endif
        gpio_set(PTD4,1);
}




void Motor_init()
{
    gpio_set(PTD4,0);
        FTM_PWM_init(FTM0,FTM_CH1,10000,75000);
	FTM_PWM_init(FTM0,FTM_CH2,10000,75000);
        FTM_PWM_Duty(FTM0,FTM_CH1,75000);
#if MotorTest_EN
	
        FTM_QUAD_Init(FTM2);
        
        int dutyCatch;
	int duty;
	int dutyMax = 100;
	
	printf("Motor TEST start in 3 second \r\n");
	systick_delay_ms(3000);
	
	printf("Pwm1 START \r\n");
	for (int duty = 10; duty < 50; ++duty)						//pwm1百分十到百分百测试
	{
		FTM_PWM_Duty(FTM0,FTM_CH1,duty*1000);
                dutyCatch = FTM_QUAD_get(FTM2);
		printf("%d \r\n", duty);
		FLOAT_delay_us(30);
	}

	FTM_PWM_Duty(FTM1,FTM_CH0,0);									//pwm1归零

	printf("Pwm2 START \r\n");

	for (int duty = 10; duty < dutyMax; ++duty)
	{
		FTM_PWM_Duty(FTM0,FTM_CH2,duty*1000);
		printf("%d \r\n", duty);
		FLOAT_delay_us(30);
	}

	FTM_PWM_Duty(FTM0,FTM_CH2,0);
#endif
        gpio_set(PTD4,1);
}

void Encode_init()													//编码器
{
        gpio_set(PTD4,0);
	FTM_PWM_init(FTM2,FTM_CH0,3000000,75000);
	FTM_PWM_init(FTM2,FTM_CH1,3000000,75000);
#if EncodeTest_EN

#endif
        gpio_set(PTD4,1);
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
  int i;
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
    LDC_result = filter(SPI0) - filter(SPI1);

    frist_steering_error = expectation_steering_value - LDC_result;	

    p_steering_value = kp_steering * frist_steering_error;

    if(d_flag == 0)
    {
      d_steering_value = (frist_steering_error - second_steering_error) * kd_steering;
    
    }
    else
    {
      d_flag = 0;
      d_steering_value = 0;
    }
    
    caculate_steering_value = (p_steering_value + d_steering_value);
    second_steering_error = frist_steering_error;
     
//    if(fg == 3)
//    {
//            if(LDC_result == 0)
//            caculate_steering_value = STEERING_MAX;
//            else
//            caculate_steering_value = STEERING_MIN;	
//            fg = 0;
//    }
//      if(fg == 2)		left
//    {
//            caculate_steering_value = 3425 - ad_resultk;	//2275
//            fg = 0;
//            d_flag = 1;
//    }
//     if(fg == 1)		right
//    {
//            caculate_steering_value = 2275 + ad_resultk;	//3425
//            fg = 0;
//            d_flag = 1;	
//    }
    LDC_result = caculate_steering_value/600;
    if(LDC_result >= STEERING_ANGLE_MAX)
     {
                    LDC_result = STEERING_ANGLE_MAX;
     }
     if(LDC_result <= STEERING_ANGLE_MIN )
     {
                    LDC_result = STEERING_ANGLE_MIN;
     } 
    
    FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result));
    printf("angle :%d\r\n",LDC_result);
#endif	
}

void Device_init()
{       
        gpio_init(PTC15,GPI,1);                                         //拨码开关第二位
        gpio_init(PTD2,GPI,1);                                          //拨码开关第一位
        gpio_init(PTD1,GPO,1);						//三个指示灯依次亮起
        gpio_init(PTD3,GPO,1);
        gpio_init(PTD4,GPO,1);						//每进入一次初始化PTD4灯灭，初始化完成亮起
	Steering_init();
	Motor_init();
	Encode_init();
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
	Device_init();
        FLOAT_LDC_init(SPI1);
        systick_delay_ms(100);
        FLOAT_LDC_init(SPI0);
        systick_delay_ms(4000);
        uart_init(UART3,115200);
#if LDC_EVM_TEST_EN
     
     evm_test(SPI1); 
     
     evm_test(SPI0);    
#endif     
//     EnableInterrupts;
	while(1)
	{
             TEST_mode();
             if(MODE)
             {
                LDC_SPI0_val = filter(SPI0);
                systick_delay_ms(60);
                LDC_SPI1_val = filter(SPI1);
                printf("ldc0_val");
                printf("%d   \t",LDC_SPI0_val);
                printf("ldc1_val");
                printf("%d   \t",LDC_SPI1_val);
                LDC_result = LDC_SPI0_val-LDC_SPI1_val;
                printf("%d\r\n",LDC_result);
                Steering_Change();
                systick_delay_ms(30);
                
//                if(MODE2)
//                {
//                Data_average = average(LDC_SPI0_val);
//                if(Data_average) printf("Data0_average is :%d\r\n",Data_average);
//                }
//                else
//                {
//                Data_average = average(LDC_SPI1_val);
//                if(Data_average) printf("Data1_average is :%d\r\n",Data_average);
//                }
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
                //printf("Runing");
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