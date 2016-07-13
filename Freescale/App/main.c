
#include "common.h"
#include "include.h"
#include "MK60_uart.h"
#include "MK60_port.h"
#include "MK60_FTM.h"
#include "MK60_PIT.h"
#include "LDC1000.H"
#include "calculate.h"
#include "Virtual_Oscilloscope.h"

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




//正交解码
int guad_val = 0;

//LDC_val采样值
uint8 RPMIN_tmp_SPI0 =0;                      //TI 14 
uint8 RPMIN_tmp_SPI1 =0;                      //TI 3B
extern uint8 orgVal[12];
extern int  LDC_val;
float  LDC_SPI0_val;                                    //int
float  LDC_SPI1_val;                                    //int
float  LDC_result;
float first_Steering_Error;
float p_Steering_Value;
float expectation_Steering_Value = 0;
float kp_Steering = 0.15;   //0.25 0.8
float d_flag;
float d_Steering_Value;
float second_Steering_Error;
float kd_Steering_Value;
float caculate_Steering_Value;
float middle_Steering_Value;
float kd_Steering = 0.27;
float left_Lost;
unsigned char state;
#define left_add 50
float right_Lost;
#define right_add 50
int LDC_ca;
float adjustment[10] = {0,0,0,0,0,0,0,0,0,0};
float flag = 0,flag1 = 0;
float LDC_SPI0_pre;
float LDC_SPI1_pre;
float divisor = 0;
float divisor2 = 0;
float divisor_end = 0;
float LDC_cha[2] = {0};
unsigned int LDC_SPI0_COUNT = 0;
unsigned int LDC_SPI1_COUNT = 0;
#define LDC_MAX 5000

//Motor_pid初始值
float p = 0.8,i =0.1,d = 1, imax =  1;
float AimSpeed = 1200;
float mid = 0;
float next = 0, iErr = 0, LastErr = 0, PreErr = 0, tiffer = 0;

float Result_list[5] = { 0 };
float result_abs = 0;
float result_pre = 0;


uint32 Motor_result = 0;

//拨码开关
  uint8 MODE;
  uint8 MODE2;
  uint8 Stop_flag;
  
  extern uint8 orgVal[12];
  extern LDC_ParameterPtr LDC_buff[50];
  //uint8 thr_data[2];
  //volatile uint32 irqflag=0;

  
//ENABLE_SET
#define STEERING_CHANGE_TEST_EN 0
#define LDC_INTB_EN 0                           //INTB模式
#define LDC_EVM_TEST_EN 0                       //是否开启test
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
#define STEERING_MIN_FLAG 100
#define ANGLE_LEFT_MIN -7
#define ANGLE_RIGHT_MIN 7
#define ANGLE_LEFT_MAX -20
#define ANGLE_RIGHT_MAX 12
//MOTOR
#define MOTOR_FTM_N FTM0
#define MOTOR_FTM_CHN FTM_CH2
#define MOTOR_FREQUENCY 10000
#define MOTOR_DUTY 50000
#define SET_MAX 1600
  
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
float angle_to_period(float deg);
void delay(uint16 n);
void Steering_Change();
void Motor_stop();
void Motor_PID_init();
void Motor_PID();
void PWM_output();
void Quad_count();
void TEST_display();
void LDC_get();
void Virtual_Osc(void);
void Result_collect();
void key_choice();
float Abs(float a);

void Motor_PID(void)
{
  next = (uint32)guad_val;
  //if(iErr - LastErr <= -1 || iErr - PreErr >= 1) return next;
  iErr = AimSpeed - next; 
  LastErr = iErr - LastErr;
  PreErr = iErr - 2 * LastErr + PreErr;
  tiffer = p * iErr + i * LastErr + d * PreErr;
  tiffer = tiffer * 60;
  if(tiffer >= 60000) tiffer = 60000;
  if(tiffer >= AimSpeed) tiffer = AimSpeed;
  PreErr = LastErr;
  LastErr = iErr;
  //printf("encode=%d\r\n",(int)guad_val);
  //FTM_PWM_Duty(FTM0,FTM_CH2,tiffer*120);
  //FTM_PWM_Duty(FTM0,FTM_CH2,60000);
  //systick_delay_ms(5000);
  FTM_PWM_Duty(FTM0,FTM_CH2,tiffer);
  //systick_delay_ms(1000);
//  FTM_PWM_Duty(FTM0,FTM_CH2,80000);
//  systick_delay_ms(1000);
  //printf("tiffer: %d\n\r",(int)tiffer);
}

void Motor_ctl()
{
  Motor_result = (uint32)Speed_Smooth_Ctrl((signed int)(mid));
  if(Motor_result >= 1600)Motor_result = 1600;
  FTM_PWM_Duty(FTM0,FTM_CH2,Motor_result * 60);//(uint32)Speed_Smooth_Ctrl((signed int)
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
    

  disable_irq(PIT0_IRQn);
#endif
}

void Steering_Change()
{
	if(flag) LDC_SPI1_val += adjustment[0];
        else LDC_SPI0_val += adjustment[0];
        
	LDC_result = LDC_SPI0_val - LDC_SPI1_val;
        
        if(LDC_result > 0) divisor_end = divisor2;
        else divisor_end = divisor;
        
        first_Steering_Error = LDC_result;
	p_Steering_Value = kp_Steering * first_Steering_Error;
	d_Steering_Value = (first_Steering_Error - second_Steering_Error) * kd_Steering;
	caculate_Steering_Value = p_Steering_Value + d_Steering_Value;
	
        LDC_result = caculate_Steering_Value / divisor_end;
	switch (state)
	{
                            case 0:
                            if (LDC_result > ANGLE_LEFT_MIN && LDC_result < ANGLE_RIGHT_MIN)
                            {
                                    //kp_Steering = 0.85;
                                    //SetSpeed_speed(1000);
                            }
                                    else
                            {
                                    state++;
                                    //kp_Steering = 0.85;
                                    //SetSpeed_speed(500);
                                    FTM_PWM_Duty(FTM0,FTM_CH2,50000);
                            }
                                    break;

                            case 1:
                            if (LDC_result > ANGLE_LEFT_MIN && LDC_result < ANGLE_RIGHT_MIN) state = 0;
                            else if(LDC_result > ANGLE_RIGHT_MAX) state = 0;
                            else if(LDC_result < ANGLE_LEFT_MAX) state = 0;
                            FTM_PWM_Duty(FTM0,FTM_CH2,50000);
                            //SetSpeed_speed(500);
                            break;
                            case 2:
                              if(LDC_result > ANGLE_LEFT_MAX)
                              {
                                state = 0;
                              }
                              else 
                              {
                                LDC_result = STEERING_ANGLE_MIN;
//                                if(LDC_SPI0_val <= adjustment[8] && LDC_SPI1_val <= adjustment[7])
//                                  LDC_result = Result_list[0];
                              }
                              //SetSpeed_speed(500);
                              break;
                              
                            case 3:
                              if(LDC_result > ANGLE_RIGHT_MAX) 
                              { 
                                state = 0;
                              }
                              else 
                              {
                                LDC_result = STEERING_ANGLE_MAX;
//                                if(LDC_SPI0_val <= adjustment[8] && LDC_SPI1_val <= adjustment[7])
//                                  LDC_result = Result_list[0];
                              } 
                              //SetSpeed_speed(500);
                              break;
	}
	FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result+1));
	LCD_BL(0,0,(uint16)state);
	LCD_BL(30,0,(uint16)Abs(LDC_result));
        result_pre = LDC_result;
}

float Abs(float a)
{
  if(a < 0) return -a;
  else return a;
}

void Result_collect()
{
        Result_list[4] = Result_list[3];
        result_abs += Abs(Result_list[4] - Result_list[3]);
        Result_list[3] = Result_list[2];
        result_abs += Abs(Result_list[3] - Result_list[2]);
        Result_list[2] = Result_list[1];
        result_abs += Abs(Result_list[2] - Result_list[1]);
        Result_list[1] = Result_list[0];
        result_abs += Abs(Result_list[1] - Result_list[0]);
        Result_list[0] = LDC_result;
        result_abs = result_abs/5;
        LCD_BL(70,0,(uint16)result_abs*10);
        LCD_BL(0,2,(uint16)Result_list[1]);
}


void Device_init()
{       
        gpio_init(PTC15,GPI,1);                                         //拨码开关第二位
        gpio_init(PTD2,GPI,1);                                          //拨码开关第一位
        gpio_init(PTD1,GPO,1);						//三个指示灯依次亮起
        gpio_init(PTD3,GPO,1);
        gpio_init(PTD4,GPO,1);						//每进入一次初始化PTD4灯灭，初始化完成亮起
        gpio_init(PTC5,GPO,1);
        gpio_init(PTC4,GPI,0);                                          //干簧管

	FTM_PWM_init(FTM1,FTM_CH0,50,0);				//舵机初始化，从百分之十占空比到百分之七十占空比测试
	
	FTM_PWM_init(FTM0,FTM_CH1,10000,0);			//电机pwm初始化
	FTM_PWM_init(FTM0,FTM_CH2,10000,0);
	
	//FTM_PWM_init(FTM2,FTM_CH0,3000000,93000);		//编码器初始化
	//FTM_PWM_init(FTM2,FTM_CH1,3000000,93000);
          FTM_QUAD_Init(FTM2);							//初始化为正交解码
	LCD_Init();
     
     PIT_init();
    gpio_set(PTD4,1);
}

float angle_to_period(float deg)
{
  float pulse_width = 5000+20000*(90+deg)/180;
  return (100000*pulse_width/200000);
}



void TEST_mode()
{  
  Stop_flag = gpio_get(PTC4);
  MODE = gpio_get(PTD2);
  MODE2 = gpio_get(PTC15);
  printf("%d\r\n",(int)Stop_flag);
}

void LDC_EVM_TEST()
{
   evm_test(SPI0);
   evm_test(SPI1);
}

void main(void)
{
	int i1=0;
	
	Device_init();
        disable_irq(PIT0_IRQn);
        systick_delay_ms(50);
        FLOAT_LDC_init(SPI1);
        systick_delay_ms(50);
        FLOAT_LDC_init(SPI0);
        systick_delay_ms(1000);
        uart_init(UART3,115200);
        key_init(0);
        key_init(1);
        key_init(2);
        Speed_Ctl_Init(AimSpeed, p, i, d, imax);  
     
      while(1)
      {
        LCD_P6x8Str(0,2,"Rmax Lmin");
        printf("Rmax Rmin\r\n");
        systick_delay_ms(1500);
        i1=50;
        
        while(i1--)
       {
           adjustment[2]=(float)filter(SPI1)/10;//右max
           LCD_BL(55,2,(uint16)adjustment[2]);
           printf("%d\r\n",adjustment[2]);
           adjustment[9] += adjustment[2];
       }
        
       i1=50;
       while(i1--)
       {
          adjustment[1]=(float)filter(SPI0)/10;//左min
          LCD_BL(90,2,(uint16)adjustment[1]);
          printf("%d\r\n",adjustment[1]);
       }
       
       LCD_BL(55,2,(uint16)adjustment[1]);
       LCD_BL(90,2,(uint16)adjustment[2]);
    

    
       LCD_P6x8Str(0,4,"Lmax Rmin");
       printf("Lmax Rmin\r\n");
       systick_delay_ms(1500);
       i1=50;
       while(i1--)
       {
       adjustment[3]=(float)filter(SPI1)/10;//右min
       LCD_BL(55,4,(uint16)adjustment[3]);
       printf("%d\r\n",adjustment[3]);
       }
        
       i1=50;
       while(i1--)
       {
        adjustment[4]=(float)filter(SPI0)/10;//左max
        LCD_BL(90,4,(uint16)adjustment[4]);
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
       
       divisor = (adjustment[2] - adjustment[3])/68;                              //48
       divisor2 = (adjustment[4] - adjustment[1])/48;                             //48
       
//       for(int i=0;i<10;i++)
//       {
//         printf("%d\r\n",adjustment[i]);
//       }
//       systick_delay_ms(1000);
       
       
          
       LCD_P6x8Str(0,6,"Mid value");
        i1 = 50;
       systick_delay_ms(1500);
       while(i1--)
       {
        adjustment[7]=(float)filter(SPI0)/10;//右
        adjustment[8]=(float)filter(SPI1)/10;//左
        LCD_BL(55,6,(uint16)adjustment[7]);
        LCD_BL(90,6,(uint16)adjustment[8]);
       }
            if(adjustment[7]>=adjustment[8])
       { flag=1;
       adjustment[0]=adjustment[7]-adjustment[8];
       }
       else
       {
         flag=0;
       adjustment[0]=adjustment[8]-adjustment[7]; 
       }
       
       LCD_Fill(0x00);
       
       enable_irq(PIT0_IRQn);
          while(1)
          { 
                  if(PIT_1sFlag == 1)
                  {
                        gpio_turn(PTD4);
                        PIT_1sFlag = 0;
                  }
                  if(key_check(2) == 0)
                  {
                    LCD_Fill(0x00);
                    Motor_stop();
                    break;
                  }
                  else
                  {
                    if(PIT_5msFlag == 1)
                    {
                      LDC_get();
                      PIT_5msFlag = 0;
                    }
                    if(PIT_10msFlag == 1)
                    {
                      
                      PIT_10msFlag = 0;
                    }
                    if(PIT_20msFlag == 1)
                    {
                      Steering_Change();
                      PIT_20msFlag = 0;
                    }
                    if(PIT_50msFlag == 1)
                    {
                      Quad_count();
                      PIT_50msFlag = 0;
                    }
                    Motor_ctl();
                    //Virtual_Osc();
                    if(kd_Steering < 0) LCD_P6x8Str(67,2,"-");
                    else LCD_P6x8Str(67,2,"  ");
                    LCD_BL(70,2,(uint16)(Abs(kd_Steering)*100));
                    
                    TEST_display();
                    
                    key_choice();
                  
                //    Motor_PID();
//                    if(gpio_get(PTC4) == 0)
//                    {
//                      while(1)
//                      {
//                        Motor_stop();
//                      }
//                    }
                  }
          }
      }
}

void Quad_count()
{
    guad_val =  FTM_QUAD_get(FTM2);
    FTM_QUAD_clean(FTM2);

}

void TEST_display()
{
	TEST_mode();
             if(MODE)
             {
				printf("ldc0_val");
				printf("%d   \t",(int)LDC_SPI0_val);
				printf("ldc1_val");
				printf("%d   \t",(int)LDC_SPI1_val);
				printf("%d\r\n",(int)LDC_result);
                                LCD_P6x8Str(0,4,"ldc0_val:");
                                LCD_BL(55,4,(uint16)LDC_SPI0_val);
                                LCD_P6x8Str(0,6,"ldc1_val:");
                                LCD_BL(55,6,(uint16)LDC_SPI1_val);
             }
             else
             {
                 FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPI0);//orgVal[]对应上面写入的值说明初始化正常
               
                 for(int i=0;i<3;i++)
                {
                  printf("%c0",orgVal[i]);
                  LCD_BL(0+(i*10),8,(uint16)orgVal[i]);
                }
                RPMIN_tmp_SPI0 = orgVal[2];
                systick_delay_ms(150);
                
                FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPI1);//orgVal[]对应上面写入的值说明初始化正常
               
                 for(int i=0;i<3;i++)
                {
                  printf("%c1",orgVal[i]);
                  LCD_BL(0+(i*10),9,(uint16)orgVal[i]);
                  Motor_stop();
                }
                systick_delay_ms(150);
             }
}

void key_choice()
{
  if(key_check(0)==0)           //检测PB16按键按下
                {
//                  orgVal[2]++;
//                  RPMIN_tmp_SPI0++;
//                 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x00,SPI1);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX,orgVal[2],SPI1);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x01,SPI1);
//                 systick_delay_ms(50);
//                 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x00,SPI0);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX,RPMIN_tmp_SPI0,SPI0);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x01,SPI0);
                  kd_Steering+=0.005;
                }
                if(key_check(1)==0)           //检测PB17按键按下
                {
                  kd_Steering-=0.005;
//                   orgVal[2]--;
//                 RPMIN_tmp_SPI0--;
//                 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x00,SPI1);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX,orgVal[2],SPI1);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x01,SPI1);
//                 systick_delay_ms(50);
//                 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x00,SPI0);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX,RPMIN_tmp_SPI0,SPI0);
//		 FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x01,SPI0);
                }
}

void Virtual_Osc(void)
{
  OutData[0] = (float)guad_val;
  OutData[1] = LDC_SPI0_val;
  OutData[2] = LDC_SPI1_val;
  OutData[3] = (float)mid;
  OutPut_Data();
}

void LDC_get()
{
	LDC_SPI0_val = (float)filter(SPI0)/10;
	systick_delay_ms(0);
	LDC_SPI1_val = (float)filter(SPI1)/10;
	LDC_result = LDC_SPI0_val-LDC_SPI1_val;
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
                //Motor_ctl();
                //Motor_PID();
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
                mid = Speed_Ctrl((float)guad_val);
		PIT_50msCount = 0;
		PIT_50msFlag = 1;
                Result_collect();
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