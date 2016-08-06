
#include "common.h"
#include "include.h"
#include "MK60_uart.h"
#include "MK60_port.h"
#include "MK60_FTM.h"
#include "MK60_PIT.h"
#include "LDC1000.H"
#include "calculate.h"
#include "Virtual_Oscilloscope.h"
//#include "motor_speed_pid.h"

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
float kp_Steering = 0.12;   //0.25 0.8
float d_flag;
float d_Steering_Value;
float second_Steering_Error;
float kd_Steering_Value;
float caculate_Steering_Value;
float middle_Steering_Value;
float kd_Steering = 0.1539;                                       //0.1614 1539   //2619
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
float ldc0_side = 0;
float ldc1_side = 0;
float ldcValuetemp0[100] = 0;
float ldcValuetemp1[100] = 0;
unsigned int LDC_SPI0_COUNT = 0;
unsigned int LDC_SPI1_COUNT = 0;
#define LDC_MAX 5000


//Motor_pid初始值/************************************************************/
float p = 70.5,i =78.4,d = 40.0;//float p = 70.5,i =78.4,d = 40.0;
float AimSpeed = 450.0;
float now = 0.0, thisErr = 0.0, LastErr = 0.0, PreErr = 0.0, out = 0.0;
float pErr = 0.0, iErr = 0.0, dErr = 0.0;

float Result_list[5] = { 0 };
float result_abs = 0;
float result_pre = 0;

float zengfuzhi= 0;


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
#define STEERING_ANGLE_MAX 20
#define STEERING_ANGLE_MIN -24
#define STEERING_MIN_FLAG 100
#define ANGLE_LEFT_MIN -13
#define ANGLE_RIGHT_MIN 9
#define ANGLE_LEFT_MAX -15
#define ANGLE_RIGHT_MAX 7
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
void Left_Out_Side();
void Right_Out_Side();
void Out_side_collect();

void Motor_PID()
{
  //if(iErr - LastErr <= -1 || iErr - PreErr >= 1) return next;
  thisErr = now - AimSpeed; 
  pErr = LastErr - thisErr;
  iErr = thisErr;
  dErr = thisErr - 2 * LastErr + PreErr;
  out += p * pErr - i * iErr + d * dErr;
  if(out >= 75000) out = 75000;
  PreErr = LastErr;
  LastErr = thisErr;
  //printf("encode=%d\r\n",(int)guad_val);
  //FTM_PWM_Duty(FTM0,FTM_CH2,tiffer*120);
  //FTM_PWM_Duty(FTM0,FTM_CH2,60000);
  //systick_delay_ms(5000);
  FTM_PWM_Duty(FTM0,FTM_CH2,out);
  //systick_delay_ms(1000);
//  FTM_PWM_Duty(FTM0,FTM_CH2,80000);
//  systick_delay_ms(1000);
  //printf("tiffer: %d\n\r",(int)tiffer);
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

void Set_SteeringPD(float PP,float DD)
{
  kp_Steering = PP;
  kd_Steering = DD;
}

void Steering_Change()
{
  
	if(flag) LDC_SPI1_val += adjustment[0];
        else LDC_SPI0_val += adjustment[0];
        
	LDC_result = LDC_SPI0_val - LDC_SPI1_val;
        
        if(LDC_SPI0_val > adjustment[6]+adjustment[0]-60 && LDC_SPI1_val > adjustment[6]+adjustment[0]-60)
        {
          Set_SteeringPD(0.1200,0.00);
        }
        else
        {
          if(LDC_result > 0) 
          {
            //Set_SteeringPD(0.1200,0.1553);
            divisor_end = divisor2;
          }
          else
          {
            //Set_SteeringPD(0.1200,0.2813);
            divisor_end = divisor;
          }
        }
        
        
        first_Steering_Error = LDC_result;
	p_Steering_Value = kp_Steering * first_Steering_Error;
	d_Steering_Value = (first_Steering_Error - second_Steering_Error) * kd_Steering;
	caculate_Steering_Value = p_Steering_Value + d_Steering_Value;
	
        LDC_result = caculate_Steering_Value / divisor_end;
        
        
//        switch(state)
//        {
//        case 0:
////          if(LDC_result > ANGLE_LEFT_MIN && LDC_result < ANGLE_RIGHT_MIN)
////          {
////              //AimSpeed = 300.0;
////          }          
////          else state = 1;
//          
//          if(LDC_result > ANGLE_LEFT_MIN && LDC_result < ANGLE_RIGHT_MIN)
//            state = 0;
//          if(LDC_result <= ANGLE_LEFT_MIN) state = 2;
//          if(LDC_result >= ANGLE_RIGHT_MIN) state = 3;
//            //AimSpeed = 300.0;
//          
//          break;
//        case 1:
//          if(LDC_result > ANGLE_LEFT_MIN && LDC_result < ANGLE_RIGHT_MIN)
//            state = 0;
//          if(LDC_result <= ANGLE_LEFT_MIN) state = 2;
//          if(LDC_result >= ANGLE_RIGHT_MIN) state = 3;
//            //AimSpeed = 300.0;
//          break;
//        case 2:
//          if(LDC_result > ANGLE_LEFT_MIN && LDC_result <ANGLE_RIGHT_MIN)
//            state = 0;
//          //AimSpeed = 400.0;
//          Left_Out_Side();
//          break;
//        case 3:
//          if(LDC_result > ANGLE_LEFT_MIN && LDC_result <ANGLE_RIGHT_MIN)
//            state = 0;
//          //AimSpeed = 300.0;
//          Right_Out_Side();
//          //Set_SteeringPD(0.1254,0.3689);
//          break;
//        }
        
        Left_Out_Side();
        Right_Out_Side();

        if(LDC_result >= STEERING_ANGLE_MAX) LDC_result = STEERING_ANGLE_MAX;
        else if(LDC_result <= STEERING_ANGLE_MIN) LDC_result = STEERING_ANGLE_MIN;
        
	FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(LDC_result+1));
        
	//LCD_BL(0,0,(uint16)Abs(state));
	//LCD_BL(30,0,(uint16)Abs(LDC_result));
        
        result_pre = LDC_result;
        LDC_SPI0_pre = LDC_SPI0_val;
        LDC_SPI1_pre = LDC_SPI1_val;
}

void Out_side_Motor_ctl()
{
  if(PIT_10msFlag == 1)
  {
    Quad_count();
    now = (float)guad_val;
    Motor_PID(now);
    PIT_10msFlag = 0;
  }
}

void Right_Out_Side()
{
  //if(LDC_SPI1_val < 1756 && LDC_SPI0_val < 1907 && LDC_SPI0_pre > LDC_SPI0_val)
  if(LDC_SPI0_val > adjustment[4]-60 && LDC_SPI1_val < (adjustment[3]+20))                     //90      70                      //左边 小于1748是说明 左边在铝膜上 不打死  右边小于1906 且 值在变小 说明 丢线了
  {
    FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(STEERING_ANGLE_MAX));
    //LCD_P6x8Str(90,6,"ROut");
    while(1)
    {
      LDC_get();
      
      
      if(flag) LDC_SPI1_val += adjustment[0];
      else LDC_SPI0_val += adjustment[0];
      //TEST_display();
      Out_side_Motor_ctl();
      //if(LDC_SPI0_val > 2240 | LDC_SPI1_val > 1740)
      if( LDC_SPI1_val > adjustment[3]+30)                                                      //70
      {
//        LCD_Fill(0x00);
        break;
      }
    }
  }
}

void Left_Out_Side()
{
  //if(LDC_SPI0_val < ldc0_side && LDC_SPI1_val < adjustment[8]+adjustment[0]+150 && LDC_SPI1_pre > LDC_SPI1_val)                                                 //左边 小于1748是说明 左边在铝膜上 不打死  右边小于1906 且 值在变小 说明 丢线了
  
  if(LDC_SPI0_val < (adjustment[1]+20) && LDC_SPI1_val > adjustment[2]-60)                   //90  //70                            //左边 小于1748是说明 左边在铝膜上 不打死  右边小于1906 且 值在变小 说明 丢线了
  {
    //LCD_P6x8Str(86,6,"leftOut");
    FTM_PWM_Duty(FTM1,FTM_CH0,angle_to_period(STEERING_ANGLE_MIN));
    while(1)
    {
      LDC_get();
      if(flag) LDC_SPI1_val += adjustment[0];
      else LDC_SPI0_val += adjustment[0];
      //TEST_display();
      Out_side_Motor_ctl();
      //if(LDC_SPI0_val > adjustment[7] | LDC_SPI1_val > adjustment[8]-50)
      if(LDC_SPI0_val > adjustment[1]+30)                                                       //50
      {
//        LCD_Fill(0x00);
        break;
      }
      
    }
  }
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
        //LCD_BL(70,0,(uint16)result_abs*10);
        //LCD_BL(0,2,(uint16)Result_list[1]);
}

void Out_side_collect()
{
    int count = 50;
    LCD_P6x8Str(0,6,"out side");
       while(count--)
       {
        ldc0_side=(float)filter(SPI0)/10;//左
        ldc1_side=(float)filter(SPI1)/10;//右
        LCD_BL(55,6,(uint16)ldc0_side);
        LCD_BL(90,6,(uint16)ldc1_side);
        
        if(flag) ldc1_side += adjustment[0];
        else ldc0_side += adjustment[0];
       }
       
       ldc1_side  += 50;
       ldc0_side  += 50;
}


void Device_init()
{       
        gpio_init(PTC11,GPI,1);                                         //拨码开关5
        gpio_init(PTC12,GPI,1);                                         //拨码开关4
        gpio_init(PTC13,GPI,1);                                         //拨码开关3
        gpio_init(PTC15,GPI,1);                                         //拨码开关2
        gpio_init(PTD2,GPI,1);                                          //拨码开关1
        gpio_init(PTD1,GPO,1);						//三个指示灯依次亮起
        gpio_init(PTD3,GPO,1);
        gpio_init(PTD4,GPO,1);						//每进入一次初始化PTD4灯灭，初始化完成亮起
        gpio_init(PTC5,GPO,1);
        gpio_init(PTC4,GPI,0);                                          //干簧管

	FTM_PWM_init(FTM1,FTM_CH0,50,angle_to_period(0));				//舵机初始化，从百分之十占空比到百分之七十占空比测试
	
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
  //printf("%d\r\n",(int)Stop_flag);
}

void LDC_EVM_TEST()
{
   evm_test(SPI0);
   evm_test(SPI1);
}

void adjustment_change()
{
  if(flag)
  {
    adjustment[2] += adjustment[0];
    adjustment[3] += adjustment[0];
  } 
  else 
  {
    adjustment[1] += adjustment[0];
    adjustment[4] += adjustment[0];
  }
}


void speed_control()
{
  if(gpio_get(PTC15))
  {
    AimSpeed = 150;
  }
    if(gpio_get(PTC13))
  {
    AimSpeed = 160;
  }
    if(gpio_get(PTC12))
  {
    AimSpeed = 170;
  }
  if(gpio_get(PTC11))
  {
    AimSpeed = 250;
  }
}

void value_collect()                                            //收集得出最大值放在0 和 1 位置
{
  int i=0;
  int i1 =0;
  float max,min;
  
  LCD_P6x8Str(0,2,"Rmax Lmin");
  LCD_P6x8Str(0,4,"Lmax Rmin");
  systick_delay_ms(1500);
  
  i = 0;
  
  while(i<100)
 {
    ldcValuetemp1[i]=(float)filter(SPI1)/10;
    LCD_BL(55,2,(uint16)ldcValuetemp1[i]);
    systick_delay_ms(30);
    ldcValuetemp0[i]=(float)filter(SPI0)/10;
    LCD_BL(90,2,(uint16)ldcValuetemp0[i]);
     i++;
 }
  

 
  i = 1;
  
  max = ldcValuetemp0[0];
  min = ldcValuetemp0[0];
  while(i<100)
  {
    if(max<ldcValuetemp0[i])
      max = ldcValuetemp0[i];
    if(min>ldcValuetemp0[i]);
      min = ldcValuetemp0[i];
    
    i++;
  }
  
  adjustment[4] = max;
  adjustment[1] = min;
  
  max = ldcValuetemp1[0];
  min = ldcValuetemp1[0];
  
  i = 1;
  while(i<100)
  {
    if(max<ldcValuetemp1[i])
      max = ldcValuetemp1[i];
    if(min>ldcValuetemp1[i]);
      min = ldcValuetemp1[i];
      
    i++;
  }
  
  adjustment[2] = max;
  adjustment[3] = min;
  
 LCD_BL(55,2,(uint16)adjustment[1]);
 LCD_BL(90,2,(uint16)adjustment[2]);
 LCD_BL(55,4,(uint16)adjustment[4]);
 LCD_BL(90,4,(uint16)adjustment[3]);
}

  void main(void)
{
	int i1=0;
	int count10S = 0;
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
        //LDC_EVM_TEST();
        
        //Speed_Ctl_Init(short point_value, double dKpp, double dKii, double dKdd);
        
      while(1)
      {
        //value_collect();
    
       LCD_P6x8Str(0,2,"Rmax Lmin");
        printf("Rmax Rmin\r\n");
        systick_delay_ms(1500);
        i1=50;
        
        while(i1--)
       {
           adjustment[2]=(float)filter(SPI1)/10;//右max
           LCD_BL(55,2,(uint16)adjustment[2]);
           printf("%d\r\n",adjustment[2]);
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
       
       divisor = (adjustment[2] - adjustment[3])/60;                              //48 //50
       divisor2 = (adjustment[4] - adjustment[1])/59;                             //48 //45
       
       zengfuzhi =  (adjustment[2] - adjustment[3])/(adjustment[4] - adjustment[1]);
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
        adjustment[7]=(float)filter(SPI0)/10;//左
        adjustment[8]=(float)filter(SPI1)/10;//右
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
       
       Out_side_collect();
       
       LCD_Fill(0x00);
       
       adjustment_change();
       
       enable_irq(PIT0_IRQn);
          while(1)
          { 
                  
                  if(PIT_1sFlag == 1)
                  {
                        gpio_turn(PTD4);
                        PIT_1sFlag = 0;
                        count10S ++;
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
                      Steering_Change();
                    }
                    if(count10S > 10)
                    {  
                      if(gpio_get(PTC4) == 0)
                      {
                        while(1)
                        {
                          Motor_stop();
                          Steering_Change();
                        }
                      }
                    }
                    if(PIT_10msFlag == 1)
                    {
                      Quad_count();
                      now = (float)guad_val;
                      Motor_PID(now);
                      PIT_10msFlag = 0;
                      
                    }
                    if(PIT_20msFlag == 1)
                    {

                      PIT_20msFlag = 0;
                    }
                    if(PIT_50msFlag == 1)
                    {
                      
                      //Result_collect();
                      PIT_50msFlag = 0;
                    }
 
                    
                    //printf("guad_val:%d \r\n",(guad_val));
                    
                    //TEST_display();
                    speed_control();
                    
                    //key_choice();
                    uint16 i23 = gpio_get(PTC4); 
                    
                    //LCD_BL(50,2,(uint16)(i23));
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
//				printf("ldc0_val");
//				printf("%d   \t",(int)LDC_SPI0_val);
//				printf("ldc1_val");
//				printf("%d   \t",(int)LDC_SPI1_val);
//				printf("%d\r\n",(int)LDC_result);
                                LCD_P6x8Str(0,4,"ldc0_val:");
                                LCD_BL(55,4,(uint16)LDC_SPI0_val);
                                LCD_P6x8Str(0,6,"ldc1_val:");
                                LCD_BL(55,6,(uint16)LDC_SPI1_val);
                                
                                LCD_BL(90,4,(uint16)adjustment[1]);
                                LCD_BL(90,6,(uint16)adjustment[2]);
                                LCD_BL(70,2,(uint16)(divisor*100));
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
                  kd_Steering+=0.05;
                }
                if(key_check(1)==0)           //检测PB17按键按下
                {
                  kd_Steering-=0.05;
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
  //OutData[3] = (float)mid;
  OutPut_Data();
}

void LDC_get()
{
	LDC_SPI0_val = (float)filter(SPI0)/10;
	systick_delay_ms(0);
	LDC_SPI1_val = (float)filter(SPI1)/10;
	LDC_result = LDC_SPI0_val-LDC_SPI1_val;
        if(LDC_result > 500) LDC_result = result_pre;

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