extern short Speed_Ctrl(float speed) ; 
extern short coder_read();
extern void Speed_Ctl_Init(short point_value, double dKpp, double dKii, double dKdd);
extern void motor(short pwm_l);
extern void speed_play();
//NRF24L01+ 的 中断服务函数
typedef struct
{
	double SetPoint;	 	//设定值
	double Proportion;	//比例系数
	double Integral;	  //积分系数
	double Derivative;	//微分系数
	double LastErr;			//前一次误差
	double PrevErr;			//前两次误差
	double pErr;				//比例误差
	double iErr;				//积分误差
	double dErr;				//微分误差
	double Pid_OutPut;	//pid输出
	double Integral_edge_Max;		//变速积分边沿最大值
	double Integral_edge_Min;		//变速积分边沿最小值
}_PID;
/***********************************分割线***********************************/
_PID pid_speed;
short read_speed,speed_pwm;
/*****************************PID计算部分************************************/
short Speed_Ctrl(float speed)
{
	static short speed_pid_out = 0;
	float ThisErr;
	//增量式pid
	ThisErr = speed - pid_speed.SetPoint;
	pid_speed.pErr = pid_speed.LastErr-ThisErr;
        pid_speed.iErr = ThisErr;
        pid_speed.dErr = ThisErr - 2*pid_speed.LastErr + pid_speed.PrevErr;
	speed_pid_out += pid_speed.Proportion*pid_speed.pErr - pid_speed.Integral*pid_speed.iErr + pid_speed.Derivative*pid_speed.dErr; 
	pid_speed.PrevErr = pid_speed.LastErr;
	pid_speed.LastErr = ThisErr;
        if(speed_pid_out >= 9900)
        {
          speed_pid_out = 9900;
        }
        else if(speed_pid_out <= -9900)
        {
          speed_pid_out = -9900;
        }
          
        //else if( || (-9999 < speed_pid_out && speed_pid_out <= 0))))
	return speed_pid_out;
}
/***********************************分割线***********************************/


/********************************速度脉冲读取********************************/
short coder_read()
{
  uint16 count;
  count   =  tpm_pulse_get(TPM2);
  tpm_pulse_clean(TPM2);
  PIT_Flag_Clear(PIT0);
  return count;
}
/***********************************分割线***********************************/

/********************************速度控制初始化********************************/

void Speed_Ctl_Init(short point_value, double dKpp, double dKii, double dKdd)
{
      
  tpm_pwm_init(TPM1, TPM_CH0,10*1000,0);      //初始化 PWM
  tpm_pwm_init(TPM1, TPM_CH1,10*1000,0);      //初始化 PWM
  tpm_pulse_init(TPM2,TPM_CLKIN0,TPM_PS_1); //初始化 TPM2 为脉冲累加，输入管脚为 TPM_CLKIN0_PIN ，分频系数为 1
  tpm_pulse_clean(TPM2);
  pid_speed.SetPoint   = (double)point_value;	//设定值
  pid_speed.Proportion = dKpp;				//比例系数
  pid_speed.Integral   = dKii;	  		//积分系数
  pid_speed.Derivative = dKdd;				//微分系数 
  pid_speed.iErr		 = 0;	
  pid_speed.LastErr		 = 0;
  pid_speed.PrevErr		 = 0;
}
/***********************************分割线***********************************/

/********************************速度反馈后输入********************************/
void motor(short pwm_l)
{
  if(pwm_l>9900)
    pwm_l=9900;
  else if(pwm_l<-9900)
    pwm_l=-9900;
//  printf("%d\n",pwm_l);//与PWM设定的值相差多少      
 
  if(pwm_l>0)
  {
    tpm_pwm_duty(TPM1, TPM_CH0,pwm_l);
    tpm_pwm_duty(TPM1, TPM_CH1,0);
  }
  else
  {
    tpm_pwm_duty(TPM1, TPM_CH1,-pwm_l);
    tpm_pwm_duty(TPM1, TPM_CH0,0);
  } 
 
}
void speed_play()
{
      read_speed=coder_read();
      tpm_pulse_clean(TPM2);
      speed_pwm=Speed_Ctrl(read_speed);
      motor(speed_pwm);
   //  printf("%d\n",read_speed);   
    // printf("%d\n",speed_pwm); 
   
}
