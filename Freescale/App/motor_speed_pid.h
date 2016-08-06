extern short Speed_Ctrl(float speed) ; 
extern short coder_read();
extern void Speed_Ctl_Init(short point_value, double dKpp, double dKii, double dKdd);
extern void motor(short pwm_l);
extern void speed_play();
//NRF24L01+ �� �жϷ�����
typedef struct
{
	double SetPoint;	 	//�趨ֵ
	double Proportion;	//����ϵ��
	double Integral;	  //����ϵ��
	double Derivative;	//΢��ϵ��
	double LastErr;			//ǰһ�����
	double PrevErr;			//ǰ�������
	double pErr;				//�������
	double iErr;				//�������
	double dErr;				//΢�����
	double Pid_OutPut;	//pid���
	double Integral_edge_Max;		//���ٻ��ֱ������ֵ
	double Integral_edge_Min;		//���ٻ��ֱ�����Сֵ
}_PID;
/***********************************�ָ���***********************************/
_PID pid_speed;
short read_speed,speed_pwm;
/*****************************PID���㲿��************************************/
short Speed_Ctrl(float speed)
{
	static short speed_pid_out = 0;
	float ThisErr;
	//����ʽpid
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
/***********************************�ָ���***********************************/


/********************************�ٶ������ȡ********************************/
short coder_read()
{
  uint16 count;
  count   =  tpm_pulse_get(TPM2);
  tpm_pulse_clean(TPM2);
  PIT_Flag_Clear(PIT0);
  return count;
}
/***********************************�ָ���***********************************/

/********************************�ٶȿ��Ƴ�ʼ��********************************/

void Speed_Ctl_Init(short point_value, double dKpp, double dKii, double dKdd)
{
      
  tpm_pwm_init(TPM1, TPM_CH0,10*1000,0);      //��ʼ�� PWM
  tpm_pwm_init(TPM1, TPM_CH1,10*1000,0);      //��ʼ�� PWM
  tpm_pulse_init(TPM2,TPM_CLKIN0,TPM_PS_1); //��ʼ�� TPM2 Ϊ�����ۼӣ�����ܽ�Ϊ TPM_CLKIN0_PIN ����Ƶϵ��Ϊ 1
  tpm_pulse_clean(TPM2);
  pid_speed.SetPoint   = (double)point_value;	//�趨ֵ
  pid_speed.Proportion = dKpp;				//����ϵ��
  pid_speed.Integral   = dKii;	  		//����ϵ��
  pid_speed.Derivative = dKdd;				//΢��ϵ�� 
  pid_speed.iErr		 = 0;	
  pid_speed.LastErr		 = 0;
  pid_speed.PrevErr		 = 0;
}
/***********************************�ָ���***********************************/

/********************************�ٶȷ���������********************************/
void motor(short pwm_l)
{
  if(pwm_l>9900)
    pwm_l=9900;
  else if(pwm_l<-9900)
    pwm_l=-9900;
//  printf("%d\n",pwm_l);//��PWM�趨��ֵ������      
 
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
