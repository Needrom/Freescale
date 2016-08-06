#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "LDC1000.H"
#include "MK60_spi.h"
#include "MK60_uart.h"

#define TEST_RP_MSB_MIN 0x36            //0A   //07
#define TEST_RP_MSB_MAX 0x38            //12 //08
#define TEST_FC_MAX 0x0D5D
#define TEST_FC_MIN 0x0D39
#define TEST_RPMIN_MIN 0x20             //3A  //34
#define TEST_RPMIN_MAX 0x3F             //3D
#define TEST_RPMAX_MIN 0x00             //10
#define TEST_RPMAX_MAX 0x1F             //13
#define uchar uint8 
#define NN  20

unsigned char posi = 0;

LDC_ParameterPtr LDC_buff[50];

uint8 orgVal[12]={0}; 

uint8 RPMAX =0x07;                      //TI 14    yoiu 1136  
uint8 RPMIN =0x30;                      //TI 3B
uint8 rpi_max=10;
uint8 proximtyData[3]={0};
unsigned long proximtyDataTEMP=0,proximtyDataMAX,proximtyDataMIN,proximtyDataSUM,proximtyDataAVE,proximtyDataAVE_LAS;

int LDC_val=0;

unsigned long value_buf[NN],new_value_buf[NN],linearSmooth_buf[NN];

/*!
 *  @brief      浮点科技LDC1000电轨传感器模块初始化
 *  @param      
 *  @param      
 *  @param      
 *  @since      
 *  @note       包含SPI初始化
 *  Sample usage:  
 * @淘宝官方店铺 https://floats.taobao.com/
 */

void FLOAT_LDC_init(SPIn_e SPIn)
{
 
         FLOAT_SPI_init(SPIn);
         uart_init(UART3,115200);
         FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX, RPMAX,SPIn);
         FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMIN, RPMIN,SPIn);//0x14
         FLOAT_Singal_SPI_Write(LDC1000_CMD_SENSORFREQ,  0xA9,SPIn);  //谐振频率计算方法见《浮点科技电轨传感器调试手册》
         FLOAT_Singal_SPI_Write(LDC1000_CMD_LDCCONFIG,   0x17,SPIn);  //0x1B
         FLOAT_Singal_SPI_Write(LDC1000_CMD_CLKCONFIG,   0x00,SPIn);  //0x01        
         FLOAT_Singal_SPI_Write(LDC1000_CMD_INTCONFIG,   0x02,SPIn);
         FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,   0x01,SPIn);
         FLOAT_Singal_SPI_Write(LDC1000_CMD_THRESHILSB,  0x50,SPIn);
	 FLOAT_Singal_SPI_Write(LDC1000_CMD_THRESHIMSB,  0x14,SPIn);
	 FLOAT_Singal_SPI_Write(LDC1000_CMD_THRESLOLSB,  0xC0,SPIn);
	 FLOAT_Singal_SPI_Write(LDC1000_CMD_THRESLOMSB,  0x12,SPIn);
         FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPIn);//orgVal[]对应上面写入的值说明初始化正常
       
         for(int i=0;i<12;i++)
        {
          printf("%c",orgVal[i]);
        }
        
  
} 


int ldc_read_avr(SPIn_e SPIn)
{

    char rpi=0;  //取rpi次平均值    
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf(LDC1000_CMD_PROXLSB,&proximtyData[0],2,SPIn);  
      proximtyDataTEMP =((unsigned)(proximtyData[1])<<8) + proximtyData[0]+(unsigned)(proximtyData[2]<<16); 
      proximtyDataSUM += proximtyDataTEMP;
      if (proximtyDataTEMP < proximtyDataMIN)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN = proximtyDataTEMP;
      if (proximtyDataTEMP > proximtyDataMAX)
        proximtyDataMAX = proximtyDataTEMP;
    }
     proximtyDataAVE = proximtyDataSUM /rpi_max;
     proximtyDataSUM=0;
     proximtyDataAVE_LAS=proximtyDataAVE;
  
    return   proximtyDataAVE; 

}

void linearSmooth7 ( unsigned long in[], unsigned long out[], int N )
{
    int i;
    if ( N < 7 )
    {
        for ( i = 0; i <= N - 1; i++ )
        {
            out[i] = in[i];
        }
    }
    else
    {
        out[0] = ( 13.0 * in[0] + 10.0 * in[1] + 7.0 * in[2] + 4.0 * in[3] +
                  in[4] - 2.0 * in[5] - 5.0 * in[6] ) / 28.0;

        out[1] = ( 5.0 * in[0] + 4.0 * in[1] + 3 * in[2] + 2 * in[3] +
                  in[4] - in[6] ) / 14.0;

        out[2] = ( 7.0 * in[0] + 6.0 * in [1] + 5.0 * in[2] + 4.0 * in[3] +
                  3.0 * in[4] + 2.0 * in[5] + in[6] ) / 28.0;

        for ( i = 3; i <= N - 4; i++ )
        {
            out[i] = ( in[i - 3] + in[i - 2] + in[i - 1] + in[i] + in[i + 1] + in[i + 2] + in[i + 3] ) / 7.0;
        }

        out[N - 3] = ( 7.0 * in[N - 1] + 6.0 * in [N - 2] + 5.0 * in[N - 3] +
                      4.0 * in[N - 4] + 3.0 * in[N - 5] + 2.0 * in[N - 6] + in[N - 7] ) / 28.0;

        out[N - 2] = ( 5.0 * in[N - 1] + 4.0 * in[N - 2] + 3.0 * in[N - 3] +
                      2.0 * in[N - 4] + in[N - 5] - in[N - 7] ) / 14.0;

        out[N - 1] = ( 13.0 * in[N - 1] + 10.0 * in[N - 2] + 7.0 * in[N - 3] +
                      4 * in[N - 4] + in[N - 5] - 2 * in[N - 6] - 5 * in[N - 7] ) / 28.0;
    }
}


long int filter(SPIn_e SPIn)
{
   char count,i,j,count1;
   char count2=0;
 
   long int temp;
   long int sum=0;
   for(count=0;count<NN;count++)
   {
      value_buf[count] = ldc_read_avr(SPIn);
   }
   
   for(count1=0;count1<NN;count1++)
   {  
   if(value_buf[count1]<32768)
   {
   new_value_buf[count2]=value_buf[count1];
   count2++;
   }  
   }
   
   
   for (j=0;j<count2-1;j++)
   {
      for (i=0;i<count2-j;i++)
      {
        if ( new_value_buf[i]>new_value_buf[i+1] )
         {
            temp = new_value_buf[i];
            new_value_buf[i] = new_value_buf[i+1];
            new_value_buf[i+1] = temp;
         }
      }
   }

   for(count=1;count<count2-1;count++)
   {
      sum += new_value_buf[count];
   }
  
     
   return (long int)(sum/(count2-2)*0.9);


}

void FLOAT_SPI_init(SPIn_e SPIn)
{  
  if(SPIn==SPI0){
         gpio_init (LDC1000_SPI0_MISO_PIN, GPI,1);//MISO
         gpio_init (LDC1000_SPI0_MOSI_PIN ,GPO,1);//MOSI
         gpio_init (LDC1000_SPI0_CSB_PIN, GPO,1);// CSN
         gpio_init (LDC1000_SPI0_CLK_PIN, GPO,0);//SCK
  
         CSN_H;
         SCK_L;
         MOSI_H;
         
  }
  if(SPIn==SPI1){
         gpio_init (LDC1000_SPI1_MISO_PIN, GPI,1);//MISO
         gpio_init (LDC1000_SPI1_MOSI_PIN, GPO,1);//MOSI
         gpio_init (LDC1000_SPI1_CSB_PIN, GPO,1);//CSB
         gpio_init (LDC1000_SPI1_CLK_PIN, GPO,0);//CLK
//         spi_init(SPI1,SPIn_PCS1, MASTER,115200);
         
         
         CSB_H_SPI1;
         CLK_L_SPI1;
         MOSI_H_SPI1;
  }
}
/****************************************************************************************************
* Function Name: uchar FLOAT_SPI_RW(uchar wdata)
* Description  : read and write of SPI.
* Arguments    : wdata
* Return Value : rdata
/****************************************************************************************************/
uchar FLOAT_SPI_RW(uchar rwdata)
{  
    
	uchar spi_rw_i=0;	
        uchar temp=0;
        for(spi_rw_i=0;spi_rw_i<8;spi_rw_i++)   	// output 8-bit
   	{
   	        /*** prepare the write data of read before the coming of rising up******/
          if(rwdata & 0x80){
                    MOSI_H;
                    MOSI_H_SPI1;
          }
          else{ 
                    MOSI_L;
                    MOSI_L_SPI1;
          }
   		  rwdata<<=1;           		// shift next bit to MSB
                  temp<<=1;
		SCK_L;             //Set SCK high    Rising up 
                CLK_L_SPI1;
   		if(MISO) 
                  temp|=1;
   		SCK_H;            //set  SCK low     Falling down
                CLK_H_SPI1;
   	}
    return(temp);           		  		// return read byte
    
 
}
/****************************************************************************************************
* Function Name: uchar FLOAT_Singal_SPI(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
/****************************************************************************************************/
uchar FLOAT_Singal_SPI_Read(uchar reg,SPIn_e SPIn)
{
	uchar rdata;
	if(SPIn==SPI0)
	CSN_L;                // CSN low, initialize SPI communication...
        else
        CSB_L_SPI1;  
        
        FLOAT_delay_us(2);
         
         reg=reg|0x80;//read
	FLOAT_SPI_RW(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         
	rdata = FLOAT_SPI_RW(NULL);    // ..then read registervalue
       
        FLOAT_delay_us(1700);
	CSN_H;                // CSN high, terminate SPI communication
	CSB_H_SPI1;
	return rdata;        // return register value
}
/****************************************************************************************************
* Function Name: void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
/****************************************************************************************************/
void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata,SPIn_e SPIn)
{
	
	if(SPIn==SPI0)
	CSN_L;                // CSN low, initialize SPI communication...
        else
        CSB_L_SPI1;  
              
        FLOAT_delay_us(2);//2us
        reg=reg&~0x80;
	FLOAT_SPI_RW(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
        
	FLOAT_SPI_RW(wdata);    // ..then read registervalue
        FLOAT_delay_us(1700);//875us
	CSN_H;              // CSN high, terminate SPI communication
        CSB_H_SPI1;
	
}

/****************************************************************************************************
* Function Name: void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len)
* Description  : read muche registers
* Arguments    : reg,len
* Return Value : *pBuf
/****************************************************************************************************/
void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len,SPIn_e SPIn)
{
	uchar spi_rw_i;
	
	if(SPIn==SPI0)
	CSN_L;                // CSN low, initialize SPI communication...
        else
        CSB_L_SPI1;  
        
        reg=reg|0x80;//read
	FLOAT_SPI_RW(reg);       		// Select register to write to and read status uchar
	
	for(spi_rw_i=0;spi_rw_i<len;spi_rw_i++)
        {  
	pBuf[spi_rw_i] = FLOAT_SPI_RW(NULL);    // 
	 }
	CSN_H;     
        CSB_H_SPI1;

}



//////////////////////////////////////////////////////////////////////////////////////////////////////

//void LDC_Val_Printf(SPIn_e SPIn)
//{
//  if(SPIn==SPI0){
//    LDC_val=filter(SPI0)/10;//采样滤波
//  
//  printf("val:");
//    printf("%d",LDC_val);
//  }
//  
//  if(SPIn==SPI0){
//    LDC_val1=filter(SPI1)/10;
//  
//    printf("val1:");
//    printf("%d",LDC_val1);
//  }
//  
//
//  //    printf("val-compartor:");
////    printf("%d",LDC_val-LDC_val1);
//
//    
//
//    
//}


void Set_reg(SPIn_e SPIn)
{
  FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX,LDC_buff[0]->RPMAX,SPIn);
  FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMIN,LDC_buff[0]->RPMIN,SPIn);
}

void Save_the_set(SPIn_e SPIn)
{
  LDC_ParameterPtr temp;
   FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPIn);
   LDC_buff[posi]->RPMAX      = orgVal[1];
   LDC_buff[posi]->RPMIN      = orgVal[2];
   LDC_buff[posi]->PROXfreq   = filter(SPIn);
   LDC_buff[posi]->RP_NOSIE   = RP_test_noise(SPIn);
   
   posi++;
   
   if(posi)
   {
     for(int i=0;i<posi-1;i++)
      for(int j=0;j<posi-i;j++)
      {
        if(LDC_buff[j]->RP_NOSIE>LDC_buff[j+1]->RP_NOSIE)
        {
            temp = LDC_buff[j];
            LDC_buff[j] = LDC_buff[j+1];
            LDC_buff[j+1] = temp;
        }
      }
   }
}

void Reset_buff()
{
  for(int i=0;i<50;i++)
  {
    LDC_buff[i] = 0;
  }
  posi = 0;
}

int RP_test_noise(SPIn_e SPIn)
{

    char rpi=0;  //取rpi次平均值    
    char pre_error;
    FLOAT_SPI_Read_Buf(LDC1000_CMD_PROXLSB,&proximtyData[0],2,SPIn);  
      pre_error = ((unsigned char)proximtyData[1]); 
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf(LDC1000_CMD_PROXLSB,&proximtyData[0],2,SPIn);  
      proximtyDataTEMP = ((unsigned char)proximtyData[1]); 
      proximtyDataSUM += proximtyDataTEMP - pre_error;
      pre_error = proximtyDataTEMP;
      if (proximtyDataTEMP < proximtyDataMIN)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN = proximtyDataTEMP;
      if (proximtyDataTEMP > proximtyDataMAX)
        proximtyDataMAX = proximtyDataTEMP;
    }
     proximtyDataAVE = proximtyDataSUM /rpi_max;
     proximtyDataSUM=0;
     proximtyDataAVE_LAS=proximtyDataAVE;
  
    return   proximtyDataAVE; 

}

static uint8_t evm_Test_Rp_Sample(uint8 *t_buf,SPIn_e SPIn) {
	uint8_t i,j,diff;
	uint16_t avg;
	for (i = 0; i < 16; i++) {
//		evm_LED_Toggle();
		avg = 0;
		for (j = 0; j < 16; j++) {
			FLOAT_SPI_Read_Buf(LDC1000_CMD_PROXLSB,&t_buf[0],2,SPIn);
//			FLOAT_delay_us(2000);
			avg += t_buf[1];
		}
		avg = avg / 16;
		if(avg < TEST_RP_MSB_MIN || avg > TEST_RP_MSB_MAX) {
			printf("value of comp: \n\n");
                        printf("%d",avg);
                        return TRUE;
		}
//                if(SPIn==SPI1)
//                {
//                      if((filter(SPI0)-filter(SPI1))>0)
//                    {
//                      diff=(filter(SPI0)-filter(SPI1));
//                    }
//                    else
//                    {
//                      diff=(filter(SPI1)-filter(SPI0));
//                    }
//                    if(diff>60)
//                    {
//                      return TRUE;
//                    }
//                }
	}
	return FALSE;
}

// LDC1000 Test Routine
uint8 evm_test(SPIn_e SPIn) {

	uint8_t t_buf[4] = {0x00, 0x00, 0x00, 0x00},test_flag=0;
	//uint8_t rpmin,rpmax;
    uint16_t i,j;//,noise,min;

//    FLOAT_delay_us(2500); // need this many cycles to settle
//    FLOAT_delay_us(2500); // need this many cycles to settle
    // test the default params
	// Rpmin & Rpmax tuning
	// if Rp data MSB is out of range, reprogram
	for (i = TEST_RPMIN_MIN; i <= TEST_RPMIN_MAX; i++) {
		for (j = TEST_RPMAX_MIN; j <= TEST_RPMAX_MAX; j++) {
                        test_flag = evm_Test_Rp_Sample(&t_buf[0],SPIn);
			if (!test_flag && RP_test_noise(SPIn) < 20) {
                          //Save_the_set(SPIn);        
                          break; 
			}
			else {
                                LCD_Fill(0x00);
                                printf("%d \r\n",filter(SPIn));
                                LCD_BL(0,0,(uint16)filter(SPIn));
                                systick_delay_ms(50);
				FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x00,SPIn);
				FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMAX,j,SPIn);
				FLOAT_Singal_SPI_Write(LDC1000_CMD_RPMIN,i,SPIn);
				FLOAT_Singal_SPI_Write(LDC1000_CMD_PWRCONFIG,0x01,SPIn);
				systick_delay_ms(50); // need this many cycles to settle
                                FLOAT_SPI_Read_Buf(LDC1000_CMD_REVID,&orgVal[0],12,SPIn);
                                printf("%c",orgVal[1]);
                                printf("%c",orgVal[2]);
			}
			test_flag = evm_Test_Rp_Sample(&t_buf[0],SPIn);
		}
	}
//	EVM_RED_LED_OFF();
//	EVM_GRN_LED_OFF();
	test_flag = 0;
	// validate range is good with another 1024 sampling window
//	for (i = 0; i < 1024; i++) {
//		FLOAT_SPI_Read_Buf(LDC1000_CMD_PROXLSB,&t_buf[0],sizeof(t_buf));
//		FLOAT_delay_us(8000);
		// verify Rp Data within Range
//		j = t_buf[1];
//		if (j >= TEST_RP_MSB_MIN && j <= TEST_RP_MSB_MAX) {
//			EVM_GRN_LED_ON();
//		}
//		else {
			// verify Frequency Counter with Range
//			j = ((uint16_t) t_buf[2]) | (((uint16_t) t_buf[3]) << 8);
//			if (j >= TEST_FC_MIN && j <= TEST_FC_MAX) {
//				EVM_RED_LED_ON();
//				FLOAT_delay_us(2000);
//				EVM_RED_LED_OFF();
//				FLOAT_delay_us(2000);
//			}
//			EVM_RED_LED_ON();
//			test_flag = 1;
//		}
//	}
	if (test_flag) {
//		EVM_RED_LED_ON();
//		for (i = 0; i < 1024; i++) {
//			FLOAT_delay_us(1000*2.0);
//		}
		return FALSE;
	}
	else {
//		EVM_GRN_LED_ON();
//		for (i = 0; i < 1024; i++) {
//			FLOAT_delay_us(1000*2.0);
//		}
//		EVM_GRN_LED_OFF();
//		EVM_RED_LED_OFF();
		return TRUE;
	}
}
