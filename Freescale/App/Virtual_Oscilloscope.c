#include <common.h>      /* common defines and macros */
#include <MK60DZ10.h>     /* derivative information */
#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include "Virtual_Oscilloscope.h"
#include "MK60_uart.h"


/***********************************************************************************************************/

//extern void uart_putchar (UARTn uratn, char ch);
float OutData[4] = { 0 };

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
  {
  //uart_putchar(UART3, databuf[i]); //用户发送databuf[];
  // printp("%d",databuf[i]);
  uart_putchar(UART3,databuf[i])  ;//修改为各自单片机串口发送整型数据的函数
  }
} 

/************************************************************************************************************/
/************************************************************************************************************/

void out_HY(void) 
{
    uart_putchar(UART3,0xFF);
    uart_putchar(UART3,0xFF);
    uart_putchar(UART3,(int)OutData[0]);
    uart_putchar(UART3,(int)OutData[1]);
    uart_putchar(UART3,(int)OutData[2]);
    uart_putchar(UART3,(int)OutData[3]);

}



