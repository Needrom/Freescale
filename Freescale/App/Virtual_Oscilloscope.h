/************************************************************************************************************/
//                             虚拟示波器接口头文件

/***********************************************************************************************************/

/***********************************************************************************************************/

#ifndef  _Virtual_Oscilloscope_H_
#define  _Virtual_Oscilloscope_H_

/***********************************************************************************************************/

//extern void uart_putchar (UARTn uratn, char ch);
extern float OutData[4] ;

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);

void OutPut_Data(void) ;

void out_HY(void);
/************************************************************************************************************/
#endif
/************************************************************************************************************/