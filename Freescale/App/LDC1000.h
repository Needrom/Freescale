#ifndef LDC1000_CMD_H_
#define LDC1000_CMD_H_
#define uchar uint8
#include "MK60_spi.h"





//FLOAT LDC COMMANDS
#define LDC1000_CMD_REVID               0x00
#define LDC1000_CMD_RPMAX 	        0x01
#define LDC1000_CMD_RPMIN 	        0x02
#define LDC1000_CMD_SENSORFREQ 	0x03               //谐振频率
#define LDC1000_CMD_LDCCONFIG 	0x04
#define LDC1000_CMD_CLKCONFIG 	0x05
#define LDC1000_CMD_THRESHILSB 	0x06
#define LDC1000_CMD_THRESHIMSB 	0x07
#define LDC1000_CMD_THRESLOLSB 	0x08
#define LDC1000_CMD_THRESLOMSB 	0x09
#define LDC1000_CMD_INTCONFIG 	0x0A
#define LDC1000_CMD_PWRCONFIG 	0x0B
#define LDC1000_CMD_STATUS	0x20
#define LDC1000_CMD_PROXLSB 	0x21
#define LDC1000_CMD_PROXMSB 	0x22
#define LDC1000_CMD_FREQCTRLSB	0x23
#define LDC1000_CMD_FREQCTRMID	0x24
#define LDC1000_CMD_FREQCTRMSB	0x25

//FLOAT LDC BITMASKS
#define LDC1000_BIT_AMPLITUDE    0x18
#define LDC1000_BIT_RESPTIME     0x07
#define LDC1000_BIT_CLKSEL       0x02
#define LDC1000_BIT_CLKPD        0x01
#define LDC1000_BIT_INTMODE      0x07
#define LDC1000_BIT_PWRMODE      0x01
#define LDC1000_BIT_STATUSOSC    0x80
#define LDC1000_BIT_STATUSDRDYB  0x40
#define LDC1000_BIT_STATUSWAKEUP 0x20
#define LDC1000_BIT_STATUSCOMP   0x10

//LDC_SPI_PIN
#define LDC1000_SPI0_MOSI_PIN PTA17 
#define LDC1000_SPI0_MISO_PIN PTA16
#define LDC1000_SPI0_CSB_PIN PTC3
#define LDC1000_SPI0_CLK_PIN PTA15

#define LDC1000_SPI1_MOSI_PIN PTA17 
#define LDC1000_SPI1_MISO_PIN PTA16
#define LDC1000_SPI1_CSB_PIN PTC0
#define LDC1000_SPI1_CLK_PIN PTA15




///////////////////////////////////////////LDC测试参数
typedef struct LDC_Parameter
{
	int RPMAX;
	int RPMIN;
	int PROXfreq;
	int RP_NOSIE;
}volatile *LDC_ParameterPtr;

/**********************************************************SPI 管脚定义*******************************************************/
/***************经测试各家核心板IO驱动能力有区别建议大家不要使用下面注释掉的方式，使用gpio_get（）方式数据会更稳定************/



#define MISO   gpio_get(LDC1000_SPI0_MISO_PIN)//(GPIO_PDIR_REG(GPIOX_BASE(PTD0)) >> PTn(PTD0 )) & 0x01  

#define MOSI_H  gpio_set(LDC1000_SPI0_MOSI_PIN,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD1))  |= (1 << PTn(PTD1))
#define MOSI_L  gpio_set(LDC1000_SPI0_MOSI_PIN,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD1)) &= ~(1 << PTn(PTD1))

#define CSN_H   gpio_set(LDC1000_SPI0_CSB_PIN,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD2))  |= (1 << PTn(PTD2))
#define CSN_L   gpio_set(LDC1000_SPI0_CSB_PIN,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD2)) &= ~(1 << PTn(PTD2))

#define SCK_H   gpio_set(LDC1000_SPI0_CLK_PIN,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD3))  |= (1 << PTn(PTD3))
#define SCK_L   gpio_set(LDC1000_SPI0_CLK_PIN,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD3)) &= ~(1 << PTn(PTD3))

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MISO_SPI1 gpio_get(LDC1000_SPI1_MISO_PIN)           //MISO

#define MOSI_H_SPI1 gpio_set(LDC1000_SPI1_MOSI_PIN,1)
#define MOSI_L_SPI1 gpio_set(LDC1000_SPI1_MOSI_PIN,0)       //MOSI

#define CSB_H_SPI1 gpio_set(LDC1000_SPI1_CSB_PIN,1)        //CSB
#define CSB_L_SPI1 gpio_set(LDC1000_SPI1_CSB_PIN,0)

#define CLK_H_SPI1 gpio_set(LDC1000_SPI1_CLK_PIN,1)
#define CLK_L_SPI1 gpio_set(LDC1000_SPI1_CLK_PIN,0)        //CLK


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void FLOAT_LDC_init(SPIn_e SPIn);
int ldc_read_avr(SPIn_e SPIn);
long int filter(SPIn_e SPIn);

void FLOAT_SPI_init(SPIn_e SPIn);
uchar FLOAT_SPI_RW(uchar rwdata);
uchar FLOAT_Singal_SPI_Read(uchar reg,SPIn_e SPIn);
void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata,SPIn_e SPIn);
void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len,SPIn_e SPIn);
int RP_test_noise(SPIn_e SPIn);
void Save_the_set(SPIn_e SPIn);





#endif
