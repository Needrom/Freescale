#ifndef  CCD_H
#define  CCD_H  

#define CCD_SI_H   PTE3_OUT = 1    //定义线性传感器的端口 SIE3
#define CCD_SI_L   PTE3_OUT = 0    //定义线性传感器的端口 SI
#define CCD_CLK_H  PTE2_OUT = 1    //定义线性传感器的端口 CLKE2
#define CCD_CLK_L  PTE2_OUT = 0    //定义线性传感器的端口 CLK


void CCD_init(void) ;
void CCDDelay(void) ;
void CCD_RD(unsigned char * ImageData) ;
void Pixel_erzhihua(unsigned char * CCD);
void Get_Img(void);
void yuzhi(void);

#endif