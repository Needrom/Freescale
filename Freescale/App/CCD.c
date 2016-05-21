#include "include.h"
#include "CCD.h"


extern uint8 yuzhi_max1;     //CCD1阈值上限
extern uint8 yuzhi_min1;     //CCD1阈值下限
extern uint8 yz;
extern uint8 Pixel1[131];    //CCD1数组(0~127像素数据、128min、129max、130阈值)
extern uint8 CCD_Flag;
uint8 WhiteNum=0;

/*************************************************************************
*                            ZJL
*
*  函数名称：CCD_init
*  功能说明：CCD初始化
*  参数说明：
*  函数返回：无
*  修改时间：2015-10-20
*  备    注：
*************************************************************************/
void CCD_init(void)
{
  gpio_init (PTE3, GPO,LOW);     //SI信号输入    E3口
  gpio_init (PTE2, GPO,LOW);    //CLK信号输入    E2口
  adc_init(ADC1_SE4a) ;         //AO口输出       E0口
}

/*************************************************************************
*                           ZJL
*
*  函数名称：CCDDelay
*  功能说明：CCD程序延时
*  参数说明：
*  函数返回：无
*  修改时间：2015-10-20
*  备    注：
*************************************************************************/
void CCDDelay(void)
 {
   volatile uint8 i ;
   for(i=0;i<1;i++) 
   {
    asm("nop");
   }
}

/*************************************************************************
*                           ZJL
*
*  函数名称：CCD_RD
*  功能说明：CCD采样程序
*  参数说明：
*  函数返回：无
*  修改时间：2015-10-20
*  备    注：
*ImageData =  ad_once(ADC1_AD4a, ADC_8bit);
*************************************************************************/
void CCD_RD(unsigned char * ImageData)
{
  unsigned char i;
  CCD_SI_H;    //SI=1
  CCDDelay();
  CCD_CLK_H;   //CLK=1
  CCDDelay();
  CCD_SI_L;    //SI=0
  *ImageData =  adc_once(ADC1_SE4a, ADC_8bit);
  CCD_CLK_L;   //CLK=0
  for(i=0; i<127; i++)
  {
    CCDDelay();
    CCD_CLK_H; //CLK=1
    CCDDelay();
    ImageData ++ ;
    *ImageData =  adc_once(ADC1_SE4a, ADC_8bit);
    CCD_CLK_L; //CLK=0
  }
  CCDDelay();
  CCD_CLK_H; //CLK=1
  CCDDelay();
  CCD_CLK_L; //CLK=0
}
void Pixel_erzhihua(unsigned char * CCD)
{
    uint8 j;
    uint8 max;
    uint8 min;
    uint8 *CCD_0;           //CCD存储数组指针
    CCD_0=CCD;           //指针赋值
    max=*CCD;
    min=*CCD;
    CCD++;
    for(j=1;j<128;j++)
    {
      max=(max<*CCD)?*CCD:max;
      min=(min>*CCD)?*CCD:min;	
      CCD++;
    }
    *CCD=min;            //存于第129个地址
    CCD++;
    *CCD=max;            //存于第130个地址
    CCD++;
    yz=8*(max+min)/20;
    if(CCD_0==Pixel1)
    {
      yz=(yz<yuzhi_min1)?yuzhi_min1:yz;
      yz=(yz>yuzhi_max1)?yuzhi_max1:yz;
    }
    *CCD=yz;             //存于第131个地址
  for(j=0;j<128;j++)
    {
     
       if(*CCD_0 >*CCD)
       { *CCD_0=1;
          WhiteNum++;
       }
       else
       {
        *CCD_0=0;
       
       }
      CCD_0++;
     
    }
    
}
void Get_Img(void)
{  int lv_num=1;
   CCD_RD(Pixel1);
   for(lv_num=1;lv_num<127;lv_num++)
   Pixel1[lv_num]=get_mid(Pixel1[lv_num-1],Pixel1[lv_num],Pixel1[lv_num+1]);
   Pixel_erzhihua(Pixel1);
}
void yuzhi(void)
{       
        int temp1;
        int i;
        int temp2;
        int min;
        int max;
        int pj1;
        for(i=0;i<5;i++)
        {
         while(CCD_Flag ==0);
         CCD_Flag = 0;
          CCD_RD(Pixel1);
          Pixel_erzhihua(Pixel1);
          temp1 = temp1 + Pixel1[129] ;
          temp2 = temp2 + Pixel1[128] ;
          
        }
        max = temp1/5;
        min = temp2/5;
        pj1 = 8*(max+min)/20;
        yuzhi_max1 = (8*(max + min)/20) * 1.1;
        yuzhi_min1 = (8*(max + min)/20) * 0.8;        
}