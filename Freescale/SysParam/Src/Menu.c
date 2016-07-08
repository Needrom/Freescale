#include "common.h"
#include "OLED.h"
#include "SysParam.h"
#include "Control.h"
#include "Timer.h"
#include "BoardInit.h"
#include "ADCFilter.h"
#include "Angle.h"
#include "ENC03.h"
#include "ElecDetecter.h"
#include "Menu.h"
#include "BalanceTick.h"

void DelayMS(int count);

/**************全局变量声明**********/

static signed int siUserChoose=0;
static signed char siConfirmFlag=0;

/**********函数说明:主菜单目录初始化函数 *************/

struct MenuItem  
{
  unsigned char ucMenuCount;						//当前层节点数，即每层菜单最多能显示的条目数         
  unsigned char *ucDisplayString;					//指向菜单标题字符串的指针
  double (*GetData)(void);						//获取数据
  signed char (*Subs)(signed char scDirection);    	//指向当前状态应该执行功能函数的指针
  struct MenuItem *ChildrenMenus; 					//指向当前菜单的下级菜单
  struct MenuItem *ParentMenus;   					//指向当前菜单的上级菜单
}Null;//主菜单



struct MenuItem MainMenu[5];			//主菜单

struct MenuItem PIDMenu[3];			//PID菜单
struct MenuItem AutoAdjustMenu[4];	     //自动校正菜单
struct MenuItem SpeedMenu[2];		     //运行速度菜单
struct MenuItem AboutMenu[4];		     //信息菜单

struct MenuItem SpeedPIDMenu[4];		//速度PID菜单
struct MenuItem AnglePIDMenu[2];		//角度PID主菜单
struct MenuItem DirectionPIDMenu[2];	//方向PID主菜单
struct MenuItem ElecDetecterMenu[4];	//磁场菜单

struct MenuItem (*MenuPoint) = MainMenu;

signed char AdjustBalanceAngle(signed char scDirection);
signed char AdjustENC03(signed char scDirection);
signed char AdjustMagneticField(signed char scDirection);

signed char ReadVol(signed char scDirection);
/**********函数说明:主菜单目录初始化函数 *************/
void MainMenuInit()
{
  MainMenu[0].ucMenuCount= 5;
  MainMenu[0].ucDisplayString="PID";
  MainMenu[0].Subs=NULL;
  MainMenu[0].GetData = NULL;
  MainMenu[0].ChildrenMenus=PIDMenu;
  MainMenu[0].ParentMenus=NULL;
  
  MainMenu[1].ucMenuCount = 5;
  MainMenu[1].ucDisplayString = "AutoAdjust";
  MainMenu[1].Subs = NULL;
  MainMenu[1].GetData = NULL;
  MainMenu[1].ChildrenMenus = AutoAdjustMenu;
  MainMenu[1].ParentMenus = NULL;
  
  MainMenu[2].ucMenuCount = 5;
  MainMenu[2].ucDisplayString = "Speed";
  MainMenu[2].Subs = NULL;
  MainMenu[2].GetData = NULL;
  MainMenu[2].ChildrenMenus = SpeedMenu;
  MainMenu[2].ParentMenus = NULL;
  
  MainMenu[3].ucMenuCount = 5;
  MainMenu[3].ucDisplayString = "About";
  MainMenu[3].Subs = NULL;
  MainMenu[3].GetData = NULL;
  MainMenu[3].ChildrenMenus =AboutMenu;
  MainMenu[3].ParentMenus = NULL;
  
  MainMenu[4].ucMenuCount = 5;
  MainMenu[4].ucDisplayString = "Finish";
  MainMenu[4].Subs = Finishing;
  MainMenu[4].GetData = NULL;
  MainMenu[4].ChildrenMenus = NULL;
  MainMenu[4].ParentMenus = NULL;
  
}
void PIDMenuInit()
{
  PIDMenu[0].ucMenuCount=3;
  PIDMenu[0].ucDisplayString="Direction";
  PIDMenu[0].Subs=NULL;
  PIDMenu[0].GetData = NULL;
  PIDMenu[0].ChildrenMenus=DirectionPIDMenu;
  PIDMenu[0].ParentMenus=MainMenu;
  
  PIDMenu[1].ucMenuCount =3;
  PIDMenu[1].ucDisplayString = "Speed";
  PIDMenu[1].Subs = NULL;
  PIDMenu[1].GetData = NULL;
  PIDMenu[1].ChildrenMenus = SpeedPIDMenu;
  PIDMenu[1].ParentMenus = MainMenu;
  
  PIDMenu[2].ucMenuCount = 3;
  PIDMenu[2].ucDisplayString = "Angle";
  PIDMenu[2].Subs = NULL;
  PIDMenu[2].GetData = NULL;
  PIDMenu[2].ChildrenMenus = AnglePIDMenu;
  PIDMenu[2].ParentMenus = MainMenu;
}

void AutoAdjustMenuInit()
{
  AutoAdjustMenu[0].ucMenuCount=4;
  AutoAdjustMenu[0].ucDisplayString= "Angle";
  AutoAdjustMenu[0].Subs= AdjustBalanceAngle;
  AutoAdjustMenu[0].GetData = GetSysBalanceAngle;
  AutoAdjustMenu[0].ChildrenMenus=NULL;
  AutoAdjustMenu[0].ParentMenus=MainMenu;
  
  AutoAdjustMenu[1].ucMenuCount =4;
  AutoAdjustMenu[1].ucDisplayString = "ElecDetecter";
  AutoAdjustMenu[1].Subs = NULL;
  AutoAdjustMenu[1].GetData = NULL;
  AutoAdjustMenu[1].ChildrenMenus = ElecDetecterMenu;
  AutoAdjustMenu[1].ParentMenus = MainMenu;
   
  AutoAdjustMenu[2].ucMenuCount =4;
  AutoAdjustMenu[2].ucDisplayString = "ENC03X";
  AutoAdjustMenu[2].Subs = AdjustENC03;
  AutoAdjustMenu[2].GetData = GetSysENC03X;
  AutoAdjustMenu[2].ChildrenMenus = NULL;
  AutoAdjustMenu[2].ParentMenus = MainMenu;
  
  AutoAdjustMenu[3].ucMenuCount =4;
  AutoAdjustMenu[3].ucDisplayString = "ENC03Z";
  AutoAdjustMenu[3].Subs = AdjustENC03;
  AutoAdjustMenu[3].GetData = NULL;
  AutoAdjustMenu[3].ChildrenMenus = NULL;
  AutoAdjustMenu[3].ParentMenus = MainMenu;
  
 
}

void SpeedMenuInit()
{
  SpeedMenu[0].ucMenuCount=2;
  SpeedMenu[0].ucDisplayString="Target";
  SpeedMenu[0].Subs=SetSysTargetSpeed;
  SpeedMenu[0].GetData = GetSysTargetSpeed;
  SpeedMenu[0].ChildrenMenus=NULL;
  SpeedMenu[0].ParentMenus=MainMenu;
  
  SpeedMenu[1].ucMenuCount =2;
  SpeedMenu[1].ucDisplayString = "SpeedUp";
  SpeedMenu[1].Subs = SetSysSpeedUp;
  SpeedMenu[1].GetData = GetSysSpeedUp;
  SpeedMenu[1].ChildrenMenus = NULL;
  SpeedMenu[1].ParentMenus = MainMenu;
}

void AboutMenuInit()
{
  AboutMenu[0].ucMenuCount= 4;
  AboutMenu[0].ucDisplayString="Team info";
  AboutMenu[0].Subs=NULL;
  AboutMenu[0].GetData = NULL;
  AboutMenu[0].ChildrenMenus=NULL;
  AboutMenu[0].ParentMenus=MainMenu;
  
  AboutMenu[1].ucMenuCount = 4;
  AboutMenu[1].ucDisplayString = "Load default";
  AboutMenu[1].Subs = LoadDefaultParam;
  AboutMenu[1].GetData = NULL;
  AboutMenu[1].ChildrenMenus = NULL;
  AboutMenu[1].ParentMenus = MainMenu;
  
  AboutMenu[2].ucMenuCount = 4;
  AboutMenu[2].ucDisplayString = "Vol";
  AboutMenu[2].Subs = ReadVol;
  AboutMenu[2].GetData = GetSysVol;
  AboutMenu[2].ChildrenMenus = NULL;
  AboutMenu[2].ParentMenus = NULL;
  
  AboutMenu[3].ucMenuCount = 4;
  AboutMenu[3].ucDisplayString = "TestParam";
  AboutMenu[3].Subs = TestParam;
  AboutMenu[3].GetData = NULL;
  AboutMenu[3].ChildrenMenus = NULL;
  AboutMenu[3].ParentMenus = NULL;
}


void SpeedPIDMenuInit()
{
  SpeedPIDMenu[0].ucMenuCount=4;
  SpeedPIDMenu[0].ucDisplayString="P";
  SpeedPIDMenu[0].Subs= SetSysSpeedP;
  SpeedPIDMenu[0].GetData = GetSysSpeedP;
  SpeedPIDMenu[0].ChildrenMenus=NULL;
  SpeedPIDMenu[0].ParentMenus=PIDMenu;
  
  SpeedPIDMenu[1].ucMenuCount =4;
  SpeedPIDMenu[1].ucDisplayString = "I";
  SpeedPIDMenu[1].Subs = SetSysSpeedI;
  SpeedPIDMenu[1].GetData = GetSysSpeedI;
  SpeedPIDMenu[1].ChildrenMenus = NULL;
  SpeedPIDMenu[1].ParentMenus = PIDMenu;
  
  SpeedPIDMenu[2].ucMenuCount = 4;
  SpeedPIDMenu[2].ucDisplayString = "D";
  SpeedPIDMenu[2].Subs = SetSysSpeedD;
  SpeedPIDMenu[2].GetData = GetSysSpeedD;
  SpeedPIDMenu[2].ChildrenMenus = NULL;
  SpeedPIDMenu[2].ParentMenus = PIDMenu;

  SpeedPIDMenu[3].ucMenuCount = 4;
  SpeedPIDMenu[3].ucDisplayString = "IMax";
  SpeedPIDMenu[3].Subs = SetSysSpeedIMax;
  SpeedPIDMenu[3].GetData = GetSysSpeedIMax;
  SpeedPIDMenu[3].ChildrenMenus = NULL;
  SpeedPIDMenu[3].ParentMenus = PIDMenu;
}

void AnglePIDMenuInit()
{
  AnglePIDMenu[0].ucMenuCount=2;
  AnglePIDMenu[0].ucDisplayString="P";
  AnglePIDMenu[0].Subs=SetSysAngleP;
  AnglePIDMenu[0].GetData = GetSysAngleP;
  AnglePIDMenu[0].ChildrenMenus=NULL;
  AnglePIDMenu[0].ParentMenus=PIDMenu;
  
  AnglePIDMenu[1].ucMenuCount =2;
  AnglePIDMenu[1].ucDisplayString = "D";
  AnglePIDMenu[1].Subs = SetSysAngleD;
  AnglePIDMenu[1].GetData = GetSysAngleD;
  AnglePIDMenu[1].ChildrenMenus = NULL;
  AnglePIDMenu[1].ParentMenus = PIDMenu;
}

void DirectionPIDMenuInit()
{
  DirectionPIDMenu[0].ucMenuCount=2;
  DirectionPIDMenu[0].ucDisplayString="P";
  DirectionPIDMenu[0].Subs=SetSysDirectionP;
  DirectionPIDMenu[0].GetData = GetSysDirectionP;
  DirectionPIDMenu[0].ChildrenMenus=NULL;
  DirectionPIDMenu[0].ParentMenus=PIDMenu;
  
  DirectionPIDMenu[1].ucMenuCount =2;
  DirectionPIDMenu[1].ucDisplayString = "D";
  DirectionPIDMenu[1].Subs = SetSysDirectionD;
  DirectionPIDMenu[1].GetData = GetSysDirectionD;
  DirectionPIDMenu[1].ChildrenMenus = NULL;
  DirectionPIDMenu[1].ParentMenus = PIDMenu;
}

void ElecDetecterMenuInit()
{
  ElecDetecterMenu[0].ucMenuCount= 4;
  ElecDetecterMenu[0].ucDisplayString="ElecA";
  ElecDetecterMenu[0].Subs=AdjustMagneticField;
  ElecDetecterMenu[0].GetData = GetSysElecADetecter;
  ElecDetecterMenu[0].ChildrenMenus=NULL;
  ElecDetecterMenu[0].ParentMenus=AutoAdjustMenu;
  
  ElecDetecterMenu[1].ucMenuCount = 4;
  ElecDetecterMenu[1].ucDisplayString = "ElecB";
  ElecDetecterMenu[1].Subs = AdjustMagneticField;
  ElecDetecterMenu[1].GetData = GetSysElecBDetecter;
  ElecDetecterMenu[1].ChildrenMenus = NULL;
  ElecDetecterMenu[1].ParentMenus = AutoAdjustMenu;
  
  ElecDetecterMenu[2].ucMenuCount = 4;
  ElecDetecterMenu[2].ucDisplayString = "ElecC";
  ElecDetecterMenu[2].Subs = AdjustMagneticField;
  ElecDetecterMenu[2].GetData = GetSysElecCDetecter;
  ElecDetecterMenu[2].ChildrenMenus = NULL;
  ElecDetecterMenu[2].ParentMenus = AutoAdjustMenu;
  
  ElecDetecterMenu[3].ucMenuCount = 4;
  ElecDetecterMenu[3].ucDisplayString = "ElecD";
  ElecDetecterMenu[3].Subs = AdjustMagneticField;
  ElecDetecterMenu[3].GetData = GetSysElecDDetecter;
  ElecDetecterMenu[3].ChildrenMenus = NULL;
  ElecDetecterMenu[3].ParentMenus = AutoAdjustMenu;
  

}
/******************************************/

void MenuInitialation() 
{ 
  MainMenuInit(); 
  PIDMenuInit();
  AutoAdjustMenuInit();
  SpeedMenuInit();
  AboutMenuInit();
  SpeedPIDMenuInit();
  AnglePIDMenuInit();
  DirectionPIDMenuInit();
  ElecDetecterMenuInit();
}

void ShowNumber(float fData, unsigned char ucY)
{
  char cDisplayData[20]={'\0'};
  unsigned char ucLocation = 0;
  signed int siValue=0;

  ucLocation = DATA_LOCATION;
  
  if(fData<0&&(fData>(-1)))
  {    
      OLED_Print(ucLocation,ucY,"-",SELECT_STATE);
      ucLocation=ucLocation+8;
  }
  sprintf(cDisplayData, "%d", (signed int)fData);
  
  OLED_Print(ucLocation,ucY,(unsigned char*)cDisplayData,SELECT_STATE);
  
  ucLocation = ucLocation + strlen(cDisplayData) * 8;
 
  if(fData < 0)
  {
    	fData *= -1;
  }
  siValue=(signed int)((fData - (signed int)fData)*1000);
  if(siValue!=0)
  {
    OLED_Print(ucLocation ,ucY,".",SELECT_STATE);
    ucLocation = ucLocation + 8;
    cDisplayData[0] = siValue/100%10+'0';
    cDisplayData[1] = siValue/10%10+'0';
    cDisplayData[2] = siValue%10+'0';
    OLED_Print(ucLocation,ucY,(unsigned char*)cDisplayData,SELECT_STATE);
  }
}

signed char AdjustBalanceAngle(signed char scDirection)
{ 
  BuzzOff();
  OLED_Fill(0x00);  //初始清屏
  OLED_Print(0,0,"Start Adjust!",SELECT_STATE);

  float fCarAngle = 0;                          //小车的角度
  float fCarAngleDot = 0;                       //角加速度
  int i=0;
  
  for(i=0;i<(1000);)
  {
    if(Is5msReady())
    {
      CalAngle(&fCarAngle, &fCarAngleDot);
      OLED_Print(ITEM_LOCATION, 4, "Angle", SELECT_STATE);
      ShowNumber(fCarAngle, 4);
      i++;
    }
  }
  
  SetSysBalanceAngle(fCarAngle);
  SetDefaultAngle(fCarAngle);

  return 0;
}
signed char AdjustENC03(signed char scDirection)
{ 
  BuzzOff();
  OLED_Fill(0x00);  //初始清屏
  //OLED_Print(0,0,"Start Adjust !",SELECT_STATE);
  
  int i=0;
  unsigned int uiENC03X;
  for(i=0;i<(1000);)
  {
    if(Is5msReady())
    {
      uiENC03X=GetBChannelData();
      OLED_Print(ITEM_LOCATION, 0, "ENC03X", SELECT_STATE);
      ShowNumber(uiENC03X, 0);
      i++;
     
    }
  }
  SetSysENC03X(uiENC03X);

  return 0;
}

signed char ReadVol(signed char scDirection)
{
  ReadSysVol();  
  return 0;
}

signed char AdjustMagneticField(signed char scDirection)
{
  BuzzOff();
  OLED_Fill(0x00);  //初始清屏
 // OLED_Print(0,0,"Start Adjust !",SELECT_STATE);
  
  unsigned int uiElecADetecter=0;
  unsigned int uiElecBDetecter=0;
  unsigned int uiElecCDetecter=0;
  unsigned int uiElecDDetecter=0;
  unsigned int uiElecEDetecter=0;
  unsigned int uiElecAMax=0;
  unsigned int uiElecBMax=0;
  unsigned int uiElecCMax=0;
  unsigned int uiElecDMax=0;
  unsigned int uiElecEMax=0;
  unsigned int key=0;
  while(1)
  {
      uiElecADetecter=GetDChannelData();
      uiElecBDetecter=GetEChannelData();
      uiElecCDetecter=GetFChannelData();
      uiElecDDetecter=GetGChannelData();
      uiElecEDetecter=GetCChannelData();
      if(uiElecADetecter>uiElecAMax)
      {
        uiElecAMax=uiElecADetecter;
      }
       if(uiElecBDetecter>uiElecBMax)
      {
        uiElecBMax=uiElecBDetecter;
      }
        if(uiElecCDetecter>uiElecCMax)
      {
        uiElecCMax=uiElecCDetecter;
      }
        if(uiElecDDetecter>uiElecDMax)
      {
        uiElecDMax=uiElecDDetecter;
      }
       if(uiElecEDetecter>uiElecEMax)
      {
        uiElecEMax=uiElecEDetecter;
      }
      OLED_Print(ITEM_LOCATION, 0, "ElecA", SELECT_STATE);
      ShowNumber(uiElecADetecter, 0);
      OLED_Print(ITEM_LOCATION, 2, "ElecB", SELECT_STATE);
      ShowNumber(uiElecBDetecter, 2);
      OLED_Print(ITEM_LOCATION, 4, "ElecC", SELECT_STATE);
      ShowNumber(uiElecCDetecter, 4);
      OLED_Print(ITEM_LOCATION, 6, "ElecD", SELECT_STATE);
      ShowNumber(uiElecDDetecter, 6);
      key = ReadKeyBoard();
      if(key!=0)
      {
        break;
      }  
  }
  SetSysElecADetecter(uiElecAMax);
  SetSysElecBDetecter(uiElecBMax);
  SetSysElecCDetecter(uiElecCMax);
  SetSysElecDDetecter(uiElecDMax);
  SetSysElecEDetecter(uiElecEMax);
  SetElecProportion(uiElecAMax,uiElecBMax,uiElecCMax,uiElecDMax,uiElecEMax);
  return 0;

}

void Show_Menu()
{
  unsigned long int i;
  unsigned long int uliDisplay = 0;
  
  OLED_Fill(0x00);  //初始清屏
  
  if(MenuPoint[siUserChoose].ucMenuCount >= 4)
  {
    if(siUserChoose >=4)
    {
      uliDisplay = siUserChoose-3;
    }
    for(i = 0; i < 4; i++)
    {
      if((i + uliDisplay) == siUserChoose)
      {
        OLED_Print(CURSOR_LOCATION,i*2,"*",SELECT_STATE);
        if(siConfirmFlag == SET_STATE) OLED_Print(ITEM_LOCATION,i*2,MenuPoint[i + uliDisplay].ucDisplayString,SET_STATE);
        else OLED_Print(ITEM_LOCATION,i*2,MenuPoint[i + uliDisplay].ucDisplayString,SELECT_STATE);
      }
      else
      {
        OLED_Print(ITEM_LOCATION,i*2,MenuPoint[i + uliDisplay].ucDisplayString,SELECT_STATE);
      }   
      
      if(MenuPoint[i + uliDisplay].GetData != NULL)
      {
		ShowNumber(MenuPoint[i + uliDisplay].GetData(), i*2);
      }
    }
  }
  else
  {
    for(i=0;i<MenuPoint[siUserChoose].ucMenuCount;i++)
    {
      if(i == siUserChoose)
      {
        OLED_Print(CURSOR_LOCATION,i*2,"*",SELECT_STATE);
        if(siConfirmFlag == SET_STATE) OLED_Print(ITEM_LOCATION,i*2,MenuPoint[i + uliDisplay].ucDisplayString,SET_STATE);
        else OLED_Print(ITEM_LOCATION,i*2,MenuPoint[i + uliDisplay].ucDisplayString,SELECT_STATE);
      }
      else
      {
        OLED_Print(ITEM_LOCATION,i*2,MenuPoint[i].ucDisplayString,SELECT_STATE);
      } 
      
      if(MenuPoint[i + uliDisplay].GetData != NULL)
      {
	   ShowNumber(MenuPoint[i + uliDisplay].GetData(), i*2);
      }
    }
  }
}

signed char MenuSelect()
{
  signed char ret=0;
  uint8 key; 
  key = ReadKeyBoard(); //获取按键值
  
  switch(key)
  {
  case 1://确认
    BuzzOn();
    if ( (MenuPoint[siUserChoose].GetData == NULL) && (MenuPoint[siUserChoose].Subs != NULL) )
    {
      ret = (*MenuPoint[siUserChoose].Subs)(DIRECTION_UP); 
    }
    else if((MenuPoint[siUserChoose].GetData != NULL) && (MenuPoint[siUserChoose].Subs != NULL))
    {
      if(siConfirmFlag == SELECT_STATE) siConfirmFlag = SET_STATE;
      else if(siConfirmFlag == SET_STATE)siConfirmFlag = SELECT_STATE;
    }
    else if(MenuPoint[siUserChoose].ChildrenMenus != NULL) 
    { 
      MenuPoint = MenuPoint[siUserChoose].ChildrenMenus; 
      siUserChoose = 0;
    }
    Show_Menu();
    
    BuzzOff();
    break;
    
  case 2://向上选择
    BuzzOn();
    if(siConfirmFlag == SET_STATE)
    {
      ret = (*MenuPoint[siUserChoose].Subs)(DIRECTION_UP); 
    }
    else
    {
      siUserChoose --; 
      if (siUserChoose <0) 
      { 
        siUserChoose = MenuPoint[0].ucMenuCount-1; 
      }
    }
    Show_Menu();
    BuzzOff();
    
    break;
    
  case 3://向下选择
    BuzzOn();
    if(siConfirmFlag == SET_STATE)
    {
      ret = (*MenuPoint[siUserChoose].Subs)(DIRECTION_DOWN); 
    }
    else
    {
      siUserChoose ++; 
      if(siUserChoose == MenuPoint[0].ucMenuCount) 
      {
        siUserChoose = 0; 
      } 
    }
    Show_Menu();
    BuzzOff();
    break;
    
  case 4://后退
    BuzzOn();
    if(siConfirmFlag == SET_STATE)
    {
	 siConfirmFlag = SELECT_STATE;
    }
    else
    {
	 if(MenuPoint[0].ParentMenus != NULL) 
	 {
	   MenuPoint = MenuPoint[0].ParentMenus; 
	   siUserChoose = 0; 
	 }
    }
    Show_Menu();
    BuzzOff();
    break;
  }
  
  return ret;
}

void SetMode(signed char scState)
{
  switch(scState)
  {
  case NO_ERROR:
    Show_Menu();
    break;
    
  case FINISH:// 正常结束
      OLED_Fill(0x00);	//清屏
      OLED_Print(ITEM_LOCATION, 0, "FINISH", SELECT_STATE);   
      BuzzOn();
      DelayMS(1);
      BuzzOff();
      DelayMS(1);
      BuzzOn();
      DelayMS(1);
      BuzzOff();
    break;
    
  case OVER_SPEED://出界

      OLED_Fill(0x00);	//清屏
      OLED_Print(ITEM_LOCATION, 0, "OVER_SPEED", SELECT_STATE);    
      BuzzOn();
      DelayMS(2);
      BuzzOff();
      
    break;
  }

  while(1)
  {
      if(COMPLETE == MenuSelect())
      {
        break;
      }
  }
  
  OLED_Fill(0x00);	//清屏
  OLED_Print(ITEM_LOCATION, 0, " RUN ", SELECT_STATE);
}
void DelayMS(int count)
{
  int i;
  for (i=0;i<(2*count);)
  {
    if(Is50msReady())
    {
    	i++;
    }
  }
}