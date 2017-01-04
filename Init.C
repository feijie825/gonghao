/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
* File Name          : Init.c
* Author             : 张力阵
* 初始化
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "stdio.h"
#include "ENG_ERR.h"   //电能处理声明文件
#include "define.h"
#include "vari.h"
/*****************************************************************************
* 版本号
*****************************************************************************/
const u8 VER_TAB[]={"V1.0.4\r"};

/*****************************************************************************
* 检测标准表编号保存表格状态
* 返回Current_Save_Tab当前使用的表位号表格ID
* Save_Tab_Sts 标准表编号状态表格状态列表
*****************************************************************************/
void Check_SMtr_Tab_Sts(void)
{
    u16 m,n;
    u16 Len;
    const u8 *Str;
    u8    c;
    Len=sizeof(SAVE_S);
    Len+=0x03;
    Len&=0xFFFC;			                              //4字节对齐
    Current_Save_Tab=0;				                      //默任无保存值
    for(n=0;n<SAVE_TAB_NUMB;n++)
    {
      Str=(u8 *)&Save_Tab;
   
      Str+=n*Len;
      c=*Str;
      if(c==YES)                                 //有修正值
      {
        memcpy(&XIUZ,Str,sizeof(SAVE_S));        //拷贝保存值
        Current_Save_Tab=(n+1);		                //表位号标志为DATA_YES 该修正表格有效
        Save_Tab_Sts[n]=VALIDE;                  
      }                                         
      else if(c!=0xff)                           
       Save_Tab_Sts[n]=NOT_BLACK;	               //表位号标志不为0xff 直接置不空标志
      else                                       
      {                                         
        Save_Tab_Sts[n]=BLANK;                   //先设置表位号表格为空 可以写入表位号数据
        for(m=0;m<(Len/4);m++)
        {                                       //检查后续字节
          if(*((u32 *)Str)!=0xFFFFFFFF)
          {
            Save_Tab_Sts[n]=NOT_BLACK;
            break;
          }
          Str+=4; 
        }
      }
    }
    if(Current_Save_Tab)                         //当前保存有修正数据
    {                                           //判断修正值数据是否可用
      if(XIUZ.U_PXZ<0.2)                         //修正值是否太小
       XIUZ.U_PXZ=1.000;                         //默认为1.0
      if(XIUZ.U_SXZ<0.2)                         //修正值是否太小
       XIUZ.U_SXZ=1.000;                         //默认为1.0
      if(XIUZ.I_SXZ<0.2)                         //修正值是否太小
       XIUZ.I_SXZ=1.000;                         //默认为1.0
			if(XIUZ.U_XZ<0.2)                         //修正值是否太小
       XIUZ.U_XZ=1.000;                         //默认为1.0
      if(XIUZ.I_XZ<0.2)                         //修正值是否太小
       XIUZ.I_XZ=1.000;                         //默认为1.0
      return;                                    //有保存值退出
    } 
    XIUZ.Flag=YES;                               //标志
    XIUZ.CH0_MTR[0]=1;                           //通道1默认对应1表位
    XIUZ.CH0_MTR[1]=2;                           //通道2默认对应2表位
    XIUZ.CH0_MTR[2]=3;                           //通道3默认对应3表位
    XIUZ.U_PXZ=1.000;                            //默认修正值电压有功功率修正值
    XIUZ.U_SXZ=1.000;                            //默认修正值电压视在功率修正值	
    XIUZ.I_SXZ=1.000;                            //默认修正值电流视在功率修正值	
		XIUZ.U_XZ=1.000;                            //默认修正值电压修正值	
    XIUZ.I_XZ=1.000;                            //默认修正值电流修正值
    for(m=0;m<SAVE_TAB_NUMB;m++)                 //查看是否有空位置
    {
      if(Save_Tab_Sts[m]==BLANK)                 //是否空(可写)
       return;                                   //有空的表位号表格退出
    }                                           
    FlashErase(SAVE_BASE);                       //没有表位值 且没有空表位位置 擦除表位BANK
}
//固化参数值 即RAM->FLASH
void Solid_Save_Tab(void)
{
    u32 *Ptr,Addr,Data=0;
    u16 Len;
    u8  n;
    Len=sizeof(SAVE_S);                          //保存值长度
    Len+=0x03;
    Len&=0xFFFC;                                 //4字节对齐
    for(n=0;n<SAVE_TAB_NUMB;n++)
     {
      if(Save_Tab_Sts[n]==BLANK)
       break;                                    //有空的修正表格跳出循环
     }
    if(n==SAVE_TAB_NUMB)                         //判断是否没有空表格
     {
      FlashErase(SAVE_BASE);                     //没有表位值 且没有空表位位置 擦除表位BANK
      for(n=0;n<SAVE_TAB_NUMB;n++)
       Save_Tab_Sts[n]=BLANK;
      Current_Save_Tab=0;
      n=0;
     } 
    if(Current_Save_Tab!=0)                        
     {                                             //当前有修正值 
      Addr=(u32)(&Save_Tab);    
      Addr+=(Current_Save_Tab-1)*Len;              //当前保存值地址(需要擦除) 
      FlashProgram(&Data,Addr,4);                  //写入0 表示表位号无效
      Save_Tab_Sts[Current_Save_Tab-1]=NOT_BLACK;  //not blank
     }
    Ptr=(u32 *)&XIUZ;
    Addr=(u32)&Save_Tab;
    Addr+=n*Len;
    Save_Tab_Sts[n]=VALIDE;                        //数据有效
    Current_Save_Tab=n+1;                          //更新当前保存表格ID号
    FlashProgram(Ptr,Addr,Len);                    //编程 烧写当前值
}
//读表位号
void Read_Mtr_Numb(void)
{
    u8 m,n;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);  //使能端口G时钟
    SysCtlDelay(500);                             //延时0.08ms
//初始化GPIOG口                             
    GPIODirModeSet(GPIOG,
                   BW,
                   GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIOG,                       //端口 设置管脚类型    
                     BW,                          //管脚                 
                     GPIO_STRENGTH_8MA,           //驱动能力             
                     GPIO_PIN_TYPE_STD_WPD);      //下拉            
    SysCtlDelay(500);                             //延时0.08ms
    for(;;)                                 
     {                                      
      m=GPIOPinRead(GPIOG,BW);                    //读表位号
      SysCtlDelay(5000);                          //延时0.5ms
      n=GPIOPinRead(GPIOG,BW);                    //读表位号  
      if(m==n)                                    //稳定
       break;	   
     }
    if(m!=0xFF)
     m++;	                                 //
    Mtr_Numb_ID=m;                         //
    Mtr_Numb_Str[2]=m%10;                  //低位
    m/=10;                               
    Mtr_Numb_Str[1]=m%10;                  //低位
    m/=10;                               
    Mtr_Numb_Str[0]=m%10;                  //高位
    if(m==0)                             
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOG);//禁能端口G时钟
}
/****************************************************************************
* 字符串拷贝 将Str1 拷贝到Str2中
* Str1 待拷贝的字符串
* Str2 待写入的字符串
* Len Str2 最大长度
* 待拷贝字符串长度不够 前面填充空格 ' ' 为了显示后对齐
****************************************************************************/
void Fill_Space_StrCpy(u8 *Str1,u8 *Str2,u8 Len)
{
    u8 m,n=0;
    if(Len<2)                        //数据太短
     {	
      *Str2='\0';	
      return;
     } 
    Len--;                           //字符串包含结束符 '\0' 
    for(n=0;n<Len;n++)               //循环剔除Str1中开头的空字符' '
     {
      if(*Str1!=' ')
       break;	
      Str1++;
     }
    if(n==Len)                       //判断是否没有有效数据
     {
      for(m=0;m<Len;m++)
       {
        *Str2=' ';                   //填充空格
        Str2++;
       }
      *Str2='\0';  	 	
      return; 
     }
    else 	
     m=n=strlen((const char*)Str1);  //检查字符串Str1 实际长度
    if(m<Len)
     {
      for(;m<Len;m++)
       {
        *Str2=' ';                   //填充空格
        Str2++;
       }
      Len=n;                         //数据实际长度  	
     }
    if(Str1[Len-1]=='.')             //判断要拷贝的最后一个字符是否为'.' 
     {
      *Str2=' ';                     //多填充一个空格
      Str2++;
      Len--;	
     }
    for(m=0;m<Len;m++)
     {
      *Str2=*Str1;
      Str1++;
      Str2++;
     } 	
    *Str2='\0';  	 	
}

/*****************************************************************************
* RAM和变量初始化
*****************************************************************************/
void Init_Ram(void)
{   
    Read_Mtr_Numb();                        //读表号
	   Check_SMtr_Tab_Sts();                   //读标准表配置
    SINGLE_OR_THREE=SINGLE;                 //默认单相台
    CAN_STS.BYTE =0;                        //清除错误状态标志
    CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];    //初始化接收报文处理指针
    CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];    //初始化发送报文处理指针
    CAN_RX_OVTimer=(u16)Timer_1ms;          //初始化CAN超时定时器
    CAN_ERR=0;                              //
    CAN_SEND_DTIME=(Mtr_Numb_ID%10);        //10个为一组
    CAN_SEND_DELAY=CAN_SEND_DTIME;          //延时 避开
	   ADC_Data.Trig=1;                        //已经触发标志
    CS5460A_Vref=2.5;                       //CS5460A基准电压
    ADC0_Vref=3.0;                          //LM3S2139片内ADC基准电压

    Current_I_Value=5.0;                    //当前电流值
    memset(Rx_Com,0x00,RX_CMD_LEN);
    CHK_OneTime_Flag=0x00;  
    Current_Wire_Type=WIRE_P1;
}

