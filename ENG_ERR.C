/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : ENG_ERR.c
;* Author             : 张力阵
;* 电能脉冲处理函数库
;* 误差板工作模式 见 vari.h 中MODE 定义
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "stdio.h"	      //工程中头文件
#include "string.h"
#include "define.h"
#include "vari.h"
#include "ENG_ERR.h"
/********************************************************
* 检测PLL时钟 200ms 检测一次 寄存器
* 实时检测 标准晶振定时(131ms一次)
* 默认不使用 RCC2
* 有标准晶振时 STD_CLK_Cnt 相当于 131ms定时器
* 没有标准晶振 不影响检测(无法通过标准晶振检测PLL错误)
* 但是标准晶振频率超过1000k时可能有问题
* 标准晶振频率改变时 要调整比较值 即if(t<76)中的76
* 经测试 正常计数值为131 PLL异常失败 计数值约为42
* 即 CPU速度变慢(8mHz) Timer_1ms 计数速度为PLL正常时的 8/25=0.32(32%)
* 标准晶振中断时间不变 仍为131ms Timer_1ms 计数值减少 时间时间为3.125ms
* 检测失锁标志
********************************************************/
void Check_PLL(void)
{
    if(((u8)(Timer_8ms-PLL_CHK_Timer))>=PLL_CHK_TIME)//判断显示定时是否到
     {
      u32  ulRCC,ulRCC2;
      PLL_CHK_Timer=Timer_8ms;                    //定时到重启定时
      if(!(SYSCTL->RIS & SYSCTL_INT_PLL_LOCK))    //PLL失锁标志
       {
        PLL_ERR_Cnt=0;                            //	
        Init_Pll();                               //初始化锁相环	 
        return;	
       }
      ulRCC = SYSCTL->RCC;                        //读取RCC  运行模式时钟配置寄存器
      ulRCC2 = SYSCTL->RCC2;                      //读取RCC2 运行模式时钟配置寄存器2
      ulRCC&=((SYSCTL_RCC_ACG|                    //时钟门控域 
               SYSCTL_RCC_SYSDIV_M|               //系统分频域
               SYSCTL_RCC_USESYSDIV|              //使用分频域
               SYSCTL_RCC_PWRDN|                  //掉电域
               SYSCTL_RCC_BYPASS|                 //PLL旁路域
               SYSCTL_RCC_XTAL_M|                 //晶振域
               SYSCTL_RCC_OSCSRC_M|               //振荡器选择域
               SYSCTL_RCC_IOSCDIS|                //内部振荡器禁能域
               SYSCTL_RCC_MOSCDIS));              //主振荡器禁能域
      if((ulRCC2&SYSCTL_RCC2_USERCC2)||           //不使用RCC2
         (ulRCC!=SysRCC_CFG))                     
       {   
        PLL_ERR_Cnt=0;                            //清除错误计数
        Init_Pll();                               //初始化锁相环	 
       }  
     }  
}
/*****************************************************************************
* 功耗测试结果发送,通过 CAN总线。
*****************************************************************************/
void Power_Mea_Resualt_Send(void)
{
   if(ADC_Start!='Y')
   {
     Rslt_Send_Timer=(u16)Timer_1ms;
     return;
   }  
   else
   {
     if((u16)(Timer_1ms-Rslt_Send_Timer)<RSLT_SEND_TIME)   //没有到发送时间，返回
       return;                                             //2秒钟发送一次
     else
       Rslt_Send_Timer=(u16)Timer_1ms;
       TEMP_STR[0]=Current_Mea_MetNUM;            //测量表位
       TEMP_STR[1]=U_Pr_Str[1];                   //
       TEMP_STR[2]=U_Pr_Str[0];                   //电压有功功耗
       TEMP_STR[3]=U_Pm_Str[1];                   //
       TEMP_STR[4]=U_Pm_Str[0];                   //电压视在功耗
       TEMP_STR[5]=I_Pm_Str[1];                   //
       TEMP_STR[6]=I_Pm_Str[0];                   //
       Send_Data(CAN_OCMD_MEA,                    //命令
                7,
                TEMP_STR);
   }
}

//字符串连接 被粘字符串长度小于20 粘贴字符串长度小于10 
void Rmstrcat (unsigned char *str1,unsigned char *str2)
{
     unsigned char Len=0;
     for(;;)
      {
       if(*str1=='\0')
        break;
       str1++;
       Len++;
       if(Len>40)
        return; 
      }
     for(Len=0;Len<15;Len++)
      {
       *str1=*str2;
       if(*str2=='\0')
        return;
       str1++;
       str2++;  
      } 
     if(Len==15)
      *str1='\0'; 
}


/*****************************************************************************
* 功耗测试结果查询时发送,通过RS485总线。
*****************************************************************************/
void Power_Mea_Resualt_Com_Send(void)
{
   if(ADC_Start=='Y')                                         //
   {
     if(CHK_OneTime_Flag=='Y')
     {
       CHK_OneTime_Flag=0x00;
       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
       UART_TEMP_STR[0]='C';
       UART_TEMP_STR[1]='W';
       UART_TEMP_STR[2]=':';
       UART_TEMP_STR[3]=(Current_Mea_MetNUM/10)|'0';
       UART_TEMP_STR[4]=(Current_Mea_MetNUM%10)|'0';
       UART_TEMP_STR[5]=',';
       Rmstrcat(UART_TEMP_STR,U_Pr_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,U_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Pm_Str_Com);
			  Rmstrcat(UART_TEMP_STR,"\r");
//       SendStr(COM0_BUF,                  //命令
//              UART_TEMP_STR,
///              0);

//       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
			 Rmstrcat(UART_TEMP_STR,U_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Str_Com);
       Rmstrcat(UART_TEMP_STR,"\r");
     
       SendStr(COM0_BUF,                  //命令
              UART_TEMP_STR,
              0);
       COM0_OCoun=0x01;
     }  
   } 
}
/*****************************************************************************
* 功耗测试结果查询时发送,通过RS485总线。
*****************************************************************************/
void Ui_Mea_Resualt_Com_Send(void)
{
   if(ADC_Start=='Y')                                         //
   {
     if(CHK_ONETIME_FLAG1=='Y')
     {
       CHK_ONETIME_FLAG1=0x00;
       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
       UART_TEMP_STR[0]='Q';
       UART_TEMP_STR[1]='W';
       UART_TEMP_STR[2]=':';
       UART_TEMP_STR[3]=(Current_Mea_MetNUM/10)|'0';
       UART_TEMP_STR[4]=(Current_Mea_MetNUM%10)|'0';
       UART_TEMP_STR[5]=',';
       Rmstrcat(UART_TEMP_STR,U_Pr_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,U_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Pm_Str_Com);
			 Rmstrcat(UART_TEMP_STR,",");
//       SendStr(COM0_BUF,                  //命令
//              UART_TEMP_STR,
///              0);

//       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
			 Rmstrcat(UART_TEMP_STR,U_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Str_Com);
       Rmstrcat(UART_TEMP_STR,"\r");
     
       SendStr(COM0_BUF,                  //命令
              UART_TEMP_STR,
              UART_TEMP_STR_LEN);
       COM0_OCoun=0x01;
     }  
   } 
}


/*
void Power_Mea_Resualt_Com_Send(void)
{
   if(ADC_Start!='Y')                                         //
   {
     Rslt_Com_Send_Timer=(u16)Timer_1ms;
     return;
   }
   else
   {
     if((u16)(Timer_1ms-Rslt_Com_Send_Timer)<RSLT_SEND_TIME)   //没有到发送时间，返回
       return;                                             //2秒钟发送一次
     else
       Rslt_Com_Send_Timer=(u16)Timer_1ms;
       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
       UART_TEMP_STR[0]='C';
       UART_TEMP_STR[1]='W';
       UART_TEMP_STR[2]=':';
       UART_TEMP_STR[3]=(Current_Mea_MetNUM/10)|'0';
       UART_TEMP_STR[4]=(Current_Mea_MetNUM%10)|'0';
       UART_TEMP_STR[5]=',';
       Rmstrcat(UART_TEMP_STR,U_Pr_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,U_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,"\r");
     
       SendStr(COM0_BUF,                  //命令
              UART_TEMP_STR,
              0);
       COM0_OCoun=0x01;
      
   } 
}
*/
/*****************************************************************************
* AD采样定时超时处理
*****************************************************************************/
void Proc_ADC_Timer(void)
{
    if(ADC_Data.Trig)                             //是否已经触发AD采样
     {
       if((u16)(Timer_1ms-ADC_Timer)<AD_OVER_TIME)//判断AD采样是否超时
        return;
       Init_Adc();                                //重新初始化ADC 
     }		
    else
     {	
       if((u16)(Timer_1ms-ADC_Timer)<AD_TRIG_TIME)//判断AD采样定时是否到
        return;
       ADCProcessorTrigger(0);                    //启动采样
     	 ADC_Timer=(u16)Timer_1ms;                  //重置采样定时 
       ADC_Data.Trig=1;
     }
}   
/*****************************************************************************
* 处理AD采样值 
* 是否累加
*****************************************************************************/
void Proc_ADC_Data(void)
{
    if(ADC_Start!='Y')
    	return;
    if(ADC_Data.New_Data)            //判断是否有新数据
    {
       u16 Data;
       float f,f_Temp;
       u8 lon,temp; 
       u8 str[20];
     
       ADC_Data.New_Data=0;          //清除新数据标志 
       Data=ADC_Data.Data;           //拷贝采样值
       f=Data*ADC0_Vref*XIUZ.I_SXZ/1024;
	      I_Pmax[I_Mea_Cnt]=f*Current_I_Value/20;  //乘电流值(默认5A).
	      I_Mea_Cnt++;                  //电流测量次数
	      f=0;
	      if(I_Mea_Cnt>=I_MEA_CNT)      //测量次数达到5次
	      {
	        I_Mea_Cnt=0;
	        lon=I_MEA_CNT;
         temp=lon;
         for(;lon>0;lon--)
          f+=I_Pmax[lon-1];
         f/=temp;                    //取5次测量的平均值 
         f_Temp*=0;
         f_Temp=f;
         f+=0.0005;                  // 
         f*=1000;                    //这里添加浮点数转换程序
         Data=(u16)f;
         I_Pm_Str[0]=(u8)((Data&0xff00)>>8);
         I_Pm_Str[1]=(u8)(Data&0x00ff);
        
         sprintf(str,"%6.4f",f_Temp);
         memcpy(I_Pm_Str_Com,str,7);
         I_Data_Ready='Y';
         ADC_Data.Data=0;
	      }   
    }
}

/*****************************************************************************
* 外置看门狗喂狗
* 
*****************************************************************************/
void Ext_WDT_Feed(void)
{
  if((u8)(Timer_1ms-Ext_WDT_Timer)<EXT_WDT_TIME)            //判断AD采样是否超时
    return;
  Ext_WDT_Timer=(u8)Timer_1ms;
  WDI_CHANGE;
}
