/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
* File Name          : main.c
* Author             : 张力阵
* 主函数
*******************************************************************************/
#include "LM3S2139.h"   // include <*.h> 表示系统文件夹 include "*.h"  表示工程文件夹头文件
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
#include "Disp.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
#include "math.h"
#include "stdmtr.h"
#include "CS5460A.h"
int main(void)
{  
    SysCtlDelay(3125000);                  //延时500mS
    IntMasterDisable();                    //开机禁能中断
    Init_Pll();                            //初始化锁相环
    Init_Gpio();                           //初始化I/O口
    SysCtlDelay(625000);                   //延时100ms
    Init_SysTick();                        //初始化系统节拍定时器 并使能中断
    Init_CS5460A();                        //初始化CS5460A
    Init_Uart();                           //初始化串口
    Init_Adc();                            //初始化ADC
    Init_Ram();                            //初始化RAM区和变量
    Init_CAN();                            //测试CAN接口
    //Init_Wdt();                            //初始化看门狗并启动
    Init_Int();                            //初始化NVIC中断使能和优先级配置
    IntMasterEnable();                     //CPU中断允许
	   ADC_Start='Y';                         //开启ADC转换
    Start_CS5460AConv();                   //开启CS5460A转换
    for(;;)                                
     {	                                    
       //WDTFeed();                          
       Ext_WDT_Feed();                     
                                           
       Proc_SDATA_IBUF();                  //处理CAN短数据接收指令缓冲区
       Proc_SDATA_OBUF();                  //处理CAN短数据发送指令缓冲区
       Proc_CAN_OvTm();                    //CAN 总线超时处理
       Proc_CAN_STS();                     //处理CAN总线状态
                                           
       Proc_COM0_IBuf();                   //处理串口1接收缓冲区
       Proc_COM0_OBuf();                   //处理串口1发送缓冲区
       Proc_ADC_Timer();                   //AD采样定时/超时处理
                                           
       Check_PLL();                        //检测PLL时钟
                                           
       New_Data_Prc();                     //CS5460A转换程序
       Proc_ADC_Data();                    //ADC数据处理
                                           
       Power_Mea_Resualt_Send();           //功耗测试数据(CAN)定时发送
       Power_Mea_Resualt_Com_Send();       //功耗测试数据(RS485)定时发送
			 Ui_Mea_Resualt_Com_Send();
      
     }
}
