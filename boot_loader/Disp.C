/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.c
;* Author             : 张力阵
;* 显示程序
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "Disp.h"
//*********************取设备所在DCx**************************************
#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 8) & 0xf)

//*********************取设备所在位*******************************************
#define SYSCTL_PERIPH_MASK(a)   ( 1<< ((a) & 0xff))

extern void Delay(unsigned long ulCount);

u8          Disp_Buf[8];                    //显示缓冲区
extern unsigned char Board_Id;

/*****************************************************************************
* 设备时钟使能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralEnable(u32 ulPeripheral)
{
	SYSCTL->RCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}
/*****************************************************************************
* 7279复位 初始化
*****************************************************************************/
void Reset_HD7279(void)
{
    SSIDataLen(SSI0,SSI_CR0_DSS_8);	      //设置数据长度为8bit
    GPIOPinWrite(GPIOA,DISP_RST,0);       //复位管脚置0
    Delay(625);                           //每个数延时160ns  延时100us
    GPIOPinWrite(GPIOA,DISP_RST,DISP_RST);//复位管脚置1
    Delay(6250);                          //每个数延时160ns  延时1mS
    SSIDataPut(SSI0,LED_RESET);           //送数据  
    Delay(62500);                         //每个数延时160ns  延时10mS
//    Disp_Data(LED_SEND_DATA_CODE2);       //复位并显示 按方式0译码(与原误差板译码方式相同     
}
/*****************************************************************************
* 方式3译码时,数据转换表
*****************************************************************************/
const u8 Disp_Code_Tab[]=
{
  // 0     1    2    3   4     5   6    7     8   9
    0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,
  // A    B     C    D   E     F   g    H    L     R 
    0x77,0x7F,0x4E,0x7E,0x4F,0x47,0x7B,0x37,0x0E,0x77, 
  // -   空格   b    d   U    t    n    o    P    N
    0x01,0x00,0x1F,0x3D,0x3e,0x0F,0x15,0x1d,0x67,0x76      
};
/*****************************************************************************
* 刷新显示缓冲区
* 刷新数据区
* Disp_Mode 数据刷新方式
* Disp_Buf[0] 对应 最左面数码管(最高位) Disp_Buf[7] 对应 最右面数码管(最低位)
*****************************************************************************/
void Disp_Data(u8 Disp_Mode)
{
    u8 m;
    u16 t;
    SSIDataLen(SSI0,SSI_CR0_DSS_16);	  //设置数据长度为8bit
    for(m=0;m<8;m++)
     {
      t=((Disp_Mode|m)<<8);
      if(Disp_Mode==LED_SEND_DATA_CODE2)
       {
        if((Disp_Buf[m]&0x7f)<sizeof(Disp_Code_Tab))
         t|=Disp_Code_Tab[Disp_Buf[m]&0x7f];
        else
         t|=Disp_Code_Tab[DISP_BLANK];
        t|=(Disp_Buf[m]&0x80);         //判断是否有小数点
       }
      else    
       t|=Disp_Buf[m];
      SSIDataPut(SSI0,t);
     }
}
//显示引导状态
void Disp_Boot(void)
{
    u8 m;
    Disp_Buf[0]=DISP_b;                  //负误差
    Disp_Buf[1]=DISP_o;                  //负误差
    Disp_Buf[2]=DISP_o;                  //负误差
    Disp_Buf[3]=DISP_t;                  //负误差
    Disp_Buf[4]=DISP_BLANK;              //负误差
    m=Board_Id;                          //表位号
    Disp_Buf[7]=m%10;                    //表位号最低位
    m/=10;                               
    Disp_Buf[6]=m%10;                    //表位号高位
    m/=10;                               
    if(m!=0)                             
     Disp_Buf[5]=m%10;                   //表位号最高位
    else                                 
     Disp_Buf[5]=DISP_BLANK;             //表位号最高位
    Disp_Data(LED_SEND_DATA_CODE2);      //按方式0译码(与原误差板译码方式相同)	
}
//显示空白状态
void Disp_Blank(void)
{
    u8 n;
    for(n=0;n<8;n++)
     {
      Disp_Buf[n]=DISP_BLANK;            //显示空白	
     }	
    Disp_Data(LED_SEND_DATA_CODE2);      //按方式0译码(与原误差板译码方式相同)	
}
