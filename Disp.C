/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.c
;* Author             : 张力阵
;* 显示程序
;* 没有标准电能脉冲   显示 no bEP
;* 没有标准时钟脉冲   显示 no bCP
;* 没有被检表时钟脉冲 显示 no cLP
;* 时钟脉冲没有稳定   显示 no Stb
;* 电压跌落完成       显示 dL   End
;* 时钟频率           显示 Fxxxxxxx
;* 日计时误差         显示 dxxxxxxx
;* 需量周期           显示 Lxxxxxxx
;* 收到合闸脉冲       显示 H  R  H2
;* 收到投切脉冲       显示 5  R  Sd
;* 内置跳闸           OPEN      01
;* 继电器故障         ERR       01
** 
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "stdlib.h"
#include "Disp.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* 7279复位 初始化
*****************************************************************************/
void Reset_HD7279(void)
{
    SSIDataLen(SSI0,SSI_CR0_DSS_8);	      //设置数据长度为8bit
    DISP_RST_EN;
    SysCtlDelay(625);                     //每个数延时160ns  延时100us
    DISP_RST_DN;
    SysCtlDelay(6250);                    //每个数延时160ns  延时1mS
    SSIDataPut(SSI0,LED_RESET);           //送数据  
    SysCtlDelay(62500);                   //每个数延时160ns  延时10mS
//    Disp_Data(Disp_Code_Mode);            //复位并显示 按方式0译码(与原误差板译码方式相同     
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
    0x01,0x00,0x1F,0x3D,0x3e,0x0F,0x15,0x1d,0x67,0x76,
  // c   8.
    0x0d,0xFF        
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
    if(Disp_En_Timer)                   //延时未到
     return;	 
    SSIDataLen(SSI0,SSI_CR0_DSS_16);	  //设置数据长度为16bit
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
      SysCtlDelay(250);                //延时40uS
     }
    Disp_Timer=Timer_8ms;              //刷新显示定时器
    Disp_En_Timer=DISP_EN_TIME;        //显示使能定时
}
/*****************************************************************************
* 更新当前校验圈数显示缓冲区
* 只显示后两位 显示在显示窗最左端
*****************************************************************************/
void Update_N_Buf(void)
{
    u8 m=CURRENT_N;                        //当前圈数
    CURRENT_N_ASC[1]=(m%10)|'0';           //圈数低位
    m/=10;                                 //圈数高位
    if(m>9)                                //判断当前圈数是否超过100
     m%=10;                                //
    CURRENT_N_ASC[0]=(m|'0');              //圈数高位
    Disp_Buf[0]=(CURRENT_N_ASC[0]&0x0f);   //圈数高位
    Disp_Buf[1]=(CURRENT_N_ASC[1]&0x0f);   //圈数最低位
}
/*****************************************************************************
* 更新表位号显示缓冲区
*****************************************************************************/
void Update_Mtr_Num(void)
{					   
      Disp_Buf[5]=Mtr_Numb_Str[0];         //高位
      Disp_Buf[6]=Mtr_Numb_Str[1];         //中位
      Disp_Buf[7]=Mtr_Numb_Str[2];         //低位
}
/*****************************************************************************
* 拷贝字符串数据到显示缓冲区
* 入口: Len    要拷贝数据的长度
* 人口: Offset 显示缓冲区起始地址 显示低位(缓冲区高位)
* 人口：*Ptr   要拷贝数据的指针 要显示的数据  
*****************************************************************************/
void Copy_Str_To_DSBUF(u8 Len,u8 Offset,u8 *Ptr)
{
    u8 m,n=Offset,d;
    if(GZ_FLAG||NZTZ_FLAG)              //是否处于故障和跳闸状态
     return;                            //退出 不再显示其他内容	 
    for(m=0;(m<Len&&n<8);m++,n++)       //循环处理显示
     {
      if(Ptr[m]!='.')                   //判断是否为小数点
       {                                
        if((Ptr[m]==' ')||              //是否为空格
           (Ptr[m]=='\0')||             //字符串结束  
           (Ptr[m]=='+'))               //是否为'+'
         d=DISP_BLANK;                   //不显示
        else if(Ptr[m]=='-')            //负号
         d=DISP_MINUS;                   //'-'
        else if(Ptr[m]=='H')            //
         d=DISP_H;                      //显示'H'
        else if(Ptr[m]=='L')
         d=DISP_L;                      //显示'L'
        else                         
         d=Ptr[m]-'0';                  //ASC变为
        Disp_Buf[n]=d;
       }
      else                              //小数点处理
       {                                
        if(n==Offset)                   
         {                              
          Disp_Buf[2]=0x80;             //显示小数点
          n=3;                          
         }                              
        else                            
         {                              
          if(n>Offset)                  
           n--;                         //回到上个字符
          else                          
           n=Offset;                    // 
          Disp_Buf[n]|=0x80;            //显示小数点
         } 
       }
     }
} 
/*****************************************************************************
* 显示长整型数据
* 入口: Len  最大显示长度
* 人口: Addr 显示起始地址 显示低位(缓冲区高位) 1--8
* 人口：Data 要显示的数据  
* 人口: Sts  0:高位补零 1:高位不显示
*****************************************************************************/
void Disp_Long_Data(u8 Len,u8 Addr,u32 Data,u8 Sts)
{
    u8 m;
    if(Addr==0)
     return;
    if(GZ_FLAG||NZTZ_FLAG)             //是否处于故障和跳闸状态
     return;                           //退出 不再显示其他内容	 
    Addr--; 
    for(m=0;m<Len;m++)
     {
      Disp_Buf[Addr]=Data%10;          //转化成十进制BCD码
      Data/=10;                        //去掉低位
      if((Addr==0)||(Data==0))         
       break;
      Addr--;
     }
   for(;m<Len;m++)
    {
     if(Addr==0)
      break;
     Addr--; 
     if(Sts)
      Disp_Buf[Addr]=DISP_BLANK;        //不显示
     else
      Disp_Buf[Addr]=0;                 //显示0
    } 
   Disp_Timer=(Timer_8ms-DISP_TIME);    //更新显示
//   Disp_Data(Disp_Code_Mode);          //按方式0译码(与原误差板译码方式相同)      
}
/*****************************************************************************
* 显示定时处理
*****************************************************************************/
void Disp_Time_Pr(void)
{
    if((u8)(Timer_8ms-Disp_Timer)<DISP_TIME)//判断显示定时是否到abs(Timer_8ms-Disp_Timer)
     return;                            //不到 退出
    Disp_Data(Disp_Code_Mode);          //复位并显示 按方式0译码(与原误差板译码方式相同     
}            

