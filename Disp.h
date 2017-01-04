/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.h
;* Author             : 张力阵
;* 显示程序声明和预定义
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"

#define  DISP_TIME              (480/TIMER_8MS) //显示定时 单位:ms 480ms
//显示选择
#define  DISP_TEST_DATA         '0'             //显示试验数据
#define  DISP_CLK_FREQ          '1'             //显示时钟频率
#define  DISP_DAY_ERR           '2'             //显示日计时误差
#define  DISP_XUL_TIME          '3'             //显示需量周期
#define  DISP_TQMC              '4'             //显示投切脉冲
#define  DISP_HZMC              '5'             //显示合闸脉冲

#define  DISP_ENG_LEN           7               //电能误差数据长度
#define  DISP_ENG_OFFSET        2               //电能误差显示偏移量

#define  DISP_FREQ_LEN          8               //时钟频率数据长度
#define  DISP_FREQ_OFFSET       1               //时钟频率显示偏移量

#define  DISP_DAY_LEN           7               //日计时误差数据长度
#define  DISP_DAY_OFFSET        2               //日计时误差显示偏移量

#define  DISP_XUL_LEN           7               //需量周期数据长度
#define  DISP_XUL_OFFSET        2               //需量周期显示偏移量

#define  DISP_PWR_LEN           9               //电能误差数据长度
#define  DISP_PWR_OFFSET        0               //电能误差显示偏移量

#define  DISP_PLS_LEN           8               //脉冲显示长度
#define  DISP_PLS_OFFSET        0               //脉冲显示偏移量

#define  DISP_TIME_LEN          8               //脉冲周期数据长度
#define  DISP_TIME_OFFSET       0               //脉冲周期显示偏移量

//显示代码
#define  DISP_0                 0x00            //显示 0
#define  DISP_1                 0x01            //显示 1
#define  DISP_2                 0x02            //显示 2
#define  DISP_3                 0x03            //显示 3
#define  DISP_4                 0x04            //显示 4
#define  DISP_5                 0x05            //显示 5
#define  DISP_6                 0x06            //显示 6
#define  DISP_7                 0x07            //显示 7
#define  DISP_8                 0x08            //显示 8
#define  DISP_9                 0x09            //显示 9
#define  DISP_A                 0x0A            //显示 A
#define  DISP_B                 0x0B            //显示 B
#define  DISP_C                 0x0C            //显示 C
#define  DISP_D                 0x0D            //显示 D
#define  DISP_E                 0x0E            //显示 E
#define  DISP_F                 0x0F            //显示 F
#define  DISP_G                 0x10            //显示 G
#define  DISP_H                 0x11            //显示 H
#define  DISP_L                 0x12            //显示 L
#define  DISP_R                 0x13            //显示 R
#define  DISP_MINUS             0x14            //显示 '-' 负号
#define  DISP_BLANK             0x15            //清除显示缓冲区数据 空格
#define  DISP_b                 0x16            //显示 b
#define  DISP_d                 0x17            //显示 d
#define  DISP_U                 0x18            //显示 U
#define  DISP_t                 0x19            //显示 t
#define  DISP_n                 0x1A            //显示 n
#define  DISP_o                 0x1B            //显示 o
#define  DISP_P                 0x1C            //显示 P
#define  DISP_N                 0x1D            //显示 N
#define  DISP_c                 0x1E            //显示 c
#define  DISP_ALL               0x1F            //显示 8.
                 
#define  DISP_FILL_0             0              //补0
#define  DISP_FILL_BLANK         1              //补空格
//HD7279命令
#define  LED_RESET              0xA4            //HD7279复位纯指令
#define  LED_TEST               0xBF            //HD7279测试纯指令
#define  LED_LEFT               0xA1            //左移指令
#define  LED_RIGHT              0xA0            //右移指令
#define  LED_LOOP_LEFT          0xA3            //循环左移指令
#define  LED_LOOP_RIGHT         0xA2            //循环右移指令
#define  LED_SEND_DATA_CODE0    0x80            //下载数据并按方式0译码 指令后三位代表LED位置 1字节数据 
                                                //数据位 定义 d7 DP LED小数点  d0~d3 显示数据
                                                // 0~9 对应 LED 0~9
                                                // 0x0A(-) 
                                                // 0x0B(E) 
                                                // 0x0C(H) 
                                                // 0x0D(L)
                                                // 0x0E(P)
                                                // 0x0F( ) 
#define  LED_SEND_DATA_CODE1    0xC8            //下载数据并按方式1译码 
                                                //数据位 定义 d7 DP LED小数点  d0~d3 显示数据
                                                // 0~9 对应 LED 0~9
                                                // 0x0A(A) 
                                                // 0x0B(b) 
                                                // 0x0C(C) 
                                                // 0x0D(d)
                                                // 0x0E(E)
                                                // 0x0F(F) 
#define  LED_SEND_DATA_CODE2    0x90            //下载数据不译码
                                                //数据位 定义 d7 DP 
                                                //数据位 定义 d6 SEGA 
                                                //数据位 定义 d5 SEGB 
                                                //数据位 定义 d4 SEGC 
                                                //数据位 定义 d3 SEGD 
                                                //数据位 定义 d2 SEGE 
                                                //数据位 定义 d1 SEGF 
                                                //数据位 定义 d0 SEGG 
#define  LED_BLINK              0x88            //闪烁控制
                                                //D7 数码管8 D0 数码管1
#define  LED_OFF                0x98            //消隐控制 关闭数码管
                                                //D7 数码管8 D0 数码管1  
#define  LED_SEG_ON             0xE0            //LED段点亮指令
                                                //D0~D5 段寻址 0~LED1-G 1~LED1-F ... 7~LED-. 0x3F~LED8-.
#define  LED_SEG_OFF            0xC0            //LED段关闭指令
                                                //D0~D5 段寻址 0~LED1-G 1~LED1-F ... 7~LED-. 0x3F~LED8-.
#define  READ_KEY               0x15            //读键盘命令


void Reset_HD7279(void);                 //7279复位 
void Disp_Data(u8 Disp_Mode);            //刷新显示缓冲区
void Disp_Mtr_Num(void);                 //显示表位号 
/*****************************************************************************
* 更新当前校验圈数显示缓冲区
* 只显示后两位 显示在显示窗最左端
*****************************************************************************/
void Update_N_Buf(void);
/*****************************************************************************
* 更新表位号显示缓冲区
*****************************************************************************/
void Update_Mtr_Num(void);
/*****************************************************************************
* 显示长整型数据
* 入口: Len  最大显示长度
* 人口: Addr 显示起始地址 显示低位(缓冲区高位)
* 人口：Data 要显示的数据  
* 人口: Sts  0:高位补零 1:高位不显示
*****************************************************************************/
void Disp_Long_Data(u8 Len,u8 Addr,u32 Data,u8 Sts);
/*****************************************************************************
* 显示定时处理
*****************************************************************************/
void Disp_Time_Pr(void);             
/*****************************************************************************
* 拷贝字符串数据到显示缓冲区
* 入口: Len    要拷贝数据的长度
* 人口: Offset 显示缓冲区起始地址 显示低位(缓冲区高位)
* 人口：*Ptr   要拷贝数据的指针 要显示的数据  
*****************************************************************************/
void Copy_Str_To_DSBUF(u8 Len,u8 Offset,u8 *Ptr);
/*****************************************************************************
* 显示走字脉冲数
*****************************************************************************/
void Disp_ZZ_Pls(void);                    
                   
