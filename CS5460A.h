/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CLK_ERR.h
;* Author             : 张力阵
;* 时钟处理函数库头文件和宏定义
*******************************************************************************/

/********************************************************
* 写一个字节到CS5460A
********************************************************/
u8 Write_Read_OneByte_5460A(u8 Byte);
/********************************************************
* 初始化CS5460A
********************************************************/
void Init_CS5460A(void);
/********************************************************
* 使能CS5460A
********************************************************/
void Enable_5460(void);
/********************************************************
* 禁止 CS5460A
********************************************************/
void Disable_5460(void);
/********************************************************
* 长整型转换成ASCII码
********************************************************/
u8 LongInt_To_Bcd(u8 *Ptr,long int d);
/********************************************************
* 开始CS5460A转换
********************************************************/
void Start_CS5460AConv(void);
/********************************************************
* 关闭CS5460A转换
********************************************************/
void Stop_CS5460AConv(void);
/********************************************************
* 处理CS5460A转换数据
********************************************************/
void New_Data_Prc(void);

