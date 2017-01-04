/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CS5460A.c
;* Author             : 张林
;* CS5460A处理程序
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
#include "stdio.h"
/*****************************************************************************
* 写1个字节到CS5460A
* 人口:要写入的字节
* 出口:SDO引脚读回的数据
*****************************************************************************/
u8 Write_Read_OneByte_5460A(u8 Byte)
{
  u8 m,n;
  n=0;
  for(m=0;m<8;m++)
  {
  	 n<<=1;
		  if(CS5460A_SDO_B)
			   n|=0x01;
		  if(Byte&0x80)
	     CS5460A_SDI_H;
	   else
      CS5460A_SDI_L;
      
    CS5460A_SCLK_H;
   	SysCtlDelay(CS5460A_RW_Delay);
   	CS5460A_SCLK_L;
   	SysCtlDelay(CS5460A_RW_Delay);
    Byte<<=1;
	 }	  
  return(n);    
}
/*****************************************************************************
* 初始化CS5460A
* 人口:
* 出口:
*****************************************************************************/
void Init_CS5460A(void)
{ 
  u8 Dat;
  Dat*=0;	
  CS5460A_RESET_L;
  SysCtlDelay(200000);           //25ms
	 CS5460A_RESET_H; 
	 SysCtlDelay(400000);           //50ms
	 CS5460A_SCLK_L;
	 CS5460A_CS_L;
	 SysCtlDelay(CS5460A_RW_Delay);

	 Dat=Write_Read_OneByte_5460A(INIT_SYNC1_START);
	 Dat=Write_Read_OneByte_5460A(INIT_SYNC1_START);
	 Dat=Write_Read_OneByte_5460A(INIT_SYNC1_START);
	 Dat=Write_Read_OneByte_5460A(INIT_SYNC0_END);                 //初始化5460A输出口
	  
  Dat=Write_Read_OneByte_5460A(REGISTER_WRITE|CONFIG_REG_ADDR); //init configure reg
	 Dat=Write_Read_OneByte_5460A(0x00);      //(0x00),高位字节
	 Dat=Write_Read_OneByte_5460A(0x10);      //下降沿中断,中间字节 0x10
	 Dat=Write_Read_OneByte_5460A(0x62);      //dividor=1 ;V I h-pass enable低位字节
 
	 Dat=Write_Read_OneByte_5460A(REGISTER_WRITE|POFF_REG_ADDR);
	 Dat=Write_Read_OneByte_5460A(0x00);
	 Dat=Write_Read_OneByte_5460A(0x00);
	 Dat=Write_Read_OneByte_5460A(0x00);      //init poffset reg
  
  Dat=Write_Read_OneByte_5460A(REGISTER_WRITE|PULSE_RATE_REG_ADDR);
	 Dat=Write_Read_OneByte_5460A(0x18);      //init pow freq reg
	 Dat=Write_Read_OneByte_5460A(0x6a);      //init pow freq reg
	 Dat=Write_Read_OneByte_5460A(0x00);      //init pow freq reg
  
	 Dat=Write_Read_OneByte_5460A(REGISTER_WRITE|MASK_REG_ADDR);	
	 Dat=Write_Read_OneByte_5460A(0x80);      //init mask reg
	 Dat=Write_Read_OneByte_5460A(0x00);      //init mask reg
	 Dat=Write_Read_OneByte_5460A(0x00);      //init mask reg

  SysCtlDelay(CS5460A_RW_Delay);
	 CS5460A_CS_H;	  
}
/*****************************************************************************
* 使能CS5460A
* 人口:
* 出口:
*****************************************************************************/
void Enable_5460(void)
{
  GPIOPinIntDisable(GPIOD,CS5460A_INT);   //禁能，外部中断

  CS5460A_SCLK_L;
	 CS5460A_CS_L;
}
/*****************************************************************************
* 禁能CS5460A
* 人口:
* 出口:
*****************************************************************************/
void Disable_5460(void)
{
	 CS5460A_CS_H;
  GPIOPinIntClear(GPIOD,CS5460A_INT);      //清除外部中断标志位
  GPIOPinIntEnable(GPIOD,CS5460A_INT);     //使能外部中断
}
/*****************************************************************************
* 停止CS5460A转换
* 人口:
* 出口:
*****************************************************************************/
void Stop_CS5460AConv(void)
{
  u8 Dat;
  Dat*=0;
	 Enable_5460();
	 SysCtlDelay(CS5460A_RW_Delay);
	 Dat=Write_Read_OneByte_5460A(HALT_AND_SLEEP);   //休眠
	 SysCtlDelay(CS5460A_RW_Delay);
	 Disable_5460();
}
/*****************************************************************************
* 开始CS5460A转换
* 人口:
* 出口:
*****************************************************************************/
void Start_CS5460AConv(void)
{
  u8 Dat;
  Dat*=0;
  Enable_5460();
  SysCtlDelay(CS5460A_RW_Delay);
	 Dat=Write_Read_OneByte_5460A(POWER_UP);                //上电命令
	 SysCtlDelay(CS5460A_RW_Delay);
	 CS5460A_CS_H;
	 SysCtlDelay(CS5460A_RW_Delay);
	 CS5460A_SCLK_L;
  CS5460A_CS_L;
	 SysCtlDelay(CS5460A_RW_Delay);
	 Dat=Write_Read_OneByte_5460A(START_MULTI_CONVERSION);  //启动连续转换
	 SysCtlDelay(CS5460A_RW_Delay);
	 Disable_5460();
}

//5460新数据处理
void New_Data_Prc(void)
{
  float f1,f2,f3,f4;
	float U_tmp=0,I_tmp=0;
	 u8 Lon,temp,str[20];
  u16  Data;
	 if(ADC_Start!='Y')
   return;
	 if((u16)(Timer_1ms-Timer_CS5460A_INT)>2000)               //5460中断超时 2s
  {
	   Init_CS5460A();                            //重新初始化5460 
	   Timer_CS5460A_INT=(u16)Timer_1ms; 
		  SysCtlDelay(1560);                       //250us
		  Start_CS5460AConv();
	 }
  if(CS5460A_New_Data!='Y')
	  return;
	 CS5460A_New_Data=0x00;
  Temp_Cal1.Chrd[3]=0;
	 Temp_Cal1.Chrd[2]=CS5460A_Vrms_Reg[0];
	 Temp_Cal1.Chrd[1]=CS5460A_Vrms_Reg[1];
	 Temp_Cal1.Chrd[0]=CS5460A_Vrms_Reg[2];
	 f1=47.02/0.2*CS5460A_Vref;                          //电压线圈电压有效值
	 f1=(float)Temp_Cal1.Intd*f1/0x1000000;
	 f1*=XIUZ.U_XZ;
	 f1-=XIUZ.U_OFFSET;           //减去电压零点修正值//y=kx-b
    // if(f1>=0.1)
	 //f1+=((1-f1/XIUZ.XZDU)*XIUZ.U_OFFSET); //减去斜率误差
		 
	 Temp_Cal1.Chrd[3]=0;
	 Temp_Cal1.Chrd[2]=CS5460A_Irms_Reg[0];
	 Temp_Cal1.Chrd[1]=CS5460A_Irms_Reg[1];
	 Temp_Cal1.Chrd[0]=CS5460A_Irms_Reg[2];
	 f2=CS5460A_Vref*100;
	 f2=(float)Temp_Cal1.Intd*f2/0x1000000;    //电压线圈电流有效值 MA
	 f2*=XIUZ.I_XZ;
		f2-=XIUZ.I_OFFSET;         //减去电流零点修正值
    // if(f2>=0.001)
	 //f2+=((1-f2/XIUZ.XZDI)*XIUZ.I_OFFSET);   //减去斜率误差
		 
	 Temp_Cal1.Chrd[3]=0;
	 Temp_Cal1.Chrd[2]=CS5460A_E_Reg[0]&0x7f;
	 Temp_Cal1.Chrd[1]=CS5460A_E_Reg[1];
	 Temp_Cal1.Chrd[0]=CS5460A_E_Reg[2];
	 U_Prms[U_Mea_Cnt]=-1*((CS5460A_E_Reg[0]&0x80)/0x80)+(float)Temp_Cal1.Intd/IVPE_DIVED_VALUE;
//	 U_Prms[U_Mea_Cnt]=-1*2*(4.702/0.2)*U_Prms[U_Mea_Cnt]*CS5460A_Vref*CS5460A_Vref*XIUZ.U_PXZ;//-f1*f1/470201;  //电压线圈有功功率 -f1*f1/470201
	 U_Prms[U_Mea_Cnt]=-1*2*(4.702/0.2)*U_Prms[U_Mea_Cnt]*CS5460A_Vref*CS5460A_Vref*XIUZ.U_PXZ-f1*f1/4001000;  //电压线圈有功功率 -f1*f1/470201
//	 U_Pmax[U_Mea_Cnt]=f1*f2/1000;//-f1*f1/470201;//*XIUZ.U_SXZ  2*                                             //电压线圈电流放大比例改为1:2 -f1*f1/470201;
	 U_Pmax[U_Mea_Cnt]=f1*f2/1000-f1*f1/4001000;//*XIUZ.U_SXZ  2*                                             //电压线圈电流放大比例改为1:2 -f1*f1/470201;
	 	/********************/
	U_P[U_Mea_Cnt]=f1;//取电压
	I_P[U_Mea_Cnt]=f2;//取电流
	/********************/
	 
  U_Mea_Cnt++;

	 f1=f2=0;
	 if(U_Mea_Cnt>=U_MEA_CNT)
	 {
	   U_Mea_Cnt=0;
	   Lon=U_MEA_CNT;
    temp=Lon;
    for(;Lon>0;Lon--)
    {
      f1+=U_Prms[Lon-1];           //电压有功功率
      f2+=U_Pmax[Lon-1];           //电压视在功率
			U_tmp+=U_P[Lon-1];
			I_tmp+=I_P[Lon-1];
    }
    f1/=temp;                      //电压线圈有功功率
    f2/=temp;                      //电压线圈视在功率
		U_tmp/=temp; 
		I_tmp/=temp; 
    f3=f1;
    f4=f2;
    f1+=0.0005;                    //电压线圈有功功率保留小数点后3位有效数字，四舍五入
    f2+=0.0005;                    //电压线圈视在功率保留小数点后3位有效数字，四舍五入 
    f1*=1000;                      //
    f2*=1000;
    Data=(u16)f1;
    U_Pr_Str[0]=(u8)((Data&0xFF00)>>8);
    U_Pr_Str[1]=(u8)(Data&0x00FF);
    Data=(u16)f2;
    U_Pm_Str[0]=(u8)((Data&0xFF00)>>8);
    U_Pm_Str[1]=(u8)(Data&0x00FF);
    /**************************************/
		sprintf(str,"%6.4f",		U_tmp		);
    memcpy(U_Str_Com ,str,7);
		sprintf(str,"%6.4f",		I_tmp		);
		memcpy(I_Str_Com ,str,7);
		U_current=U_tmp;
		I_current=I_tmp;
		/*****************************/
    sprintf(str,"%6.4f",f3);
    memcpy(U_Pr_Str_Com,str,7);
    sprintf(str,"%6.4f",f4);
    memcpy(U_Pm_Str_Com,str,7);
    
    U_Data_Ready='Y';
	 }   
}


