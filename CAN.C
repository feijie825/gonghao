/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CAN.C
;* Author             : 张力阵
;* CAN总线数据处理程序 
;* 集成了CAN接口的LM3S 系列处理器 由于CAN时钟和CPU时钟之间频率的不一致,造成访问CAN寄存器时
;* 要添加额外延时 CPU 时钟频率高于CAN时钟频率
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
#include "加密.h"
/****************************************************************************
* CAN短数据帧发送
****************************************************************************/
const CAN_MSG CAN_TX_SMSG=
{   
    0x00030050,    //控制域 EXD_ID=1 TX_INT_EN=1 
    0x18000002,    //仲裁域 EXD=1 DIR=1 DEV=2
    0x0000,        //DA1
    0x0000,        //DA2
    0x0000,        //DA3
    0x0000,        //DA4
}; 
/*****************************************************************************
* 从机CAN数据发送超时处理
*****************************************************************************/
void Proc_CAN_OvTm(void)
{
    if(CAN_SMSG_TX_STS)                              //短数据帧是否处于发送状态
     {
      if((u16)(Timer_1ms-CAN_STX_OVTimer)>=CAN_TX_OVTM)//短数据发送超时处理
       {
        CAN_SMSG_TX_STS=COM_TX_IDLE;                   //清除短数据发送标志      置为发送空闲状态
       }		
     }		
}
/*****************************************************************************
* 处理主机查询信息
*****************************************************************************/
void Proc_Mst_Check(void)
{
/*
	if(Echo_Sts==MST_CHK_RCVD)
	 {                                            //收到主机查询帧
      CAN_MSG CAN_ECHO_MSG;
      memcpy(&CAN_ECHO_MSG,
             &CAN_MSG_SET_TAB[SLV_CHK_ECHO-1],
             8);                                //导入控制域和仲裁域
      CAN_ECHO_MSG.ID.BIT.NUM = Mtr_Numb_ID;    //表位号
      CAN_Tx_Msg(CAN0,&CAN_ECHO_MSG,MSG_OBJ_TYPE_TX);
      Echo_Sts= SLV_ECHO_SEND;                  //已经申请发送 送入MSG RAM
      CAN_RX_OVTimer=(u16)Timer_1ms;            //初始化CAN超时定时器
	 }
*/
}
/********************************************************
* IO口基址定义
********************************************************/
const u32 PORT_BASE_ADDR_TAB[]=
{
    GPIO_PORTA_BASE,            //GPIOA 基址
    GPIO_PORTB_BASE,            //GPIOB 基址
    GPIO_PORTC_BASE,            //GPIOC 基址
    GPIO_PORTD_BASE,            //GPIOD 基址
    GPIO_PORTE_BASE,            //GPIOE 基址
    GPIO_PORTF_BASE,            //GPIOF 基址
    GPIO_PORTG_BASE,            //GPIOG 基址
    GPIO_PORTH_BASE,            //GPIOH 基址
};
/********************************************************
* 发送数据到CAN缓冲区
* 人口: CMD 命令编号
* 人口: Len 数据长度
* 人口: Ptr 待发送数据指针
********************************************************/
void Send_Data(u16 CMD,u8 Len,u8 *Ptr)
{
    u8 m;
    if(CAN_ERR)                           //总线错误
     return;                              //退出
    if(Len>8)                             //判断长度是否超过最大数据长度
     Len=8;                               
    memcpy(CAN_MSG_OPtr,                  //CAN发送指针
           &CAN_TX_SMSG,                  //默认帧
           16);                           
    CAN_MSG_OPtr->ID.BIT.NUM=Mtr_Numb_ID; //表位号   仲裁域
    CAN_MSG_OPtr->ID.BIT.CMD=CMD;         //命令     仲裁域
    if(Len!=0)
     {
      CAN_MSG_OPtr->CTL.BIT.LEN=Len;      //数据长度 控制域
      for(m=0;m<Len;m++)                  //拷贝有效数据
       {
       	CAN_MSG_OPtr->Data.BYTE[m]=*Ptr;
       	Ptr++;
       }	 
     }         
    SDATA_MSG_OHead_ADD_ONE();            //发送指针加1处理           
}                                    
/********************************************************
* 总控中心启动命令
********************************************************/
void Set_Master_Start(void)
{
    MASTER_START=1;                             	//总控中心已经启动标志
}
/******************************************************
* 单三相台设置命令
******************************************************/
void Set_TS(void)
{
    if(CAN_MSG_IPtr->Data.BYTE[0]=='3')          //是否设置为三相台
     {	         
      if(SINGLE_OR_THREE!=THREE)                 //单三相设置发生改变
       {
        SINGLE_OR_THREE=THREE;  
       }                               
     } 
    else if(CAN_MSG_IPtr->Data.BYTE[0]=='1')
     {	
      if(SINGLE_OR_THREE!=SINGLE)                //台体状态改变
       {
        SINGLE_OR_THREE=SINGLE;  
       }
     }
    else
     return;                       			       
}
/********************************************************
* 取测量表位号和测量通道的对应关系 
* 返回0，说明没有对应的表位
* 返回1，有对应的表位
********************************************************/
u8 Get_Mea_Met(u8 Met)
{
  u8 n;
  for(n=0;n<3;n++)
	 {
	   if(Met==0)
	   {
		    Current_Mea_MetNUM=0;           //当前测量的表位号       
		    Current_Mea_Channel=0;          //当前测量的通道号
		    break;
		  }
	   if(Met==XIUZ.CH0_MTR[n])
	   {
		    Current_Mea_MetNUM=Met;         //当前测量的表位号        
		    Current_Mea_Channel=(n+1);      //当前测量的通道号
		    break;
		  } 
  }
	 if(n==3)
	  return 0;                          //没有对应的表位号，不改变原测量单元 
  else
   return 1;
}
/********************************************************
* 功耗测试 表位和相别选择 处理 
* XXXY或XXY  XXX或XX 要测量的表位
* Y 要测量的相 0:停止测量 1:A相 2:B相 3:C相
********************************************************/
void Power_Test_Pr(void)
{
    u8 ID=0,c;
    ID=CAN_MSG_IPtr->Data.BYTE[0];             //取出测量表位号 
    c=CAN_MSG_IPtr->Data.BYTE[1];              //取测量控制字
    if(c>3)                                    //判断控制字是否错误
     return;
    Current_Mea_Phase=c;                       //存储测试相别
    CNCL_CHNL_Sel1;                             //释放所有测量通道
    CNCL_CHNL_Sel2;
    Sel_Phase_NO;                              //不测量任何相别
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;
    if((Get_Mea_Met(ID)==0)||                  //要测量的表位号不在功耗模块的测量范围内
    	  (ID==0)||                               //测量的表位号是0，退出功耗测量
    	  (c==0))                                 //测量控制字为0，退出功耗测量
    {
      ADC_Start=0x00;                          //退出功耗测量    	
      Stop_CS5460AConv();
      return;
    } 	 
    else
    {
      if(Current_Mea_Phase==1)                 //测试A相电压电流功耗
     	  Sel_Phase_A; 
      else if(Current_Mea_Phase==2)            //测试B相电压电流功耗
    		  Sel_Phase_B;                           
      else if(Current_Mea_Phase==3)            //测试C相电压电流功耗
    		  Sel_Phase_C;                          
       
      if(Current_Mea_Channel==1)               //通道1测量
      {
        Sel_Channel1;	
        if((Current_Wire_Type==WIRE_P1)||                //单相有功
          (Current_Wire_Type==WIRE_Q1)||                //单相无功
          (Current_Wire_Type==WIRE_P4)||                //三相四线有功
          (Current_Wire_Type==WIRE_Q4_3)||              //三相四线3元件无功
          (Current_Wire_Type==WIRE_Q4_R))               //三相四线真无功
          P4_Sel1;
        else
          P3_Sel1;
      }
      else if(Current_Mea_Channel==2)          //通道2测量
      {
        Sel_Channel2;	
        if((Current_Wire_Type==WIRE_P1)||                //单相有功
          (Current_Wire_Type==WIRE_Q1)||                //单相无功
          (Current_Wire_Type==WIRE_P4)||                //三相四线有功
          (Current_Wire_Type==WIRE_Q4_3)||              //三相四线3元件无功
          (Current_Wire_Type==WIRE_Q4_R))               //三相四线真无功
          P4_Sel2;
        else
          P3_Sel2;
      }
      else if(Current_Mea_Channel==3)          //通道3测量
      {
        Sel_Channel3;	
        if((Current_Wire_Type==WIRE_P1)||                //单相有功
          (Current_Wire_Type==WIRE_Q1)||                //单相无功
          (Current_Wire_Type==WIRE_P4)||                //三相四线有功
          (Current_Wire_Type==WIRE_Q4_3)||              //三相四线3元件无功
          (Current_Wire_Type==WIRE_Q4_R))               //三相四线真无功
          P4_Sel3;
        else
          P3_Sel3;
      }
      
      ADC_Start='Y';                           //开启ADC转换
      Start_CS5460AConv();                     //开启CS5460A转换
   }
} 
/********************************************************
* 设置功耗测试通道对应表位号
********************************************************/
void Set_Mea_Met_Tab(void)
{
  u8 chnl,Met;
  chnl=CAN_MSG_IPtr->Data.BYTE[0];             //取出要设置的通道号 
  Met=CAN_MSG_IPtr->Data.BYTE[1];              //取测量控制字
  if((chnl==0)||(chnl>3))                      //通道号的范围1-3
	  return; 	
	 if((Met==0)||Met>255)                            //表位号的范围1-255
	  return;
  chnl--; 
	 if(Met==XIUZ.CH0_MTR[chnl])                        //设置的值与原对应值一致，
	  return;
	 XIUZ.CH0_MTR[chnl]=Met;
}
/********************************************************
* 设置修正值
* 0:设置电压有功功率修正值
* 1:设置电压视在功率修正值
* 2:设置电流有功功率修正值
********************************************************/
void Set_Vref(u8 type)
{
	 float f;
	 if(type>2)
	 	return;                                       //修正值类型只能是0-2之间的数字
	 memcpy(&f,
	        &CAN_MSG_IPtr->Data.BYTE,
	        4);                                     //取出功耗修正系数
  if(type==UP_TYP)
  {
    f*=XIUZ.U_PXZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>2))   
     return;
    XIUZ.U_PXZ=f;
  }
  else if(type==US_TYP)
  {
    f*=XIUZ.U_SXZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>2))   
     return;
    XIUZ.U_SXZ=f;
  }
  else if(type==IS_TYP)
  {
    f*=XIUZ.I_SXZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>2))   
     return;
    XIUZ.I_SXZ=f;
  }  
}
/******************************************************
* 读版本号,只有第一表位返回版本号
******************************************************/
void Set_Read_Ver(void)             
{
    if(Mtr_Numb_ID==1)                       //是否为第1表位
     {
      u8 n,d;
      for(n=0;n<10;)
       {
        d=VER_TAB[n];
        if(d=='\0')
         break;
        TEMP_STR[n]=d; 
        n++;
       }
      if(n==0||n==10)                          //长度错误
       return;	    
      Send_Data(CAN_OCMD_VER,                  //命令
                n,
                TEMP_STR);
     }           
}
/******************************************************
* 固化修正值数据及通道对应关系数据 
******************************************************/
void Set_SOLID(void)
{
    Solid_Save_Tab();             //固化标准表编号 
}
/******************************************************
* 设置接线方式
******************************************************/
void Set_Wire_Type(void)
{
  u8 type;
  type=CAN_MSG_IPtr->Data.BYTE[0];           //取出接线方式控制字
  if((type<WIRE_P1)||(type>WIRE_Q1)||(Current_Wire_Type==type))
   return;
  if(SINGLE_OR_THREE==SINGLE)                //单相台只能一种接线方式
  {
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;  
    if((type!=WIRE_P1)&&(type!=WIRE_Q1))   
     Current_Wire_Type=WIRE_P1;
    else
     Current_Wire_Type=type;
  }
  else                                       //三相台
     Current_Wire_Type=type;     
}
/*****************************************************************************
* 处理CAN短数据接收指令缓冲区
*****************************************************************************/
void Proc_SDATA_IBUF(void)
{
    if(CAN_SDATA_MSG_IHead==CAN_SDATA_MSG_ITail)
     return;                                                       //没有收到短数据退出
    CAN_RX_OVTimer=(u16)Timer_1ms;                                 //初始化CAN超时定时器
    MASTER_START=1;                                                //主机已启动
    switch(CAN_MSG_IPtr->ID.BIT.CMD)                               //根据命令跳转到相应位置处理
     {
      case CAN_ICMD_CHK:    Set_Master_Start();            break;  //总控中心启动命令
      case CAN_ICMD_SETST:  Set_TS();                      break;  //设置单三相命令
      case CAN_ICMD_PMTR:   Set_Mea_Met_Tab();             break;  //设置表位对应关系
      case CAN_ICMD_MEA:    Power_Test_Pr();               break;  //开始测量功耗命令
      case CAN_ICMD_XUP:    Set_Vref(UP_TYP);              break;  //修正电压有功功耗
      case CAN_ICMD_XUS:    Set_Vref(US_TYP);              break;  //修正电压视在功耗
      case CAN_ICMD_XIS:    Set_Vref(IS_TYP);              break;  //修正电流视在功耗
      case CAN_ICMD_VER:    Set_Read_Ver();                break;  //查版本号
      case CAN_ICMD_SOLID:  Set_SOLID();                   break;  //固化数据
      case CAN_ICMD_BOOT:                                  break;  //进入引导程序
      case CAN_ICMD_TYPE:   Set_Wire_Type();               break;  //接线选择
      default:break;     
     }
    CAN_SDATA_MSG_ITail++;
    if(CAN_SDATA_MSG_ITail>=CAN_SDILEN)
     {
      CAN_SDATA_MSG_ITail=0;
      CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];//初始化接收报文处理指针
     }                                    
    else                                  
     CAN_MSG_IPtr+=1;                     //指针 

}
/*****************************************************************************
* CAN短数据发送指令缓冲区头指针加1处理
*****************************************************************************/
void SDATA_MSG_OHead_ADD_ONE(void)
{
    CAN_SDATA_MSG_OHead++;
    if(CAN_SDATA_MSG_OHead>=CAN_SDOLEN)
     {
      CAN_SDATA_MSG_OHead=0;              //发送指针
      CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];//初始化发送报文处理指针
     }
    else
     CAN_MSG_OPtr+=1;
}           
/*****************************************************************************
* 处理CAN短数据发送指令缓冲区
*****************************************************************************/
void Proc_SDATA_OBUF(void)
{
    if(CAN_SDATA_MSG_OHead==CAN_SDATA_MSG_OTail)
     return;                                  //没有短数据要发送退出
    if(CAN_SMSG_TX_STS != COM_TX_IDLE )       //CAN控制器是否在发送状态
     return;
    if(!MASTER_START)                         //总控中心是否启动
     return;	
    if(CAN_SEND_DELAY)                        //为防止过渡竞争 添加表位延时
     {
      CAN_SEND_DELAY--;
      return;
     }	
    CAN_SEND_DELAY=CAN_SEND_DTIME;            // 	
    CAN_SMSG_TX_STS = SLV_SMSG_TX_IS;
    CAN_STX_OVTimer=(u16)Timer_1ms;           //重置发送定时
    CAN_Tx_Msg(CAN0,&CAN_SDATA_MSG_OBUF[CAN_SDATA_MSG_OTail],MSG_OBJ_TYPE_TX);  //发送短帧
    CAN_SDATA_MSG_OTail++;                    //指针加1 指向下一帧 
    if(CAN_SDATA_MSG_OTail>=CAN_SDOLEN)       //判断是否超出循环   
     CAN_SDATA_MSG_OTail=0;                   //从头开始循环       
}
/*****************************************************************************
* 处理CAN总线状态
* 处理超时
*****************************************************************************/
void Proc_CAN_STS(void)        
{
    if((CAN_STS.BYTE&(CAN_STS_BOFF|CAN_STS_EWARN|CAN_STS_EPASS))||
      ((u16)(Timer_1ms-CAN_RX_OVTimer)>CAN_RX_OVTM)) 
     {
      CAN_RX_OVTimer=(u16)Timer_1ms;      //超时定时	
      CAN_STS.BYTE=0;                     //清除错误
      CAN_ERR=1;                          //CAN错误状态标志
      CAN_SMSG_TX_STS=COM_TX_IDLE;        //清除短数据发送标志      置为发送空闲状态
      CAN_LMSG_TX_STS=COM_TX_IDLE;        //清除CAN长数据帧发送状态 置为发送空闲状态
      CAN_LDATA_TX_STS = SLV_LDATA_TX_NO; //清除长数据发送状态      置为发送空闲状态         
      Init_CAN();                         //重新初始化总线
     } 	
}
