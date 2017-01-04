/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Interrupt.c
;* Author             : 张力阵
;* 中断处理函数库
;* 要改变发送或接收FIFO触发设置 要更改相应程序 
*******************************************************************************/			   
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "CLK_ERR.h"
#include "ENG_ERR.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
/********************************************************
* 错误中断服务程序
********************************************************/
void IntDefaultHandler(void)
{
    for(;;)
     {}
}

/********************************************************
* 看门狗中断服务程序
********************************************************/
void WatchDogHandler(void)
{
    WatchdogIntClear();	  //清除看门狗中断
}
/********************************************************
* 系统节拍时钟定时器中断服务程序
********************************************************/
void SysTickIntHandler(void)
{
    Timer_1ms++;                           //1ms定时器
//    STD_CLK_Timer++;                       //检测标准时钟脉冲定时
    CLK_Timer++;                           //时钟脉冲定时
    if(WORK_Timer)                         //进入工作模式定时
     WORK_Timer--;                         //	
    if(Disp_En_Timer)                      //显示使能定时
     Disp_En_Timer--;                      //
    if(!(Timer_1ms%8))
     Timer_8ms++;
}
/********************************************************
* 采样序列发生器0中断服务程序
********************************************************/
void ADC0Handler(void)
{
//#ifdef _ADC
    ADCIntClear(0);                    //清除中断
    ADCSequenceDataGet(0,ADC_SEQ_Data);//读值
    ADC_Data.Data=(u16)ADC_SEQ_Data[0];
    ADC_Data.New_Data=1;               //新数据标志
    ADC_Data.Trig=0;                   //ADC软件触发标志
//#endif
}
/*****************************************************************************
*
* 串口0中断服务程序
*
*****************************************************************************/
void UART0IntHandler(void)
{
    u32 ulStatus;
    ulStatus = UARTIntStatus(UART0, true);
    UARTIntClear(UART0, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {       //发送FIFO中断
      COM0_STimer=(u16)Timer_1ms;      //重启定时 	    
      if(COM0_InSend!=COM_TX_IS)       //不在发送状态
       return;
      if(COM0_OHead!=COM0_OTail)       //判断是否有数据要发送
       {	
        if(Send_One_Uart_Frm(COM0_BUF)==BUF_FIFO_NBLK)  //启动发送一帧 判断是否有后续数据
         return;                       //有后续数据 退出 等待程序下次循环 发送	
       }
      else 
       {	
        COM0_InSend=0;                 //清除正在发送标志
        COM0_OCoun=0;                  //清除待发送命令计数
       }  	 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {	                     //接收FIFO中断或接收超时中断
      u8 m,c;
      u16 t;
      m=16; 	 
      for(;UARTCharsAvail(UART0);) //循环读接收FIFO
       {
        c=(u8)UARTCharGet(UART0); //读出字符
        if((c=='\r')||            //收到回车符
           ((c>' ')&&(c<='~')))  
         {                        //收到的字符为换行符'\r'或ASC码
          t=COM0_IHead+1;
          if(t>=COM0_ILEN)
           t=0;
          if(t==COM0_ITail)
           continue;               //没有空间 丢弃该数据 	
          COM0_IBuf[COM0_IHead]=c; //保存数据
          COM0_IHead=t;
          if(c=='\r')
           COM0_ICoun++;         //收到一条完整命令
         }
        m--;
        if(m==0)
         break;
       }  
     }
}
/*****************************************************************************
*
* 串口1中断服务程序
*
*****************************************************************************/
void UART1IntHandler(void)
{
    u32 ulStatus;
    ulStatus = UARTIntStatus(UART1, true);
    UARTIntClear(UART1, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {       //发送FIFO中断
      COM1_STimer=(u16)Timer_1ms;      //重启定时 	    
      if(COM1_InSend!=COM_TX_IS)       //不在发送状态
       return;
      if(COM1_OHead!=COM1_OTail)       //判断是否有数据要发送
       {	
        if(Send_One_Uart_Frm(COM1_BUF)==BUF_FIFO_NBLK)  //启动发送一帧 判断是否有后续数据
         return;                       //有后续数据 退出 等待程序下次循环 发送	
       }
      else 
       {	
        COM1_InSend=0;                 //清除正在发送标志
        COM1_OCoun=0;                  //清除待发送命令计数
       }  
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {	                     //接收FIFO中断或接收超时中断
      u8 m,c;
      u16 t;
      m=16; 	 
      for(;UARTCharsAvail(UART1);) //循环读接收FIFO
       {
        c=(u8)UARTCharGet(UART1); //读出字符
        if((c=='\r')||            //收到回车符
           ((c>' ')&&(c<='~')))  
         {                        //收到的字符为换行符'\r'或ASC码
          t=COM1_IHead+1;
          if(t>=COM1_ILEN)
           t=0;
          if(t==COM1_ITail)
           continue;               //没有空间 丢弃该数据 	
          COM1_IBuf[COM1_IHead]=c; //保存数据
          COM1_IHead=t;
          if(c=='\r')
           COM1_ICoun++;         //收到一条完整命令
         }
        m--;
        if(m==0)
         break;
       }  
     }
}
/*****************************************************************************
*
* 定时器0-A中断服务程序
* 用于电子脉冲计数
*****************************************************************************/
void Timer0AIntHandler(void)
{
    TIMER0->ICR = TIMER_TIMA_TIMEOUT; //清溢出中断
}
/*****************************************************************************
*
* 定时器0-B中断服务程序
* 用于标准晶振高频计数 正常131.07ms中断一次
* 计数分频值0xFFFF 
* 当前计数值=STD_CLK_Cnt*0xFFFF+(0xFFFF-TIMER0B)
*           =((STD_CLK_Cnt+1)*0xFFFF)-TIMER0B
*           =(STD_CLK_Cnt+1)*(0x10000-1)-TIMER0B
*           =(STD_CLK_Cnt+1)*0x10000-(STD_CLK_Cnt+1)-TIMER0B 
*           =(STD_CLK_Cnt+1)*0x10000-STD_CLK_Cnt-TIMER0B-1
*           =(STD_CLK_Cnt+1)*0x10000-STD_CLK_Cnt+(~TIMER0B+1)-1
*           =(STD_CLK_Cnt<<16)-STD_CLK_Cnt+0x10000+(~TIMER0B)
* 连续两次计数差值 第一次计数高位 STD_CLK_Cnt1 TIMER0B1
*                  第二次计数高位 STD_CLK_Cnt2 TIMER0B2
*                  计算差值时 常量0x10000 消掉
* 差值=
*****************************************************************************/
void Timer0BIntHandler(void)
{
    TIMER0->ICR = TIMER_TIMB_TIMEOUT;               //清溢出中断
    TIMER0->TBILR = 0xFFFF;                         //重置计数 0xFFFF
    TIMER0->CTL &=~TIMER_CTL_TBEN;                  //Timer1-B启动
}
/*****************************************************************************
* 定时器1-A中断服务程序
* 用于时钟脉冲计数
*****************************************************************************/
void Timer1AIntHandler(void)
{
    TIMER1->ICR = TIMER_TIMA_TIMEOUT;               //清溢出中断
    TIMER1->TAILR = 60000;                          //重置计数
    TIMER1->CTL |=TIMER_CTL_TAEN;                   //Timer1-A启动
    TIMER1->CTL &=~TIMER_CTL_TAEN;                  //Timer1-A 停止计数		
    return;
}
/*****************************************************************************
*
* 定时器1-B中断服务程序
* 用于被检表高频计数
*****************************************************************************/
void Timer1BIntHandler(void)
{
#ifdef __TEST
    TIMER1->ICR = TIMER_TIMB_TIMEOUT;           //溢出中断
    TIMER1->TBILR = 24;                         //重置计数
    TIMER1->TBMATCHR=12;                        //50%
    TIMER1->CTL |=TIMER_CTL_TBEN;               //Timer2-B启动
#else
    TIMER1->ICR = TIMER_TIMB_TIMEOUT;               //清溢出中断
    TIMER1->TBILR = 0xFFFF;                         //重置计数
    TIMER1->CTL |=TIMER_CTL_TBEN;                   //Timer1-B启动
#endif
}
/*****************************************************************************
*
* 定时器2-A中断服务程序
* 用于标准表高频计数 处理时间1.2us
* 30K 中断时间为 2.1845s
* 10K 中断时间为 6.5535s
*****************************************************************************/
void Timer2AIntHandler(void)
{
    TIMER2->ICR = TIMER_TIMA_TIMEOUT;               //清溢出中断
    TIMER2->TAILR = 0xFFFF;                         //重置计数
    TIMER2->CTL |=TIMER_CTL_TAEN;                   //Timer2-A启动
}
/*****************************************************************************
*
* 定时器2-B中断服务程序
* 用于PWM输出
*****************************************************************************/
void Timer2BIntHandler(void)
{
    TIMER2->ICR = TIMER_TIMB_TIMEOUT;           //溢出中断
    TIMER2->TBILR = 19;                         //重置计数
    TIMER2->TBMATCHR=9;                         //50%
    TIMER2->CTL |=TIMER_CTL_TBEN;               //Timer2-B启动
}
/*****************************************************************************
* GPIOB 端口 中断处理函数
* 光电头脉冲输入中断
* 电子脉冲中断(测量电子脉冲周期) 
* 时钟脉冲中断输入(测量时钟脉冲周期)
*****************************************************************************/
void GPIOBHandler(void)
{
    u8  Int;                            //中断状态
    Int=GPIOB->MIS;                     //读取I/O口中断状态
    GPIOB->ICR=Int;                     //清除GPIOB I/O口中断
}
/*****************************************************************************
* GPIOC 端口 中断处理函数
* 按键中断
*****************************************************************************/
void GPIOCHandler(void)
{
    u8 Int;
    Int=GPIOC->MIS;                   //读取中断状态
    GPIOC->ICR=Int;                   //清除端口中断
    if(Int&KEY_IN)                     
     {
      GPIOC->IM&=~KEY_IN;             //按键中断口中断禁能  
      KEY_Timer=Timer_8ms;            //重启中断使能定时
      KEY_INT_REEN=1;                 //清除重使能标志 
      if(NEW_KEY_FLAG)                //上次按键没有处理 认为是连续按键 挂表or解除挂表
       {	
        NEW_KEY_FLAG=0;               //收到按键中断标志
        PLUG_CHG_FLAG=1;	
       }
      else
       {	
        NEW_KEY_FLAG=1;               //收到按键中断标志
        PLUG_CHG_FLAG=0;	
       }
      Con_KEY_Timer=Timer_8ms;        //连续按键定时
     }
}
/*****************************************************************************
* GPIOD 端口 中断处理函数
* CS5460A  INT中断
*****************************************************************************/
void GPIODHandler(void)
{
  u8 Int;
  Int=GPIOD->MIS;                            //读取中断状态
  GPIOD->ICR=Int;                                //清除端口中断
  if(Int&CS5460A_INT) 
  {
    u8 Dat;
    Dat*=0;
	   Timer_CS5460A_INT=(u16)Timer_1ms;
    CS5460A_SCLK_L;
    CS5460A_CS_L;
    Dat=Write_Read_OneByte_5460A(REGISTER_READ|STATUS_REG_ADDR);
	   CS5460A_Sts_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Sts_Reg[1]=Write_Read_OneByte_5460A(0xff);  //read status reg
	   CS5460A_Sts_Reg[2]=Write_Read_OneByte_5460A(REGISTER_WRITE|STATUS_REG_ADDR);
	   Dat=Write_Read_OneByte_5460A(CS5460A_Sts_Reg[0]);
	   Dat=Write_Read_OneByte_5460A(CS5460A_Sts_Reg[1]);
	   Dat=Write_Read_OneByte_5460A(CS5460A_Sts_Reg[2]);   //clr status reg
   
	   CS5460A_CS_H;
	   if((CS5460A_Sts_Reg[0]&0x80)==0x00)        //一个计算周期没有结束，返回
	     return;
    CS5460A_SCLK_L;
    CS5460A_CS_L;
    Dat=Write_Read_OneByte_5460A(REGISTER_READ|IRMS_REG_ADDR);
	   CS5460A_Irms_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Irms_Reg[1]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Irms_Reg[2]=Write_Read_OneByte_5460A(REGISTER_READ|VRMS_REG_ADDR);
	   CS5460A_Vrms_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Vrms_Reg[1]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Vrms_Reg[2]=Write_Read_OneByte_5460A(REGISTER_READ|E_REG_ADDR);
	   CS5460A_E_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_E_Reg[1]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_E_Reg[2]=Write_Read_OneByte_5460A(0xfe);
	   CS5460A_CS_H;
    
	   if(ADC_Start!='Y')
	    return;
	   CS5460A_New_Data='Y'; 
  }
}
/*****************************************************************************
* GPIOF 端口 中断处理函数
* 需量脉冲输入中断
* 时段投切脉冲
* 合闸脉冲
*****************************************************************************/
void GPIOFHandler(void)
{
    u8 Int;
    Int=GPIOF->MIS;                         //读取中断状态
    GPIOF->ICR=Int;                         //清除端口中断
}
/*****************************************************************************
* CAN 中断处理函数
*****************************************************************************/
void CANHandler(void)
{
    u32 ulStatus;
    ulStatus = CANIntStatus(CAN0, CAN_INT_STS_CAUSE);  //读引起中断的报文对象的编号
    if(ulStatus==0)                                    //误中断
     return;                                           //返回
    if(ulStatus==0x8000)                               //判断是否为状态中断
     {                                                 //状态中断
      CAN_STS.BYTE=CANStatusGet(CAN0, CAN_STS_CONTROL);//读总线错误状态
      CANErrCntrGet(CAN0,&CANERR_CNT);
      return;
     }                                                 
    if(ulStatus>MSG_OBJ_NUMB)                          
     {                                                 //误中断
      CANStatusGet(CAN0, CAN_STS_CONTROL);             //
      return;                                          //超出报文对象定义个数 认为为错误接收或发送
     }
    CAN_ERR=0;                                         //总线连接 
    CAN_STS.BYTE =0;                                   //正确接收或发送 清除状态中断
    CANERR_CNT=0;                                      //错误计数清零 
    if(CAN_MSG_SET_TAB[(ulStatus-1)].CTL.BIT.RX_INT_EN)//判断是否为接收报文中断
     {                                                 //接收中断
      memset(&CAN_MSG_Rx,0,8);                         //清除接收报文控制域和仲裁域
      CAN_MSG_Rx.CTL.BIT.IDx=ulStatus;                 //报文对象编号
      CAN_Rx_Msg(CAN0, &CAN_MSG_Rx, 1);                //接收帧 并清除中断
      switch(ulStatus)                                 //0x8000状态中断  0x01---0x20 引起中断的报文对象
       {	                                                                  
        case MST_SCDATA_TX:                             //主机正在发送短广播数据串ID8
        case MST_SSDATA_TX:                             //主机正在发送短单播数据串ID9
         {                                              //短数据接收处理 接收中断
          if((CAN_SDATA_MSG_IHead+1)!=CAN_SDATA_MSG_ITail)//判断是否有空闲空间
           {
            memcpy(&CAN_SDATA_MSG_IBUF[CAN_SDATA_MSG_IHead],
                   &CAN_MSG_Rx,16);                    //拷贝数据
            CAN_SDATA_MSG_IHead++;                     //指针加1 指向下一帧
            if(CAN_SDATA_MSG_IHead>=CAN_SDILEN)        //判断是否超出循环
             CAN_SDATA_MSG_IHead=0;                    //从头开始循环 
           }
           break;
         }
        default:
         {
          if((CAN_LDATA_MSG_IHead+1)!=CAN_LDATA_MSG_ITail)//判断是否有空闲空间
           {
            memcpy(&CAN_LDATA_MSG_IBUF[CAN_LDATA_MSG_IHead],
                   &CAN_MSG_Rx,16);                    //拷贝数据
            CAN_LDATA_MSG_IHead++;                     //指针加1 指向下一帧
            if(CAN_LDATA_MSG_IHead>=CAN_LDILEN)        //判断是否超出循环
             CAN_LDATA_MSG_IHead=0;                    //从头开始循环 
           }
          break;
         }
       }
     }
    else
     {                                                 //发送中断
      switch(ulStatus)                                 //0x8000状态中断  0x01---0x20 引起中断的报文对象
       {	                                             
        case SLV_SDATA_TX:                          //从机发送短数据串单帧完成 ID13
         {                                          //发送中断
          CAN_SMSG_TX_STS =COM_TX_IDLE;             //短帧发送完成
          CAN_STX_OVTimer=(u16)Timer_1ms;           //重置发送定时
          break;
         }
        default:
         break;
       }
      CANIntClear(CAN0,ulStatus);                      //清除发送中断
     }  
}
