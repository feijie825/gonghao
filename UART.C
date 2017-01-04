/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : UART.c
;* Author             : 张力阵
;* 串口数据处理程序
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "DISP.h"
#include "eng_err.h"
#include "stdlib.h"
#include "CS5460A.h"
#include "stdio.h"
//CAN 接口地址表格
const u32 UART_PORT[UART_NUMB]=
{
	UART0_BASE,
	UART1_BASE,
};
/****************************************************************************
* COM0接收命令表格
****************************************************************************/
const u8 COM0_ICMD_TAB[][8]=
{
    {"C00\r"},        //
    {"SEQUP:"},       //设置单三相
    {"Gd:"},
    {"Mtr:"},
    {"Ge:"},
    {"Gf:"},
    {"Gg:"},
		
		{"Gh:"},          //修正电压
    {"Gi:"},          //修正电流
		{"Gou:"},         //修正电压零点
		{"Goi:"},         //修正电流零点
    {"Gj:"},
    {"VER\r"},
    {"SOLID\r"},
    {"BOOT:"},        //进入引导程序
    {"$I1:"},         //A相电流有效值
    {"$I2:"},         //B相电流有效值
    {"$I3:"},         //C相电流有效值
    {"I:"},           //单相电流有效值
};
/****************************************************************************
* COM1接收命令表格
****************************************************************************/
const u8 COM1_ICMD_TAB[][8]=
{
    {"C00\r"},        //
    {"SEQUP:"},       //设置单三相
    {"Gd:"},
    {"Mtr:"},
    {"Ge:"},
    {"Gf:"},
    {"Gg:"},
		{"Gh:"},          //修正电压
    {"Gi:"},          //修正电流
		{"Gou:"},         //修正电压零点
		{"Goi:"},         //修正电流零点
    {"Gj:"},
    {"VER\r"},
    {"SOLID\r"},
    {"BOOT:"},        //进入引导程序
    {"$I1:"},         //A相电流有效值
    {"$I2:"},         //B相电流有效值
    {"$I3:"},         //C相电流有效值
    {"I:"},           //单相电流有效值
};
/****************************************************************************
* 各缓冲区设定的接收匹配命令个数表格
****************************************************************************/
const u16 CMD_NUM_TAB[]=
{
    sizeof(COM0_ICMD_TAB)/sizeof(COM0_ICMD_TAB[0]),       //COM0接收缓冲区        
    sizeof(COM1_ICMD_TAB)/sizeof(COM1_ICMD_TAB[0]),       //COM1接收缓冲区        
};

/****************************************************************************
* 缓冲区命令带数据标志表格
****************************************************************************/
const u8 Data_Flag_Tab[]=
{
    ':',                           //COM0接收缓冲区  标准表   
    ':',                           //COM1接收缓冲区              
};
/****************************************************************************
* 命令头地址表格
****************************************************************************/
const CChar_Ptr CMD_ADDR_TAB[]=
{
    COM0_ICMD_TAB[0],              //COM0接收缓冲区        
    COM1_ICMD_TAB[0],              //COM1接收缓冲区        
};
/****************************************************************************
* 各缓冲区每条命令头长度表格
****************************************************************************/
const u8 CMD_LEN_TAB[]=
{
    sizeof(COM0_ICMD_TAB[0]),         //COM0接收缓冲区  标准表    
    sizeof(COM1_ICMD_TAB[0]),         //COM1接收缓冲区        
};
/****************************************************************************
* 接收缓冲区处理 表格
****************************************************************************/
const IBUF_Pr IBUF_Ptr_TAB[]=
{
    {//COM0接收缓冲区
     COM0_IBuf,    
     &COM0_IHead,
     &COM0_ITail,
     COM0_ILEN,
    },
    {//COM1接收缓冲区
     COM1_IBuf,    
     &COM1_IHead,
     &COM1_ITail,
     COM1_ILEN,
    },
};
/****************************************************************************
* 发送缓冲区处理 表格
****************************************************************************/
const OBUF_Pr OBUF_Ptr_TAB[]=
{
    {//COM0发送缓冲区
     COM0_OBuf,    
     &COM0_OHead,
     &COM0_OTail,
     &COM0_OHead,         //无效 为了匹配结构体
     &COM0_InSend,        //无效 为了匹配结构体
     COM0_OLEN,
    },
    {//COM1发送缓冲区
     COM1_OBuf,    
     &COM1_OHead,
     &COM1_OTail,
     &COM1_OHead,         //无效 为了匹配结构体
     &COM1_InSend,        //无效 为了匹配结构体
     COM1_OLEN,
    },
};
/*****************************************************************************
* 进入BOOT 关闭中断
*****************************************************************************/
void BOOT_INT_DIS(void)
{
    u32 *Ptr;
    Ptr=(u32*)0x20000000;
    *Ptr     = 'E';                 //位变量地址分配0x20000000 
    *(Ptr+1) = 'E';                 //位变量地址分配0x20000004
    *(Ptr+2) = Rx_Para[0][0];       //回送数据的ID  
    IntMasterDisable();          //进入BOOT前 禁能中断
    IntDisable(FAULT_SYSTICK);   //系统节拍发生器NVIC中断使能  系统定时器中断
    IntDisable(INT_UART0);       //UART0         NVIC中断使能  UART0中断
    IntDisable(INT_UART1);       //UART1         NVIC中断使能  UART1中断
    IntDisable(INT_TIMER0A);     //TIMER0-A      NVIC中断使能  电子脉冲计数中断
    IntDisable(INT_TIMER0B);     //TIMER0-B      NVIC中断使能  标准晶振高频计数溢出中断
    IntDisable(INT_TIMER1A);     //TIMER1-A      NVIC中断使能  时钟脉冲计数溢出中断
#ifndef __TEST
    IntDisable(INT_TIMER1B);     //TIMER1-B      NVIC中断使能  被检表高频计数溢出中断
#endif
    IntDisable(INT_TIMER2A);     //TIMER2-A      NVIC中断使能  标准表高频计数溢出中断 
//    IntDisable(INT_TIMER2B);   //TIMER2-B      NVIC中断使能  PWM 输出
    IntDisable(INT_CAN0);        //CAN           NVIC中断使能  CAN 中断
//    IntDisable(INT_WATCHDOG);    //看门狗        NVIC中断使能
    IntDisable(INT_GPIOB);       //GPIOB         NVIC中断使能  GPIOB中断 光电头脉冲输入
    IntDisable(INT_GPIOC);       //GPIOC         NVIC中断使能  GPIOC中断 按键输入
    IntDisable(INT_GPIOF);       //GPIOF         NVIC中断使能  GPIOF中断 需量脉冲 时段投切 合闸脉冲
//    IntMasterEnable();           //CPU中断允许
   	for(;;)	                        //死循环等待看门狗复位
    {}
}
/****************************************************************************
* 从接收缓冲区中取一个字符
* 用到 IBUF_Ptr 接收缓冲区处理结构体
****************************************************************************/
u8 Get_Char(void)
{
    u8 c;
    if(*IBUF_Ptr.ITail==*IBUF_Ptr.IHead) //指针头是否=指针尾
     return('\0');	
    c=IBUF_Ptr.IBuf[*IBUF_Ptr.ITail];    // 取一个字符
    IBUF_Ptr.IBuf[*IBUF_Ptr.ITail]=0x00; // 缓冲区清零
    (*IBUF_Ptr.ITail)++;                 // 处理指针加1 必须加括号 否则指针++ 而不是指针的指针+1
    if(*IBUF_Ptr.ITail>=IBUF_Ptr.IBLEN)  // 处理指针循环
     *IBUF_Ptr.ITail=0;                       
    return c;                            // 返回字符
}
/****************************************************************************
* 发送一字符
****************************************************************************/
void PutChar(u8 c)
{
    OBUF_Ptr.OBuf[*OBUF_Ptr.OHead]=c;
    (*OBUF_Ptr.OHead)++;
    if(*OBUF_Ptr.OHead>=OBUF_Ptr.OBLEN)
     *OBUF_Ptr.OHead=0;	
}
/****************************************************************************
* 字符串发送
* 入口: Str 要发送的数据
* 入口: Len 要发送数据的长度
        当Len=0时  按字符串发送 直到遇到结束符'\0' '\r' 0xff
        当Len!=0时 直接发送Len长度个字符
* 标准表数据 暂不用该程序发送
* 该子程序将数据发送到数据发送缓冲区
****************************************************************************/
void SendStr(u8 Buf_Id,u8 *Str,u8 Len)
{
    u8 c;
    if(Buf_Id>COM1_BUF)                    //判断发送缓冲区是否合法
     return;            	  
    memcpy(&OBUF_Ptr,                      //装载发送缓冲区处理结构体
           &OBUF_Ptr_TAB[Buf_Id],          //发送缓冲区处理结构体表格
           sizeof(OBUF_Pr));               //长度
    if(Len)                                //发送Len个字符
     {
      for(;;)
       {
        PutChar(*(Str++));
        Len--;
        if(Len==0)
         return;	
       }
     }
		else
		{
			for(;;)                                
			 {
				c=*Str;                              //取出字符串
				Str++;
				if((c=='\0')||                       //字符串结束符 
					 (c==0xFF))
				 c='\r';                                      
				PutChar(c);
				if(c=='\r')                          //判断字符串是否结束
				 return; 
			 }
	 }
}
/****************************************************************************
* 串口处理程序:找出一个指令头字符串在一个指令表数组字符串中的序号
* 返回在命令表格中对应的序号
* 超长命令头已在Get_Com_Cmd()函数中剔除
****************************************************************************/
u16 Case_num(u8 Buf_Id)
{
    u8 m,n,k; 
    const u8 *Ptr1,*Ptr2;         //命令头表格指针
    u8 *Str;                      //命令头指针
      Ptr1=CMD_ADDR_TAB[Buf_Id].Ptr;//比较命令头起始位置
      Ptr2=Ptr1;                    //保存地址
      k=CMD_NUM_TAB[Buf_Id];        //命令个数
      n=CMD_LEN_TAB[Buf_Id];        //命令头长度
    for(m=0;m<k;m++)
     {                 
      Str=Rx_Com;                   //命令头指针
      for(;;)                       
       {                            
        if(*Str!=*Ptr1)             
         {                          
          Ptr1=Ptr2+n;              //指向下一组命令
          Ptr2=Ptr1;                //保存当前指针
          break;
         }
        if((*Ptr1=='\r')||
           (*Ptr1==Data_Flag_Tab[Buf_Id]))
         return(m);
        Str++;
        Ptr1++;
       }				
     }
    return(0xFFFF);
}
/****************************************************************************
* 从接收缓冲区中取出参数
* m 参数 第几个参数
* 
* 出口:参数取完返回0 还有参数返回1 参数错误返回0xFF
****************************************************************************/
u8 Get_Para(u8 Buf_Id,u8 m)
{
    u8 s;
    u8 n;
    if(m>=RX_PARA_NUM)
     return 0;
    if(Buf_Id>COM1_BUF)	           //判断端口是否错误
     return 0;	 
    memcpy(&IBUF_Ptr,              //接收缓冲区处理结构体
           &IBUF_Ptr_TAB[Buf_Id],  
           sizeof(IBUF_Pr));
    for(n=0;n<(RX_PARA_LEN-1);n++)
     {
      s=Get_Char();               //调用取出字符
      if(s=='\r')                 //参数读取结束
       {
        Rx_Para[m][n]='\0';
        return 0;                 //所有参数取完反回
       }
      if((s>0x1f)&&(s<0x7F))      //判断是否为其它合法字符
       {
        Rx_Para[m][n]=s;
        if(s==',')
         {
          Rx_Para[m][n]='\0';
          return 1;               //本参数取完 还有后续参数 返回1
         }
       } 
     }
    Rx_Para[m][n]='\0';
    for(;;)			                  //扔掉多余的数据
     {
      s=Get_Char();
      if((s=='\0')||
         (s=='\r'))
       return 0xFF;               //参数错误
     }		
}
/****************************************************************************
* 从接收缓冲区取一条指令
* 指令头放在Rx_Com 中
* 参数放在Rx_Para中
* 指令的允许最大长度由RX_CMD_LEN定义
****************************************************************************/
void Get_Com_Cmd(u8 Buf_Id)   
{
    u8 n,s;
    if(Buf_Id>COM1_BUF)
     return;	 
    memcpy(&IBUF_Ptr,           //接收缓冲区处理结构体
           &IBUF_Ptr_TAB[Buf_Id],  
           sizeof(IBUF_Pr));
    	          
      for(n=0;n<(RX_CMD_LEN-1);n++)
       {
        s=Get_Char();             //取一字符
        if(s>'~')
         continue;
        Rx_Com[n]=s;
        if(s=='\r')               //回车符
         {
          Rx_Com[n]='\r';
          Rx_Para[0][0]='\0';
          return;               //无参数返回
         }
        else if(s==Data_Flag_Tab[Buf_Id])
          goto G_Para;
       }
      Rx_Com[n]='\0';             //命令头超长
      for(;;)                     //扔掉多余的数据
       {
        s=Get_Char();             //取一字符
        if(s=='\r')
         return;           //命令头超长
        if(s==':')
         break;           //命令头超长
       }
G_Para:
      if(!Get_Para(COM0_BUF,0))    return;
      if(Get_Para(COM0_BUF,1)) 
	     {
		      for(;;)			                  //扔掉多余的数据
		      {
			       s=Get_Char();
			       if(s=='\r')  return;
		      }
	     }	      
}
/****************************************************************************
* 没有匹配命令处理
* 人口:Buf_Id 缓冲区编号 
* 本程序不适用于 长数据端口缓冲区
* 长数据端口缓冲区清除 请用 Clear_Ldata()函数
****************************************************************************/
void No_Match_Cmd_Pr(u8 Buf_Id)
{
    u8 c; 
    if(Buf_Id>COM1_BUF)            //判断端口是否错误
     return ;	 
    memcpy(&IBUF_Ptr,              //接收缓冲区处理结构体
           &IBUF_Ptr_TAB[Buf_Id],  
           sizeof(IBUF_Pr));
    for(;;)                       //扔掉多余的数据
     {
      c=Get_Char();               //取字符并清除该单元
      if((c=='\0')||
         (c=='\r'))
       return;  
     }
}
/****************************************************************************
* 发送一串口数据帧到发送FIFO
* 入口: Buf 缓冲区
*                  COM0_BUF        COM0接收缓冲区ID号       COM0
*                  COM1_BUF        COM1接收缓冲区ID号       COM1
* 出口: 0 缓冲区 UART FIFO 都空                 BUF_FIFO_BLK
*       1 缓冲区 UART FIFO 至少一个不空         BUF_FIFO_NBLK
*       2 缓冲区 UART FIFO 一条完整命令发送完毕 ONE_CMD_BLK
****************************************************************************/
u8 Send_One_Uart_Frm(u8 Buf)
{
    u8  c,n=0;
    UART_Typedef* UARTx;
    if(Buf>COM1_BUF)                                             //判断缓冲区是否错误
     return(BUF_FIFO_BLK);                                       //错误退出
    UARTx=(UART_Typedef*)UART_PORT[Buf];                         //UART口硬件地址
    for(;n<(UART_TFIFO_LEN-2);)                                  //循环发送(FIFO长度-2)个字节
     {
      if(*OBUF_Ptr_TAB[Buf].OTail==*OBUF_Ptr_TAB[Buf].OHead)     //判断所有数据是否发送完毕
       {
        *OBUF_Ptr_TAB[Buf].OCoun=0;                              //待发送命令条数清零
        n=0;
        break;	
       }  
      c=OBUF_Ptr_TAB[Buf].OBuf[*OBUF_Ptr_TAB[Buf].OTail];        //送入一字符
      OBUF_Ptr_TAB[Buf].OBuf[*OBUF_Ptr_TAB[Buf].OTail]=0;        //清空字符
      (*OBUF_Ptr_TAB[Buf].OTail)++;                              //处理指针+1 括号不能去掉 否则处理的是指针
      if(*OBUF_Ptr_TAB[Buf].OTail>=OBUF_Ptr_TAB[Buf].OBLEN)      //判断是否超出循环
       *OBUF_Ptr_TAB[Buf].OTail=0;
      if(c=='\r')                                                //判断是否为回车符
       {                                                         
        (*OBUF_Ptr_TAB[Buf].OCoun)--;                            //命令数减1
        UARTCharPut(UARTx,c);                                    //送入一字符
        n=0;
        break;
       }                                                         
      else if(c>=' ')                                            //ASC码
       {	 
        n++;
        UARTCharPut(UARTx,c);                                    //送入一字符
       }	  
     } 
    COM1_STimer=(u16)Timer_1ms;                                  //重启定时 	    
    if(n==0)                                                     //实际写入串口发送FIFO的字节数
     return(BUF_FIFO_BLK);                                       //无数据要发送 
    else
     return(BUF_FIFO_NBLK);                                      //缓冲区不空 	 	
}


/******************************************************
* 查询功耗测试数据
******************************************************/
void Com_CHK_Pr(void)
{
  u8 n;
  if((Rx_Com[0]!='C')||(ADC_Start!='Y'))                                       //功耗测试是否开始
		{
//			 memset(Rx_Com,0x00,RX_CMD_LEN);
//			 CHK_OneTime_Flag=0x00;
		 	return;
		}

  if((0x2f<Rx_Com[1]<0x3a)&&(0x2f<Rx_Com[2]<0x3a))       //查询的表位号是否为合法数据
	 {
	   n=(Rx_Com[1]&0x0f)*10+(Rx_Com[2]&0x0f);
		  if(n!=Mtr_Numb_ID)                                   //是否查询该功耗模块
				{
//					 memset(Rx_Com,0x00,RX_CMD_LEN);
//					 CHK_OneTime_Flag=0x00; 
					 return;
				}	
    else
    {
      memset(Rx_Com,0x00,RX_CMD_LEN);
      CHK_OneTime_Flag='Y';  
    }
  }  
//		else
//		{
//				memset(Rx_Com,0x00,RX_CMD_LEN);
//				CHK_OneTime_Flag=0x00; 
//		}
}  
/******************************************************
* 查询电压电流值
******************************************************/
void Com_CHK_Ui(void)
{
  u8 n;
  if((Rx_Com[0]!='Q')||(ADC_Start!='Y'))                                       //功耗测试是否开始
		{
//			 memset(Rx_Com,0x00,RX_CMD_LEN);
//			 CHK_OneTime_Flag=0x00;
		 	return;
		}

  if((0x2f<Rx_Com[1]<0x3a)&&(0x2f<Rx_Com[2]<0x3a))       //查询的表位号是否为合法数据
	 {
	   n=(Rx_Com[1]&0x0f)*10+(Rx_Com[2]&0x0f);
		  if(n!=Mtr_Numb_ID)                                   //是否查询该功耗模块
				{
//					 memset(Rx_Com,0x00,RX_CMD_LEN);
//					 CHK_OneTime_Flag=0x00; 
					 return;
				}	
    else
    {
      memset(Rx_Com,0x00,RX_CMD_LEN);
      CHK_ONETIME_FLAG1='Y';  
    }
  }  
//		else
//		{
//				memset(Rx_Com,0x00,RX_CMD_LEN);
//				CHK_OneTime_Flag=0x00; 
//		}
}  

/******************************************************
* 单三相台设置命令
******************************************************/
void Com_Set_ST(void)
{
    if(Rx_Para[0][0]=='3')                          //是否设置为三相台
     {	         
      if(SINGLE_OR_THREE!=THREE)                 //单三相设置发生改变
       {
        SINGLE_OR_THREE=THREE;  
       }                               
     } 
    else if(Rx_Para[0][0]=='1')
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
u8 Com_Get_Mea_Met(u8 Met)
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
* Gd:XXA(0d)  XX要测量的表位
* A 要测量的相 0:停止测量 1:A相 2:B相 3:C相
********************************************************/
void Com_Power_Test_Pr(void)
{
    u8 ID=0,c;
    ID=(Rx_Para[0][0]&0x0f)*10+(Rx_Para[0][1]&0x0f);             //取出测量表位号 
    c=Rx_Para[0][2]&0x0f;                                       //取测量控制字
    if((c>3)||(Com_Get_Mea_Met(ID)==0))                         //判断控制字是否错误,要测量的表位号不在功耗模块的测量范围内
     return;
    Current_Mea_Phase=c;                       //存储测试相别
    CNCL_CHNL_Sel1;                             //释放所有测量通道
    CNCL_CHNL_Sel2;
    Sel_Phase_NO;                              //不测量任何相别
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;
    if((ID==0)||                               //测量的表位号是0，退出功耗测量
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
void Com_Set_Mea_Met_Tab(void)
{
  u8 chnl,Met;
  chnl=(Rx_Para[0][0]&0x0f)*10+(Rx_Para[0][1]&0x0f);             //取出要设置的通道号 
	 Met=(Rx_Para[0][2]&0x0f)*10+(Rx_Para[0][3]&0x0f);              //取测量控制字
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
* 3:设置电压修正值
* 4:设置电流修正值
* 5:设置电压零点修正值
* 6:设置电流零点修正值
********************************************************/
void Com_Set_Vref(u8 type)
{
	 float f,f2;
	 if(type>6)
	 	return;                                       //修正值类型只能是0-6之间的数字
	 f=atof(Rx_Para[0]);                            //取出功耗修正系数
  if(type==UP_TYP)
  {
		f2=atof(U_Pr_Str_Com);
		f/=f2;
    f*=XIUZ.U_PXZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>3))   
     return;
    XIUZ.U_PXZ=f;
  }
  else if(type==US_TYP)
  {
		f2=atof(U_Pm_Str_Com);
		f/=f2;
    f*=XIUZ.U_SXZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>3))   
     return;
    XIUZ.U_SXZ=f;
  }
  else if(type==IS_TYP)
  {
		f2=atof(I_Pm_Str_Com);
		f/=f2;
    f*=XIUZ.I_SXZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>3))   
     return;
    XIUZ.I_SXZ=f;
  } 
  else if(type==U_TYP)
  {
		XIUZ.XZDU=f;                            //记下修正点电压值
		f/=U_current;                    //自动计算修正系数
    f*=XIUZ.U_XZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>3))   
     return;
    XIUZ.U_XZ=f;
		
  } 
  else if(type==I_TYP)
  {
		XIUZ.XZDI=f;                            //记下修正点电流值
		f/=I_current;                    //自动计算修正系数
    f*=XIUZ.I_XZ;                      //当前系数再乘要修正的系数
    if((f<0.2)||(f>4))   
     return;
    XIUZ.I_XZ=f;
  }  	
	else if(type==OU_TYP)               //当前零点修正值再加要修正的值
	{
		XIUZ.U_OFFSET+=f;
		
	}
	else if(type==OI_TYP)								//当前零点修正值再加要修正的值
	{
		XIUZ.I_OFFSET+=f;
	}
}
/******************************************************
* 接线方式设置
******************************************************/
void Com_Set_Wire_Type(void)
{
  u8 type;
  if(Rx_Para[0][1]=='\0')
    type=Rx_Para[0][0];
  else if(Rx_Para[0][3]=='\0')
    type=Rx_Para[0][2];
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
/******************************************************
* 读版本号,只有第一表位返回版本号
******************************************************/
void Com_Set_Read_Ver(void)             
{
    if(Mtr_Numb_ID==1)                       //是否为第1表位
    {    
      u8 n,d;
      for(n=0;n<10;)
       {
        d=VER_TAB[n];
        if(d=='\0')
         break;
        UART_TEMP_STR[n]=d; 
        n++;
       }
      if(n==0||n==10)                          //长度错误
       return;	    
     
      SendStr(COM0_BUF,                  //命令
              UART_TEMP_STR,
              7);
      COM0_OCoun=0x01;
    }
}
/****************************************************************************
* 接收电流有效值处理
****************************************************************************/
void Com_RCV_CRT(u8 Phase)
{
  float f=0.0;
	 if(ADC_Start!='Y'||Phase!=Current_Mea_Phase)
	  return;
	 f=atof(Rx_Para[0]);
	 if(f<0||f>200)
	  return;
	 Current_I_Value=f; 	
}
/****************************************************************************
* 处理COM0接收缓冲区
****************************************************************************/
void Proc_COM0_IBuf(void)  
{
    u16 m;
    if(!COM0_ICoun)                                                    //是否收到波形合成箱数据
     return;                                                          
    COM0_ICoun--;                                                     
    Get_Com_Cmd(COM0_BUF);                                             //取命令头
  
    if(Rx_Para[0][0]=='\0')                                            //无参数处理
    {
      if(Rx_Com[0]=='C') 
      {
        Com_CHK_Pr();
        return;       
      }
			else if(Rx_Com[0]=='Q')
			{
				Com_CHK_Ui();
				return;
			}
    }  
    
    m=Case_num(COM0_BUF);                                              //命令头比较 
    if(m==0xffff)   return;
    memset(Rx_Com,0x00,RX_CMD_LEN);
    switch(m)                                                          //
     {
      case UART_ICMD_CHK:                               break;         //接收查询信息命令ID
      case UART_ICMD_SETST:      Com_Set_ST();          break;         //设置单三相命令
      case UART_ICMD_MEA:        Com_Power_Test_Pr();   break;         //开始测量功耗命令
      case UART_ICMD_PMTR:       Com_Set_Mea_Met_Tab(); break;         //设置表位对应关系
      case UART_ICMD_XUP:        Com_Set_Vref(UP_TYP);  break;         //修正电压有功功耗
      case UART_ICMD_XUS:        Com_Set_Vref(US_TYP);  break;         //修正电压视在功耗
      case UART_ICMD_XIS:        Com_Set_Vref(IS_TYP);  break;         //修正电流视在功耗
			case UART_ICMD_XU:         Com_Set_Vref(U_TYP);   break;         //修正电压
			case UART_ICMD_XI:	       Com_Set_Vref(I_TYP);   break;         //修正电流
			case UART_ICMD_XOU:        Com_Set_Vref(OU_TYP);   break;         //修正电压零点
			case UART_ICMD_XOI:        Com_Set_Vref(OI_TYP);   break;         //修正电流零点
      case UART_ICMD_TYPE:       Com_Set_Wire_Type();   break;
      case UART_ICMD_VER:        Com_Set_Read_Ver();    break;         //查版本号
      case UART_ICMD_SOLID:      Set_SOLID();           break;         //固化数据
      case UART_ICMD_BOOT:       BOOT_INT_DIS();        break;         //进入引导程序
      case UART_ICMD_CRT_A:      Com_RCV_CRT(Phase_A);  break;         //接收A相电流有效值
      case UART_ICMD_CRT_B:      Com_RCV_CRT(Phase_B);  break;         //接收B相电流有效值
      case UART_ICMD_CRT_C:	     Com_RCV_CRT(Phase_C);  break;         //接收C相电流有效值
						case UART_ICMD_CRT_S:      Com_RCV_CRT(Phase_A);  break;         //接收单相电流有效值
      default:                                          break;                                               
     }
     	    	
}         
/****************************************************************************
* 处理COM0发送缓冲区
* 命令定时发送
* 2倍命令发送间隔时间 还没有发完 清除正在发送标志 防止中断不清除正在发送标志
****************************************************************************/
void Proc_COM0_OBuf(void)
{
    u16 t;
    if(!COM0_OCoun)                    //判断是否有需要发送的命令
     return;
    t=((u16)Timer_1ms-COM0_STimer);    // 	
    if(t<COM0_TIME)
     return;
    if(!COM0_InSend)                   //判断是否不在发送状态
     {
      if(COM0_OHead!=COM0_OTail)       //判断是否有数据要发送
       {	
        if(UARTBusy(UART0))            //检查UART是否正在发送数据
         return;                       //有数据要发送 发送缓冲区不空
        UARTIntDisable(UART0,UART_INT_TX); //先屏蔽发送中断 以免发送时申请中断
        t=Send_One_Uart_Frm(COM0_BUF);
        UARTIntEnable(UART0,UART_INT_TX);  //数据写入完毕打开发送中断 等待中断
        if(t==BUF_FIFO_NBLK)             //启动发送一帧 判断是否有后续数据
         return;                         //有后续数据 退出 等待程序下次循环 发送	
       }
      else 
       {	
        COM0_InSend=0;                 //清除正在发送标志
        COM0_OCoun=0;                  //清除待发送命令计数
       }  
      COM0_STimer=(u16)Timer_1ms;      //重启定时 	    
     }   		
    else if(t>COM0_OVTM)
     {	
      COM0_InSend=0;                   //清除正在发送标志
      COM0_STimer=(u16)Timer_1ms;      //重启定时 	    
     }	 
}           
           
