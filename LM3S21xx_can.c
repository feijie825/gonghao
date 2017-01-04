/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_can.c
;* Author             : 张力阵
;* CAN总线接口驱动程序 
;* 集成了CAN接口的LM3S 系列处理器 由于CAN时钟和CPU时钟之间频率的不一致,造成访问CAN寄存器时
;* 要添加额外延时 CPU 时钟频率高于CAN时钟频率
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "stdlib.h"
#include "string.h"
#include "define.h"
#include "vari.h"

//CAN寄存器访问延时定义
#define CAN_RW_DELAY            (1)
/******************************************************************************
* CAN波特率设置表
* 与CANBIT寄存器数据一致
* 位顺序 TSEG1 TSEG2  SJW PRE(预分频值)
* CAN位时间=(TSEG1+1)+(TSEG2+1)+1 
* 采样点=((TSEG1+1)+1)/CAN 份额时间个数
* CAN 核CLK 位8MHz 内部产生
* 由于CAN核时钟核CPU时钟间的差异 
* 当系统时钟频率大于8MHz时 访问CAN寄存器时要增加对应的延时
******************************************************************************/
const u32 CANBitClkSettings[]=
{
    CAN_TIMING(5,2,2,10),  // CANBAUD_100K CAN位时间=(4+1)+(1+1)+1=8个CAN份额时间  份额时钟=8/10=0.8MHz 采样点6/8=75%
    CAN_TIMING(5,2,2,8),   // CANBAUD_125K CAN位时间=(4+1)+(1+1)+1=8个CAN份额时间  份额时钟=8/8=1MHz    采样点6/8=75%
    CAN_TIMING(5,2,2,4),   // CANBAUD_250K CAN位时间=(4+1)+(1+1)+1=8个CAN份额时间  份额时钟=8/4=2MHz    采样点6/8=75%
    CAN_TIMING(5,2,2,2),   // CANBAUD_500K CAN位时间=(4+1)+(1+1)+1=8个CAN份额时间  份额时钟=8/2=4MHz    采样点6/8=75%
    CAN_TIMING(5,2,2,1)    // CANBAUD_1M   CAN位时间=(4+1)+(1+1)+1=8个CAN份额时间  份额时钟=8/1=8MHz    采样点6/8=75%
};
/*****************************************************************************
* CAN 报文对象处理设置表格 MSG RAM 
* 接收报文对象需要设置滤波 
* 发送报文对象不需设置滤波
* 各域位定义见 LM3S21xx_can.h 中报文结构体定义 
* 增加报文对象时 依次添加
*****************************************************************************/
const CAN_MSG_SET  CAN_MSG_SET_TAB[]=
{    //控制域     仲裁域     过滤屏蔽
    {0x000103E0,0x10000002,0x1800FFFF},  //扩展帧 主机发送短广播数据帧 MST_SCDATA_TX   MSG OBJ ID1  接收
    {0x000203E0,0x10000002,0x1800FFFF},  //扩展帧 主机发送短单播数据帧 MST_SSDATA_TX   MSG OBJ ID2  接收
    {0x00030050,0x10000002,0x1FFFFFFF},  //扩展帧 从机发送短数据帧     SLV_SDATA_TX    MSG OBJ ID3  发送
};
/*****************************************************************************
* 初始化CAN接口
*****************************************************************************/
void Init_CAN(void)
{
    CANBit_Timing  CANBit;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);  //使能CAN设备SYCTL_RCGC(使能时钟) 
    CANDisable(CAN0);                            //进入初始化模式
    CANBit.WORD = CANBitClkSettings[CANBAUD];
    Clr_MsgRam(CAN0);                            //清除CAN MSG RAM
    CANBitTimingSet(CAN0,&CANBit);               //设置波特率
    CANIntEnable(CAN0, CAN_INT_MASTER |	         //使能中断
                       CAN_INT_ERROR );          //使能错误计数中断 Boff Ewarn 
    Set_MsgRam(CAN0);                            //初始化can网络 设置接收报文对象过滤
    CANEnable(CAN0);                             
    CANStatusGet(CAN0, CAN_STS_CONTROL);         //清除状态中断
}
/*****************************************************************************
* 清除CAN MSG RAM区
* 入口:CANx CAN结构体
*****************************************************************************/
void Clr_MsgRam(CAN_Typedef *CANx)
{
    u32 iMsg;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    CANRegWrite((u32)(&CANx->CTL),CAN_CTL_INIT);//进入初始化模式 允许访问CAN RAM区
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {                                        //等待访问接口空闲
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK,      //访问命令屏蔽寄存器
                      CAN_IF1CMSK_WRNRD |     //读写控制 WRNRD 0 读 1写 本次写 INF->报文RAM
                      CAN_IF1CMSK_ARB |       //传输仲裁位 ID+Dir+Xtd+MsgVal位->报文RAM
                      CAN_IF1CMSK_CONTROL);   //传输控制位 ->报文RAM
    CANRegWrite((u32)&CANx->INF[0].ARB2 , 0); //仲裁寄存器2 包含 MsgVal Xtd Dir ID[28:16] 标准帧ID ID[28:18]
    CANRegWrite((u32)&CANx->INF[0].MCTL , 0); //报文控制寄存器
    for(iMsg = 1; iMsg <= 32; iMsg++)
     {                                        //初始化报文RAM
      while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
       {                                      //等待访问接口空闲 
       }
      CANRegWrite((u32)&CANx->INF[0].CRQ , iMsg); //写入报文ID
     }

    CANRegWrite((u32)&CANx->INF[0].CMSK ,     //访问命令屏蔽寄存器 WRNRD 0 读 报文RAM->INF
                      CAN_IF1CMSK_NEWDAT |    //访问报文RAM 清除NewDat 位
                      CAN_IF1CMSK_CLRINTPND); //访问报文RAM 清除IntPnd 中断挂起位
    for(iMsg = 1; iMsg <= 32; iMsg++)
     {
      while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
       {                                      //等待访问接口空闲
       }
      CANRegWrite((u32)&CANx->INF[0].CRQ , iMsg); //清除所有报文的 NEWDAT 位和 IntPnd位
     }
    CANRegRead(IntNumber,(u32)&CANx->STS);    //读状态寄存器 清除CAN中断寄存器 CAN->INT
}
/*****************************************************************************
* 设置CAN MSG 接收RAM区过滤设置
* 入口:CANx CAN结构体
* 长数据 命令 CMD=3预留
*****************************************************************************/
void Set_MsgRam(CAN_Typedef *CANx)
{
    CAN_MSG_SET RX_MSG_SET;    //接收报文
/*********设置主机发送短广播数据串 过滤  MSG RAM 编号MST_SCDATA_TX(8) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_SCDATA_TX-1],sizeof(CAN_MSG_SET));
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机正在发送短单播数据串 过滤  MSG RAM 编号MST_SSDATA_TX(9) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_SSDATA_TX-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.NUM = Mtr_Numb_ID;               //仲裁域	表位号
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
}
/*****************************************************************************
* 检查CAN设备基址是否正确
* CANx 结构体
* 调试时有效
*****************************************************************************/
#ifdef DEBUG
u8 CANBaseValid(CAN_Typedef *CANx)
{
    return(((u32)CANx == CAN0_BASE) || ((u32)CANx == CAN1_BASE) ||
           ((u32)CANx == CAN2_BASE));
}
#endif
/*****************************************************************************
* 获取CAN接口中断ID号 
* CANx 结构体
* 返回ID号
* 2008.12.5 为了提高处理速度zlz 处理直接返回中断在NVIC->EN 中的位置
*****************************************************************************/
u32 CANIntNumberGet(CAN_Typedef *CANx)
{
    return(((u32)CANx == CAN0_BASE) ? ~(1<<(INT_CAN0-48)) :
           (((u32)CANx == CAN1_BASE) ? ~(1<<(INT_CAN1-48)) :
           (((u32)CANx == CAN2_BASE) ? ~(1<<(INT_CAN2-48)) : ~(1<<(INT_CAN0-48)))));
}

/*****************************************************************************
* 读CAN寄存器
* 返回 寄存器值
*****************************************************************************/
u32	CANRegRead(u32 ulIntNumber,u32 ulRegAddress)
{
    u32 ulRetVal;
    u32 ReCANEnInt;                   //CAN中断重使能
    ReCANEnInt = HWREG(NVIC_EN1) ;
    HWREG(NVIC_EN1) = (ReCANEnInt & ulIntNumber);
    HWREG(ulRegAddress);              //第一次读CAN寄存器 (数据不正确 通知CAN控制器读请求)
    SysCtlDelay(CAN_RW_DELAY);        //延时
    ulRetVal = HWREG(ulRegAddress);		//第二次读CAN寄存器 (数据正确)
    HWREG(NVIC_EN1) = ReCANEnInt ;
    return(ulRetVal);
}

/*****************************************************************************
* 写CAN寄存器
* 入口:ulRegAddress寄存器地址
* ulRegValue 寄存器值
*****************************************************************************/
void CANRegWrite(u32 ulRegAddress, u32 ulRegValue)
{
    HWREG(ulRegAddress) = ulRegValue;
    SysCtlDelay(CAN_RW_DELAY);
}
/*****************************************************************************
* 写CAN数据寄存器
* 入口:pucData 待发送数据指针
* 入口:pulRegister 数据寄存器地址
* 入口:iSize 数据长度
* 实际发送数据长度受发送DLC控制位控制
*****************************************************************************/
void CANDataRegWrite(u8 *pucData, u32 pulRegister, u32 iSize)
{
    u32 iIdx;
    u32 ulValue;
    for(iIdx = 0; iIdx < iSize; )
     {
      ulValue = pucData[iIdx++];
      if(iIdx < iSize)
       ulValue |= (pucData[iIdx++] << 8);
      CANRegWrite(pulRegister, ulValue);
      pulRegister += 4;   //CAN数据寄存器地址+4
     }
}
/*****************************************************************************
* 读CAN数据寄存器
* 入口:IntNumber CAN中断号
* 入口:pucData 待返回数据的指针
* 入口:pulRegister 数据寄存器地址指针
* 入口:iSize 数据长度
* 实际接收数据长度受DLC控制位控制
*****************************************************************************/
void CANDataRegRead(u32 IntNumber,u8 *pucData, u32 pulRegister, u32 iSize)
{
    u32 iIdx;
    u32 ulValue;
    for(iIdx = 0; iIdx < iSize; )
     {
      ulValue = CANRegRead(IntNumber,pulRegister);
  	  pulRegister += 4;   //CAN数据寄存器地址+4
      pucData[iIdx++] = (u8)ulValue;
      if(iIdx < iSize)
       pucData[iIdx++] = (u8)(ulValue >> 8);
     }
}

/*****************************************************************************
* CAN控制器使能
* CANx 结构体
*****************************************************************************/
void CANEnable(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) & ~CAN_CTL_INIT));
}

/*****************************************************************************
* CAN控制器禁能
*****************************************************************************/
void CANDisable(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) | CAN_CTL_INIT));
}

/*****************************************************************************
* 获取 CAN 位定时设置
* 入口:CANx CAN结构体
* 返回 指针pClkParms
* 2008.11.24 张力阵修改 CANBPRE	默认为0 不扩展分频 分频系数由CANBIT产生
*****************************************************************************/
void CANBitTimingGet(CAN_Typedef *CANx, CANBit_Timing *CANBit_Time)
{
    u32 IntNumber = CANIntNumberGet(CANx);           //获取CAN中断ID号
    ASSERT(CANBaseValid(CANx));
    ASSERT(pClkParms != 0);
    CANBit_Time->WORD = CANRegRead(IntNumber,(u32)&CANx->BIT); //读取CAN bittime设置寄存器
}

/*****************************************************************************
* 设置 CAN 位定时
* 入口:CANx CAN结构体
* 入口: 指针pClkParms
*****************************************************************************/
void CANBitTimingSet(CAN_Typedef *CANx, CANBit_Timing *CANBit_Time)
{
    u32 uSavedInit;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    uSavedInit = CANRegRead(IntNumber,(u32)&CANx->CTL); //读控制寄存器
    CANRegWrite((u32)&CANx->CTL , (uSavedInit |
                                   CAN_CTL_INIT |       //写入初始化开始
                                   CAN_CTL_CCE));       //改变使能
    CANRegWrite((u32)&CANx->BIT , CANBit_Time->WORD);   //写入位设置
    CANRegWrite((u32)&CANx->BRPE , 0);                  //预分频=1
    uSavedInit &= ~CAN_CTL_CCE;                         //清除改变使能位
//    if(uSavedInit & CAN_CTL_INIT)                       //判断进入设置时是否在初始化状态
//     uSavedInit &= ~CAN_CTL_INIT;                       //退出初始化状态
    CANRegWrite((u32)&CANx->CTL , uSavedInit);          //写控制寄存器
}
/*****************************************************************************
* 设置CAN中断复位程序入口
* 入口:CANx CAN结构体
* pfnHandler 服务程序入口
* 使能 NVIC中断 中断向量表在RAM中时 使用
*****************************************************************************/
void CANIntRegister(CAN_Typedef *CANx, void (*pfnHandler)(void))
{
    u32 IntNumber;
    ASSERT(CANBaseValid(CANx));
    IntNumber = CANIntNumberGet(CANx);
    IntRegister(IntNumber, pfnHandler);
    IntEnable(IntNumber);
}
/*****************************************************************************
* 撤销CAN中断复位程序入口
* 入口:CANx CAN结构体
* 禁能 NVIC中断 中断向量表在RAM中时 使用
*****************************************************************************/
void CANIntUnregister(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    IntUnregister(IntNumber);
    IntDisable(IntNumber);
}
/*****************************************************************************
* 设置CAN中断使能位
* 入口:CANx CAN结构体
* 入口:ulIntFlags 需要中断的位
*****************************************************************************/
void CANIntEnable(CAN_Typedef *CANx, u32 ulIntFlags)
{
    u32 IntNumber = CANIntNumberGet(CANx);

    ASSERT(CANBaseValid(CANx));
    ASSERT((ulIntFlags & ~(CAN_CTL_EIE | CAN_CTL_SIE | CAN_CTL_IE)) == 0);

    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) | ulIntFlags));
}

/*****************************************************************************
* 禁能CAN中断位
* 入口:CANx CAN结构体
* 入口:ulIntFlags 需要中断的位
*****************************************************************************/
void CANIntDisable(CAN_Typedef *CANx, u32 ulIntFlags)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulIntFlags & ~(CAN_CTL_EIE | CAN_CTL_SIE | CAN_CTL_IE)) == 0);
    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) & ~(ulIntFlags)));
}

/*****************************************************************************
* 获取CAN 报文中断状态
* 入口:CANx CAN结构体
* 入口:eIntStsReg 需要读取中断报文ID 还是 中断报文位
*****************************************************************************/
u32	CANIntStatus(CAN_Typedef *CANx, CANIntSts eIntStsReg)
{
    u32 ulStatus;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    switch(eIntStsReg)
     {
      case CAN_INT_STS_CAUSE:
       {                //读取中断ID号
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->INT);
        return(ulStatus);
       }
      case CAN_INT_STS_OBJECT:
       {                //读取中断报文信息
        ulStatus = (CANRegRead(IntNumber,(u32)&CANx->MSG1INT) &
                    CAN_MSG1INT_INTPND_M);
        ulStatus |= (CANRegRead(IntNumber,(u32)&CANx->MSG2INT) << 16);
        return(ulStatus);
       }
      default:
       return(0);
     }
    
}
/*****************************************************************************
* 清除CAN 报文中断状态位
* 入口:CANx CAN结构体
* 入口:ulIntClr 清除中断状态位选择
* ulIntClr= CAN_INT_INTID_STATUS 清除状态中断
* ulIntClr= 1~32 清除报文中断状态
* 2011.3.22 zlz 将清除中断端口用1(中断中) 防止与CAN_Tx_Msg()程序中使用端口冲突
*****************************************************************************/
void CANIntClear(CAN_Typedef *CANx, u32 ulIntClr)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulIntClr == CAN_INT_INTID_STATUS) ||
           ((ulIntClr>=1) && (ulIntClr <=32)));
    if(ulIntClr == CAN_INT_INTID_STATUS)
     CANRegRead(IntNumber,(u32)&CANx->STS);
    else
     {
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {              //等待接口寄存器空闲
       }
      CANRegWrite((u32)&CANx->INF[1].CMSK , CAN_IF1CMSK_CLRINTPND);         //访问报文CLRINTPND位
      CANRegWrite((u32)&CANx->INF[1].CRQ , (ulIntClr & CAN_IF1CRQ_MNUM_M)); //设置要访问的报文
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {
       }
     }
}
/*****************************************************************************
* 设置CAN禁止自动重发位
* 入口:CANx CAN结构体
* 入口:bAutoRetry=1 使能自动重发 bAutoRetry=0禁能自动重发
*****************************************************************************/
void CANRetrySet(CAN_Typedef *CANx, u8 bAutoRetry)
{
    u32 ulCtlReg;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ulCtlReg = CANRegRead(IntNumber,(u32)&CANx->CTL);
    if(bAutoRetry)
     ulCtlReg &= ~CAN_CTL_DAR;               //允许自动重发
    else
     ulCtlReg |= CAN_CTL_DAR;                //禁止自动重发    
    CANRegWrite((u32)&CANx->CTL , ulCtlReg); //写控制寄存器
}

/*****************************************************************************
* 获取CAN自动重发状态
* 入口:CANx CAN结构体
* 返回 true 自动重发使能 false自动重发禁能
*****************************************************************************/
u8 CANRetryGet(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));

    if(CANRegRead(IntNumber,(u32)&CANx->CTL) & CAN_CTL_DAR)
     return(false);
    return(true);
}
/*****************************************************************************
* 获取CAN 中断状态
* 入口:CANx CAN结构体
* 入口:eStatusReg 需要获取状态位的类型 CAN_STS_CONTROL...
*****************************************************************************/
u32	CANStatusGet(CAN_Typedef *CANx, CANSts eStatusReg)
{
    u32 ulStatus;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    switch(eStatusReg)
     {
      case CAN_STS_CONTROL:
       {                 //读取系统状态寄存器 同时清除状态位
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->STS);
        CANRegWrite((u32)&CANx->STS , (~(CAN_STS_RXOK | CAN_STS_TXOK | CAN_STS_LEC_M)));
        return(ulStatus);
       }
      case CAN_STS_TXREQUEST:
       {                 //读取发送请求状态
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->TXRQ1);
        ulStatus |= CANRegRead(IntNumber,(u32)&CANx->TXRQ2) << 16;
        return(ulStatus);
       }
      case CAN_STS_NEWDAT:
       {                 //读取新数据接收状态
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->NWDA1);
        ulStatus |= CANRegRead(IntNumber,(u32)&CANx->NWDA2) << 16;
        return(ulStatus);
       }
      case CAN_STS_MSGVAL:
       {                 //读取报文RAM有效状态
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->MSG1VAL);
        ulStatus |= CANRegRead(IntNumber,(u32)&CANx->MSG2VAL) << 16;
        return(ulStatus);
       }
      default:
       return(0);       //非法 返回0
     }
}

/*****************************************************************************
* 获取CAN 错误计数值
* 入口:CANx CAN结构体
* 人口*CanErr_Cnt 错误计数返回指针
* 出口:true 达到错误认可门限 false未达到
*****************************************************************************/
u8 CANErrCntrGet(CAN_Typedef *CANx, u16 *CanErr_Cnt)
{
    u32 ulCANError;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ulCANError = CANRegRead(IntNumber,(u32)&CANx->ERR); //读错误计数寄存器
    *CanErr_Cnt = (ulCANError &(CAN_ERR_REC_M|CAN_ERR_TEC_M));  //错误计数
    if(ulCANError & CAN_ERR_RP)                         //判断是否达到错误认可门限
     return(true);
    return(false);
}

/*****************************************************************************
* 设置 CAN 接收报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针 
* 入口: eMsgType 报文类型
* 关闭接收到远程帧自动回送功能 ZLZ 2008.11.27
*****************************************************************************/
void CAN_RxMsg_Set(CAN_Typedef *CANx, CAN_MSG_SET *CAN_MSG, MsgType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usMaskReg[2]={0,0};
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_RX) ||
           (eMsgType == MSG_OBJ_TYPE_RX_REMOTE) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE) ||)
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {                                    //等待接口寄存器空闲
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //方向 INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //允许访问MSG 数据A
                    CAN_IF1CMSK_DATAB |   //允许访问MSG 数据B
                    CAN_IF1CMSK_CONTROL|  //允许访问MSG 控制位
					CAN_IF1CMSK_ARB);     //允许访问MSG 仲裁位
    switch(eMsgType)
     {                    //根据报文类型跳转
         case MSG_OBJ_TYPE_RX:
          {                //接收数据帧
           usArbReg[1] = 0;//设置方向位为接收数据帧
           break;
          }
         case MSG_OBJ_TYPE_RX_REMOTE:
          {                //接收远程帧
           usArbReg[1] = CAN_IF1ARB2_DIR;    //设置方向位为接收远程帧
           usMsgCtrl   = CAN_IF1MCTL_UMASK;  //访问屏蔽位
           usMaskReg[0] = 0xffff;            //远程帧默认ID过滤使能
           usMaskReg[1] = 0x1fff;            //远程帧默认ID过滤使能
           usCmdMaskReg |= CAN_IF1CMSK_MASK; //访问仲裁位
           break;
          }
         case MSG_OBJ_TYPE_RXTX_REMOTE:
          {                //接收远程帧后自动回送
           usArbReg[1] = CAN_IF1ARB2_DIR;
           usMsgCtrl = CAN_IF1MCTL_RMTEN | CAN_IF1MCTL_UMASK;
           break;
          }
         default:
          return;              //非法 退出
     }
     if(CAN_MSG->CTL.BIT.ID_FLT_EN )//判断是否使用ID过滤屏蔽
      {                        //使用ID过滤屏蔽
       if(CAN_MSG->CTL.BIT.EXD_ID)
        {                      //扩展帧	29bit ID
          usMaskReg[0] = CAN_MSG->IDMask.WORD & CAN_IF1MSK1_IDMSK_M;
          usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 16) & CAN_IF1MSK2_IDMSK_M);
        }
       else
        {                      //标准帧  
         usMaskReg[0] = 0;
         usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 6 ) & CAN_IF1MSK2_IDMSK_M);
        }
      }
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.EXT_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MXTD;  //使用扩展ID过滤
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.DIR_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MDIR;  //传输方向用于过滤
    if(CAN_MSG->CTL.BIT.ID_FLT_EN  ||
       CAN_MSG->CTL.BIT.EXT_FLT_EN ||
       CAN_MSG->CTL.BIT.DIR_FLT_EN)
     {
      usMsgCtrl |= CAN_IF1MCTL_UMASK;
      usCmdMaskReg |= CAN_IF1CMSK_MASK;
     }
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //扩展帧 29BIT ID
      usArbReg[0] |= CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD;
     }
    else
     {                      //标准帧
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL;
     }
    usMsgCtrl |= (CAN_MSG->CTL.BIT.LEN  | CAN_IF1MCTL_EOB); //设置控制位 数据长度 单次发送
    if(CAN_MSG->CTL.BIT.RX_INT_EN )                       //判断接收中断是否使能
     usMsgCtrl |= CAN_IF1MCTL_RXIE;                       //接收中断使能        
    CANRegWrite((u32)&CANx->INF[0].CMSK , usCmdMaskReg);  //写命令屏蔽寄存器
    CANRegWrite((u32)&CANx->INF[0].MSK1 , usMaskReg[0]);  //写屏蔽寄存器1
    CANRegWrite((u32)&CANx->INF[0].MSK2 , usMaskReg[1]);  //写屏蔽寄存器2
    CANRegWrite((u32)&CANx->INF[0].ARB1 , usArbReg[0]);   //写仲裁寄存器1
    CANRegWrite((u32)&CANx->INF[0].ARB2 , usArbReg[1]);   //写仲裁寄存器2
    CANRegWrite((u32)&CANx->INF[0].MCTL , usMsgCtrl);     //写报文控制寄存器
    CANRegWrite((u32)&CANx->INF[0].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));//INF->MSG RAM
}
/*****************************************************************************
* 设置 CAN 报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针 
* 入口: eMsgType 报文类型 MSG_OBJ_TYPE_TX  MSG_OBJ_TYPE_TX_REMOTE
* 2008.12.5 为提高处理速度 去掉远程帧发送 如果需要发送远程帧 要修改程序
*****************************************************************************/
void CAN_Tx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, MsgType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_TX) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE))
    
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {        //等待接口寄存器空闲
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //方向 INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //允许访问MSG 数据A
                    CAN_IF1CMSK_DATAB |   //允许访问MSG 数据B
                    CAN_IF1CMSK_CONTROL|  //允许访问MSG 控制位
                    CAN_IF1CMSK_ARB);     //允许访问MSG 仲裁位 
    if(eMsgType != MSG_OBJ_TYPE_TX)
     return;
    usArbReg[1] = (CAN_IF1ARB2_DIR|CAN_IF1ARB2_MSGVAL);    //设置方向位为发送 发送数据帧
    usMsgCtrl = (CAN_IF1MCTL_TXRQST| CAN_IF1MCTL_EOB);     //置位发送请求  发送数据帧
    usMsgCtrl |= CAN_MSG->CTL.BIT.LEN  ;                   //设置控制位 数据长度 单次发送
    if(CAN_MSG->CTL.BIT.TX_INT_EN )                        //判断发送中断是否使能
     usMsgCtrl |= CAN_IF1MCTL_TXIE;                        //发送中断使能
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //扩展帧 29BIT ID
      usArbReg[0]  = CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_XTD;
     }
    else
     {                      //标准帧
      usArbReg[0] = 0;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
     }
    if(CAN_MSG->CTL.BIT.LEN!=0)
     {                                         //数据帧 数据长度不为0导入数据
      CANDataRegWrite(&CAN_MSG->Data.BYTE[0],  //待发送数据指针
                     (u32)&CANx->INF[0].DA1,   //CAN数据寄存器地址
                     CAN_MSG->CTL.BIT.LEN);    //数据长度)
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK , usCmdMaskReg);  //写命令屏蔽寄存器
    CANRegWrite((u32)&CANx->INF[0].ARB1 , usArbReg[0]);   //写仲裁寄存器1
    CANRegWrite((u32)&CANx->INF[0].ARB2 , usArbReg[1]);   //写仲裁寄存器2
    CANRegWrite((u32)&CANx->INF[0].MCTL , usMsgCtrl);     //写报文控制寄存器
    CANRegWrite((u32)&CANx->INF[0].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));//INF->MSG RAM
}
/*****************************************************************************
* 读取 CAN 报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针
* 入口: bClrPendingInt 是否清除挂起位
* 2008.11.27 zlz 取消了屏蔽码读取 不接收远程帧 
*****************************************************************************/
void CAN_Rx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, u8 bClrPendingInt)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    usCmdMaskReg = (CAN_IF1CMSK_DATAA |   //允许访问DA
                    CAN_IF1CMSK_DATAB |   //允许访问DB
                    CAN_IF1CMSK_CONTROL | //访问控制位
                    CAN_IF1CMSK_MASK |    //访问屏蔽位
                    CAN_IF1CMSK_ARB|      //访问仲裁位
                    CAN_IF1CMSK_NEWDAT);  //清除新数据标志
    if(bClrPendingInt)                    //判断是否清除pending位
     usCmdMaskReg |= CAN_IF1CMSK_CLRINTPND;//清除
    CANRegWrite((u32)&CANx->INF[1].CMSK , usCmdMaskReg); //写命令屏蔽寄存器
    CANRegWrite((u32)&CANx->INF[1].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M)); //MSG RAM->INF
    while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
     {                                     //等待 MSG RAM->INF 完成
     }
    usArbReg[0]  = CANRegRead(IntNumber,(u32)&CANx->INF[1].ARB1); //读仲裁寄存器1
    usArbReg[1]  = CANRegRead(IntNumber,(u32)&CANx->INF[1].ARB2); //读仲裁寄存器2
    usMsgCtrl    = CANRegRead(IntNumber,(u32)&CANx->INF[1].MCTL); //读报文控制寄存器
/*
    CAN_MSG->CTL.WORD = MSG_OBJ_NO_FLAGS;//清除报文标志 
    if((!(usMsgCtrl & CAN_IF1MCTL_TXRQST) && (usArbReg[1] & CAN_IF1ARB2_DIR))
     || ((usMsgCtrl & CAN_IF1MCTL_TXRQST) && (!(usArbReg[1] & CAN_IF1ARB2_DIR))))
     CAN_MSG->CTL.BIT.RMT_FRM = 1;       //远程帧标志
*/
    if(usArbReg[1] & CAN_IF1ARB2_XTD)
     {                                 //扩展帧
      CAN_MSG->ID.WORD = (((usArbReg[1] & CAN_IF1ARB2_ID_M) << 16) | usArbReg[0]);//ID
      CAN_MSG->CTL.BIT.EXD_ID = 1;  //控制域中扩展帧标志
     }
    else							   //标准帧
     CAN_MSG->ID.WORD = (usArbReg[1] & CAN_IF1ARB2_ID_M) << 6;//ID
/* 2008.12.5 zlz 为提高处理速度去掉
    if(usMsgCtrl & CAN_IF1MCTL_MSGLST)             //判断是否有报文丢失 类似Overrun 
     CAN_MSG->CTL.BIT.DATA_LOST = 1;            //报文丢失标志 
*/
    if(usMsgCtrl & CAN_IF1MCTL_NEWDAT)              //判断是否有新数据
     {                                              //有新数据
      CAN_MSG->CTL.BIT.LEN = (usMsgCtrl & CAN_IF1MCTL_DLC_M);//数据长度
//      if(CAN_MSG->CTL.BIT.RMT_FRM == 0)
       {                                            //不为远程帧 
        CANDataRegRead(IntNumber,					          //读CAN数据寄存器 中断号
                         &CAN_MSG->Data.BYTE[0],	//保存数据指针
                         (u32)&CANx->INF[1].DA1,		//CAN数据寄存器地址
                         CAN_MSG->CTL.BIT.LEN);	//数据长度
       }
/*       
      CANRegWrite((u32)&CANx->INF[1].CMSK , CAN_IF1CMSK_NEWDAT);//清除新数据标志
      CANRegWrite((u32)&CANx->INF[1].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {              //等待写操作完成
       }
      CAN_MSG->CTL.BIT.NEW_DATA = 1;              //置位有新数据标志  
*/
     }
    else                                             //没有数据
     CAN_MSG->CTL.BIT.LEN = 0;                    //数据长度0 
}
/*****************************************************************************
* 清除 CAN 报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: ulObjID 报文编号
*****************************************************************************/
void CANMessageClear(CAN_Typedef *CANx, u32 ulObjID)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulObjID >= 1) && (ulObjID <= 32));
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {             //等待接口寄存器组完成
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK , (CAN_IF1CMSK_WRNRD | //写 INF->MSG RAM
                                           CAN_IF1CMSK_ARB));  //传送 ARB仲裁位
    CANRegWrite((u32)&CANx->INF[0].ARB1 , 0);                  // 
    CANRegWrite((u32)&CANx->INF[0].ARB2 , 0);                  //  
    CANRegWrite((u32)&CANx->INF[0].CRQ , (ulObjID & CAN_IF1CRQ_MNUM_M));
}
/*****************************************************************************
* 清除 报文发送
* 入口:CANx CAN结构体
* 入口: ulObjID 报文编号
*****************************************************************************/
void CAN_MSG_TX_CANCEL(CAN_Typedef *CANx, u32 ulObjID)
{
    u32 usMsgCtrl =	 CAN_IF1MCTL_EOB;                          //清除Txrqst 标志
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulObjID >= 1) && (ulObjID <= 32));
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {             //等待接口寄存器组完成
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK , (CAN_IF1CMSK_WRNRD | //写 INF->MSG RAM
                                           CAN_IF1CMSK_CONTROL|
										   CAN_IF1CMSK_CLRINTPND));//访问控制位
    CANRegWrite((u32)&CANx->INF[0].MCTL , usMsgCtrl);          //写报文控制寄存器
    CANRegWrite((u32)&CANx->INF[0].CRQ , (ulObjID & CAN_IF1CRQ_MNUM_M));
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {             //等待接口寄存器组完成
     }
}
