/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_can.c
;* Author             : 张力阵
;* CAN总线接口驱动程序声明和预定义 
;* 2008.11.26 为了方便操作和减少存储空间 张力阵重新定义CAN结构体
;* CAN ID域前7位不能全为1(扩展帧和标准帧都是如此)
*******************************************************************************/

#ifndef __CAN_H__
#define __CAN_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
/*****************************************************************************
* 读取中断状态选择
* CANIntStatus()入口
*****************************************************************************/
typedef enum
{
    CAN_INT_STS_CAUSE,	   //读CAN中断ID    CANINT
    CAN_INT_STS_OBJECT	   //读报文中断状态 CANMSG
}CANIntSts;

/*****************************************************************************
* 定义读取状态寄存器类型
* CANStatusGet() 入口
*****************************************************************************/
typedef enum
{
    CAN_STS_CONTROL,    //读取CAN核状态寄存器
    CAN_STS_TXREQUEST,  //读取发送请求状态寄存器
    CAN_STS_NEWDAT,     //读取新数据状态寄存器
    CAN_STS_MSGVAL      //读取报文RAM是否有效状态寄存器 
}CANSts;

/*****************************************************************************
* 设置中断标志
* CANIntEnable()  人口
* CANIntDisable() 人口
*****************************************************************************/
typedef enum
{
    CAN_INT_ERROR =   8,  //错误中断  CANSTS.Boff CANSTS.EWarn 改变中断使能
    CAN_INT_STATUS =  4,  //状态中断  TxOK RxOK 或总线错误中断使能
    CAN_INT_MASTER =  2	  //总中断	  中断总使能
}
CANIntFlags;

/*****************************************************************************
* 报文类型
* CAN_RxMsg_Set()人口
*****************************************************************************/
typedef enum
{
    MSG_OBJ_TYPE_TX,            //发送报文
    MSG_OBJ_TYPE_TX_REMOTE,     //发送远程报文
    MSG_OBJ_TYPE_RX,            //接收报文			     当DIR滤波使能后 只匹配数据帧
    MSG_OBJ_TYPE_RX_REMOTE,     //接收远程报文		     当DIR滤波使能后 只匹配远程帧
    MSG_OBJ_TYPE_RXTX_REMOTE    //接收远程报文后自动回送
}MsgType;

/*****************************************************************************
* CAN 状态
* CANStatusGet()出口
*****************************************************************************/
typedef enum
{
    CAN_BUS_OFF  = (1<<7), //总线脱离
    CAN_EWARN    = (1<<6), //达到报警门限错误计数>=96
    CAN_EPASS    = (1<<5), //错误认可 错误计数>=128
    CAN_RXOK     = (1<<4), //成功收到报文
    CAN_TXOK     = (1<<3), //成功发送报文
    CAN_ERR_NONE =   0,    //没有错误
    CAN_ERR_STUFF=   1,    //位填充错误
    CAN_ERR_FORM =   2,    //格式错误
    CAN_ERR_ACK  =   3,    //响应错误
    CAN_ERR_BIT1 =   4,    //位1错误
    CAN_ERR_BIT0 =   5,    //位0错误
    CAN_ERR_CRC  =   6,    //校验错误
    CAN_ERR_MASK =   7     //错误代码屏蔽域
}
CANStatus;

/*****************************************************************************
* 标志
* CANMessageSet() 人口
* CANMessageGet() 出口
* CAN_MSG_SET->ulFlags 
* 
*****************************************************************************/
typedef enum
{
    CAN_TX_INT_EN   =  (1<<4),		 //CAN 发送使能
    CAN_RX_INT_EN   =  (1<<5),		 //CAN 接收使能
    CAN_EXD_FRM     =  (1<<6),		 //CAN 扩展帧
    CAN_ID_FILT_EN  =  (1<<7),		 //CAN 滤波使能
    CAN_DIR_FILT_EN =  ((1<<8)|CAN_ID_FILT_EN), //方向位滤波使能
    CAN_EXT_FILT_EN =  ((1<<9)|CAN_ID_FILT_EN), //扩展位滤波使能
    CAN_RMT_FRM     =  (1<<10),      //CAN 远程帧 
    CAN_NEW_DATA    =  (1<<11),		 //CAN 收到新帧标志
    CAN_DATA_LOST   =  (1<<12),		 //CAN 帧丢失标志
    CAN_NO_FLAGS    =    0			 //CAN 没有标志
}CANFlags;

//CAN错误状态结构体定义	与 CANStatus定义一致
typedef struct
{
    u32  LEC  :3; //最新的总线错误代码
    u32  TxOK :1; //报文成功发送标志
    u32  RxOK :1; //报文成功接收标志 与过滤结果无关
    u32  EPass:1; //错误认可状态 1 发送或接收错误>127(认可门限)
    u32  EWarn:1; //错误警告状态 1 发送或接收错误>96(警告门限)
    u32  Boff :1; //总线脱离状态 1 模块CAN总线脱离
}CAN_STS_S;
//CAN错误状态共用体定义 方便操作
typedef union
{
    u8        BYTE;
    CAN_STS_S BIT;
}CAN_STS_U;
//控制域结构体定义
typedef struct
{
    u32	 LEN          :4;     //数据长度 0-8 4bit
    u32  TX_INT_EN    :1;     //发送中断允许位
    u32  RX_INT_EN    :1;     //接收中断允许位
    u32  EXD_ID       :1;     //扩展帧标志
    u32  ID_FLT_EN    :1;     //ID滤波使能位
    u32  DIR_FLT_EN   :1;     //方向位滤波使能位
    u32  EXT_FLT_EN   :1;     //扩展位滤波使能位
    u32  RMT_FRM      :1;     //远程帧标志 
    u32  NEW_DATA     :1;     //新数据标志
    u32  DATA_LOST    :1;     //数据丢失标志
    u32  RESV         :3;     //保留
    u32  IDx          :5;     //报文对象编号 MSG RAM ID
}CTL_S;
//控制域联合体定义 方便操作
typedef union
{
    u32  WORD;
    CTL_S BIT;       
}CTL_U;
//ID结构体定义
typedef struct
{
    u32  DEV   :8  ;//设备类型号
    u32  NUM   :8  ;//表编号 0 广播命令 1~255 表位号
    u32  CMD   :11 ;//命令区 11bit 0~1983 b17~b27 11110111111(0x1BF) 
    u32  DIR   :1  ;//方向位 DIR=0 总控->单元 DIR=1 单元->总控
    u32  EXD   :1  ;//ID中帧标识 0 标准帧 1 扩展帧 用于控制发送优先级
    u32  RESV  :3  ;//保留
}ID_S;
//ID联合体定义
typedef union
{
    u32  WORD;
    ID_S BIT;       
}ID_U;
typedef struct
{
    u32  DEV   :8  ;//设备类型号
    u32  NUM   :8  ;//表编号 0 广播命令 1~255 表位号
    u32  CMD   :11 ;//命令区 11bit 0~1983 b17~b27 11110111111(0x1BF) 
    u32  DIR   :1  ;//方向位 DIR=0 总控->单元 DIR=1 单元->总控
    u32  EXD   :1  ;//ID中帧标识 0 标准帧 1 扩展帧 用于控制发送优先级
    u32  RESV  :3  ;//保留
}MASK_S;
typedef union
{
    u32    WORD;
    MASK_S BIT;
}MASK_U;
//报文数据结构体
typedef struct
{
    u16  DA1;
    u16  DA2;
    u16  DB1;
    u16  DB2;
}DATA_S;
//CAN报文数据共用体 方便访问
typedef union
{
    DATA_S WORD;        //
    u8     BYTE[8];	    //发送顺序 Byte[0](1) ... Byte[7] DA1.L DA1.H ... DB2.L DB2.H
}DATA_U;
// 报文接收屏蔽设置
typedef struct
{
    CTL_U  CTL;         //控制域 包含数据长度
    ID_U   ID;          //报文ID ID.31=0 扩展报文 ID.31=0 标准报文
    MASK_U IDMask;      //报文ID屏蔽
}CAN_MSG_SET;
//发送和接收到的报文结构体定义 短帧直接存入接收报文区
typedef struct
{
    CTL_U  CTL;         //控制域 包含数据长度
    ID_U   ID;          //报文ID ID.31=0 扩展报文 ID.31=0 标准报文
    DATA_U Data;        //报文数据
}CAN_MSG;
//CAN长数据处理结构体
typedef struct
{
    u8   *BUF;    //指向长数据存放位置  如:指向Com0_Obuf
    u16  *HEAD;   //指向数据头  指针	如:指向Com0_OHead
    u16  *TAIL;	  //指向数据尾  尾指针	如:指向Com0_OTail
    u8   *STS;    //指向缓冲区状态 指针	如:指向Com0_Tx_Sts
}CAN_LMSG_PR;

/*****************************************************************************
* CAN位定时结构体 
* 修改成与CANBIT 寄存器顺序一致
* CAN位时间= (TSEG1+1)+(TSEG2+1)+1
* 2008.11.24 张力阵调整
*****************************************************************************/
typedef struct
{
    u32 Prescaler: 6;    //预分频 产生位份额时间
    u32 SJW      : 2;    //同步跳转宽度
    u32 TSEG1    : 4;    //采样点之前的时间段
    u32 TSEG2    : 3;    //采样点之后的时间段
}CANBit_Time;
/*****************************************************************************
* CAN位定时共用体
* 为了操作方便
* 2008.11.24 张力阵调整
*****************************************************************************/
typedef union
{
    u32 WORD;
    CANBit_Time BIT;
}CANBit_Timing;
//产生CANBIT写入值宏定义
#define CAN_TIMING(tseg1, tseg2, sjw, brp)  (((tseg2-1) & 0x07) << 12)\
                                          | (((tseg1-1) & 0x0F) << 8) \
                                          | (((sjw-1) & 0x03) << 6)   \
                                          | ((brp-1) & 0x3F)

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern void CANBitTimingGet(CAN_Typedef *CANx, CANBit_Timing *pClkParms);
extern void CANBitTimingSet(CAN_Typedef *CANx, CANBit_Timing *pClkParms);
extern void CANDisable(CAN_Typedef *CANx);
extern void CANEnable(CAN_Typedef *CANx);
extern u8   CANErrCntrGet(CAN_Typedef *CANx, u16 *CanErr_Cnt);
extern void Init_CAN(void);
extern void CANIntClear(CAN_Typedef *CANx, u32 ulIntClr);
extern void CANIntDisable(CAN_Typedef *CANx, u32 ulIntFlags);
extern void CANIntEnable(CAN_Typedef *CANx, u32 ulIntFlags);
extern void CANIntRegister(CAN_Typedef *CANx, void (*pfnHandler)(void));
extern u32 CANIntStatus(CAN_Typedef *CANx,CANIntSts eIntStsReg);
extern void CANIntUnregister(CAN_Typedef *CANx);
extern void CANMessageClear(CAN_Typedef *CANx, u32 ulObjID);
extern void CAN_RxMsg_Set(CAN_Typedef *CANx, CAN_MSG_SET *CAN_MSG, MsgType eMsgType);
extern void CAN_Rx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, u8 ClrPending);
extern void CAN_Tx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, MsgType eMsgType);
extern u8 CANRetryGet(CAN_Typedef *CANx);
extern void CANRetrySet(CAN_Typedef *CANx, u8 bAutoRetry);
extern u32 CANStatusGet(CAN_Typedef *CANx, CANSts eStatusReg);
extern u32 CANIntNumberGet(CAN_Typedef *CANx);
extern void CANRegWrite(u32 ulRegAddress, u32 ulRegValue);
extern u32	CANRegRead(u32 ulIntNumber,u32 ulRegAddress);
extern void Clr_MsgRam(CAN_Typedef *CANx);//清除CAN MSG RAM区
extern void Set_MsgRam(CAN_Typedef *CANx);//设置CAN MSG RAM区
extern void CAN_ECHO(void);               //收到主机查询信息回应
extern void CAN_TEST(void);
/*****************************************************************************
* CAN短数据发送指令缓冲区头指针加1处理
*****************************************************************************/
extern void SDATA_MSG_OHead_ADD_ONE(void);
/*****************************************************************************
* 清除 报文发送
*****************************************************************************/
void CAN_MSG_TX_CANCEL(CAN_Typedef *CANx, u32 ulObjID);
/*****************************************************************************
* 从机CAN数据发送请求超时处理
*****************************************************************************/
void Proc_CAN_OvTm(void);  

/*****************************************************************************
* 长数据接收处理
*****************************************************************************/
void CAN_LDATA_RX_Pr(CAN_MSG *CAN_RX_MSG);
/*****************************************************************************
* 长数据发送处理
*****************************************************************************/
void CAN_LDATA_TX_Pr(void);
/*****************************************************************************
* 处理主机查询信息
*****************************************************************************/
void Proc_Mst_Check(void);
void Proc_LDATA_MSG_IBUF(void);
/*****************************************************************************
* 处理CAN短数据接收指令缓冲区
*****************************************************************************/
void Proc_SDATA_IBUF(void);
/*****************************************************************************
* 处理CAN短数据发送指令缓冲区
*****************************************************************************/
void Proc_SDATA_OBUF(void);
/*****************************************************************************
* 从机发送重发请求帧
*****************************************************************************/
void SLV_REQ_RETX(void);
/*****************************************************************************
* 从机发送数据请求帧
*****************************************************************************/
void SLV_REQ_TX(void);
/****************************************************************************
* CAN总线错误状态处理
****************************************************************************/
void Proc_CAN_ERR_STS(void);  

/*****************************************************************************
* CAN长数据发送处理结构体表格
*****************************************************************************/
extern const CAN_LMSG_PR CAN_LMSG_TX_TAB[];
/*****************************************************************************
* CAN长数据接收处理结构体表格
*****************************************************************************/
extern const CAN_LMSG_PR CAN_LMSG_RX_TAB[];
/******************************************************************************
* CAN波特率设置表
******************************************************************************/
extern const u32 CANBitClkSettings[];
/*****************************************************************************
* CAN 报文对象处理设置表格 MSG RAM 
*****************************************************************************/
extern const CAN_MSG_SET CAN_MSG_SET_TAB[];

//*****************************************************************************
//
// Several CAN APIs have been renamed, with the original function name being
// deprecated.  These defines provide backward compatibility.
//
//*****************************************************************************
#ifndef DEPRECATED
#define CANSetBitTiming(a, b)   CANBitTimingSet(a, b)
#define CANGetBitTiming(a, b)   CANBitTimingGet(a, b)
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __CAN_H__
