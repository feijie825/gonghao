/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_can.c
;* Author             : ������
;* CAN���߽ӿ���������������Ԥ���� 
;* 2008.11.26 Ϊ�˷�������ͼ��ٴ洢�ռ� ���������¶���CAN�ṹ��
;* CAN ID��ǰ7λ����ȫΪ1(��չ֡�ͱ�׼֡�������)
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
* ��ȡ�ж�״̬ѡ��
* CANIntStatus()���
*****************************************************************************/
typedef enum
{
    CAN_INT_STS_CAUSE,	   //��CAN�ж�ID    CANINT
    CAN_INT_STS_OBJECT	   //�������ж�״̬ CANMSG
}CANIntSts;

/*****************************************************************************
* �����ȡ״̬�Ĵ�������
* CANStatusGet() ���
*****************************************************************************/
typedef enum
{
    CAN_STS_CONTROL,    //��ȡCAN��״̬�Ĵ���
    CAN_STS_TXREQUEST,  //��ȡ��������״̬�Ĵ���
    CAN_STS_NEWDAT,     //��ȡ������״̬�Ĵ���
    CAN_STS_MSGVAL      //��ȡ����RAM�Ƿ���Ч״̬�Ĵ��� 
}CANSts;

/*****************************************************************************
* �����жϱ�־
* CANIntEnable()  �˿�
* CANIntDisable() �˿�
*****************************************************************************/
typedef enum
{
    CAN_INT_ERROR =   8,  //�����ж�  CANSTS.Boff CANSTS.EWarn �ı��ж�ʹ��
    CAN_INT_STATUS =  4,  //״̬�ж�  TxOK RxOK �����ߴ����ж�ʹ��
    CAN_INT_MASTER =  2	  //���ж�	  �ж���ʹ��
}
CANIntFlags;

/*****************************************************************************
* ��������
* CAN_RxMsg_Set()�˿�
*****************************************************************************/
typedef enum
{
    MSG_OBJ_TYPE_TX,            //���ͱ���
    MSG_OBJ_TYPE_TX_REMOTE,     //����Զ�̱���
    MSG_OBJ_TYPE_RX,            //���ձ���			     ��DIR�˲�ʹ�ܺ� ֻƥ������֡
    MSG_OBJ_TYPE_RX_REMOTE,     //����Զ�̱���		     ��DIR�˲�ʹ�ܺ� ֻƥ��Զ��֡
    MSG_OBJ_TYPE_RXTX_REMOTE    //����Զ�̱��ĺ��Զ�����
}MsgType;

/*****************************************************************************
* CAN ״̬
* CANStatusGet()����
*****************************************************************************/
typedef enum
{
    CAN_BUS_OFF  = (1<<7), //��������
    CAN_EWARN    = (1<<6), //�ﵽ�������޴������>=96
    CAN_EPASS    = (1<<5), //�����Ͽ� �������>=128
    CAN_RXOK     = (1<<4), //�ɹ��յ�����
    CAN_TXOK     = (1<<3), //�ɹ����ͱ���
    CAN_ERR_NONE =   0,    //û�д���
    CAN_ERR_STUFF=   1,    //λ������
    CAN_ERR_FORM =   2,    //��ʽ����
    CAN_ERR_ACK  =   3,    //��Ӧ����
    CAN_ERR_BIT1 =   4,    //λ1����
    CAN_ERR_BIT0 =   5,    //λ0����
    CAN_ERR_CRC  =   6,    //У�����
    CAN_ERR_MASK =   7     //�������������
}
CANStatus;

/*****************************************************************************
* ��־
* CANMessageSet() �˿�
* CANMessageGet() ����
* CAN_MSG_SET->ulFlags 
* 
*****************************************************************************/
typedef enum
{
    CAN_TX_INT_EN   =  (1<<4),		 //CAN ����ʹ��
    CAN_RX_INT_EN   =  (1<<5),		 //CAN ����ʹ��
    CAN_EXD_FRM     =  (1<<6),		 //CAN ��չ֡
    CAN_ID_FILT_EN  =  (1<<7),		 //CAN �˲�ʹ��
    CAN_DIR_FILT_EN =  ((1<<8)|CAN_ID_FILT_EN), //����λ�˲�ʹ��
    CAN_EXT_FILT_EN =  ((1<<9)|CAN_ID_FILT_EN), //��չλ�˲�ʹ��
    CAN_RMT_FRM     =  (1<<10),      //CAN Զ��֡ 
    CAN_NEW_DATA    =  (1<<11),		 //CAN �յ���֡��־
    CAN_DATA_LOST   =  (1<<12),		 //CAN ֡��ʧ��־
    CAN_NO_FLAGS    =    0			 //CAN û�б�־
}CANFlags;

//CAN����״̬�ṹ�嶨��	�� CANStatus����һ��
typedef struct
{
    u32  LEC  :3; //���µ����ߴ������
    u32  TxOK :1; //���ĳɹ����ͱ�־
    u32  RxOK :1; //���ĳɹ����ձ�־ ����˽���޹�
    u32  EPass:1; //�����Ͽ�״̬ 1 ���ͻ���մ���>127(�Ͽ�����)
    u32  EWarn:1; //���󾯸�״̬ 1 ���ͻ���մ���>96(��������)
    u32  Boff :1; //��������״̬ 1 ģ��CAN��������
}CAN_STS_S;
//CAN����״̬�����嶨�� �������
typedef union
{
    u8        BYTE;
    CAN_STS_S BIT;
}CAN_STS_U;
//������ṹ�嶨��
typedef struct
{
    u32	 LEN          :4;     //���ݳ��� 0-8 4bit
    u32  TX_INT_EN    :1;     //�����ж�����λ
    u32  RX_INT_EN    :1;     //�����ж�����λ
    u32  EXD_ID       :1;     //��չ֡��־
    u32  ID_FLT_EN    :1;     //ID�˲�ʹ��λ
    u32  DIR_FLT_EN   :1;     //����λ�˲�ʹ��λ
    u32  EXT_FLT_EN   :1;     //��չλ�˲�ʹ��λ
    u32  RMT_FRM      :1;     //Զ��֡��־ 
    u32  NEW_DATA     :1;     //�����ݱ�־
    u32  DATA_LOST    :1;     //���ݶ�ʧ��־
    u32  RESV         :3;     //����
    u32  IDx          :5;     //���Ķ����� MSG RAM ID
}CTL_S;
//�����������嶨�� �������
typedef union
{
    u32  WORD;
    CTL_S BIT;       
}CTL_U;
//ID�ṹ�嶨��
typedef struct
{
    u32  DEV   :8  ;//�豸���ͺ�
    u32  NUM   :8  ;//���� 0 �㲥���� 1~255 ��λ��
    u32  CMD   :11 ;//������ 11bit 0~1983 b17~b27 11110111111(0x1BF) 
    u32  DIR   :1  ;//����λ DIR=0 �ܿ�->��Ԫ DIR=1 ��Ԫ->�ܿ�
    u32  EXD   :1  ;//ID��֡��ʶ 0 ��׼֡ 1 ��չ֡ ���ڿ��Ʒ������ȼ�
    u32  RESV  :3  ;//����
}ID_S;
//ID�����嶨��
typedef union
{
    u32  WORD;
    ID_S BIT;       
}ID_U;
typedef struct
{
    u32  DEV   :8  ;//�豸���ͺ�
    u32  NUM   :8  ;//���� 0 �㲥���� 1~255 ��λ��
    u32  CMD   :11 ;//������ 11bit 0~1983 b17~b27 11110111111(0x1BF) 
    u32  DIR   :1  ;//����λ DIR=0 �ܿ�->��Ԫ DIR=1 ��Ԫ->�ܿ�
    u32  EXD   :1  ;//ID��֡��ʶ 0 ��׼֡ 1 ��չ֡ ���ڿ��Ʒ������ȼ�
    u32  RESV  :3  ;//����
}MASK_S;
typedef union
{
    u32    WORD;
    MASK_S BIT;
}MASK_U;
//�������ݽṹ��
typedef struct
{
    u16  DA1;
    u16  DA2;
    u16  DB1;
    u16  DB2;
}DATA_S;
//CAN�������ݹ����� �������
typedef union
{
    DATA_S WORD;        //
    u8     BYTE[8];	    //����˳�� Byte[0](1) ... Byte[7] DA1.L DA1.H ... DB2.L DB2.H
}DATA_U;
// ���Ľ�����������
typedef struct
{
    CTL_U  CTL;         //������ �������ݳ���
    ID_U   ID;          //����ID ID.31=0 ��չ���� ID.31=0 ��׼����
    MASK_U IDMask;      //����ID����
}CAN_MSG_SET;
//���ͺͽ��յ��ı��Ľṹ�嶨�� ��ֱ֡�Ӵ�����ձ�����
typedef struct
{
    CTL_U  CTL;         //������ �������ݳ���
    ID_U   ID;          //����ID ID.31=0 ��չ���� ID.31=0 ��׼����
    DATA_U Data;        //��������
}CAN_MSG;
//CAN�����ݴ���ṹ��
typedef struct
{
    u8   *BUF;    //ָ�����ݴ��λ��  ��:ָ��Com0_Obuf
    u16  *HEAD;   //ָ������ͷ  ָ��	��:ָ��Com0_OHead
    u16  *TAIL;	  //ָ������β  βָ��	��:ָ��Com0_OTail
    u8   *STS;    //ָ�򻺳���״̬ ָ��	��:ָ��Com0_Tx_Sts
}CAN_LMSG_PR;

/*****************************************************************************
* CANλ��ʱ�ṹ�� 
* �޸ĳ���CANBIT �Ĵ���˳��һ��
* CANλʱ��= (TSEG1+1)+(TSEG2+1)+1
* 2008.11.24 ���������
*****************************************************************************/
typedef struct
{
    u32 Prescaler: 6;    //Ԥ��Ƶ ����λ�ݶ�ʱ��
    u32 SJW      : 2;    //ͬ����ת���
    u32 TSEG1    : 4;    //������֮ǰ��ʱ���
    u32 TSEG2    : 3;    //������֮���ʱ���
}CANBit_Time;
/*****************************************************************************
* CANλ��ʱ������
* Ϊ�˲�������
* 2008.11.24 ���������
*****************************************************************************/
typedef union
{
    u32 WORD;
    CANBit_Time BIT;
}CANBit_Timing;
//����CANBITд��ֵ�궨��
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
extern void Clr_MsgRam(CAN_Typedef *CANx);//���CAN MSG RAM��
extern void Set_MsgRam(CAN_Typedef *CANx);//����CAN MSG RAM��
extern void CAN_ECHO(void);               //�յ�������ѯ��Ϣ��Ӧ
extern void CAN_TEST(void);
/*****************************************************************************
* CAN�����ݷ���ָ�����ͷָ���1����
*****************************************************************************/
extern void SDATA_MSG_OHead_ADD_ONE(void);
/*****************************************************************************
* ��� ���ķ���
*****************************************************************************/
void CAN_MSG_TX_CANCEL(CAN_Typedef *CANx, u32 ulObjID);
/*****************************************************************************
* �ӻ�CAN���ݷ�������ʱ����
*****************************************************************************/
void Proc_CAN_OvTm(void);  

/*****************************************************************************
* �����ݽ��մ���
*****************************************************************************/
void CAN_LDATA_RX_Pr(CAN_MSG *CAN_RX_MSG);
/*****************************************************************************
* �����ݷ��ʹ���
*****************************************************************************/
void CAN_LDATA_TX_Pr(void);
/*****************************************************************************
* ����������ѯ��Ϣ
*****************************************************************************/
void Proc_Mst_Check(void);
void Proc_LDATA_MSG_IBUF(void);
/*****************************************************************************
* ����CAN�����ݽ���ָ�����
*****************************************************************************/
void Proc_SDATA_IBUF(void);
/*****************************************************************************
* ����CAN�����ݷ���ָ�����
*****************************************************************************/
void Proc_SDATA_OBUF(void);
/*****************************************************************************
* �ӻ������ط�����֡
*****************************************************************************/
void SLV_REQ_RETX(void);
/*****************************************************************************
* �ӻ�������������֡
*****************************************************************************/
void SLV_REQ_TX(void);
/****************************************************************************
* CAN���ߴ���״̬����
****************************************************************************/
void Proc_CAN_ERR_STS(void);  

/*****************************************************************************
* CAN�����ݷ��ʹ���ṹ����
*****************************************************************************/
extern const CAN_LMSG_PR CAN_LMSG_TX_TAB[];
/*****************************************************************************
* CAN�����ݽ��մ���ṹ����
*****************************************************************************/
extern const CAN_LMSG_PR CAN_LMSG_RX_TAB[];
/******************************************************************************
* CAN���������ñ�
******************************************************************************/
extern const u32 CANBitClkSettings[];
/*****************************************************************************
* CAN ���Ķ��������ñ�� MSG RAM 
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
