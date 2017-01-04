/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : vari.c
;* Author             : ������
;* ��������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"
//λ���� �����ֱ��� ����32��λ����	4�ֽڶ��� ��LM3S2139.h �ж���
//��������

u32         ADC_SEQ_Data[8];                //AD ��������
Adc_Dat     ADC_Data;                       //AD ��������
u16         ADC_Timer;                      //AD������ʱ
u16         ADC_SUM_Data;                   //�ۼ�����
u8          ADC_SUM_Cnt;                    //AD�ۼӼ���
                                            
u8          Current_Save_Tab;               //��ǰ����ֵ���ڱ��
u8          Save_Tab_Sts[SAVE_TAB_NUMB];    //��λ��״̬���

SAVE_S      XIUZ;                           //

u32         SysRCC_CFG;                     //ϵͳRCC����
u8          PLL_ERR_Cnt;                    //���໷�������
u8          CMD_DATA;                       //���ֽ���������
u8          WIRE_TYPE;                      //���߷�ʽ
  
u8          Ext_WDT_Timer;                  //���ÿ��Ź���ʱ��

u8          Mtr_Numb_ID;                    //��λ��
u8          Mtr_Numb_Str[3];                //��λ��ASC��

u32         Sysclk;                         //ϵͳƵ��
//�ж����õ��Ķ�ʱ�� ��volatile ���� �������ѭ���ȴ� ��ȡ��ǰ��ʱֵ����
vu32        Timer_1ms;                      //1ms��ʱ��
vu32        CLK_Timer;                      //ʱ�Ӳ�����ʱ
vu8         Timer_8ms;                      //8ms��ʱ�� ��չ��ʱʱ�� ���ֽ���չ��2.048s
vu8         GDT_Timer;                      //���ͷ���ж�������ʱ��
vu8         DZ_Timer;                       //����������ж�������ʱ��
vu8         SZ_Timer;                       //ʱ������ܽ��ؿ��ж϶�ʱ
vu8         XUL_Timer;                      //�������ж�������ʱ��
vu8         TQ_Timer;                       //Ͷ�п��ж�������ʱ��
vu8         HZ_Timer;                       //��բ���ж�������ʱ��
vu8         KEY_Timer;                      //�������ж�������ʱ��
//vu8         HC165_Timer;                    //HC165������ʱ
//vu8         CD4094_Timer;                   //CD4094�����ʱ
vu8         PLL_CHK_Timer;                  //PLL��鶨ʱ

//���¶�ʱ��Timer_1ms ��ʱ ���Ƚϸ� ��ʱ�䳤��2.048s
u32         CYCLE_OLD_Timer;                //���ڲ�����ʱ��(��)
u32         CYCLE_NEW_Timer;                //���ڲ�����ʱ��(��)
//u32         CLK_Timer_Max;                  //ʱ�Ӳ����������ֵ
//u32         CLK_Timer_Min;                  //ʱ�Ӳ���������Сֵ
//u16         SY_Timer;                       //ʧѹ��ʱ
//u16         NY_SEND_Timer;                  //������Ͷ�ʱ
u16         CAN_STX_OVTimer;                //CAN �����ݷ��ͳ�ʱ��ʱ��
u16         CAN_LTX_OVTimer;                //CAN �����ݷ��ͳ�ʱ��ʱ��
u16         CAN_RX_OVTimer;                 //CAN ���߽��ճ�ʱ��ʱ�� ���ڴ������ߴ���
u16         WORK_Timer;                     //���빤��ģʽ��ʱ�� ��λ:ms ���65.535S
u16         STD_ECLK_Timer;                 //��׼���������ⶨʱ
//u16         MBJ_Send_Timer;                 //�������ݻ��Ͷ�ʱ
//u16         WZTZ_Send_Timer;                //������բ���ݻ��Ͷ�ʱ
//u16         NZTZ_Send_Timer;                //������բ���ݻ��Ͷ�ʱ
//u16         XLJDQ_ERR_Timer;                //�����̵������ϱ�����ʱ
u16         NEW_PLL_Ref_Timer;              //���໷�ο���ʱ��
u16         OLD_PLL_Ref_Timer;              //���໷�ο���ʱ��
//���¶�ʱ�� ��Timer_8ms ��ʱ
u8          Disp_Timer;                     //��ʾ��ʱ����
u8          GDT_RST_Timer;                  //���ͷ��λ��ʱ
//u8          Com_Rx_Time[UART_NUMB];         //COM ���ն�ʱ
//u8          Com_Rx_OvTm[UART_NUMB];         //UART���ճ�ʱ����
//u8          UJDQ_Timer;                     //��ѹ�̵�����ʱ
//u8          IJDQ_Timer;                     //�����̵�����ʱ
//u8          ESwitch_Timer;                  //���ӿ��ض�ʱ
//u8          NY_CHK_Timer;                   //��ѹ��ѯ��ʱ
u8          Con_KEY_Timer;                  //����������ʱ
u8          Disp_En_Timer;                  //��ʾʹ�ܶ�ʱ ��ֹ����д��SSI
u8          STD_CLK_Timer;                  //��׼ʱ�������ⶨʱ
u8          SMtr_Set_Timer;                 //��׼�����ö�ʱ

u8          Disp_Buf[8];                    //��ʾ������
UART_SET    Uart_Para[UART_NUMB];           //UART ����
//CANͨ�Ŷ���                               
u16         CANERR_CNT;                     //CAN�������  
u8          CAN_LMSG_TX_TYPE;               //��ǰ����CAN����������
u8          CAN_LMSG_RX_TYPE;               //��ȡ����CAN����������
CAN_LMSG_PR CAN_LMSG_RX_Ptr;                //CAN�����ݽ��սṹ��ָ��
CAN_LMSG_PR CAN_LMSG_TX_Ptr;                //CAN�����ݷ��ͽṹ��ָ��
CAN_STS_U   CAN_STS;                        //����״̬
u8          CAN_LDATA_TX_STS;               //CAN�����ݷ���״̬ ��define.h CAN�����ݷ���״̬����
u8          CAN_NEXT_MSG_IDx;               //CAN��һ֡���ĵ������� ���ݸú��ж��Ƿ�ʧ����
u8          CAN_LMSG_TX_STS;                //CAN��֡����״̬
u8          CAN_SMSG_TX_STS;                //CAN��֡����״̬
CAN_MSG     CAN_MSG_Rx;                     //������ʱ����CAN����
CAN_MSG     CAN_SMSG_Tx;                    //������ʱ����CAN�����ݱ���
CAN_MSG     CAN_LMSG_Tx;                    //������ʱ����CAN�����ݱ���
u8          Echo_Sts;                       //��Ӧ״̬  0 �޶��� 'C':�յ���ѯ 'S':�Ѿ����ͻ�Ӧ���� 'A':�Ѿ���Ӧ
u8          CAN_SEND_DTIME;                 //CAN֡������ʱʱ��
u8          CAN_SEND_DELAY;                 //CAN֡������ʱ��ʱ�� ��ֹͬʱ��������
CAN_MSG     CAN_SDATA_MSG_IBUF[CAN_SDILEN]; //CAN�����ݽ���ָ�����
u8          CAN_SDATA_MSG_IHead;            //CAN�����ݽ���ָ�����ͷָ�� ����ָ��
u8          CAN_SDATA_MSG_ITail;            //CAN�����ݽ���ָ�����βָ�� ����ָ��
CAN_MSG     *CAN_MSG_IPtr;                  //CAN������֡���մ���ָ��
CAN_MSG     CAN_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN�����ݷ���ָ�����
u8          CAN_SDATA_MSG_OHead;            //CAN�����ݷ���ָ�����ͷָ�� ����ָ��
u8          CAN_SDATA_MSG_OTail;            //CAN�����ݷ���ָ�����βָ�� ����ָ��
CAN_MSG     *CAN_MSG_OPtr;                  //CAN������֡���ʹ���ָ��

CAN_MSG     CAN_LDATA_MSG_IBUF[CAN_LDILEN]; //CAN�����ݽ���ָ�����               
u8          CAN_LDATA_MSG_IHead;            //CAN�����ݽ���ָ�����ͷָ�� ����ָ��
u8          CAN_LDATA_MSG_ITail;            //CAN�����ݽ���ָ�����βָ�� ����ָ��
u8          CAN_LDATA_SERR_Cnt;             //CAN�����ݷ��ʹ������


//Ϊ��д������ ���õķ��ͺͽ��ճ�����Ľṹ�� ����ָ���ָ��
IBUF_Pr     IBUF_Ptr;                        //���ջ���������ṹ��
OBUF_Pr     OBUF_Ptr;                        //���ͻ���������ṹ�� 
OBUF_Pr     UART_Ptr[4];                     //���ڷ��ͻ���������ṹ�� �������ж��е���                                                                   

u8          COM0_InSend;                     //COM0���ݷ���״̬ 'Y'���ڷ������� ���� ���ڷ�������
u8          COM0_IBuf[COM0_ILEN];            //COM0���ջ�����
u8          COM0_OBuf[COM0_OLEN];            //COM0���ͻ�����
u16         COM0_IHead;                      //COM0���ջ�����ͷָ�� ����ָ��
u16         COM0_ITail;                      //COM0���ջ�����βָ�� ����ָ��
u16         COM0_ICoun;                      //COM0�����������
u16         COM0_OHead;                      //COM0���ͻ�����ͷָ�����ָ��
u16         COM0_OTail;                      //COM0���ͻ�����βָ�봦��ָ��
u16         COM0_OCoun;                      //COM0�����������
u16         COM0_STimer;                     //COM0���Ͷ�ʱ

u8          COM1_InSend;                     //COM1���ݷ���״̬ 'Y'���ڷ������� ���� ���ڷ�������
u8          COM1_IBuf[COM1_ILEN];            //COM1���ջ�����
u8          COM1_OBuf[COM1_OLEN];            //COM1���ͻ�����
u16         COM1_IHead;                      //COM1���ջ�����ͷָ�� ����ָ��
u16         COM1_ITail;                      //COM1���ջ�����βָ�� ����ָ��
u16         COM1_ICoun;                      //COM1�����������
u16         COM1_OHead;                      //COM1���ͻ�����ͷָ�����ָ��
u16         COM1_OTail;                      //COM1���ͻ�����βָ�봦��ָ��
u16         COM1_OCoun;                      //COM1�����������
u16         COM1_STimer;                     //COM1���Ͷ�ʱ


u8          Rx_Com[RX_CMD_LEN];                //ȡ����ָ��ͷ//com0  com1 com2 ����
u8          Rx_Para[RX_PARA_NUM][RX_PARA_LEN]; //ȡ����ָ��ͷ ȡ�������� ���� ������������
u8          Para_Numb;                         //��������

u8          CANT_STR[8];                     //CAN֡�ַ�����ʱ������
u8          TEMP_STR[30];                    //CAN��ʱ���ַ���
u8          UART_TEMP_STR[UART_TEMP_STR_LEN];               //UART��ʱ���ַ���
u8          CHK_OneTime_Flag;                    //���յ�һ�β�ѯ�����־��
u8						CHK_ONETIME_FLAG1;
//ADC������
u8          ADC_Start;                       //ADCת����ʼ��־
//5460A������
u8          CS5460A_New_Data;                //CS5460A�������ݵȴ�����
u8          U_Mea_Cnt;                       //��ѹ��������
u8          I_Mea_Cnt;                       //������������
u8          CS5460A_E_Reg[3];                //CS5460A���ʼĴ���
u8          CS5460A_Vrms_Reg[3];             //��ѹ��Чֵ�Ĵ���
u8          CS5460A_Irms_Reg[3];             //������Чֵ�Ĵ���
u8          U_Data_Ready;                    //��ѹ��Ȧ��������׼���ñ�־
u8          I_Data_Ready;                    //������Ȧ��������׼���ñ�־
u8          U_Pr_Str[2];                     //��ѹ�й����ʴ洢�ռ�
u8          U_Pm_Str[2];                     //��ѹ���ڹ��ʴ洢�ռ�
u8          I_Pm_Str[2];                     //�������ڹ��ʴ洢�ռ�
u8          U_Pr_Str_Com[10];                 //��ѹ�й����ʴ洢�ռ�
u8          U_Pm_Str_Com[10];                 //��ѹ���ڹ��ʴ洢�ռ�
u8          I_Pm_Str_Com[10];                 //�������ڹ��ʴ洢�ռ�
u8          U_Str_Com[10];                 //��ѹ�洢�ռ�
u8          I_Str_Com[10];                 //�����洢�ռ�
u8          U_Mea_Cal;                       //��ѹ�������Կ�ʼ
u8          I_Mea_Cal;                       //�����������Կ�ʼ
u8          CS5460A_Sts_Reg[3];              //CS5460A״̬�Ĵ���
float       U_Prms[U_MEA_CNT];               //��ѹ�й�������Чֵ
float       U_Pmax[U_MEA_CNT];               //��ѹ���ڹ�����Чֵ
float       I_Pmax[I_MEA_CNT];               //�������ڹ�����Чֵ
float       U_P[U_MEA_CNT];               //��ѹ��Чֵ
float       I_P[I_MEA_CNT];               //������Чֵ
float U_current;
float I_current;
//float       Current_Adj_Val[3];              //��ǰ��������ֵϵ��
float       ADC0_Vref;                       //����������·�ο���ѹ3.0V
float       CS5460A_Vref;                    //CS5460A�ο���ѹ2.5V
float       Current_I_Value;                 //������Чֵ(Ĭ��5A)
Int_Char    Temp_Cal1;
Float_Char  Temp_Cal2;
u16         Timer_CS5460A_INT;               //CS5460A�ж϶�ʱ
//u8          Mea_Met_Tab[3];                  //����ͨ���Ͳ�����λ�Ķ�Ӧ��ϵ
u8          Current_Mea_MetNUM;              //��ǰ���ڲ����ı�λ��
u8          Current_Mea_Channel;             //��ǰ���ڲ�����ͨ����
u8          Current_Mea_Phase;               //���������
u16         Rslt_Send_Timer;                 //����������Ͷ�ʱ����2s����һ��
u16         Rslt_Com_Send_Timer;             //�������ݽ�����Ͷ�ʱ��2s����һ��
u8          Current_Wire_Type;               //��ǰ���߷�ʽ

