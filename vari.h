/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : vari.h
;* Author             : ������
;* ��������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "StdMtr.h"
typedef struct
{
    u16   Flag;          //�����־
    u16   CH0_MTR[3];    //ͨ�� ��λ ��Ӧ��ϵ Ĭ����ͨ��
    float U_PXZ;         //��ѹ�й���������ֵ
    float U_SXZ;         //��ѹ���ڹ�������ֵ	
    float I_SXZ;         //�������ڹ�������ֵ	
	  float U_XZ;          //��ѹ����ֵ	
	  float I_XZ;          //��������ֵ
		float U_OFFSET;      //��ѹ�������
	  float I_OFFSET;      //�����������
    float rset;          //�趨����ֵ	
	
		float XZDU;					//��ѹ������ֵ
		float XZDI;         //����������ֵ
}SAVE_S;
//AD���ݽṹ��
typedef struct
{
    u16 Data          :10;	 //����
    u16 New_Data      :1;    //��������
    u16 Trig          :1;    //������־
}Adc_Dat;

//����ֵ�ṹ�� ����Ϊ4byte���� ������дFLASH������ FLASH ��д��ַ������4�ı���
//IEEE��׼��������ʽ��λ--->��λ ����(1bit)+ָ��(8bit)+β��(23λ)
//TI ��������ʽ ���ֽ�(��λ--->��λ) ָ��(8bit)+7bit(β����λ)+1bit(����)+8bit(β����λ)+8bit(β�����λ)
//���� TI ������ С��ģʽ
//IEEE ������ ���ģʽ �ṹ�� 
typedef struct
{
    u32 Mant: 23;  //β��
    u32 Expo: 8 ;  //ָ��
    u32 Sign: 1 ;  //����
}IEEE_float_S;
//IEEE ������ ���ģʽ ������ 
typedef union
{
    float WORD;
    u8    BYTE[4];
    IEEE_float_S BIT;	
}IEEE_float_U ;
typedef struct
{
    u32 Expo: 8 ;  //ָ��
    u32 Mant1:7 ;  //β����7λ
    u32 Sign :1 ;  //����
    u32 Mant2:8 ;  //β����8λ
    u32 Mant3:8;   //β�����8λ
}TI_float;
//���ֹ�����
typedef union
{
    u16 WORD;
    u8  BYTE[2];     
}DB_U;
//�ֹ�����
typedef union
{
    u32 WORD;
    u8  BYTE[4];	
}WORD_U;
//���ջ���������ṹ�� ����ȡ��һ���ַ�
typedef struct
{
    u8  *IBuf;            //ָ�����IBUF��ָ��
    u16 *IHead;	          //����ָ��
    u16 *ITail;           //���մ���ָ��
    u16  IBLEN;           //���ջ���������
}IBUF_Pr;
//���ͻ���������ṹ�� ���ڷ���һ���ַ�
typedef struct
{
    u8  *OBuf;            //ָ�����IBUF��ָ��
    u16 *OHead;	          //����ָ��
    u16 *OTail;           //���մ���ָ��
    u16 *OCoun;           //�������
    u8  *InSend;          //���ڷ��ͱ�־
    u16 OBLEN;            //���ͻ���������
}OBUF_Pr;

//���巵��u8 �ĳ���ָ��
typedef struct
{
    u8 (*Ptr)(void);
}ret_u8_Ptr;
//������ �ĳ���ָ��
typedef struct
{
    void(*Ptr)(u16 a,u16 b);	
}ret_void_Ptr;
//������ �ĳ���ָ��
typedef struct
{
    void(*Ptr)(void);	
}void_void_Ptr;
//������ ��һ����������ĳ���ָ��
typedef struct
{
    void (*Ptr)(u8 a);	
}void_u8_Ptr;
//�ַ�ָ��ṹ��
typedef struct
{
    const u8 *Ptr;
}CChar_Ptr;
typedef struct
{
    u8 *Ptr;
}Char_Ptr;
typedef struct
{
    vu32 Flag;                //��־
    vu32 Numb;                //��λ��
}Mtr_Numb;
//CS5460A����
typedef union
{
  u8   Chrd[4];
  u32  Intd;
}Int_Char;
typedef union
{
  u8 Chr[4];
  float  Flt;
}Float_Char;
typedef struct 
{
    u32 Break     :1;        //������ֹλ
    u32 Parity_En :1;        //У������ 
    u32 Even      :1;        //żУ��ѡ��
    u32 Stop2     :1;        //2 stop bit
    u32 Fifo_En   :1;        //Fifo ʹ��
    u32 Data_Len  :2;        //����λ���� 0~5 3~8
    u32 Parity_01 :1;        //��żУ��̶�Ϊ0 ��1 parity_En=1 (Even=1 У��λΪ1,Even=0 У��λΪ0)
    u32 Baud      :18;       //������
}UART_PARA;
typedef struct
{
    u32 BAUD     :18;        //������
    u32 LEN      :4;         //���ݳ��� 5 6 7 8
    u32 STOP     :2;         //ֹͣλ 1 2 3=1.5   
    u32 PARITY   :3;         //У��λ 0:'N' 1:'O' 2:'E' 3:'M' 4:'S'
}UART_SET;
//����ģʽ����
typedef enum
{
    CAL_ENG_ERR_M   = 0,       //���㱻�������������
    MTR_PLUG_M      = 1,       //ѡ��ұ�״̬
    CATCH_HB_M      = 2,       //�Ժڰ�
    START_STOP_M    = 3,       //���ӵ���������� ����Ǳ��ģʽ���������
    VERIFY_READY_M  = 4,       //׼����ʼ���� �̵�������
    VERIFY_START_M  = 5,       //������������
    VOLTAGE_DOWN_M  = 6,       //�����ѹ��������
    MEASURE_ENG_M   = 7,       //����Ƶ������� ������������
    PANZHUAN_ERR_M  = 8,       //������ת�������
    MEA_CST_M       = 9,       //������������
    MEA_POWER_D_M   =10,       //���Ĳ�������
    MEA_ENG_DUTY_M  =11,       //�����趨�������ں�ռ�ձ�
    PULSE_ZZ_M      =12,       //��������������
    NYSY_M          =13,       //��ѹ���� ����¼��ѹ״̬������״̬
}MODE;
//CAN�����ݴ���ṹ��
typedef struct
{
    vu8 *Ptr;                  //ָ�򻺳���״̬ ָ��	��:ָ��Com0_Tx_Sts
}vu8_Ptr;

//CAN�����ݴ���ṹ��
typedef struct
{
    u32 *Ptr;                  //ָ�򻺳���״̬ ָ��	��:ָ��Com0_Tx_Sts
}u32_Ptr;       

extern u32         ADC_SEQ_Data[8];                //AD ��������
extern Adc_Dat     ADC_Data;                       //AD ��������
extern u16         ADC_Timer;                      //AD������ʱ
extern u16         ADC_SUM_Data;                   //�ۼ�����
extern u8          ADC_SUM_Cnt;                    //AD�ۼӼ���
                                                   
extern u8          Current_Save_Tab;               //��ǰ����ֵ���ڱ��
extern u8          Save_Tab_Sts[SAVE_TAB_NUMB];    //��λ��״̬���

extern SAVE_S      XIUZ;                           //
extern u8          Ext_WDT_Timer;                  //���ÿ��Ź���ʱ��
//ADC���ݶ���
extern u8          ADC_Start;                       //ADCת����ʼ��־
//CS5460A���ݶ���
extern u8          CS5460A_New_Data;                //CS5460A�������ݵȴ�����
extern u8          U_Mea_Cnt;                       //��ѹ��������
extern u8          I_Mea_Cnt;                       //������������
extern u8          CS5460A_E_Reg[3];                //CS5460A���ʼĴ���
extern u8          CS5460A_Vrms_Reg[3];             //��ѹ��Чֵ�Ĵ���
extern u8          CS5460A_Irms_Reg[3];             //������Чֵ�Ĵ���
extern u8          U_Data_Ready;                    //��ѹ��Ȧ��������׼���ñ�־
extern u8          I_Data_Ready;                    //������Ȧ��������׼���ñ�־
extern u8          U_Pr_Str[2];                     //��ѹ�й�����ASC��
extern u8          U_Pm_Str[2];                     //��ѹ���ڹ���ASC��
extern u8          I_Pm_Str[2];                     //�������ڹ���ASC��
extern u8          U_Pr_Str_Com[10];                 //��ѹ�й����ʴ洢�ռ�
extern u8          U_Pm_Str_Com[10];                 //��ѹ���ڹ��ʴ洢�ռ�
extern u8          I_Pm_Str_Com[10];                 //�������ڹ��ʴ洢�ռ�
extern u8          U_Str_Com[10];                 //��ѹ�洢�ռ�
extern u8          I_Str_Com[10];                 //�����洢�ռ�
extern u8          U_Mea_Cal;                       //��ѹ�������Կ�ʼ
extern u8          I_Mea_Cal;                       //�����������Կ�ʼ
extern u8          CS5460A_Sts_Reg[3];              //CS5460A״̬�Ĵ���
extern float       U_Prms[U_MEA_CNT];               //��ѹ�й�������Чֵ
extern float       U_Pmax[U_MEA_CNT];               //��ѹ���ڹ�����Чֵ
extern float       I_Pmax[I_MEA_CNT];               //�������ڹ�����Чֵ
extern float       U_P[U_MEA_CNT];               //��ѹ��Чֵ
extern float       I_P[I_MEA_CNT];               //������Чֵ
extern float U_current;
extern float I_current;
extern float       Current_Adj_Val[3];              //����ֵ
extern float       ADC0_Vref;                       //����������·�ο���ѹ��3.0V
extern float       CS5460A_Vref;                    //CS5460A�Ĳο���ѹ2.5V
extern float       Current_I_Value;                 //������Чֵ5A
extern Int_Char    Temp_Cal1;
extern Float_Char  Temp_Cal2;
extern u16         Timer_CS5460A_INT;               //CS5460A�ж϶�ʱ
//extern u8          Mea_Met_Tab[3];                  //����ͨ���Ͳ�����λ�Ķ�Ӧ��ϵ
extern u8          Current_Mea_MetNUM;              //��ǰ���ڲ����ı�λ��
extern u8          Current_Mea_Channel;             //��ǰ���ڲ�����ͨ����
extern u8          Current_Mea_Phase;               //���������
extern u16         Rslt_Send_Timer;                 //����������Ͷ�ʱ����2s����һ��
extern u16         Rslt_Com_Send_Timer;           
extern u8          Current_Wire_Type;               //��ǰ���߷�ʽ
//λ���� �����ֱ��� ����32��λ����	4�ֽڶ��� ��LM3S2139.h �ж���
extern u32         SysRCC_CFG;                     //ϵͳRCC����
extern u8          PLL_ERR_Cnt;                    //���໷�������
extern u8          Set_Uclop;                      //���õ�ѹ����
extern u8          Pwr_Phase;                      //������
extern u8          DXTZ;                           //����ѿر���բ�̵���״̬
extern u8          MBJ_DATA;                       //��������
extern u8          WZHZ_DATA[2];                   //���ú�բ�ź� [0]���� [1]���� 
extern u8          GZ_DATA;                        //��������
extern u8          NZHZ_DATA;                      //���ú�բ�ź�(������·�̵����ź�)
extern u8          TIME_ASC[8];                    //ʱ��ASC HH-MM-SS  ʱ����
extern u8          PLSGD;                          //���干�߹���
extern u8          MFClk_Mode;                     //�๦���������뷽ʽ
extern u8          MFClk_Type;                     //�๦��������������
extern u8          PLSHC_MODE;                     //����ϳɷ�ʽ
extern u8          PLS_QUAD;                       //��������
extern u8          Disp_Choose;                    //��ʾѡ��
extern u8          Disp_Code_Mode;                 //��ʾ����ģʽ
extern u16         ELEC_PLS_CNT;                   //��ǰ�����������
extern u8          ENG_ERR_ASC[9];                 //�������ASC�� ������λС���� ����λ +5λ����
extern u8          CLK_FREQ_ASC[9];                //ʱ��Ƶ��
extern u8          DAY_ERR_ASC[9];                 //�ռ�ʱ��� ���3��С����
extern u8          XUL_TIME_ASC[9];                //��������ASC�� 
extern u8          VERIFY_ENG_ASC[9];              //У�˳�ʱ���ֵ���ASC��ʾ
extern u8          CURRENT_N_ASC[2];               //��ǰȦ��ASC��
extern u8          CURRENT_ENG_ASC[9];             //��ǰ����ASC��
extern u8          CURRENT_PLS_ASC[9];             //��ǰ������ASC��
extern u8          HIGH_LVL_TIME[9];               //�ߵ�ƽʱ�� ��һ���ֽ� 'H'
extern u8          LOW_LVL_TIME[9];                //�͵�ƽʱ�� ��һ���ֽ� 'L'
extern u32         PLS_Lvl_Time[2];                //�����ƽʱ��
extern u32         High_Lvl_Time_Tp;               //��ʱ�������ߵ�ƽʱ��
extern u32         Low_Lvl_Time_Tp;                //��ʱ�������͵�ƽʱ��
extern u8          High_Lvl_CNT;                   //�ߵ�ƽ�жϲ�������
extern u8          Low_Lvl_CNT;                    //�͵�ƽ�жϲ�������
extern u8          CMD_DATA;                       //���ֽ���������
extern u8          WIRE_TYPE;                      //���߷�ʽ
extern u16         DIVIDE_Coef;                    //��Ƶϵ��
extern u8          CYCLE_MEA_SEL;                  //���ڲ���ѡ��
extern u8          CYCLE_MEA_ID;                   //���ڲ���ѡ��ID���
extern u8          SY_CNT;                         //ʧѹ����
extern u8          SY_PROCESS;                     //ʧѹ����
extern u8          SY_MODE;                        //ʧѹģʽ 
extern u8          SY_PHASE;                       //ʧѹ�� 0:����ͬʱ 1:UA 2:UB 3:UC
extern u8          PZZS;                           //������ת��� ����Ȧ������ 
extern u8          PZBZ;                           //��ת�������ֵ
extern u8          PZ_STD_CNT;                     //��ת�����׼����

extern u16         CUR_PZ_Cnt_Val;                 //��ת������   �����������ֵ
extern u16         PRE_PZ_Cnt_Val;                 //��һ�������� �����������ֵ

extern u8          PZ_ERR_ASC[3];                  //��ת���Ȧ��
extern float       VERIFY_ENG_Kwh;                 //У�˳������ֶ���
extern u32         VERIFY_PLS_Set;                 //У�˳�������������
extern u32         CURRENT_VERIFY_PLS;             //��ǰУ�˳�������������
extern u32         ZZ_PLS_Set;                     //��������������
extern u32         CURRENT_ZZ_PLS;                 //��ǰ����������    


//������������ۼӺ��ܵ���                                
extern u32         CURRENT_PLS_CNT;                //������� ������һֱ�ۼ�
extern float       CURRENT_ENG_KWh;                //��ǰ����
//���Ƶ��������                                   
extern float       GP_ENG_CST;                     //������Ƶ����
extern u32         GP_CLK_SET;                     //���Ƶ����������
extern u32         GP_CLK_ACT;                     //ʵ�ʸ�Ƶ����ֵ
extern u16         GP_RELOAD_TIME;                 //ʱ��Ƶ����װ����
extern u16         GP_RELOAD_VAL;                  //ʱ��Ƶ����װֵ
extern u16         GP_RELOAD_Cnt;                  //��Ƶ������װ����
//��׼ʱ�����������λ
extern u32         STD_CLK_Cnt;                    //��׼ʱ�Ӽ�����λ 
//ʱ������Ƶ�ʺ��ռ�ʱ�������������             
extern u8          CLK_MEA_CTL;                    //ʱ��Ƶ�ʲ�������
extern u16         CLK_STB_RNG;                    //ʱ�������ȶ������
extern u16         CLK_RELOAD_TIME_N;              //ʱ��Ƶ����װ������ֵ
extern u16         CLK_RELOAD_TIME_O;              //ʱ��Ƶ����װ������ֵ
extern u16         CLK_RELOAD_VAL_N;               //ʱ��Ƶ����װֵ��ֵ
extern u16         CLK_RELOAD_VAL_O;               //ʱ��Ƶ����װֵ��ֵ
extern u16         CLK_RELOAD_Cnt;                 //ʱ��������װ����
extern u8          CLK_MEA_TIME;                   //ʱ��Ƶ�ʲ�����ʱ ��λ:s ��  10---100s֮�� Ĭ��20s
extern float       CLK_FREQ_SET;                   //ʱ��Ƶ���趨ֵ Ĭ��1HZ
extern float       CLK_FREQ_INT;                   //ʱ��Ƶ�ʹ��ֵ Ĭ��1HZ
extern double      CLK_DAY_ERR;                    //�ռ�ʱ��� 
extern double      CLK_ACT_FREQ;                   //ʵ��Ƶ�� ������ 2009.3.13 Ϊ��߾��� ��Ϊ˫���ȸ�����
extern double      STD_CLK_VAL_ONE;                //��׼ʱ�Ӽ���ֵ ���ڼ���Ƶ��  �����ж�             ����ʱ��Ƶ�� ��׼ֵ
extern double      STD_CLK_VAL_SUM;                //��׼ʱ�Ӽ���ֵ ���ڼ���Ƶ�� CLK_RELOAD_TIME���ж� ����ʱ��Ƶ�� ��׼ֵ
extern u32         STD_CLK_CNT_ONE;                //��׼ʱ�Ӽ���ֵ ÿ���ж�                 Ӧ�Ƶı�׼������
extern u32         STD_CLK_CNT_SUM;                //��׼ʱ�Ӽ���ֵ �� CLK_RELOAD_TIME���ж� Ӧ�Ƶı�׼������
extern u32         RAW_CLK_VAL;                    //ԭʼʱ���������ֵ
extern u32         CUR_CLK_VAL;                    //��ǰʱ���������ֵ
extern u32         PRE_CLK_VAL;                    //�ϴ�ʱ���������ֵ

extern u32         SCLK_Cnt_OSub;                  //���μ���ֵ 
extern u32         SCLK_Cnt_NSub;                  //�ϴμ���ֵ
extern u32         SCLK_Cnt_RSet_Max;              //�ϼ����ı䴥������ʱ��Ƶ�����ֵ
extern u32         SCLK_STB_RNG;                   //ʱ�������ȶ������(���)

extern u32         OCLK_Cnt_OSub;                  //�ϴε���ʱ�����������ֵ �����ж��Ƿ��ȶ���
extern u32         OCLK_Cnt_NSub;                  //���ε���ʱ�����������ֵ �����ж��Ƿ��ȶ���
extern u32         OCLK_Cnt_RSet_Max;              //���μ����ı䴥������ʱ��Ƶ�����ֵ
extern u32         OCLK_STB_RNG;                   //ʱ�������ȶ������(����)
//�������ڱ�������
extern u8          XUL_MEA_CTL;                    //�������ڲ�������
extern float       XUL_TIME;                       //��������
extern u8          XUL_RELOAD_TIME;                //����������װ����
extern u8          XUL_RELOAD_Cnt;                 //�����������

extern u32         RAW_XUL_VAL;                    //ԭʼ�����жϱ�׼ʱ���������ֵ
extern u32         CUR_XUL_VAL;                    //��ǰ�����жϱ�׼ʱ���������ֵ
extern u32         PRE_XUL_VAL;                    //�ϴ������жϱ�׼ʱ���������ֵ

extern u32         SXUL_Cnt_OSub;                  //���μ���ֵ 
extern u32         SXUL_Cnt_NSub;                  //�ϴμ���ֵ

extern u32         OXUL_Cnt_OSub;                  //�ϴε���ʱ�����������ֵ �����ж��Ƿ��ȶ���
extern u32         OXUL_Cnt_NSub;                  //���ε���ʱ�����������ֵ �����ж��Ƿ��ȶ���

//������������������                           
extern u8          ENG_CLK_CH;                     //����ѡ�� 0:���ͷ���� 1:�������� 2:��׼������
extern MODE        WORK_MODE;                      //����ģʽ   
extern u16         ENG_STB_RNG;                    //�����ȶ������   
extern float       STD_ENG_CNT_VAL;                //��׼��������Ӧ�Ƹ��� = STD_ENG_CST*ACT_N/MTR_ENG_CST
extern float       STD_ENG_CST;                    //��׼���Ƶ���� ���ڼ������
extern float       MTR_ENG_CST;                    //����λ�����Ƶ����
extern float       MTR_MIN_CST;                    //���б�λ��С�����Ƶ����
extern u8          CURRENT_N;                      //��ǰȦ��
extern u8          ACT_N;                          //ʵ��У��Ȧ�� ACT_N=MTR_ENG_CST*SET_N/MTR_MIN_CST
extern u8          SET_N;                          //�趨У��Ȧ��
extern u32         STD_ENG_Cnt;                    //��׼ʱ�Ӽ�����λ

extern u32         CUR_ENG_VAL;                    //��ǰ�����������ֵ
extern u32         PRE_ENG_VAL;                    //�ϴε����������ֵ
extern u32         RAW_ENG_VAL;                    //ԭʼ�����������ֵ

extern u16         SENG_STB_RNG;                   //ʱ�������ȶ������(���)

extern u32         OEND_Cnt_OSub;                  //�ϴε���ʱ�����������ֵ �����ж��Ƿ��ȶ���
extern u32         OEND_Cnt_NSub;                  //���ε���ʱ�����������ֵ �����ж��Ƿ��ȶ���
extern u16         OEND_STB_RNG;                   //ʱ�������ȶ������(����)

extern float       ENG_ERR;                        //�������
                                                   
extern u8          Mtr_Numb_ID;                    //��λ��
extern u8          Mtr_Numb_Str[3];                //��λ��ASC��
//extern u8          Current_Mtr_Tab;                //��ǰ��λ�����ڱ��
//extern u8          Mtr_Tab_Sts[MTR_TAB_NUMB];      //��λ��״̬���

extern u32         Sysclk;                         //ϵͳƵ��

//�ж����õ��Ķ�ʱ�� ��volatile ����
extern vu32        Timer_1ms;                      //1ms��ʱ��
extern vu32        CLK_Timer;                      //ʱ�Ӳ�����ʱ
extern vu8         Timer_8ms;                      //8ms��ʱ�� ��չ��ʱʱ�� ���ֽ���չ��2.048s
extern vu8         GDT_Timer;                      //���ͷ���ж�������ʱ��
extern vu8         DZ_Timer;                       //����������ж�������ʱ��
extern vu8         SZ_Timer;                       //ʱ������ܽ��ؿ��ж϶�ʱ
extern vu8         XUL_Timer;                      //�������ж�������ʱ��
extern vu8         TQ_Timer;                       //Ͷ�п��ж�������ʱ��
extern vu8         HZ_Timer;                       //HZ�����ؿ��ж϶�ʱ
extern vu8         KEY_Timer;                      //���������ж϶�ʱ��
extern vu8         HC165_Timer;                    //HC165������ʱ
extern vu8         CD4094_Timer;                   //CD4094�����ʱ
extern vu8         PLL_CHK_Timer;                  //PLL��鶨ʱ
//���¶�ʱ��Timer_1ms ��ʱ ���Ƚϸ� ��ʱ�䳤��2.048s
extern u32         CYCLE_OLD_Timer;                //���ڲ�����ʱ��(��)
extern u32         CYCLE_NEW_Timer;                //���ڲ�����ʱ��(��)
extern u32         CLK_Timer_Max;                  //ʱ�Ӳ����������ֵ
extern u32         CLK_Timer_Min;                  //ʱ�Ӳ���������Сֵ
extern u16         SY_Timer;                       //ʧѹ��ʱ
extern u16         NY_SEND_Timer;                  //������Ͷ�ʱ
extern u16         CAN_STX_OVTimer;                //CAN �����ݷ��ͳ�ʱ��ʱ��
extern u16         CAN_LTX_OVTimer;                //CAN �����ݷ��ͳ�ʱ��ʱ��
extern u16         CAN_RX_OVTimer;                 //CAN ���߽��ճ�ʱ��ʱ��
extern u16         WORK_Timer;                     //���빤��ģʽ��ʱ�� ��λ:ms ���65.535S
extern u16         STD_ECLK_Timer;                 //��׼���������ⶨʱ
extern u16         MBJ_Send_Timer;                 //�������ݻ��Ͷ�ʱ
extern u16         WZTZ_Send_Timer;                //������բ���ݻ��Ͷ�ʱ
extern u16         NZTZ_Send_Timer;                //������բ���ݻ��Ͷ�ʱ
extern u16         XLJDQ_ERR_Timer;                //�����̵������ϱ�����ʱ
extern u16         NEW_PLL_Ref_Timer;              //���໷�ο���ʱ��
extern u16         OLD_PLL_Ref_Timer;              //���໷�ο���ʱ��
extern u16         SMtr_RCV_Timer;                 //��׼����ն�ʱ
extern u16         SMtr_TRS_Timer;                 //��׼���Ͷ�ʱ
extern u16         SMtr_RDATA_Timer;               //��׼������ݶ�ʱ
extern u16         SMtr_RSTS_Timer;                //��׼���״̬��ʱ
extern u16         SMtr_ChangeD_Timer;             //��׼��ı䶨ʱ
extern u16         SMtr_Check_Timer;               //��׼���ⶨʱ
extern u16         SMtr_TRS_OVTimer;               //��׼���ͳ�ʱ
//���¶�ʱ�� ��Timer_8ms ��ʱ
extern u8          Disp_Timer;                     //��ʾ��ʱ����
extern u8          GDT_RST_Timer;                  //���ͷ��λ��ʱ
extern u8          Com_Rx_Time[UART_NUMB];         //COM ���ն�ʱ
extern u8          Com_Rx_OvTm[UART_NUMB];         //UART���ճ�ʱ����
extern u8          UJDQ_Timer;                     //��ѹ�̵�����ʱ
extern u8          IJDQ_Timer;                     //�����̵�����ʱ
extern u8          ESwitch_Timer;                  //���ӿ��ض�ʱ
extern u8          NY_CHK_Timer;                   //��ѹ��ѯ��ʱ
extern u8          Con_KEY_Timer;                  //����������ʱ
extern u8          Disp_En_Timer;                  //��ʾʹ�ܶ�ʱ ��ֹ����д��SSI
extern u8          STD_CLK_Timer;                  //��׼ʱ�����嶨ʱ
extern u8          SMtr_Set_Timer;                 //��׼�����ö�ʱ

extern u8          Disp_Buf[8];                    //��ʾ������
extern UART_SET    Uart_Para[UART_NUMB] ;          //UART ����
//CANͨ�Ŷ���                                      
extern u16         CANERR_CNT;                     //CAN�������  
extern u8          CAN_LMSG_TX_TYPE;               //��ǰ����CAN����������
extern u8          CAN_LMSG_RX_TYPE;               //��ȡ����CAN����������
extern CAN_LMSG_PR CAN_LMSG_RX_Ptr;                //CAN�����ݽ��սṹ��ָ��
extern CAN_LMSG_PR CAN_LMSG_TX_Ptr;                //CAN�����ݷ��ͽṹ��ָ��
extern CAN_STS_U   CAN_STS;                        //����״̬
extern u8          CAN_LDATA_TX_STS;               //CAN�����ݷ���״̬ 0 �޶��� 'R':�Ѿ��������� 'S' ���ڷ������� 'E' ���ͽ���
extern u8          CAN_NEXT_MSG_IDx;               //CAN��һ֡���ĵ������� ���ݸú��ж��Ƿ�ʧ����
extern u8          CAN_LMSG_TX_STS;                //CAN��֡����״̬
extern u8          CAN_SMSG_TX_STS;                //CAN��֡����״̬
extern CAN_MSG     CAN_MSG_Rx;                     //������ʱ����CAN����
extern CAN_MSG     CAN_SMSG_Tx;                    //������ʱ����CAN�����ݱ���
extern CAN_MSG     CAN_LMSG_Tx;                    //������ʱ����CAN�����ݱ���
extern u8          Echo_Sts;                       //��Ӧ״̬  0 �޶��� 'C':�յ���ѯ 'S':�Ѿ����ͻ�Ӧ���� 'A':�Ѿ���Ӧ
extern u8          CAN_SEND_DTIME;                 //CAN֡������ʱʱ��
extern u8          CAN_SEND_DELAY;                 //CAN֡������ʱ ��ֹͬʱ��������

extern CAN_MSG     CAN_SDATA_MSG_IBUF[CAN_SDILEN]; //CAN�����ݽ���ָ�����
extern u8          CAN_SDATA_MSG_IHead;            //CAN�����ݽ���ָ�����ͷָ�� ����ָ��
extern u8          CAN_SDATA_MSG_ITail;            //CAN�����ݽ���ָ�����βָ�� ����ָ��
extern CAN_MSG     *CAN_MSG_IPtr;                  //CAN������֡���մ���ָ��
extern CAN_MSG     CAN_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN�����ݷ���ָ�����
extern u8          CAN_SDATA_MSG_OHead;            //CAN�����ݷ���ָ�����ͷָ�� ����ָ��
extern u8          CAN_SDATA_MSG_OTail;            //CAN�����ݷ���ָ�����βָ�� ����ָ��
extern CAN_MSG     *CAN_MSG_OPtr;                  //CAN������֡���ʹ���ָ��

extern CAN_MSG     CAN_LDATA_MSG_IBUF[CAN_LDILEN]; //CAN�����ݽ���ָ�����               
extern u8          CAN_LDATA_MSG_IHead;            //CAN�����ݽ���ָ�����ͷָ�� ����ָ��
extern u8          CAN_LDATA_MSG_ITail;            //CAN�����ݽ���ָ�����βָ�� ����ָ��
extern u8          CAN_LDATA_SERR_Cnt;             //CAN�����ݷ��ʹ������

//COM2 ��׼�� ֡                                             
extern u8          SMtr_IBuf[SMTR_ILEN];            //COM2���ջ�����
extern u16         SMtr_IHead;                      //COM2���ջ���������ָ��
extern u16         SMtr_ITail;                      //COM2���ջ��������մ���ָ��
extern u16         SMtr_RFrm_Ptr[SMTR_MAX_RFRAM];   //����ָ֡�� ��¼����֡��ֹλ��
extern u8          SMtr_RFrm_Cnt;                   //����֡��
extern u8          SMtr_RFrm_IHead;                 //֡����ָ��
extern u8          SMtr_RFrm_ITail;                 //֡���մ���ָ��
extern u16         SMtr_RFrm_CLen;                  //��ǰ֡����(ʣ�೤��)
extern u8          SMtr_Rx_Sts;                     //COM2����״̬     0:����״̬ ����:����״̬
extern u8          SMtr_Rx_OvTm;                    //COM2���ճ�ʱ����
extern u8          SMtr_RFrm_IS_Pr;                 //��׼�����֡���ڴ����־
extern u8          SMtr_OBuf[SMTR_OLEN];  					//COM2���ͻ�����
extern u16         SMtr_OHead;                      //COM2���ͻ���������ָ��
extern u16         SMtr_OTail;                      //COM2���ͻ��������ʹ���ָ��
extern u16         SMtr_TFrm_Ptr[SMTR_MAX_TFRAM];   //����ָ֡�� ��¼����֡��ֹλ�� 
extern u8          SMtr_TFrm_Attr[SMTR_MAX_TFRAM];  //֡���� ��������
extern u8          SMtr_InSend;                     //COM2���ݷ���״̬ 0:����״̬ ����:���ڷ���
extern u8          SMtr_TFrm_Cnt;                   //����֡��
extern u8          SMtr_TFrm_OHead;                 //֡����ָ��
extern u8          SMtr_TFrm_OTail;                 //֡���ʹ���ָ��
extern u8          SMtr_TFrm_Cmd_Cnt[SMTR_CMD_NUMB];//�ڱ�׼���ͻ�������������֡����
                                                    //Ŀ�� ֻ�������µ���������(�ɵ���������ڷ���)
                                                    //ÿ������е����ļ�����
                                                    //�������뻺�������ʱ����1
                                                    //�������������������1
                                                    //��������ʱ����������������Ϊ1 ˵�����µ����� ������ֱ�����
extern u8          SMtr_Set_Sts[SMTR_CMD_NUMB];     //��׼����������״̬ 
extern u8          SMtr_TFrm_Cnt;                   //����֡��
extern u16         SMtr_RFrm_CLen;                  //��ǰ֡����(ʣ�೤��)
extern u16         SMtr_IHead;                      //COM2���ջ���������ָ��
extern u16         SMtr_ITail;                      //COM2���ջ��������մ���ָ��
extern u16         SMtr_RFrm_Ptr[SMTR_MAX_RFRAM];   //����ָ֡�� ��¼����֡��ֹλ��
extern u16         SMtr_OHead;                      //COM2���ͻ���������ָ��
extern u16         SMtr_OTail;                      //COM2���ͻ��������ʹ���ָ��
extern u16         SMtr_TFrm_Ptr[SMTR_MAX_TFRAM];   //����ָ֡�� ��¼����֡��ֹλ�� 
                                                    //Ŀ�� ֻ�������µ���������(�ɵ���������ڷ���)
                                                    //ÿ������е����ļ�����
                                                    //�������뻺�������ʱ����1
                                                    //�������������������1
                                                    //��������ʱ����������������Ϊ1 ˵�����µ����� ������ֱ�����
extern u16         SMtr_TRS_IntVal;                 //��׼���Ͷ�ʱ����
extern u16         SMtr_RDATA_IntVal;               //��׼������ݶ�ʱ���� ��λ:ms
extern u16         SMtr_RSTS_IntVal;                //��׼������ݶ�ʱ���� ��λ:ms
extern u8          READ_WAVE_BIT;                   //Ҫ��ȡ�Ĳ��� BIT0:ua BIT1:ub BIT2:uc BIT3:ia BIT4:ib BIT5:ic
extern u8          READ_WAVE_ID;                    //��ȡ�������ݵ����             


extern IBUF_Pr     IBUF_Ptr;                        //���ջ���������ṹ��
extern OBUF_Pr     OBUF_Ptr;                        //���ͻ���������ṹ�� 
extern OBUF_Pr     UART_Ptr[4];                     //���ڷ��ͻ���������ṹ�� �������ж��е���                                                                   

extern u8          COM0_InSend;                     //COM0���ݷ���״̬ 'Y'���ڷ������� ���� ���ڷ�������
extern u8          COM0_IBuf[COM0_ILEN];            //COM0���ջ�����
extern u8          COM0_OBuf[COM0_OLEN];            //COM0���ͻ�����
extern u16         COM0_IHead;                      //COM0���ջ�����ͷָ�� ����ָ��
extern u16         COM0_ITail;                      //COM0���ջ�����βָ�� ����ָ��
extern u16         COM0_ICoun;                      //COM0�����������
extern u16         COM0_OHead;                      //COM0���ͻ�����ͷָ�����ָ��
extern u16         COM0_OTail;                      //COM0���ͻ�����βָ�봦��ָ��
extern u16         COM0_OCoun;                      //COM0�����������
extern u16         COM0_STimer;                     //COM0���Ͷ�ʱ

extern u8          COM1_InSend;                     //COM1���ݷ���״̬ 'Y'���ڷ������� ���� ���ڷ�������
extern u8          COM1_IBuf[COM1_ILEN];            //COM1���ջ�����
extern u8          COM1_OBuf[COM1_OLEN];            //COM1���ͻ�����
extern u16         COM1_IHead;                      //COM1���ջ�����ͷָ�� ����ָ��
extern u16         COM1_ITail;                      //COM1���ջ�����βָ�� ����ָ��
extern u16         COM1_ICoun;                      //COM1�����������
extern u16         COM1_OHead;                      //COM1���ͻ�����ͷָ�����ָ��
extern u16         COM1_OTail;                      //COM1���ͻ�����βָ�봦��ָ��
extern u16         COM1_OCoun;                      //COM1�����������
extern u16         COM1_STimer;                     //COM1���Ͷ�ʱ

extern u8          Rx_Com[RX_CMD_LEN];                //ȡ����ָ��ͷ//com0  com1 com2 ����
extern u8          Rx_Para[RX_PARA_NUM][RX_PARA_LEN]; //ȡ����ָ��ͷ ȡ�������� ���� ������������
extern u8          Para_Numb;                         //��������

extern u8          CANT_STR[8];                    //CAN֡�ַ�����ʱ������
extern u8          TEMP_STR[30];                   //CAN��ʱ���ַ���
extern u8          UART_TEMP_STR[UART_TEMP_STR_LEN];              //UART��ʱ���ַ���
extern u8          CHK_OneTime_Flag;                    //���յ�һ�β�ѯ�����־��
extern u8						CHK_ONETIME_FLAG1;
//λ��������
extern u32         SINGLE_OR_THREE ;               //����̨ ����̨��־ 0: ����̨ 1:����̨
extern u32         NEW_ENG_PLS     ;               //�յ��µ��������־
extern u32         NEW_CLK_PLS     ;               //�յ���ʱ�������־
extern u32         NEW_XUL_PLS     ;               //�յ������������־ 
extern u32         FIRST_ENG_PLS   ;               //�״β������������־
extern u32         FIRST_CLK_PLS   ;               //�״β���ʱ�������־
extern u32         FIRST_XUL_PLS   ;               //�״β������������־
extern u32         OVER_ERR_FLAG   ;               //�����־
extern u32         NEW_ENG_DATA    ;               //�������������
extern u32         BEEP_EN         ;               //������ʹ�ܱ�־
extern u32         MTR_PLUG        ;               //�ұ��־ 0:���ұ� 1:�ұ�
extern u32         GDT_RST_FLAG    ;               //���ͷ��λ��־
extern u32         NEW_CMD         ;               //�������־
extern u32         HB_BACK_EDGE    ;               //���Ӷ԰߱�־
extern u32         SY_START        ;               //ʧѹ������־
extern u32         ZZ_PLS_LOADED   ;               //�����Ƿ��Ѿ�Ԥ��
extern u32         HB_CATCHED      ;               //�ڰ߶�׼��־
extern u32         VERIFY_END      ;               //У�˳����������������־ 
extern u32         SY_ACT          ;               //ʧѹ�Ѷ�����־ 
extern u32         RISE_FALL_LVL   ;               //������/�½����ж�
extern u32         ENG_STB_CHK     ;               //���������ȶ�����־
extern u32         SCLK_STB_CHK    ;               //�ۺ�ʱ�������ȶ�����־
extern u32         OCLK_STB_CHK    ;               //����ʱ�������ȶ�����־
extern u32         XUL_STB_CHK     ;               //���������ȶ�����־
extern u32         NO_STD_CLK      ;               //��׼ʱ��������ڱ�־ 0:���� 1:������
extern u32         NO_CLK_PLS      ;               //û�м�⵽ʱ�������־
//extern u32         NO_XUL_PLS      ;               //û�м�⵽���������־
extern u32         NO_STD_ENG      ;               //û�м�⵽��׼���������־
extern u32         DISP_HL_LVL     ;               //��ʾ�������ڱ�־ 0: ��ʾ�͵�ƽʱ�� 1:��ʾ�ߵ�ƽʱ��
extern u32         PULSE_ZZ_END    ;               //�����������������־
extern u32         NEW_WZHZ_PLS    ;               //���ú�բ�����־
extern u32         NEW_NZHZ_PLS    ;               //���ú�բ�����־
extern u32         NEW_TQ_PLS      ;               //��ʱ��Ͷ�������־
extern u32         NEW_MBJ_PLS     ;               //���������־
extern u32         NEW_JBJ_PLS     ;               //�����̵���������־
extern u32         REF_JZ_INT      ;               //��׼�����жϱ�־

extern u32         TX_ZS_BIT       ;               //ͨ��ָʾ�ȱ�־
extern u32         SZCLK_SET_TooM_T;               //ʱ��Ƶ������̫С��־(��ʱ ��һ�μ�⵽����̫С)
extern u32         SZCLK_SET_TooM  ;               //ʱ��Ƶ������̫С��־
extern u32         NEW_KEY_FLAG    ;               //�°�����־
extern u32         PLUG_CHG_FLAG   ;               //�ұ�ı��־
extern u32         CD4094_FLAG     ;               //4094�ı������־                                                   
extern u32         KEY_PC_PLUG     ;               //����ѡ��ұ� ���� ��λ��ѡ��ұ�
extern u32         TZEN            ;               //��բ���ʹ������   
extern u32         MBJEN           ;               //�����źż��ʹ��
extern u32         GZ_FLAG         ;               //������·�̵������ϱ�־
extern u32         GZS_FLAG        ;               //��ʱ�ù��ϱ�־
extern u32         NZTZ_FLAG       ;               //������բ��־(��λ�����̵���) 
extern u32         WZTZ_FLAG       ;               //������բ��־(����բ����) 
extern u32         MASTER_START    ;               //�ܿ�����������־                                                   
extern u32         GDT_INT_REEN    ;               //���ͷ���ж�������ʱ��
extern u32         DZ_INT_REEN     ;               //����������ж�������ʱ��
extern u32         SZ_INT_REEN     ;               //ʱ������ܽ��ؿ��ж϶�ʱ
extern u32         XUL_INT_REEN    ;               //�������ж�������ʱ��
extern u32         TQ_INT_REEN     ;               //Ͷ�п��ж�������ʱ��
extern u32         HZ_INT_REEN     ;               //��բ���ж�������ʱ��
extern u32         KEY_INT_REEN    ;               //�������ж�������ʱ��
extern u32         CAN_ERR         ;               //CAN���ߴ���                                                   
                                                   
extern u32         I_JDQ           ;               //�����̵���״̬ 0: �Ͽ�(��������) 1: ����(������·)
extern u32         I_JDQ_CHG       ;               //�����̵���״̬�ı�
extern u32         I_JDQ_EN        ;               //�����̵���ʹ���ź�״̬
extern u32         UJDQ_FLAG       ;               //��ѹ�̵���״̬�����ı��־
extern u32         ESwitch_FLAG    ;               //���ӿ���״̬�����ı��־
extern u32         U_JDQ[3]        ;               //��ѹ�̵���״̬
extern u32         U_ESwitch[3]    ;               //��ѹ���ӿ���״̬


//�ڴ�0x20000004 ��Ԫ 4�ֽ� λ����
/****************************************************************************
* CAN������֡����
****************************************************************************/
extern const CAN_MSG CAN_TX_SMSG;

/********************************************************
* IO�ڻ�ַ����
********************************************************/
extern const u32 PORT_BASE_ADDR_TAB[];
/********************************************************
* ���ڲ�����������ܽű�� 
* GDT_PLS   '1'      //���ͷ���� GDT_MC
* DZ_PLS    '2'      //��������   DZ_MC
* SZ_PLS    '3'      //ʱ������   SZ_MC
* XUL_PLS   '4'      //��������   XL_MC
* TQ_PLS    '5'      //Ͷ������   TQ_MC
* HZ_PLS    '6'      //��բ����   HZ_MC
********************************************************/
extern const u8 CYCLE_PIN_TAB[];
/********************************************************
* �ܽ��ؿ���ʱ�� 
* ���ͷ�����ؿ���ʱ�� GDT_MC
* ���������ؿ���ʱ��   DZ_MC
* ʱ�������ؿ���ʱ��   SZ_MC
* ���������ؿ���ʱ��   XL_MC
* Ͷ�������ؿ���ʱ��   TQ_MC
* ��բ�����ؿ���ʱ��   HZ_MC
********************************************************/
extern const vu8_Ptr IO_Timer_Tab[];
/********************************************************
* �ܽ��ؿ��жϱ�־ 
* ���ͷ�����ؿ���ʱ�� GDT_MC
* ���������ؿ���ʱ��   DZ_MC
* ʱ�������ؿ���ʱ��   SZ_MC
* ���������ؿ���ʱ��   XL_MC
* Ͷ�������ؿ���ʱ��   TQ_MC
* ��բ�����ؿ���ʱ��   HZ_MC
********************************************************/
extern const u32_Ptr IO_REEN_TAB[];
/*****************************************************************************
* ����Ĭ�ϲ����б� MTRCOM LCTCOM
*****************************************************************************/
extern const UART_SET UART_PARA_TAB[UART_NUMB];
/****************************************************************************
* ��׼���������ͷ��ַ���
****************************************************************************/
extern const CChar_Ptr SMTR_CMD_ADDR_TAB[];
/****************************************************************************
* ��׼�����ƥ������������
****************************************************************************/
extern const u16 SMTR_CMD_NUM_TAB[];
/****************************************************************************
* ��׼��ÿ������ͷ���ȱ��
****************************************************************************/
extern const u8 SMTR_CMD_LEN_TAB[];
/****************************************************************************
* COM0����������
****************************************************************************/
extern const u8 COM0_ICMD_TAB[][8];
/****************************************************************************
* COM1����������
****************************************************************************/
extern const u8 COM1_ICMD_TAB[][8];

/****************************************************************************
* ���������趨�Ľ���ƥ������������
****************************************************************************/
extern const u16 CMD_NUM_TAB[];

extern const SAVE_S Save_Tab;

extern const u8 VER_TAB[];
