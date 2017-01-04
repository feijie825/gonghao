/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.h
;* Author             : ������
;* ��ʾ����������Ԥ����
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"

#define  DISP_TIME              (480/TIMER_8MS) //��ʾ��ʱ ��λ:ms 480ms
//��ʾѡ��
#define  DISP_TEST_DATA         '0'             //��ʾ��������
#define  DISP_CLK_FREQ          '1'             //��ʾʱ��Ƶ��
#define  DISP_DAY_ERR           '2'             //��ʾ�ռ�ʱ���
#define  DISP_XUL_TIME          '3'             //��ʾ��������
#define  DISP_TQMC              '4'             //��ʾͶ������
#define  DISP_HZMC              '5'             //��ʾ��բ����

#define  DISP_ENG_LEN           7               //����������ݳ���
#define  DISP_ENG_OFFSET        2               //���������ʾƫ����

#define  DISP_FREQ_LEN          8               //ʱ��Ƶ�����ݳ���
#define  DISP_FREQ_OFFSET       1               //ʱ��Ƶ����ʾƫ����

#define  DISP_DAY_LEN           7               //�ռ�ʱ������ݳ���
#define  DISP_DAY_OFFSET        2               //�ռ�ʱ�����ʾƫ����

#define  DISP_XUL_LEN           7               //�����������ݳ���
#define  DISP_XUL_OFFSET        2               //����������ʾƫ����

#define  DISP_PWR_LEN           9               //����������ݳ���
#define  DISP_PWR_OFFSET        0               //���������ʾƫ����

#define  DISP_PLS_LEN           8               //������ʾ����
#define  DISP_PLS_OFFSET        0               //������ʾƫ����

#define  DISP_TIME_LEN          8               //�����������ݳ���
#define  DISP_TIME_OFFSET       0               //����������ʾƫ����

//��ʾ����
#define  DISP_0                 0x00            //��ʾ 0
#define  DISP_1                 0x01            //��ʾ 1
#define  DISP_2                 0x02            //��ʾ 2
#define  DISP_3                 0x03            //��ʾ 3
#define  DISP_4                 0x04            //��ʾ 4
#define  DISP_5                 0x05            //��ʾ 5
#define  DISP_6                 0x06            //��ʾ 6
#define  DISP_7                 0x07            //��ʾ 7
#define  DISP_8                 0x08            //��ʾ 8
#define  DISP_9                 0x09            //��ʾ 9
#define  DISP_A                 0x0A            //��ʾ A
#define  DISP_B                 0x0B            //��ʾ B
#define  DISP_C                 0x0C            //��ʾ C
#define  DISP_D                 0x0D            //��ʾ D
#define  DISP_E                 0x0E            //��ʾ E
#define  DISP_F                 0x0F            //��ʾ F
#define  DISP_G                 0x10            //��ʾ G
#define  DISP_H                 0x11            //��ʾ H
#define  DISP_L                 0x12            //��ʾ L
#define  DISP_R                 0x13            //��ʾ R
#define  DISP_MINUS             0x14            //��ʾ '-' ����
#define  DISP_BLANK             0x15            //�����ʾ���������� �ո�
#define  DISP_b                 0x16            //��ʾ b
#define  DISP_d                 0x17            //��ʾ d
#define  DISP_U                 0x18            //��ʾ U
#define  DISP_t                 0x19            //��ʾ t
#define  DISP_n                 0x1A            //��ʾ n
#define  DISP_o                 0x1B            //��ʾ o
#define  DISP_P                 0x1C            //��ʾ P
#define  DISP_N                 0x1D            //��ʾ N
#define  DISP_c                 0x1E            //��ʾ c
#define  DISP_ALL               0x1F            //��ʾ 8.
                 
#define  DISP_FILL_0             0              //��0
#define  DISP_FILL_BLANK         1              //���ո�
//HD7279����
#define  LED_RESET              0xA4            //HD7279��λ��ָ��
#define  LED_TEST               0xBF            //HD7279���Դ�ָ��
#define  LED_LEFT               0xA1            //����ָ��
#define  LED_RIGHT              0xA0            //����ָ��
#define  LED_LOOP_LEFT          0xA3            //ѭ������ָ��
#define  LED_LOOP_RIGHT         0xA2            //ѭ������ָ��
#define  LED_SEND_DATA_CODE0    0x80            //�������ݲ�����ʽ0���� ָ�����λ����LEDλ�� 1�ֽ����� 
                                                //����λ ���� d7 DP LEDС����  d0~d3 ��ʾ����
                                                // 0~9 ��Ӧ LED 0~9
                                                // 0x0A(-) 
                                                // 0x0B(E) 
                                                // 0x0C(H) 
                                                // 0x0D(L)
                                                // 0x0E(P)
                                                // 0x0F( ) 
#define  LED_SEND_DATA_CODE1    0xC8            //�������ݲ�����ʽ1���� 
                                                //����λ ���� d7 DP LEDС����  d0~d3 ��ʾ����
                                                // 0~9 ��Ӧ LED 0~9
                                                // 0x0A(A) 
                                                // 0x0B(b) 
                                                // 0x0C(C) 
                                                // 0x0D(d)
                                                // 0x0E(E)
                                                // 0x0F(F) 
#define  LED_SEND_DATA_CODE2    0x90            //�������ݲ�����
                                                //����λ ���� d7 DP 
                                                //����λ ���� d6 SEGA 
                                                //����λ ���� d5 SEGB 
                                                //����λ ���� d4 SEGC 
                                                //����λ ���� d3 SEGD 
                                                //����λ ���� d2 SEGE 
                                                //����λ ���� d1 SEGF 
                                                //����λ ���� d0 SEGG 
#define  LED_BLINK              0x88            //��˸����
                                                //D7 �����8 D0 �����1
#define  LED_OFF                0x98            //�������� �ر������
                                                //D7 �����8 D0 �����1  
#define  LED_SEG_ON             0xE0            //LED�ε���ָ��
                                                //D0~D5 ��Ѱַ 0~LED1-G 1~LED1-F ... 7~LED-. 0x3F~LED8-.
#define  LED_SEG_OFF            0xC0            //LED�ιر�ָ��
                                                //D0~D5 ��Ѱַ 0~LED1-G 1~LED1-F ... 7~LED-. 0x3F~LED8-.
#define  READ_KEY               0x15            //����������


void Reset_HD7279(void);                 //7279��λ 
void Disp_Data(u8 Disp_Mode);            //ˢ����ʾ������
void Disp_Mtr_Num(void);                 //��ʾ��λ�� 
/*****************************************************************************
* ���µ�ǰУ��Ȧ����ʾ������
* ֻ��ʾ����λ ��ʾ����ʾ�������
*****************************************************************************/
void Update_N_Buf(void);
/*****************************************************************************
* ���±�λ����ʾ������
*****************************************************************************/
void Update_Mtr_Num(void);
/*****************************************************************************
* ��ʾ����������
* ���: Len  �����ʾ����
* �˿�: Addr ��ʾ��ʼ��ַ ��ʾ��λ(��������λ)
* �˿ڣ�Data Ҫ��ʾ������  
* �˿�: Sts  0:��λ���� 1:��λ����ʾ
*****************************************************************************/
void Disp_Long_Data(u8 Len,u8 Addr,u32 Data,u8 Sts);
/*****************************************************************************
* ��ʾ��ʱ����
*****************************************************************************/
void Disp_Time_Pr(void);             
/*****************************************************************************
* �����ַ������ݵ���ʾ������
* ���: Len    Ҫ�������ݵĳ���
* �˿�: Offset ��ʾ��������ʼ��ַ ��ʾ��λ(��������λ)
* �˿ڣ�*Ptr   Ҫ�������ݵ�ָ�� Ҫ��ʾ������  
*****************************************************************************/
void Copy_Str_To_DSBUF(u8 Len,u8 Offset,u8 *Ptr);
/*****************************************************************************
* ��ʾ����������
*****************************************************************************/
void Disp_ZZ_Pls(void);                    
                   
