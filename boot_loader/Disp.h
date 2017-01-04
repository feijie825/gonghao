/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.h
;* Author             : ������
;* ��ʾ����������Ԥ����
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
extern u8        Disp_Buf[8];                    //��ʾ������
/*****************************************************************************
* �豸ʱ��ʹ���ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralEnable(u32 ulPeripheral);

//PORTA
#define U0RX     GPIO_PIN_0	  //UART0����
#define U0TX     GPIO_PIN_1	  //UART0����
#define SSICLK   GPIO_PIN_2   //SSICLK HD7279 ʱ��
#define SSIFSS   GPIO_PIN_3   //SSIFSS HD7279 Ƭѡ
#define DISP_RST GPIO_PIN_4   //SSIRX  HD7279 ��λ
#define SSITX    GPIO_PIN_5   //SSITX  HD7279 ����
#define JZ_IN    GPIO_PIN_6   //CCP1���Ⱦ�������ӿ�
#define FH_IN    GPIO_PIN_7	  //CCP4��׼����Ƶ����
//PORTB
#define DZ_MC    GPIO_PIN_0   //CCP0��������
#define SZ_MC    GPIO_PIN_1   //CCP2ʱ������
#define GDT_MC   GPIO_PIN_2   //���ͷ����
#define GDT_RST  GPIO_PIN_3   //���ͷ��λ�ź� 
 #define J7_P6    GPIO_PIN_4   //���� * �ı� A���ѹ���ӿ��ؿ��� ����̨����  UA_ESWC
 #define J15_P6   GPIO_PIN_5   //���� * �ı� B���ѹ���ӿ��ؿ��� ����̨����  UB_ESWC
 #define J15_P5   GPIO_PIN_6   //���� * �ı� C���ѹ���ӿ��ؿ��� ����̨����  UC_ESWC
#define TRST     GPIO_PIN_7   //����TRST

//PORTC
#define TCK      GPIO_PIN_0   //����TCK SWCLK SW����
#define TMS      GPIO_PIN_1   //����TMS SWDIO SW����
#define TDI      GPIO_PIN_2   //����TDI
#define TDO      GPIO_PIN_3   //����TDO SWO   SW����
#define PWM_DAC	 GPIO_PIN_4   //CCP5 PWMģ�����
#define KEY_IN   GPIO_PIN_5   //��������
 #define NY_IN    GPIO_PIN_6   // * �ı� NY ��ѹ�������
 #define BEEP    GPIO_PIN_7   //���������� �ֹ�����

//PORTD                       
#define CANRX    GPIO_PIN_0   //CAN����
#define CANTX    GPIO_PIN_1   //CAN����
#define U1RX     GPIO_PIN_2   //UART1����
#define U1TX     GPIO_PIN_3   //UART1����
#define GP_BK    GPIO_PIN_4   //CCP3 �����׼����Ƶ���� GP 
 #define J7_P5     GPIO_PIN_5   //* ���� �ı� A���ѹ�̵������� ����̨���� UA_JC
 #define TEST_LAMP GPIO_PIN_6   //* У��ָʾ�ƿ��� ��һ��λ��
 #define P3_OR_P4  GPIO_PIN_7   //* ���Ĳ���ʱ �������� �������߿���

//PORTE                       
 #define UA_JC    GPIO_PIN_1   // A���ѹ�̵�������   ����̨���� UA_JC   U1K_C
 #define UB_JC    GPIO_PIN_0   // B���ѹ�̵�������   ����̨���� UB_JC   U2K_C
 #define UC_JC    GPIO_PIN_2   // C���ѹ�̵�������   ����̨���� UC_JC   U3K_C
 #define UA_ESWC  GPIO_PIN_3   // A���ѹ���ӿ��ؿ��� ����̨���� UA_ESWC U4K_C

//����̨����    
 #define ION_JC   GPIO_PIN_1   //�����̵���ͨ ����̨�� UB��ѹ�̵��������ź�
 #define IOFF_JC  GPIO_PIN_2   //�����̵����� ����̨�� UC��ѹ�̵��������ź�

//PORTF                       
#define GOG_KZ   GPIO_PIN_0   //��������干�߹���ѡ�����
#define MC_PN_KZ GPIO_PIN_1   //�ı䱻������������������ ���������� Positive(0) or Negative(1)
#define MC_WV_KZ GPIO_PIN_2   //�ı䱻������������������ �������޹� Watt(0) or Var(1)
#define XL_MC    GPIO_PIN_3   //     �ı�* ������������
#define TQ_MC    GPIO_PIN_4   //���� �ı�* ʱ��Ͷ������
#define HZ_MC    GPIO_PIN_5   //���� �ı�* ��բ����
#define TX_MC    GPIO_PIN_6   //���� �ı�* ͨ��ָʾ
#define WDI_MC   GPIO_PIN_7   //���� �ı�* ���Ź���λʱ��

//PORTH   
#define J15_P4   GPIO_PIN_0   //���� * �ı�
#define J15_P3   GPIO_PIN_1   //���� * �ı�
#define UC_ESWC  GPIO_PIN_2   //���� * �ı� C���ѹ���ӿ��ؿ��� ����̨����  UC_ESWC
#define UB_ESWC  GPIO_PIN_3   //���� * �ı� B���ѹ���ӿ��ؿ��� ����̨����  UB_ESWC

#define U_IN_CTL GPIO_PIN_3   //��ѹ������� 1 ����1�Ŷ���(Ĭ��) 0:����3�Ŷ��� ĳЩ����̨ʹ�� ��B����ӿ��ؿ��Ƹ���
                 

//PORTG                       
#define BW       (GPIO_PIN_0|GPIO_PIN_1| \
                  GPIO_PIN_2|GPIO_PIN_3| \
                  GPIO_PIN_4|GPIO_PIN_5| \
                  GPIO_PIN_6|GPIO_PIN_7)

//I/O�ں궨��
//PORTB
#ifdef PULSE
#define GDT_RST_EN     GPIOPinWrite(GPIOB,GDT_RST,GDT_RST)   //���ͷ��λʹ�� �ߵ�ƽʹ��
#define GDT_RST_DN     GPIOPinWrite(GPIOB,GDT_RST,0)         //���ͷ��λ���� �͵�ƽ����
#else
#define GDT_RST_EN     GPIOPinWrite(GPIOB,GDT_RST,0)         //���ͷ��λʹ�� �ߵ�ƽʹ��
#define GDT_RST_DN     GPIOPinWrite(GPIOB,GDT_RST,GDT_RST)   //���ͷ��λ���� �͵�ƽ����
#endif

//PORTC
#define BEEP_ON        GPIOPinWrite(GPIOC,BEEP_ON,BEEP_ON)   //������ ��
#define BEEP_OFF       GPIOPinWrite(GPIOC,BEEP_ON,0)         //������ ��

//PORTD 
#define WIRE_P3_CTL    GPIOPinWrite(GPIOD,P3_OR_P4,0)       //��������
#define WIRE_P4_CTL    GPIOPinWrite(GPIOD,P3_OR_P4,P3_OR_P4)//�������߻���
#define TEST_LAMP_ON   GPIOPinWrite(GPIOD,TEST_LAMP,0)         //У��ָʾ����
#define TEST_LAMP_OFF  GPIOPinWrite(GPIOD,TEST_LAMP,TEST_LAMP) //У��ָʾ����

//PORTE
#define UA_JDQ_ON      GPIOPinWrite(GPIOE,UA_JC,0)           //A���ѹ�̵�������
#define UA_JDQ_OFF     GPIOPinWrite(GPIOE,UA_JC,UA_JC)       //A���ѹ�̵����Ͽ�
#define UB_JDQ_ON      GPIOPinWrite(GPIOE,UB_JC,0)           //B���ѹ�̵�������
#define UB_JDQ_OFF     GPIOPinWrite(GPIOE,UB_JC,UB_JC)       //B���ѹ�̵����Ͽ�
#define UC_JDQ_ON      GPIOPinWrite(GPIOE,UC_JC,0)           //C���ѹ�̵�������
#define UC_JDQ_OFF     GPIOPinWrite(GPIOE,UC_JC,UC_JC)       //C���ѹ�̵����Ͽ�
#define UA_ESW_ON      GPIOPinWrite(GPIOE,UA_ESWC,0)         //A���ѹ���ӿ��ؽ���
#define UA_ESW_OFF     GPIOPinWrite(GPIOE,UA_ESWC,UA_ESWC)   //A���ѹ���ӿ��ضϿ�

//����̨����
#define I_IN_EN        GPIOPinWrite(GPIOE,ION_JC|IOFF_JC,ION_JC)        //�������� ����̨ʹ�� �͵�ƽ �̵����Ͽ�
#define I_BYPASS_EN    GPIOPinWrite(GPIOE,ION_JC|IOFF_JC,ION_JC|IOFF_JC)//������· ����̨ʹ�� �ߵ�ƽ �̵������� ������·

#define I_JDQ_EN_CNCL  GPIOPinWrite(GPIOE,ION_JC,0)          //�̵���ʹ���źų���
#define I_JDQ_CTL_CNCL GPIOPinWrite(GPIOE,IOFF_JC,0)         //�̵��������źų���   

//PORTF
#define POS_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,0)                //ѡ�������й�����
#define POS_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ)         //ѡ�������޹�����
#define NEG_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_PN_KZ)         //ѡ�����й�����
#define NEG_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ|MC_PN_KZ)//ѡ�����޹�����
#define DOWN_JOIN      GPIOPinWrite(GPIOF,GOG_KZ,GOG_KZ)       //���Ͷ� ���伫����һ�� E
#define UP_JOIN        GPIOPinWrite(GPIOF,GOG_KZ,0)            //���߶� ���缫����һ�� C

#define TX_MC_ON       GPIOPinWrite(GPIOF,TX_MC,TX_MC)         //ͨ��ָʾ����
#define TX_MC_OFF      GPIOPinWrite(GPIOF,TX_MC,0)             //ͨ��ָʾ����

//PORTH                
#define UB_ESW_ON      GPIOPinWrite(GPIOH,UB_ESWC,0)         //B���ѹ���ӿ��ؽ���
#define UB_ESW_OFF     GPIOPinWrite(GPIOH,UB_ESWC,UB_ESWC)   //B���ѹ���ӿ��ضϿ�
#define UC_ESW_ON      GPIOPinWrite(GPIOH,UC_ESWC,0)         //C���ѹ���ӿ��ؽ���
#define UC_ESW_OFF     GPIOPinWrite(GPIOH,UC_ESWC,UC_ESWC)   //C���ѹ���ӿ��ضϿ�

#define U_PORT1_IN     GPIOPinWrite(GPIOH,U_IN_CTL,U_IN_CTL) //��ѹ����1�Ŷ���
#define U_PORT3_IN     GPIOPinWrite(GPIOH,U_IN_CTL,0)        //��ѹ����3�Ŷ���

//������
#define TST_PIN_ON     GPIOPinWrite(GPIOH,J15_P4,J15_P4)     //���ԹܽŸߵ�ƽ
#define TST_PIN_OFF    GPIOPinWrite(GPIOH,J15_P4,0)          //���Թܽŵ͵�ƽ


#define  DISP_TIME              200             //��ʾ��ʱ ��λ:ms 200ms
//��ʾѡ��
#define  DISP_TEST_DATA         0               //��ʾ��������
#define  DISP_CLK_FREQ          1               //��ʾʱ��Ƶ��
#define  DISP_DAY_ERR           2               //��ʾ�ռ�ʱ���
#define  DISP_XUL_TIME          3               //��ʾ��������

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
void Disp_Boot(void);                    //��ʾ����״̬
void Disp_Blank(void);                   //��ʾ�հ�״̬
