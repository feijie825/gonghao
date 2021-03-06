/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.h
;* Author             : 张力阵
;* 显示程序声明和预定义
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
extern u8        Disp_Buf[8];                    //显示缓冲区
/*****************************************************************************
* 设备时钟使能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralEnable(u32 ulPeripheral);

//PORTA
#define U0RX     GPIO_PIN_0	  //UART0接收
#define U0TX     GPIO_PIN_1	  //UART0发送
#define SSICLK   GPIO_PIN_2   //SSICLK HD7279 时钟
#define SSIFSS   GPIO_PIN_3   //SSIFSS HD7279 片选
#define DISP_RST GPIO_PIN_4   //SSIRX  HD7279 复位
#define SSITX    GPIO_PIN_5   //SSITX  HD7279 接收
#define JZ_IN    GPIO_PIN_6   //CCP1高稳晶振输入接口
#define FH_IN    GPIO_PIN_7	  //CCP4标准表高频输入
//PORTB
#define DZ_MC    GPIO_PIN_0   //CCP0电子脉冲
#define SZ_MC    GPIO_PIN_1   //CCP2时钟脉冲
#define GDT_MC   GPIO_PIN_2   //光电头脉冲
#define GDT_RST  GPIO_PIN_3   //光电头复位信号 
 #define J7_P6    GPIO_PIN_4   //备用 * 改变 A相电压电子开关控制 三相台定义  UA_ESWC
 #define J15_P6   GPIO_PIN_5   //备用 * 改变 B相电压电子开关控制 三相台定义  UB_ESWC
 #define J15_P5   GPIO_PIN_6   //备用 * 改变 C相电压电子开关控制 三相台定义  UC_ESWC
#define TRST     GPIO_PIN_7   //调试TRST

//PORTC
#define TCK      GPIO_PIN_0   //调试TCK SWCLK SW调试
#define TMS      GPIO_PIN_1   //调试TMS SWDIO SW调试
#define TDI      GPIO_PIN_2   //调试TDI
#define TDO      GPIO_PIN_3   //调试TDO SWO   SW调试
#define PWM_DAC	 GPIO_PIN_4   //CCP5 PWM模拟输出
#define KEY_IN   GPIO_PIN_5   //按键输入
 #define NY_IN    GPIO_PIN_6   // * 改变 NY 耐压结果输入
 #define BEEP    GPIO_PIN_7   //蜂鸣器控制 手工连线

//PORTD                       
#define CANRX    GPIO_PIN_0   //CAN接收
#define CANTX    GPIO_PIN_1   //CAN发送
#define U1RX     GPIO_PIN_2   //UART1接收
#define U1TX     GPIO_PIN_3   //UART1发送
#define GP_BK    GPIO_PIN_4   //CCP3 被检标准表高频输入 GP 
 #define J7_P5     GPIO_PIN_5   //* 备用 改变 A相电压继电器控制 三相台定义 UA_JC
 #define TEST_LAMP GPIO_PIN_6   //* 校验指示灯控制 第一表位用
 #define P3_OR_P4  GPIO_PIN_7   //* 功耗测试时 三相三线 三相四线控制

//PORTE                       
 #define UA_JC    GPIO_PIN_1   // A相电压继电器控制   三相台定义 UA_JC   U1K_C
 #define UB_JC    GPIO_PIN_0   // B相电压继电器控制   三相台定义 UB_JC   U2K_C
 #define UC_JC    GPIO_PIN_2   // C相电压继电器控制   三相台定义 UC_JC   U3K_C
 #define UA_ESWC  GPIO_PIN_3   // A相电压电子开关控制 三相台定义 UA_ESWC U4K_C

//单相台定义    
 #define ION_JC   GPIO_PIN_1   //电流继电器通 单相台用 UB电压继电器控制信号
 #define IOFF_JC  GPIO_PIN_2   //电流继电器断 单相台用 UC电压继电器控制信号

//PORTF                       
#define GOG_KZ   GPIO_PIN_0   //被检表脉冲共高共低选择控制
#define MC_PN_KZ GPIO_PIN_1   //改变被检表电子脉冲输入控制 控制正反向 Positive(0) or Negative(1)
#define MC_WV_KZ GPIO_PIN_2   //改变被检表电子脉冲输入控制 控制有无功 Watt(0) or Var(1)
#define XL_MC    GPIO_PIN_3   //     改变* 需量周期输入
#define TQ_MC    GPIO_PIN_4   //备用 改变* 时段投切脉冲
#define HZ_MC    GPIO_PIN_5   //备用 改变* 合闸脉冲
#define TX_MC    GPIO_PIN_6   //备用 改变* 通信指示
#define WDI_MC   GPIO_PIN_7   //备用 改变* 看门狗复位时钟

//PORTH   
#define J15_P4   GPIO_PIN_0   //备用 * 改变
#define J15_P3   GPIO_PIN_1   //备用 * 改变
#define UC_ESWC  GPIO_PIN_2   //备用 * 改变 C相电压电子开关控制 三相台定义  UC_ESWC
#define UB_ESWC  GPIO_PIN_3   //备用 * 改变 B相电压电子开关控制 三相台定义  UB_ESWC

#define U_IN_CTL GPIO_PIN_3   //电压接入控制 1 接入1号端子(默认) 0:接入3号端子 某些单相台使用 与B相电子开关控制复用
                 

//PORTG                       
#define BW       (GPIO_PIN_0|GPIO_PIN_1| \
                  GPIO_PIN_2|GPIO_PIN_3| \
                  GPIO_PIN_4|GPIO_PIN_5| \
                  GPIO_PIN_6|GPIO_PIN_7)

//I/O口宏定义
//PORTB
#ifdef PULSE
#define GDT_RST_EN     GPIOPinWrite(GPIOB,GDT_RST,GDT_RST)   //光电头复位使能 高电平使能
#define GDT_RST_DN     GPIOPinWrite(GPIOB,GDT_RST,0)         //光电头复位禁能 低电平禁能
#else
#define GDT_RST_EN     GPIOPinWrite(GPIOB,GDT_RST,0)         //光电头复位使能 高电平使能
#define GDT_RST_DN     GPIOPinWrite(GPIOB,GDT_RST,GDT_RST)   //光电头复位禁能 低电平禁能
#endif

//PORTC
#define BEEP_ON        GPIOPinWrite(GPIOC,BEEP_ON,BEEP_ON)   //蜂鸣器 响
#define BEEP_OFF       GPIOPinWrite(GPIOC,BEEP_ON,0)         //蜂鸣器 响

//PORTD 
#define WIRE_P3_CTL    GPIOPinWrite(GPIOD,P3_OR_P4,0)       //三相三线
#define WIRE_P4_CTL    GPIOPinWrite(GPIOD,P3_OR_P4,P3_OR_P4)//三相四线或单相
#define TEST_LAMP_ON   GPIOPinWrite(GPIOD,TEST_LAMP,0)         //校验指示灯亮
#define TEST_LAMP_OFF  GPIOPinWrite(GPIOD,TEST_LAMP,TEST_LAMP) //校验指示灯灭

//PORTE
#define UA_JDQ_ON      GPIOPinWrite(GPIOE,UA_JC,0)           //A相电压继电器接入
#define UA_JDQ_OFF     GPIOPinWrite(GPIOE,UA_JC,UA_JC)       //A相电压继电器断开
#define UB_JDQ_ON      GPIOPinWrite(GPIOE,UB_JC,0)           //B相电压继电器接入
#define UB_JDQ_OFF     GPIOPinWrite(GPIOE,UB_JC,UB_JC)       //B相电压继电器断开
#define UC_JDQ_ON      GPIOPinWrite(GPIOE,UC_JC,0)           //C相电压继电器接入
#define UC_JDQ_OFF     GPIOPinWrite(GPIOE,UC_JC,UC_JC)       //C相电压继电器断开
#define UA_ESW_ON      GPIOPinWrite(GPIOE,UA_ESWC,0)         //A相电压电子开关接入
#define UA_ESW_OFF     GPIOPinWrite(GPIOE,UA_ESWC,UA_ESWC)   //A相电压电子开关断开

//单相台控制
#define I_IN_EN        GPIOPinWrite(GPIOE,ION_JC|IOFF_JC,ION_JC)        //电流接入 单相台使用 低电平 继电器断开
#define I_BYPASS_EN    GPIOPinWrite(GPIOE,ION_JC|IOFF_JC,ION_JC|IOFF_JC)//电流旁路 单相台使用 高电平 继电器吸合 电流旁路

#define I_JDQ_EN_CNCL  GPIOPinWrite(GPIOE,ION_JC,0)          //继电器使能信号撤销
#define I_JDQ_CTL_CNCL GPIOPinWrite(GPIOE,IOFF_JC,0)         //继电器控制信号撤销   

//PORTF
#define POS_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,0)                //选择正向有功脉冲
#define POS_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ)         //选择正向无功脉冲
#define NEG_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_PN_KZ)         //选择反向有功脉冲
#define NEG_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ|MC_PN_KZ)//选择反向无功脉冲
#define DOWN_JOIN      GPIOPinWrite(GPIOF,GOG_KZ,GOG_KZ)       //共低端 发射极连在一起 E
#define UP_JOIN        GPIOPinWrite(GPIOF,GOG_KZ,0)            //共高端 集电极连在一起 C

#define TX_MC_ON       GPIOPinWrite(GPIOF,TX_MC,TX_MC)         //通信指示灯亮
#define TX_MC_OFF      GPIOPinWrite(GPIOF,TX_MC,0)             //通信指示灯亮

//PORTH                
#define UB_ESW_ON      GPIOPinWrite(GPIOH,UB_ESWC,0)         //B相电压电子开关接入
#define UB_ESW_OFF     GPIOPinWrite(GPIOH,UB_ESWC,UB_ESWC)   //B相电压电子开关断开
#define UC_ESW_ON      GPIOPinWrite(GPIOH,UC_ESWC,0)         //C相电压电子开关接入
#define UC_ESW_OFF     GPIOPinWrite(GPIOH,UC_ESWC,UC_ESWC)   //C相电压电子开关断开

#define U_PORT1_IN     GPIOPinWrite(GPIOH,U_IN_CTL,U_IN_CTL) //电压接入1号端子
#define U_PORT3_IN     GPIOPinWrite(GPIOH,U_IN_CTL,0)        //电压接入3号端子

//测试用
#define TST_PIN_ON     GPIOPinWrite(GPIOH,J15_P4,J15_P4)     //测试管脚高电平
#define TST_PIN_OFF    GPIOPinWrite(GPIOH,J15_P4,0)          //测试管脚低电平


#define  DISP_TIME              200             //显示定时 单位:ms 200ms
//显示选择
#define  DISP_TEST_DATA         0               //显示试验数据
#define  DISP_CLK_FREQ          1               //显示时钟频率
#define  DISP_DAY_ERR           2               //显示日计时误差
#define  DISP_XUL_TIME          3               //显示需量周期

#define  DISP_ENG_LEN           7               //电能误差数据长度
#define  DISP_ENG_OFFSET        2               //电能误差显示偏移量

#define  DISP_FREQ_LEN          8               //时钟频率数据长度
#define  DISP_FREQ_OFFSET       1               //时钟频率显示偏移量

#define  DISP_DAY_LEN           7               //日计时误差数据长度
#define  DISP_DAY_OFFSET        2               //日计时误差显示偏移量

#define  DISP_XUL_LEN           7               //需量周期数据长度
#define  DISP_XUL_OFFSET        2               //需量周期显示偏移量

#define  DISP_PWR_LEN           9               //电能误差数据长度
#define  DISP_PWR_OFFSET        0               //电能误差显示偏移量

#define  DISP_PLS_LEN           8               //脉冲显示长度
#define  DISP_PLS_OFFSET        0               //脉冲显示偏移量

#define  DISP_TIME_LEN          8               //脉冲周期数据长度
#define  DISP_TIME_OFFSET       0               //脉冲周期显示偏移量

//显示代码
#define  DISP_0                 0x00            //显示 0
#define  DISP_1                 0x01            //显示 1
#define  DISP_2                 0x02            //显示 2
#define  DISP_3                 0x03            //显示 3
#define  DISP_4                 0x04            //显示 4
#define  DISP_5                 0x05            //显示 5
#define  DISP_6                 0x06            //显示 6
#define  DISP_7                 0x07            //显示 7
#define  DISP_8                 0x08            //显示 8
#define  DISP_9                 0x09            //显示 9
#define  DISP_A                 0x0A            //显示 A
#define  DISP_B                 0x0B            //显示 B
#define  DISP_C                 0x0C            //显示 C
#define  DISP_D                 0x0D            //显示 D
#define  DISP_E                 0x0E            //显示 E
#define  DISP_F                 0x0F            //显示 F
#define  DISP_G                 0x10            //显示 G
#define  DISP_H                 0x11            //显示 H
#define  DISP_L                 0x12            //显示 L
#define  DISP_R                 0x13            //显示 R
#define  DISP_MINUS             0x14            //显示 '-' 负号
#define  DISP_BLANK             0x15            //清除显示缓冲区数据 空格
#define  DISP_b                 0x16            //显示 b
#define  DISP_d                 0x17            //显示 d
#define  DISP_U                 0x18            //显示 U
#define  DISP_t                 0x19            //显示 t
#define  DISP_n                 0x1A            //显示 n
#define  DISP_o                 0x1B            //显示 o
#define  DISP_P                 0x1C            //显示 P
#define  DISP_N                 0x1D            //显示 N

#define  DISP_FILL_0             0              //补0
#define  DISP_FILL_BLANK         1              //补空格
//HD7279命令
#define  LED_RESET              0xA4            //HD7279复位纯指令
#define  LED_TEST               0xBF            //HD7279测试纯指令
#define  LED_LEFT               0xA1            //左移指令
#define  LED_RIGHT              0xA0            //右移指令
#define  LED_LOOP_LEFT          0xA3            //循环左移指令
#define  LED_LOOP_RIGHT         0xA2            //循环右移指令
#define  LED_SEND_DATA_CODE0    0x80            //下载数据并按方式0译码 指令后三位代表LED位置 1字节数据 
                                                //数据位 定义 d7 DP LED小数点  d0~d3 显示数据
                                                // 0~9 对应 LED 0~9
                                                // 0x0A(-) 
                                                // 0x0B(E) 
                                                // 0x0C(H) 
                                                // 0x0D(L)
                                                // 0x0E(P)
                                                // 0x0F( ) 
#define  LED_SEND_DATA_CODE1    0xC8            //下载数据并按方式1译码 
                                                //数据位 定义 d7 DP LED小数点  d0~d3 显示数据
                                                // 0~9 对应 LED 0~9
                                                // 0x0A(A) 
                                                // 0x0B(b) 
                                                // 0x0C(C) 
                                                // 0x0D(d)
                                                // 0x0E(E)
                                                // 0x0F(F) 
#define  LED_SEND_DATA_CODE2    0x90            //下载数据不译码
                                                //数据位 定义 d7 DP 
                                                //数据位 定义 d6 SEGA 
                                                //数据位 定义 d5 SEGB 
                                                //数据位 定义 d4 SEGC 
                                                //数据位 定义 d3 SEGD 
                                                //数据位 定义 d2 SEGE 
                                                //数据位 定义 d1 SEGF 
                                                //数据位 定义 d0 SEGG 
#define  LED_BLINK              0x88            //闪烁控制
                                                //D7 数码管8 D0 数码管1
#define  LED_OFF                0x98            //消隐控制 关闭数码管
                                                //D7 数码管8 D0 数码管1  
#define  LED_SEG_ON             0xE0            //LED段点亮指令
                                                //D0~D5 段寻址 0~LED1-G 1~LED1-F ... 7~LED-. 0x3F~LED8-.
#define  LED_SEG_OFF            0xC0            //LED段关闭指令
                                                //D0~D5 段寻址 0~LED1-G 1~LED1-F ... 7~LED-. 0x3F~LED8-.
#define  READ_KEY               0x15            //读键盘命令


void Reset_HD7279(void);                 //7279复位 
void Disp_Data(u8 Disp_Mode);            //刷新显示缓冲区
void Disp_Boot(void);                    //显示引导状态
void Disp_Blank(void);                   //显示空白状态

