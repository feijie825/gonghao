/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S2139.h
;* Author             : 张力阵														  
;* LM3S2139 寄存器定义
;* 优先级分组 分为8组 0:8 1:7 2:6 3:5 4:4 5:3 6:2 7:1 
;* :前面值表示抢占优先级位 参与中断仲裁
;* :后面值表示子优先级位   不参与中断仲裁

*******************************************************************************/

#ifndef __LM3S2139_H__
#define __LM3S2139_H__
#include "LM3S21xx_types.h"
/*****************************************************************************
*
* The following are defines for the fault assignments.
* 系统中断ID
*****************************************************************************/
#define FAULT_NMI               2           // NMI fault     非屏蔽异常
#define FAULT_HARD              3           // Hard fault	 硬件异常
#define FAULT_MPU               4           // MPU fault	 存储器管理异常
#define FAULT_BUS               5           // Bus fault	 总线故障
#define FAULT_USAGE             6           // Usage fault	 使用故障
#define FAULT_SVCALL            11          // SVCall		 使用SVC指令的系统服务调用
#define FAULT_DEBUG             12          // Debug monitor 调试监控器
#define FAULT_PENDSV            14          // PendSV		 系统服务的可挂起请求
#define FAULT_SYSTICK           15          // System Tick	 系统节拍定时器中断

/*****************************************************************************
* 设备中断ID
*****************************************************************************/
#define INT_GPIOA               16          // GPIO Port A
#define INT_GPIOB               17          // GPIO Port B
#define INT_GPIOC               18          // GPIO Port C
#define INT_GPIOD               19          // GPIO Port D
#define INT_GPIOE               20          // GPIO Port E
#define INT_UART0               21          // UART0 Rx and Tx
#define INT_UART1               22          // UART1 Rx and Tx
#define INT_SSI0                23          // SSI0 Rx and Tx
#define INT_I2C0                24          // I2C0 Master and Slave
#define INT_PWM_FAULT           25          // PWM Fault
#define INT_PWM0                26          // PWM Generator 0
#define INT_PWM1                27          // PWM Generator 1
#define INT_PWM2                28          // PWM Generator 2
#define INT_QEI0                29          // Quadrature Encoder 0
#define INT_ADC0                30          // ADC Sequence 0
#define INT_ADC1                31          // ADC Sequence 1
#define INT_ADC2                32          // ADC Sequence 2
#define INT_ADC3                33          // ADC Sequence 3
#define INT_WATCHDOG            34          // Watchdog timer
#define INT_TIMER0A             35          // Timer 0 subtimer A
#define INT_TIMER0B             36          // Timer 0 subtimer B
#define INT_TIMER1A             37          // Timer 1 subtimer A
#define INT_TIMER1B             38          // Timer 1 subtimer B
#define INT_TIMER2A             39          // Timer 2 subtimer A
#define INT_TIMER2B             40          // Timer 2 subtimer B
#define INT_COMP0               41          // Analog Comparator 0
#define INT_COMP1               42          // Analog Comparator 1
#define INT_COMP2               43          // Analog Comparator 2
#define INT_SYSCTL              44          // System Control (PLL, OSC, BO)
#define INT_FLASH               45          // FLASH Control
#define INT_GPIOF               46          // GPIO Port F
#define INT_GPIOG               47          // GPIO Port G
#define INT_GPIOH               48          // GPIO Port H
#define INT_UART2               49          // UART2 Rx and Tx
#define INT_SSI1                50          // SSI1 Rx and Tx
#define INT_TIMER3A             51          // Timer 3 subtimer A
#define INT_TIMER3B             52          // Timer 3 subtimer B
#define INT_I2C1                53          // I2C1 Master and Slave
#define INT_QEI1                54          // Quadrature Encoder 1
#define INT_CAN0                55          // CAN0
#define INT_CAN1                56          // CAN1
#define INT_CAN2                57          // CAN2
#define INT_ETH                 58          // Ethernet
#define INT_HIBERNATE           59          // Hibernation module
#define INT_USB0                60          // USB 0 Controller
#define INT_PWM3                61          // PWM Generator 3
#define INT_UDMA                62          // uDMA controller
#define INT_UDMAERR             63          // uDMA Error

//*****************************************************************************
//
// The following are defines for the total number of interrupts.
//
//*****************************************************************************
#define NUM_INTERRUPTS          64

//*****************************************************************************
//
// The following are defines for the total number of priority levels.
//
//*****************************************************************************
#define NUM_PRIORITY            8
#define NUM_PRIORITY_BITS       3

//*****************************************************************************
//
// The following are deprecated defines for the interrupt assignments.
//
//*****************************************************************************
#define INT_SSI                 23          // SSI Rx and Tx
#define INT_I2C                 24          // I2C Master and Slave
#define INT_QEI                 29          // Quadrature Encoder

//*****************************************************************************
//
// The following are defines for the base address of the memories and
// peripherals.
//
//*****************************************************************************
#define FLASH_BASE              0x00000000  // FLASH memory
#define SAVE_BASE               0x0000FC00  // 保存地址
#define SRAM_BASE               0x20000000  // SRAM memory
#define SRAM_BIT_BASE           0x22000000  // SRAM 对于RAM区
#define WATCHDOG_BASE           0x40000000  // Watchdog
#define GPIO_PORTA_BASE         0x40004000  // GPIO Port A
#define GPIO_PORTB_BASE         0x40005000  // GPIO Port B
#define GPIO_PORTC_BASE         0x40006000  // GPIO Port C
#define GPIO_PORTD_BASE         0x40007000  // GPIO Port D
#define SSI0_BASE               0x40008000  // SSI0
#define SSI1_BASE               0x40009000  // SSI1
#define UART0_BASE              0x4000C000  // UART0
#define UART1_BASE              0x4000D000  // UART1
#define UART2_BASE              0x4000E000  // UART2
#define I2C0_MASTER_BASE        0x40020000  // I2C0 Master
#define I2C0_SLAVE_BASE         0x40020800  // I2C0 Slave
#define I2C1_MASTER_BASE        0x40021000  // I2C1 Master
#define I2C1_SLAVE_BASE         0x40021800  // I2C1 Slave
#define GPIO_PORTE_BASE         0x40024000  // GPIO Port E
#define GPIO_PORTF_BASE         0x40025000  // GPIO Port F
#define GPIO_PORTG_BASE         0x40026000  // GPIO Port G
#define GPIO_PORTH_BASE         0x40027000  // GPIO Port H
#define PWM_BASE                0x40028000  // PWM
#define QEI0_BASE               0x4002C000  // QEI0
#define QEI1_BASE               0x4002D000  // QEI1
#define TIMER0_BASE             0x40030000  // Timer0
#define TIMER1_BASE             0x40031000  // Timer1
#define TIMER2_BASE             0x40032000  // Timer2
#define TIMER3_BASE             0x40033000  // Timer3
#define ADC_BASE                0x40038000  // ADC
#define COMP_BASE               0x4003C000  // Analog comparators
#define CAN0_BASE               0x40040000  // CAN0
#define CAN1_BASE               0x40041000  // CAN1			 F
#define CAN2_BASE               0x40042000  // CAN2
#define ETH_BASE                0x40048000  // Ethernet
#define MAC_BASE                0x40048000  // Ethernet
#define USB0_BASE               0x40050000  // USB 0 Controller
#define GPIO_PORTA_AHB_BASE     0x40058000  // GPIO Port A (high speed)
#define GPIO_PORTB_AHB_BASE     0x40059000  // GPIO Port B (high speed)
#define GPIO_PORTC_AHB_BASE     0x4005A000  // GPIO Port C (high speed)
#define GPIO_PORTD_AHB_BASE     0x4005B000  // GPIO Port D (high speed)
#define GPIO_PORTE_AHB_BASE     0x4005C000  // GPIO Port E (high speed)
#define GPIO_PORTF_AHB_BASE     0x4005D000  // GPIO Port F (high speed)
#define GPIO_PORTG_AHB_BASE     0x4005E000  // GPIO Port G (high speed)
#define GPIO_PORTH_AHB_BASE     0x4005F000  // GPIO Port H (high speed)
#define HIB_BASE                0x400FC000  // Hibernation Module
#define FLASH_CTRL_BASE         0x400FD000  // FLASH Controller
#define SYSCTL_BASE             0x400FE000  // System Control
#define UDMA_BASE               0x400FF000  // uDMA Controller
#define ITM_BASE                0xE0000000  // Instrumentation Trace Macrocell
#define DWT_BASE                0xE0001000  // Data Watchpoint and Trace
#define FPB_BASE                0xE0002000  // FLASH Patch and Breakpoint
#define NVIC_BASE               0xE000E000  // Nested Vectored Interrupt Ctrl
#define TPIU_BASE               0xE0040000  // Trace Port Interface Unit
//*****************************************************************************
//
// The following are deprecated defines for the base address of the memories
// and peripherals.
//
//*****************************************************************************
#define SSI_BASE                0x40008000  // SSI
#define I2C_MASTER_BASE         0x40020000  // I2C Master
#define I2C_SLAVE_BASE          0x40020800  // I2C Slave
#define QEI_BASE                0x4002C000  // QEI
//*****************************************************************************
// 地址通过验证
// Watchdog Timer (WATCHDOG)
// 看门狗寄存器结构体
//*****************************************************************************
typedef struct
{
	vu32 LOAD;                //看门狗重装寄存器
	vu32 VALUE;               //看门狗计数当前值寄存器
	vu32 CTL;                 //看门狗控制寄存器
	vu32 ICR;                 //看门狗中断清除寄存器
	vu32 RIS;                 //看门狗原始中断状态寄存器
	vu32 MIS;                 //看门狗屏蔽后中断状态寄存器
	u32  EMPTY1[256];         //地址填充 
	vu32 TEST;                //看门狗测试寄存器
	u32  EMPTY[505];          //地址填充
	vu32 LOCK;                //看门狗锁定寄存器
}WDG_Typedef;
#define WDG                   ((WDG_Typedef *)WDG_BASE) 
#define WDG_BASE              0x40000000
#define WATCHDOG_LOAD         (*((vu32 *)0x40000000))
#define WATCHDOG_VALUE        (*((vu32 *)0x40000004))
#define WATCHDOG_CTL          (*((vu32 *)0x40000008))
#define WATCHDOG_ICR          (*((vu32 *)0x4000000C))
#define WATCHDOG_RIS          (*((vu32 *)0x40000010))
#define WATCHDOG_MIS          (*((vu32 *)0x40000014))
#define WATCHDOG_TEST         (*((vu32 *)0x40000418))
#define WATCHDOG_LOCK         (*((vu32 *)0x40000C00))
//*****************************************************************************
//
// The following are defines for the Watchdog Timer register offsets.
// 看门狗寄存器偏移地址
//*****************************************************************************
#define WDT_O_LOAD              0x00000000  // Load register
#define WDT_O_VALUE             0x00000004  // Current value register
#define WDT_O_CTL               0x00000008  // Control register
#define WDT_O_ICR               0x0000000C  // Interrupt clear register
#define WDT_O_RIS               0x00000010  // Raw interrupt status register
#define WDT_O_MIS               0x00000014  // Masked interrupt status register
#define WDT_O_TEST              0x00000418  // Test register
#define WDT_O_LOCK              0x00000C00  // Lock register
//*****************************************************************************
//
// The following are deprecated defines for the Watchdog Timer register
// offsets.
//
//*****************************************************************************
#define WDT_O_PeriphID4         0x00000FD0
#define WDT_O_PeriphID5         0x00000FD4
#define WDT_O_PeriphID6         0x00000FD8
#define WDT_O_PeriphID7         0x00000FDC
#define WDT_O_PeriphID0         0x00000FE0
#define WDT_O_PeriphID1         0x00000FE4
#define WDT_O_PeriphID2         0x00000FE8
#define WDT_O_PeriphID3         0x00000FEC
#define WDT_O_PCellID0          0x00000FF0
#define WDT_O_PCellID1          0x00000FF4
#define WDT_O_PCellID2          0x00000FF8
#define WDT_O_PCellID3          0x00000FFC

//****************************************************************************
//
//General-Purpose Input/Outputs (PORT)
//地址通过验证
//****************************************************************************
typedef struct
{
	vu32 DATA[256];             //I/O口数据
	vu32 DIR;                   //I/O口方向
	vu32 IS;                    //I/O口中断方式 0:沿中断 1:电平中断
	vu32 IBE;                   //中断双边沿 0:中断由IEV寄存器控制 1:I/O口双边沿中断
	vu32 IEV;                   //中断事件寄存器 0:下降沿或低电平中断 1:上升沿或高电平中断
	vu32 IM;                    //中断屏蔽寄存器 0:I/O相应管脚中断禁能 1:I/O相应管脚中断使能
	vu32 RIS;                   //原始中断状态寄存器 保存原始中断状态
	vu32 MIS;                   //屏蔽后的中断状态寄存器 
	vu32 ICR;                   //中断清除寄存器 Interrutp clear 写1清除相应中断
	vu32 AFSEL;                 //备用功能选择寄存器
	vu32 EMPTY2[55];            //地址填充
	vu32 DR2R;                  //2mA驱动寄存器 写入1后清除DD4R和DD8R相应位
	vu32 DR4R;                  //4mA驱动寄存器 写入1后清除DD2R和DD8R相应位
	vu32 DR8R;                  //8mA驱动寄存器 写入1后清除DD2R和DD4R相应位
	vu32 ODR;                   //开漏选择寄存器
	vu32 PUR;                   //上拉选择寄存器
	vu32 PDR;                   //下拉选择寄存器
	vu32 SLR;                   //斜率控制寄存器 只对8mA端口有效
	vu32 DEN;                   //数字功能使能寄存器
	vu32 LOCK;                  //锁定寄存器 
	vu32 CR;                    //确认寄存器
	vu32 AMSEL;                 //GPIO 模拟模式选择寄存器 对LM3S2139无效
}GPIO_Typedef;
#define GPIOA                 ((GPIO_Typedef*)GPIO_PORTA_BASE)
#define GPIOB                 ((GPIO_Typedef*)GPIO_PORTB_BASE)
#define GPIOC                 ((GPIO_Typedef*)GPIO_PORTC_BASE)
#define GPIOD                 ((GPIO_Typedef*)GPIO_PORTD_BASE)
#define GPIOE                 ((GPIO_Typedef*)GPIO_PORTE_BASE)
#define GPIOF                 ((GPIO_Typedef*)GPIO_PORTF_BASE)
#define GPIOG                 ((GPIO_Typedef*)GPIO_PORTG_BASE)
#define GPIOH                 ((GPIO_Typedef*)GPIO_PORTH_BASE)

#define GPIO_PORTA_DATA_BITS  ((vu32 *)0x40004000)
#define GPIO_PORTA_DATA       (*((vu32 *)0x400043FC))
#define GPIO_PORTA_DIR        (*((vu32 *)0x40004400))
#define GPIO_PORTA_IS         (*((vu32 *)0x40004404))
#define GPIO_PORTA_IBE        (*((vu32 *)0x40004408))
#define GPIO_PORTA_IEV        (*((vu32 *)0x4000440C))
#define GPIO_PORTA_IM         (*((vu32 *)0x40004410))
#define GPIO_PORTA_RIS        (*((vu32 *)0x40004414))
#define GPIO_PORTA_MIS        (*((vu32 *)0x40004418))
#define GPIO_PORTA_ICR        (*((vu32 *)0x4000441C))
#define GPIO_PORTA_AFSEL      (*((vu32 *)0x40004420))
#define GPIO_PORTA_DR2R       (*((vu32 *)0x40004500))
#define GPIO_PORTA_DR4R       (*((vu32 *)0x40004504))
#define GPIO_PORTA_DR8R       (*((vu32 *)0x40004508))
#define GPIO_PORTA_ODR        (*((vu32 *)0x4000450C))
#define GPIO_PORTA_PUR        (*((vu32 *)0x40004510))
#define GPIO_PORTA_PDR        (*((vu32 *)0x40004514))
#define GPIO_PORTA_SLR        (*((vu32 *)0x40004518))
#define GPIO_PORTA_DEN        (*((vu32 *)0x4000451C))
#define GPIO_PORTA_LOCK       (*((vu32 *)0x40004520))
#define GPIO_PORTA_CR         (*((vu32 *)0x40004524))
#define GPIO_PORTA_AMSEL      (*((vu32 *)0x40004528))

//*****************************************************************************
//
// The following are defines for the GPIO Register offsets.
//
//*****************************************************************************
#define GPIO_O_DATA             0x00000000  // Data register.
#define GPIO_O_DIR              0x00000400  // Data direction register.
#define GPIO_O_IS               0x00000404  // Interrupt sense register.
#define GPIO_O_IBE              0x00000408  // Interrupt both edges register.
#define GPIO_O_IEV              0x0000040C  // Interrupt event register.
#define GPIO_O_IM               0x00000410  // Interrupt mask register.
#define GPIO_O_RIS              0x00000414  // Raw interrupt status register.
#define GPIO_O_MIS              0x00000418  // Masked interrupt status reg.
#define GPIO_O_ICR              0x0000041C  // Interrupt clear register.
#define GPIO_O_AFSEL            0x00000420  // Mode control select register.
#define GPIO_O_DR2R             0x00000500  // 2ma drive select register.
#define GPIO_O_DR4R             0x00000504  // 4ma drive select register.
#define GPIO_O_DR8R             0x00000508  // 8ma drive select register.
#define GPIO_O_ODR              0x0000050C  // Open drain select register.
#define GPIO_O_PUR              0x00000510  // Pull up select register.
#define GPIO_O_PDR              0x00000514  // Pull down select register.
#define GPIO_O_SLR              0x00000518  // Slew rate control enable reg.
#define GPIO_O_DEN              0x0000051C  // Digital input enable register.
#define GPIO_O_LOCK             0x00000520  // Lock register.
#define GPIO_O_CR               0x00000524  // Commit register.
#define GPIO_O_AMSEL            0x00000528  // GPIO Analog Mode Select

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTB)
//
//****************************************************************************
#define GPIO_PORTB_DATA_BITS  ((vu32 *)0x40005000)
#define GPIO_PORTB_DATA       (*((vu32 *)0x400053FC))
#define GPIO_PORTB_DIR        (*((vu32 *)0x40005400))
#define GPIO_PORTB_IS         (*((vu32 *)0x40005404))
#define GPIO_PORTB_IBE        (*((vu32 *)0x40005408))
#define GPIO_PORTB_IEV        (*((vu32 *)0x4000540C))
#define GPIO_PORTB_IM         (*((vu32 *)0x40005410))
#define GPIO_PORTB_RIS        (*((vu32 *)0x40005414))
#define GPIO_PORTB_MIS        (*((vu32 *)0x40005418))
#define GPIO_PORTB_ICR        (*((vu32 *)0x4000541C))
#define GPIO_PORTB_AFSEL      (*((vu32 *)0x40005420))
#define GPIO_PORTB_DR2R       (*((vu32 *)0x40005500))
#define GPIO_PORTB_DR4R       (*((vu32 *)0x40005504))
#define GPIO_PORTB_DR8R       (*((vu32 *)0x40005508))
#define GPIO_PORTB_ODR        (*((vu32 *)0x4000550C))
#define GPIO_PORTB_PUR        (*((vu32 *)0x40005510))
#define GPIO_PORTB_PDR        (*((vu32 *)0x40005514))
#define GPIO_PORTB_SLR        (*((vu32 *)0x40005518))
#define GPIO_PORTB_DEN        (*((vu32 *)0x4000551C))
#define GPIO_PORTB_LOCK       (*((vu32 *)0x40005520))
#define GPIO_PORTB_CR         (*((vu32 *)0x40005524))

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTC)
//
//****************************************************************************
#define GPIO_PORTC_DATA_BITS  ((vu32 *)0x40006000)
#define GPIO_PORTC_DATA       (*((vu32 *)0x400063FC))
#define GPIO_PORTC_DIR        (*((vu32 *)0x40006400))
#define GPIO_PORTC_IS         (*((vu32 *)0x40006404))
#define GPIO_PORTC_IBE        (*((vu32 *)0x40006408))
#define GPIO_PORTC_IEV        (*((vu32 *)0x4000640C))
#define GPIO_PORTC_IM         (*((vu32 *)0x40006410))
#define GPIO_PORTC_RIS        (*((vu32 *)0x40006414))
#define GPIO_PORTC_MIS        (*((vu32 *)0x40006418))
#define GPIO_PORTC_ICR        (*((vu32 *)0x4000641C))
#define GPIO_PORTC_AFSEL      (*((vu32 *)0x40006420))
#define GPIO_PORTC_DR2R       (*((vu32 *)0x40006500))
#define GPIO_PORTC_DR4R       (*((vu32 *)0x40006504))
#define GPIO_PORTC_DR8R       (*((vu32 *)0x40006508))
#define GPIO_PORTC_ODR        (*((vu32 *)0x4000650C))
#define GPIO_PORTC_PUR        (*((vu32 *)0x40006510))
#define GPIO_PORTC_PDR        (*((vu32 *)0x40006514))
#define GPIO_PORTC_SLR        (*((vu32 *)0x40006518))
#define GPIO_PORTC_DEN        (*((vu32 *)0x4000651C))
#define GPIO_PORTC_LOCK       (*((vu32 *)0x40006520))
#define GPIO_PORTC_CR         (*((vu32 *)0x40006524))

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTD)
//
//****************************************************************************
#define GPIO_PORTD_DATA_BITS  ((vu32 *)0x40007000)
#define GPIO_PORTD_DATA       (*((vu32 *)0x400073FC))
#define GPIO_PORTD_DIR        (*((vu32 *)0x40007400))
#define GPIO_PORTD_IS         (*((vu32 *)0x40007404))
#define GPIO_PORTD_IBE        (*((vu32 *)0x40007408))
#define GPIO_PORTD_IEV        (*((vu32 *)0x4000740C))
#define GPIO_PORTD_IM         (*((vu32 *)0x40007410))
#define GPIO_PORTD_RIS        (*((vu32 *)0x40007414))
#define GPIO_PORTD_MIS        (*((vu32 *)0x40007418))
#define GPIO_PORTD_ICR        (*((vu32 *)0x4000741C))
#define GPIO_PORTD_AFSEL      (*((vu32 *)0x40007420))
#define GPIO_PORTD_DR2R       (*((vu32 *)0x40007500))
#define GPIO_PORTD_DR4R       (*((vu32 *)0x40007504))
#define GPIO_PORTD_DR8R       (*((vu32 *)0x40007508))
#define GPIO_PORTD_ODR        (*((vu32 *)0x4000750C))
#define GPIO_PORTD_PUR        (*((vu32 *)0x40007510))
#define GPIO_PORTD_PDR        (*((vu32 *)0x40007514))
#define GPIO_PORTD_SLR        (*((vu32 *)0x40007518))
#define GPIO_PORTD_DEN        (*((vu32 *)0x4000751C))
#define GPIO_PORTD_LOCK       (*((vu32 *)0x40007520))
#define GPIO_PORTD_CR         (*((vu32 *)0x40007524))

//****************************************************************************
//
//Synchronous Serial Interface (SSI0)
//地址通过验证
//****************************************************************************
typedef struct
{
	vu32 CR0;                   //SSI控制寄存器0 包含 SSI时钟频率 时钟相位 时钟极性 帧格式 数据长度位
	vu32 CR1;                   //SSI控制寄存器1 包含 主从选择位 从机输出禁止位 SSI使能位 LOOPBACK设置位
	vu32 DR;                    //SSI数据寄存器 16位数据寄存器
	vu32 SR;                    //SSI状态寄存器  包含 BSY(忙) RFF(接收fifo满) RNE(接收fifo不空) TNF(发送fifo不满) TFE(发送fifo空)
	vu32 CPSR;                  //SSI时钟预分频寄存器 时钟预分频(2~254)间偶数
	vu32 IM;                    //SSI中断屏蔽寄存器
	vu32 RIS;                   //SSI原始中断状态寄存器
	vu32 MIS;                   //SSI屏蔽后中断状态寄存器
	vu32 ICR;                   //SSI中断清除寄存器
	vu32 DMACTL;                //SSI dma控制寄存器 LM3S2139无效
}SSI_Typedef;
#define SSI0                  ((SSI_Typedef*)SSI0_BASE)

#define SSI0_CR0              (*((vu32 *)0x40008000))
#define SSI0_CR1              (*((vu32 *)0x40008004))
#define SSI0_DR               (*((vu32 *)0x40008008))
#define SSI0_SR               (*((vu32 *)0x4000800C))
#define SSI0_CPSR             (*((vu32 *)0x40008010))
#define SSI0_IM               (*((vu32 *)0x40008014))
#define SSI0_RIS              (*((vu32 *)0x40008018))
#define SSI0_MIS              (*((vu32 *)0x4000801C))
#define SSI0_ICR              (*((vu32 *)0x40008020))
#define SSI_O_DMACTL            0x00000024  // SSI DMA Control

//****************************************************************************
//
//Universal Asynchronous Receivers/Transmitters (UART0)
//地址通过验证
//****************************************************************************
typedef struct
{
	vu32 DR;                    // 数据寄存器 接收数据时包含溢出错误(OE) 终止错误(BE) 奇偶错误(PE) 帧错误(FE)状态位
	vu32 RSR;                   // 接收错误状态寄存器 与ECR 错误清除寄存器共用地址 包含与 DR中相同的状态位
	u32  EMPTY1[4];             // 地址填充
	vu32 FR;                    // 标志寄存器 包含发送缓冲区空(TXFE) 接收缓冲区满(RXFF) 发送缓冲区满(TXFF) 接收缓冲区空(RXFE) 忙(BUSY)
	u32  EMPTY2;                // 地址填充
	vu32 ILPR;                  // Irda 低功耗寄存器
	vu32 IBRD;                  // 波特率整数寄存器 存放波特率除数整数值
	vu32 FBRD;                  // 波特率小数寄存器 存放波特率除数小数值
	vu32 LCRH;                  // 线控寄存器 包含 字长 FIFO使能 双停止位 偶校验选择 奇偶校验使能 发送终止位
	vu32 CTL;                   // UART控制寄存器 包含接收使能 发送使能 LOOPBACK使能 Irda低功耗模式 SIR使能 UART使能
	vu32 IFLS;                  // FIFO深度选择寄存器 包含接收FIFO引起中断的深度 和 发送FIFO引起中断的深度 1/8 1/4 1/2 3/4 7/8 
	vu32 IM;                    // 中断屏蔽位 包含OEIM BEIM PEIM FEIM RTIM(接收超时中断) TXIM RXIM
	vu32 RIS;                   // 原始中断状态寄存器
	vu32 MIS;                   // 屏蔽后中断状态寄存器
	vu32 ICR;                   // 中断状态清除寄存器
	vu32 DMACTL;                // DMA控制寄存器 LM3S2139无效
}UART_Typedef;
#define UART0                 ((UART_Typedef*)UART0_BASE)
#define UART0_DR              (*((vu32 *)0x4000C000))
#define UART0_RSR             (*((vu32 *)0x4000C004))
#define UART0_ECR             (*((vu32 *)0x4000C004))
#define UART0_FR              (*((vu32 *)0x4000C018))
#define UART0_ILPR            (*((vu32 *)0x4000C020))
#define UART0_IBRD            (*((vu32 *)0x4000C024))
#define UART0_FBRD            (*((vu32 *)0x4000C028))
#define UART0_LCRH            (*((vu32 *)0x4000C02C))
#define UART0_CTL             (*((vu32 *)0x4000C030))
#define UART0_IFLS            (*((vu32 *)0x4000C034))
#define UART0_IM              (*((vu32 *)0x4000C038))
#define UART0_RIS             (*((vu32 *)0x4000C03C))
#define UART0_MIS             (*((vu32 *)0x4000C040))
#define UART0_ICR             (*((vu32 *)0x4000C044))
#define UART0_DMACTL          (*((vu32 *)0x4000C048))
//*****************************************************************************
//
// The following are defines for the UART Register offsets.
//
//*****************************************************************************
#define UART_O_DR               0x00000000  // Data Register
#define UART_O_RSR              0x00000004  // Receive Status Register (read)
#define UART_O_ECR              0x00000004  // Error Clear Register (write)
#define UART_O_FR               0x00000018  // Flag Register (read only)
#define UART_O_ILPR             0x00000020  // UART IrDA Low-Power Register
#define UART_O_IBRD             0x00000024  // Integer Baud Rate Divisor Reg
#define UART_O_FBRD             0x00000028  // Fractional Baud Rate Divisor Reg
#define UART_O_LCRH             0x0000002C  // UART Line Control
#define UART_O_CTL              0x00000030  // Control Register
#define UART_O_IFLS             0x00000034  // Interrupt FIFO Level Select Reg
#define UART_O_IM               0x00000038  // Interrupt Mask Set/Clear Reg
#define UART_O_RIS              0x0000003C  // Raw Interrupt Status Register
#define UART_O_MIS              0x00000040  // Masked Interrupt Status Register
#define UART_O_ICR              0x00000044  // Interrupt Clear Register
#define UART_O_DMACTL           0x00000048  // UART DMA Control

//*****************************************************************************
//
// The following are deprecated defines for the UART Register offsets.
//
//*****************************************************************************
#define UART_O_LCR_H            0x0000002C  // Line Control Register, HIGH byte
#define UART_O_PeriphID4        0x00000FD0
#define UART_O_PeriphID5        0x00000FD4
#define UART_O_PeriphID6        0x00000FD8
#define UART_O_PeriphID7        0x00000FDC
#define UART_O_PeriphID0        0x00000FE0
#define UART_O_PeriphID1        0x00000FE4
#define UART_O_PeriphID2        0x00000FE8
#define UART_O_PeriphID3        0x00000FEC
#define UART_O_PCellID0         0x00000FF0
#define UART_O_PCellID1         0x00000FF4
#define UART_O_PCellID2         0x00000FF8
#define UART_O_PCellID3         0x00000FFC

//****************************************************************************
//
//Universal Asynchronous Receivers/Transmitters (UART1)
//
//****************************************************************************
#define UART1                 ((UART_Typedef*)UART1_BASE)
#define UART1_DR              (*((vu32 *)0x4000D000))
#define UART1_RSR             (*((vu32 *)0x4000D004))
#define UART1_ECR             (*((vu32 *)0x4000D004))
#define UART1_FR              (*((vu32 *)0x4000D018))
#define UART1_ILPR            (*((vu32 *)0x4000D020))
#define UART1_IBRD            (*((vu32 *)0x4000D024))
#define UART1_FBRD            (*((vu32 *)0x4000D028))
#define UART1_LCRH            (*((vu32 *)0x4000D02C))
#define UART1_CTL             (*((vu32 *)0x4000D030))
#define UART1_IFLS            (*((vu32 *)0x4000D034))
#define UART1_IM              (*((vu32 *)0x4000D038))
#define UART1_RIS             (*((vu32 *)0x4000D03C))
#define UART1_MIS             (*((vu32 *)0x4000D040))
#define UART1_ICR             (*((vu32 *)0x4000D044))
#define UART1_DMACTL          (*((vu32 *)0x4000D048))

//****************************************************************************
//
//Inter-Integrated Circuit (MASTER) Interface
//
//****************************************************************************
#define I2C0_MASTER_MSA       (*((vu32 *)0x40020000))
#define I2C0_MASTER_SOAR      (*((vu32 *)0x40020000))
#define I2C0_MASTER_SCSR      (*((vu32 *)0x40020004))
#define I2C0_MASTER_MCS       (*((vu32 *)0x40020004))
#define I2C0_MASTER_SDR       (*((vu32 *)0x40020008))
#define I2C0_MASTER_MDR       (*((vu32 *)0x40020008))
#define I2C0_MASTER_MTPR      (*((vu32 *)0x4002000C))
#define I2C0_MASTER_SIMR      (*((vu32 *)0x4002000C))
#define I2C0_MASTER_SRIS      (*((vu32 *)0x40020010))
#define I2C0_MASTER_MIMR      (*((vu32 *)0x40020010))
#define I2C0_MASTER_MRIS      (*((vu32 *)0x40020014))
#define I2C0_MASTER_SMIS      (*((vu32 *)0x40020014))
#define I2C0_MASTER_SICR      (*((vu32 *)0x40020018))
#define I2C0_MASTER_MMIS      (*((vu32 *)0x40020018))
#define I2C0_MASTER_MICR      (*((vu32 *)0x4002001C))
#define I2C0_MASTER_MCR       (*((vu32 *)0x40020020))

//****************************************************************************
//
//Inter-Integrated Circuit (SLAVE) Interface
//
//****************************************************************************
#define I2C0_SLAVE_MSA        (*((vu32 *)0x40020800))
#define I2C0_SLAVE_SOAR       (*((vu32 *)0x40020800))
#define I2C0_SLAVE_SCSR       (*((vu32 *)0x40020804))
#define I2C0_SLAVE_MCS        (*((vu32 *)0x40020804))
#define I2C0_SLAVE_SDR        (*((vu32 *)0x40020808))
#define I2C0_SLAVE_MDR        (*((vu32 *)0x40020808))
#define I2C0_SLAVE_MTPR       (*((vu32 *)0x4002080C))
#define I2C0_SLAVE_SIMR       (*((vu32 *)0x4002080C))
#define I2C0_SLAVE_SRIS       (*((vu32 *)0x40020810))
#define I2C0_SLAVE_MIMR       (*((vu32 *)0x40020810))
#define I2C0_SLAVE_MRIS       (*((vu32 *)0x40020814))
#define I2C0_SLAVE_SMIS       (*((vu32 *)0x40020814))
#define I2C0_SLAVE_SICR       (*((vu32 *)0x40020818))
#define I2C0_SLAVE_MMIS       (*((vu32 *)0x40020818))
#define I2C0_SLAVE_MICR       (*((vu32 *)0x4002081C))
#define I2C0_SLAVE_MCR        (*((vu32 *)0x40020820))

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTE)
//
//****************************************************************************
#define GPIO_PORTE_DATA_BITS  ((vu32 *)0x40024000)
#define GPIO_PORTE_DATA       (*((vu32 *)0x400243FC))
#define GPIO_PORTE_DIR        (*((vu32 *)0x40024400))
#define GPIO_PORTE_IS         (*((vu32 *)0x40024404))
#define GPIO_PORTE_IBE        (*((vu32 *)0x40024408))
#define GPIO_PORTE_IEV        (*((vu32 *)0x4002440C))
#define GPIO_PORTE_IM         (*((vu32 *)0x40024410))
#define GPIO_PORTE_RIS        (*((vu32 *)0x40024414))
#define GPIO_PORTE_MIS        (*((vu32 *)0x40024418))
#define GPIO_PORTE_ICR        (*((vu32 *)0x4002441C))
#define GPIO_PORTE_AFSEL      (*((vu32 *)0x40024420))
#define GPIO_PORTE_DR2R       (*((vu32 *)0x40024500))
#define GPIO_PORTE_DR4R       (*((vu32 *)0x40024504))
#define GPIO_PORTE_DR8R       (*((vu32 *)0x40024508))
#define GPIO_PORTE_ODR        (*((vu32 *)0x4002450C))
#define GPIO_PORTE_PUR        (*((vu32 *)0x40024510))
#define GPIO_PORTE_PDR        (*((vu32 *)0x40024514))
#define GPIO_PORTE_SLR        (*((vu32 *)0x40024518))
#define GPIO_PORTE_DEN        (*((vu32 *)0x4002451C))
#define GPIO_PORTE_LOCK       (*((vu32 *)0x40024520))
#define GPIO_PORTE_CR         (*((vu32 *)0x40024524))

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTF)
//
//****************************************************************************
#define GPIO_PORTF_DATA_BITS  ((vu32 *)0x40025000)
#define GPIO_PORTF_DATA       (*((vu32 *)0x400253FC))
#define GPIO_PORTF_DIR        (*((vu32 *)0x40025400))
#define GPIO_PORTF_IS         (*((vu32 *)0x40025404))
#define GPIO_PORTF_IBE        (*((vu32 *)0x40025408))
#define GPIO_PORTF_IEV        (*((vu32 *)0x4002540C))
#define GPIO_PORTF_IM         (*((vu32 *)0x40025410))
#define GPIO_PORTF_RIS        (*((vu32 *)0x40025414))
#define GPIO_PORTF_MIS        (*((vu32 *)0x40025418))
#define GPIO_PORTF_ICR        (*((vu32 *)0x4002541C))
#define GPIO_PORTF_AFSEL      (*((vu32 *)0x40025420))
#define GPIO_PORTF_DR2R       (*((vu32 *)0x40025500))
#define GPIO_PORTF_DR4R       (*((vu32 *)0x40025504))
#define GPIO_PORTF_DR8R       (*((vu32 *)0x40025508))
#define GPIO_PORTF_ODR        (*((vu32 *)0x4002550C))
#define GPIO_PORTF_PUR        (*((vu32 *)0x40025510))
#define GPIO_PORTF_PDR        (*((vu32 *)0x40025514))
#define GPIO_PORTF_SLR        (*((vu32 *)0x40025518))
#define GPIO_PORTF_DEN        (*((vu32 *)0x4002551C))
#define GPIO_PORTF_LOCK       (*((vu32 *)0x40025520))
#define GPIO_PORTF_CR         (*((vu32 *)0x40025524))

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTG)
//
//****************************************************************************
#define GPIO_PORTG_DATA_BITS  ((vu32 *)0x40026000)
#define GPIO_PORTG_DATA       (*((vu32 *)0x400263FC))
#define GPIO_PORTG_DIR        (*((vu32 *)0x40026400))
#define GPIO_PORTG_IS         (*((vu32 *)0x40026404))
#define GPIO_PORTG_IBE        (*((vu32 *)0x40026408))
#define GPIO_PORTG_IEV        (*((vu32 *)0x4002640C))
#define GPIO_PORTG_IM         (*((vu32 *)0x40026410))
#define GPIO_PORTG_RIS        (*((vu32 *)0x40026414))
#define GPIO_PORTG_MIS        (*((vu32 *)0x40026418))
#define GPIO_PORTG_ICR        (*((vu32 *)0x4002641C))
#define GPIO_PORTG_AFSEL      (*((vu32 *)0x40026420))
#define GPIO_PORTG_DR2R       (*((vu32 *)0x40026500))
#define GPIO_PORTG_DR4R       (*((vu32 *)0x40026504))
#define GPIO_PORTG_DR8R       (*((vu32 *)0x40026508))
#define GPIO_PORTG_ODR        (*((vu32 *)0x4002650C))
#define GPIO_PORTG_PUR        (*((vu32 *)0x40026510))
#define GPIO_PORTG_PDR        (*((vu32 *)0x40026514))
#define GPIO_PORTG_SLR        (*((vu32 *)0x40026518))
#define GPIO_PORTG_DEN        (*((vu32 *)0x4002651C))
#define GPIO_PORTG_LOCK       (*((vu32 *)0x40026520))
#define GPIO_PORTG_CR         (*((vu32 *)0x40026524))

//****************************************************************************
//
//General-Purpose Input/Outputs (PORTH)
//
//****************************************************************************
#define GPIO_PORTH_DATA_BITS  ((vu32 *)0x40027000)
#define GPIO_PORTH_DATA       (*((vu32 *)0x400273FC))
#define GPIO_PORTH_DIR        (*((vu32 *)0x40027400))
#define GPIO_PORTH_IS         (*((vu32 *)0x40027404))
#define GPIO_PORTH_IBE        (*((vu32 *)0x40027408))
#define GPIO_PORTH_IEV        (*((vu32 *)0x4002740C))
#define GPIO_PORTH_IM         (*((vu32 *)0x40027410))
#define GPIO_PORTH_RIS        (*((vu32 *)0x40027414))
#define GPIO_PORTH_MIS        (*((vu32 *)0x40027418))
#define GPIO_PORTH_ICR        (*((vu32 *)0x4002741C))
#define GPIO_PORTH_AFSEL      (*((vu32 *)0x40027420))
#define GPIO_PORTH_DR2R       (*((vu32 *)0x40027500))
#define GPIO_PORTH_DR4R       (*((vu32 *)0x40027504))
#define GPIO_PORTH_DR8R       (*((vu32 *)0x40027508))
#define GPIO_PORTH_ODR        (*((vu32 *)0x4002750C))
#define GPIO_PORTH_PUR        (*((vu32 *)0x40027510))
#define GPIO_PORTH_PDR        (*((vu32 *)0x40027514))
#define GPIO_PORTH_SLR        (*((vu32 *)0x40027518))
#define GPIO_PORTH_DEN        (*((vu32 *)0x4002751C))
#define GPIO_PORTH_LOCK       (*((vu32 *)0x40027520))
#define GPIO_PORTH_CR         (*((vu32 *)0x40027524))

//****************************************************************************
//
//General-Purpose Timers (TIMER0)
//地址通过验证
//****************************************************************************
typedef struct
{
	vu32 CFG;                   //通用定时/计数器配置寄存器 包含32位定时器配置 32位RTC配置 16位定时器配置 TAMR BIT0 BIT1决定
	vu32 TAMR;                  //TimerA 模式配置寄存器 包含 TAMR(工作模式) TACMR(边沿计数OR边沿定时) TAAMS(捕获模式OR PWM模式)
	vu32 TBMR;                  //TimerB 模式配置寄存器 包含 TBMR(工作模式) TBCMR(边沿计数OR边沿定时) TBAMS(捕获模式OR PWM模式)
	vu32 CTL;                   //定时器 控制寄存器 控制TimerA TimerB
	u32  EMPTY1[2];             //地址填充
	vu32 IMR;                   //中断屏蔽寄存器 控制TimerA TimerB
	vu32 RIS;                   //原始中断状态寄存器
	vu32 MIS;                   //屏蔽后中断状态寄存器
	vu32 ICR;                   //中断清除寄存器
	vu32 TAILR;                 //TimerA间隔装载寄存器 32位模式 包含 TimerA和TimerB 间隔装载值
	vu32 TBILR;                 //TimerB间隔装载寄存器 TimerB 间隔装载值
	vu32 TAMATCHR;              //TimerA匹配寄存器 
	vu32 TBMATCHR;              //TimerB匹配寄存器 
	vu32 TAPR;                  //TimerA预分频寄存器
	vu32 TBPR;                  //TimerB预分频寄存器
	vu32 TAPMR;                 //TimerA预分频匹配寄存器
	vu32 TBPMR;                 //TimerB预分频匹配寄存器
	vu32 TAR;                   //TimerA计数器当前值寄存器 边沿计数模式下为 边沿发生时的计数值
	vu32 TBR;                   //TimerB计数器当前值寄存器 边沿计数模式下为 边沿发生时的计数值
}TIMER_Typedef;
#define TIMER0                ((TIMER_Typedef*)TIMER0_BASE)
#define TIMER1                ((TIMER_Typedef*)TIMER1_BASE)
#define TIMER2                ((TIMER_Typedef*)TIMER2_BASE)

#define TIMER0_CFG            (*((vu32 *)0x40030000))
#define TIMER0_TAMR           (*((vu32 *)0x40030004))
#define TIMER0_TBMR           (*((vu32 *)0x40030008))
#define TIMER0_CTL            (*((vu32 *)0x4003000C))
#define TIMER0_IMR            (*((vu32 *)0x40030018))
#define TIMER0_RIS            (*((vu32 *)0x4003001C))
#define TIMER0_MIS            (*((vu32 *)0x40030020))
#define TIMER0_ICR            (*((vu32 *)0x40030024))
#define TIMER0_TAILR          (*((vu32 *)0x40030028))
#define TIMER0_TBILR          (*((vu32 *)0x4003002C))
#define TIMER0_TAMATCHR       (*((vu32 *)0x40030030))
#define TIMER0_TBMATCHR       (*((vu32 *)0x40030034))
#define TIMER0_TAPR           (*((vu32 *)0x40030038))
#define TIMER0_TBPR           (*((vu32 *)0x4003003C))
#define TIMER0_TAPMR          (*((vu32 *)0x40030040))
#define TIMER0_TBPMR          (*((vu32 *)0x40030044))
#define TIMER0_TAR            (*((vu32 *)0x40030048))
#define TIMER0_TBR            (*((vu32 *)0x4003004C))
//*****************************************************************************
//
// The following are defines for the timer register offsets.
//
//*****************************************************************************
#define TIMER_O_CFG             0x00000000  // Configuration register
#define TIMER_O_TAMR            0x00000004  // TimerA mode register
#define TIMER_O_TBMR            0x00000008  // TimerB mode register
#define TIMER_O_CTL             0x0000000C  // Control register
#define TIMER_O_IMR             0x00000018  // Interrupt mask register
#define TIMER_O_RIS             0x0000001C  // Interrupt status register
#define TIMER_O_MIS             0x00000020  // Masked interrupt status reg.
#define TIMER_O_ICR             0x00000024  // Interrupt clear register
#define TIMER_O_TAILR           0x00000028  // TimerA interval load register
#define TIMER_O_TBILR           0x0000002C  // TimerB interval load register
#define TIMER_O_TAMATCHR        0x00000030  // TimerA match register
#define TIMER_O_TBMATCHR        0x00000034  // TimerB match register
#define TIMER_O_TAPR            0x00000038  // TimerA prescale register
#define TIMER_O_TBPR            0x0000003C  // TimerB prescale register
#define TIMER_O_TAPMR           0x00000040  // TimerA prescale match register
#define TIMER_O_TBPMR           0x00000044  // TimerB prescale match register
#define TIMER_O_TAR             0x00000048  // TimerA register
#define TIMER_O_TBR             0x0000004C  // TimerB register

//****************************************************************************
//
//General-Purpose Timers (TIMER1)
//
//****************************************************************************
#define TIMER1_CFG            (*((vu32 *)0x40031000))
#define TIMER1_TAMR           (*((vu32 *)0x40031004))
#define TIMER1_TBMR           (*((vu32 *)0x40031008))
#define TIMER1_CTL            (*((vu32 *)0x4003100C))
#define TIMER1_IMR            (*((vu32 *)0x40031018))
#define TIMER1_RIS            (*((vu32 *)0x4003101C))
#define TIMER1_MIS            (*((vu32 *)0x40031020))
#define TIMER1_ICR            (*((vu32 *)0x40031024))
#define TIMER1_TAILR          (*((vu32 *)0x40031028))
#define TIMER1_TBILR          (*((vu32 *)0x4003102C))
#define TIMER1_TAMATCHR       (*((vu32 *)0x40031030))
#define TIMER1_TBMATCHR       (*((vu32 *)0x40031034))
#define TIMER1_TAPR           (*((vu32 *)0x40031038))
#define TIMER1_TBPR           (*((vu32 *)0x4003103C))
#define TIMER1_TAPMR          (*((vu32 *)0x40031040))
#define TIMER1_TBPMR          (*((vu32 *)0x40031044))
#define TIMER1_TAR            (*((vu32 *)0x40031048))
#define TIMER1_TBR            (*((vu32 *)0x4003104C))

//****************************************************************************
//
//General-Purpose Timers (TIMER2)
//
//****************************************************************************
#define TIMER2_CFG            (*((vu32 *)0x40032000))
#define TIMER2_TAMR           (*((vu32 *)0x40032004))
#define TIMER2_TBMR           (*((vu32 *)0x40032008))
#define TIMER2_CTL            (*((vu32 *)0x4003200C))
#define TIMER2_IMR            (*((vu32 *)0x40032018))
#define TIMER2_RIS            (*((vu32 *)0x4003201C))
#define TIMER2_MIS            (*((vu32 *)0x40032020))
#define TIMER2_ICR            (*((vu32 *)0x40032024))
#define TIMER2_TAILR          (*((vu32 *)0x40032028))
#define TIMER2_TBILR          (*((vu32 *)0x4003202C))
#define TIMER2_TAMATCHR       (*((vu32 *)0x40032030))
#define TIMER2_TBMATCHR       (*((vu32 *)0x40032034))
#define TIMER2_TAPR           (*((vu32 *)0x40032038))
#define TIMER2_TBPR           (*((vu32 *)0x4003203C))
#define TIMER2_TAPMR          (*((vu32 *)0x40032040))
#define TIMER2_TBPMR          (*((vu32 *)0x40032044))
#define TIMER2_TAR            (*((vu32 *)0x40032048))
#define TIMER2_TBR            (*((vu32 *)0x4003204C))

/****************************************************************************
*  
* Analog-to-Digital Converter (ADC)
* 地址通过验证
****************************************************************************/
typedef struct
{
	vu32 SSMUX;                //(*((vu32 *)0x40038040))
	vu32 SSCTL;                //(*((vu32 *)0x40038044))
	vu32 SSFIFO;               //(*((vu32 *)0x40038048))
	vu32 SSFSTAT;              //(*((vu32 *)0x4003804C))
	u32  EMPTY[4];             //地址填充
}ADC_SEQ;
typedef struct
{
	vu32 ACTSS;                 //采样序列使能寄存器 ASEN0~ASEN3 active sample sequence
	vu32 RIS;                   //原始中断状态寄存器 INR0~INR3
	vu32 IM;                    //中断屏蔽寄存器     MASK0~MASK3
	vu32 ISC;                   //屏蔽后的中断状态寄存器 IN0~IN3 写1清除中断
	vu32 OSTAT;                 //上溢出状态寄存器 OV0~OV3 OVERFLOW
	vu32 EMUX;                  //事件触发选择寄存器 EM0~EM3  EM(0~15) 4BITS
	vu32 USTAT;                 //下溢出状态寄存器 UV0~UV3 UNDERFLOW
	u32  EMPTY1;                //地址填充
	vu32 SSPRI;                 //ADC序列优先级
	u32  EMPTY2;                //地址填充
	vu32 PSSI;                  //处理器采样序列启动寄存器
	u32  EMPTY3;                //地址填充
	vu32 SAC;                   //硬件平均控制寄存器
	u32  EMPTY4[3];             //地址填充
	ADC_SEQ SEQ[4];             //ADC 采样序列控制寄存器组
	u32  EMPTY8[16];            //地址填充
	vu32 TMLB;                  //测试模式回送
}ADC_Typedef;
#define ADC                   ((ADC_Typedef*)ADC_BASE)
#define ADC_ACTSS             (*((vu32 *)0x40038000))
#define ADC_RIS               (*((vu32 *)0x40038004))
#define ADC_IM                (*((vu32 *)0x40038008))
#define ADC_ISC               (*((vu32 *)0x4003800C))
#define ADC_OSTAT             (*((vu32 *)0x40038010))
#define ADC_EMUX              (*((vu32 *)0x40038014))
#define ADC_USTAT             (*((vu32 *)0x40038018))
#define ADC_SSPRI             (*((vu32 *)0x40038020))
#define ADC_PSSI              (*((vu32 *)0x40038028))
#define ADC_SAC               (*((vu32 *)0x40038030))
#define ADC_SSMUX0            (*((vu32 *)0x40038040))
#define ADC_SSCTL0            (*((vu32 *)0x40038044))
#define ADC_SSFIFO0           (*((vu32 *)0x40038048))
#define ADC_SSFSTAT0          (*((vu32 *)0x4003804C))
#define ADC_SSMUX1            (*((vu32 *)0x40038060))
#define ADC_SSCTL1            (*((vu32 *)0x40038064))
#define ADC_SSFIFO1           (*((vu32 *)0x40038068))
#define ADC_SSFSTAT1          (*((vu32 *)0x4003806C))
#define ADC_SSMUX2            (*((vu32 *)0x40038080))
#define ADC_SSCTL2            (*((vu32 *)0x40038084))
#define ADC_SSFIFO2           (*((vu32 *)0x40038088))
#define ADC_SSFSTAT2          (*((vu32 *)0x4003808C))
#define ADC_SSMUX3            (*((vu32 *)0x400380A0))
#define ADC_SSCTL3            (*((vu32 *)0x400380A4))
#define ADC_SSFIFO3           (*((vu32 *)0x400380A8))
#define ADC_SSFSTAT3          (*((vu32 *)0x400380AC))
#define ADC_TMLB              (*((vu32 *)0x40038100))

//****************************************************************************
//
//Analog Comparators (COMP)
//
//****************************************************************************
#define COMP_ACMIS            (*((vu32 *)0x4003C000))
#define COMP_ACRIS            (*((vu32 *)0x4003C004))
#define COMP_ACINTEN          (*((vu32 *)0x4003C008))
#define COMP_ACREFCTL         (*((vu32 *)0x4003C010))
#define COMP_ACSTAT0          (*((vu32 *)0x4003C020))
#define COMP_ACCTL0           (*((vu32 *)0x4003C024))
#define COMP_ACSTAT1          (*((vu32 *)0x4003C040))
#define COMP_ACCTL1           (*((vu32 *)0x4003C044))
#define COMP_ACSTAT2          (*((vu32 *)0x4003C060))
#define COMP_ACCTL2           (*((vu32 *)0x4003C064))

//****************************************************************************
//
//Controller Area Network (CAN0) Module
//
//****************************************************************************
typedef struct
{
	vu32 CRQ;           //命令请求寄存器
	vu32 CMSK;          //命令屏蔽寄存器(访问控制)
	vu32 MSK1;          //仲裁屏蔽寄存器1
	vu32 MSK2;          //仲裁屏蔽寄存器2
	vu32 ARB1;          //仲裁寄存器1
	vu32 ARB2;          //仲裁寄存器2
	vu32 MCTL;          //报文控制寄存器
	vu32 DA1;           //报文数据寄存器A1
	vu32 DA2;           //报文数据寄存器A2
	vu32 DB1;           //报文数据寄存器B1
	vu32 DB2;           //报文数据寄存器B2
	vu32 EMPTY[13];	    //地址填充
}CAN_INF;             //CAN接口寄存器 IF(Interface)
typedef struct
{
	vu32 CTL;              //CAN控制寄存器
	vu32 STS;              //CAN状态寄存器
	vu32 ERR;              //CAN错误计数寄存器
	vu32 BIT;              //CAN位速率控制寄存器
	vu32 INT;              //报文中断编号寄存器 1-32 引起中断的报文ID 0x8000状态中断
	vu32 TST;              //CAN测试寄存器
	vu32 BRPE;             //波特率预分频寄存器
	vu32 EMPTY1;           //地址填充
	CAN_INF INF[2];        //CAN接口寄存器组 Interface
	vu32 EMPTY2[8];        //地址填充
	vu32 TXRQ1;            //CAN发送请求寄存器1 1-16 号报文
	vu32 TXRQ2;            //CAN发送请求寄存器2 17-32 号报文
	vu32 EMPTY3[6];        //地址填充
	vu32 NWDA1;            //有新数据寄存器1 1-16 号报文
	vu32 NWDA2;            //有新数据寄存器2 17-32 号报文
	vu32 EMPTY4[6];        //地址填充
	vu32 MSG1INT;          //中断挂起寄存器1 1-16 号报文挂起标志
	vu32 MSG2INT;          //中断挂起寄存器2 17-32 号报文挂起标志
	vu32 EMPTY5[6];        //地址填充
	vu32 MSG1VAL;          //报文有效标志寄存器1 1-16 号报文有效标志
	vu32 MSG2VAL;          //报文有效标志寄存器2 17-32 号报文有效标志
}CAN_Typedef;
#define CAN0                  ((CAN_Typedef*)CAN0_BASE)
#define CAN0_CTL              (*((vu32 *)0x40040000))
#define CAN0_STS              (*((vu32 *)0x40040004))
#define CAN0_ERR              (*((vu32 *)0x40040008))
#define CAN0_BIT              (*((vu32 *)0x4004000C))
#define CAN0_INT              (*((vu32 *)0x40040010))
#define CAN0_TST              (*((vu32 *)0x40040014))
#define CAN0_BRPE             (*((vu32 *)0x40040018))
#define CAN0_IF1CRQ           (*((vu32 *)0x40040020))
#define CAN0_IF1CMSK          (*((vu32 *)0x40040024))
#define CAN0_IF1MSK1          (*((vu32 *)0x40040028))
#define CAN0_IF1MSK2          (*((vu32 *)0x4004002C))
#define CAN0_IF1ARB1          (*((vu32 *)0x40040030))
#define CAN0_IF1ARB2          (*((vu32 *)0x40040034))
#define CAN0_IF1MCTL          (*((vu32 *)0x40040038))
#define CAN0_IF1DA1           (*((vu32 *)0x4004003C))
#define CAN0_IF1DA2           (*((vu32 *)0x40040040))
#define CAN0_IF1DB1           (*((vu32 *)0x40040044))
#define CAN0_IF1DB2           (*((vu32 *)0x40040048))
#define CAN0_IF2CRQ           (*((vu32 *)0x40040080))
#define CAN0_IF2CMSK          (*((vu32 *)0x40040084))
#define CAN0_IF2MSK1          (*((vu32 *)0x40040088))
#define CAN0_IF2MSK2          (*((vu32 *)0x4004008C))
#define CAN0_IF2ARB1          (*((vu32 *)0x40040090))
#define CAN0_IF2ARB2          (*((vu32 *)0x40040094))
#define CAN0_IF2MCTL          (*((vu32 *)0x40040098))
#define CAN0_IF2DA1           (*((vu32 *)0x4004009C))
#define CAN0_IF2DA2           (*((vu32 *)0x400400A0))
#define CAN0_IF2DB1           (*((vu32 *)0x400400A4))
#define CAN0_IF2DB2           (*((vu32 *)0x400400A8))
#define CAN0_TXRQ1            (*((vu32 *)0x40040100))
#define CAN0_TXRQ2            (*((vu32 *)0x40040104))
#define CAN0_NWDA1            (*((vu32 *)0x40040120))
#define CAN0_NWDA2            (*((vu32 *)0x40040124))
#define CAN0_MSG1INT          (*((vu32 *)0x40040140))
#define CAN0_MSG2INT          (*((vu32 *)0x40040144))
#define CAN0_MSG1VAL          (*((vu32 *)0x40040160))
#define CAN0_MSG2VAL          (*((vu32 *)0x40040164))
/*****************************************************************************
* The following are defines for the CAN register offsets.
*****************************************************************************/
#define CAN_O_CTL               0x00000000  // Control register
#define CAN_O_STS               0x00000004  // Status register
#define CAN_O_ERR               0x00000008  // Error register
#define CAN_O_BIT               0x0000000C  // Bit Timing register
#define CAN_O_INT               0x00000010  // Interrupt register
#define CAN_O_TST               0x00000014  // Test register
#define CAN_O_BRPE              0x00000018  // Baud Rate Prescaler register
#define CAN_O_IF1CRQ            0x00000020  // Interface 1 Command Request reg.
#define CAN_O_IF1CMSK           0x00000024  // Interface 1 Command Mask reg.
#define CAN_O_IF1MSK1           0x00000028  // Interface 1 Mask 1 register
#define CAN_O_IF1MSK2           0x0000002C  // Interface 1 Mask 2 register
#define CAN_O_IF1ARB1           0x00000030  // Interface 1 Arbitration 1 reg.
#define CAN_O_IF1ARB2           0x00000034  // Interface 1 Arbitration 2 reg.
#define CAN_O_IF1MCTL           0x00000038  // Interface 1 Message Control reg.
#define CAN_O_IF1DA1            0x0000003C  // Interface 1 DataA 1 register
#define CAN_O_IF1DA2            0x00000040  // Interface 1 DataA 2 register
#define CAN_O_IF1DB1            0x00000044  // Interface 1 DataB 1 register
#define CAN_O_IF1DB2            0x00000048  // Interface 1 DataB 2 register
#define CAN_O_IF2CRQ            0x00000080  // Interface 2 Command Request reg.
#define CAN_O_IF2CMSK           0x00000084  // Interface 2 Command Mask reg.
#define CAN_O_IF2MSK1           0x00000088  // Interface 2 Mask 1 register
#define CAN_O_IF2MSK2           0x0000008C  // Interface 2 Mask 2 register
#define CAN_O_IF2ARB1           0x00000090  // Interface 2 Arbitration 1 reg.
#define CAN_O_IF2ARB2           0x00000094  // Interface 2 Arbitration 2 reg.
#define CAN_O_IF2MCTL           0x00000098  // Interface 2 Message Control reg.
#define CAN_O_IF2DA1            0x0000009C  // Interface 2 DataA 1 register
#define CAN_O_IF2DA2            0x000000A0  // Interface 2 DataA 2 register
#define CAN_O_IF2DB1            0x000000A4  // Interface 2 DataB 1 register
#define CAN_O_IF2DB2            0x000000A8  // Interface 2 DataB 2 register
#define CAN_O_TXRQ1             0x00000100  // Transmission Request 1 register
#define CAN_O_TXRQ2             0x00000104  // Transmission Request 2 register
#define CAN_O_NWDA1             0x00000120  // New Data 1 register
#define CAN_O_NWDA2             0x00000124  // New Data 2 register
#define CAN_O_MSG1INT           0x00000140  // CAN Message 1 Interrupt Pending
#define CAN_O_MSG2INT           0x00000144  // CAN Message 2 Interrupt Pending
#define CAN_O_MSG1VAL           0x00000160  // CAN Message 1 Valid
#define CAN_O_MSG2VAL           0x00000164  // CAN Message 2 Valid

/****************************************************************************
*
* Internal Memory (FLASH)
* 地址通过验证
****************************************************************************/
typedef struct
{
	vu32 FMA;                   //存储器地址寄存器 Memory address register
	vu32 FMD;                   //存储器数据寄存器 Memory data register
	vu32 FMC;                   //存储器控制寄存器 包含整体擦除位,页擦除位,编程位,确认位Memory control register
	vu32 FCRIS;                 //存储器控制原始中断状态寄存器 包含编程状态位和访问状态位 Raw interrupt status register
	vu32 FCIM;                  //存储器中断屏蔽寄存器 包含编程屏蔽位和访问屏蔽位 Interrupt mask register
	vu32 FCMISC;                //存储器屏蔽后状态寄存器 包含屏蔽后编程状态位和屏蔽后状态位 Interrupt status register
	u32  EMPTY1[1078];          //地址填充
	vu32 RMCTL;                 //ROM控制寄存器 ROM Control          LM2139无效
	vu32 RMVER;                 //ROM版本寄存器 ROM Version Register LM2139无效
	u32  EMPTY2[14];            //地址填充
	vu32 FMPRE0;                //FLASH 读保护寄存器0 写0保存后 不可更改 一次性慎用 FLASH read protect register 0
	vu32 FMPPE0;                //FLASH 编程使能寄存器0 写0保存后 不可更改 一次性慎用 FLASH program protect register 0
	u32  EMPTY3[2];             //地址填充
	vu32 USECRL;                //us 重装寄存器 产生us编程周期 uSec reload register
	u32  EMPTY4[35];            //地址填充
	vu32 USERDBG;               //用户调试寄存器 一次性 慎用 可以通过写 0 禁止外部调试 User Debug
	u32  EMPTY5[3];             //地址填充
	vu32 USERREG[4];            //用户寄存器 非易失 只能写一次
	u32  EMPTY6[4];             //地址填充
	vu32 FMPRE[4];              //FLASH 读保护寄存器组  FLASH read protect register 
	u32  EMPTY7[124];           //地址填充
	vu32 FMPPE[4];              //FLASH 编程使能寄存器组 FLASH program protect register 
}FLASH_Typedef;
#define FLASH                 ((FLASH_Typedef*)FLASH_CTRL_BASE)
#define FLASH_FMA             (*((vu32 *)0x400FD000))
#define FLASH_FMD             (*((vu32 *)0x400FD004))
#define FLASH_FMC             (*((vu32 *)0x400FD008))
#define FLASH_FCRIS           (*((vu32 *)0x400FD00C))
#define FLASH_FCIM            (*((vu32 *)0x400FD010))
#define FLASH_FCMISC          (*((vu32 *)0x400FD014))
#define FLASH_RMCTL           (*((vu32 *)0x400FE0F0))
#define FLASH_RMVER           (*((vu32 *)0x400FE0F4))
#define FLASH_FMPRE           (*((vu32 *)0x400FE130))
#define FLASH_FMPPE           (*((vu32 *)0x400FE134))
#define FLASH_USECRL          (*((vu32 *)0x400FE140))
#define FLASH_USERDBG         (*((vu32 *)0x400FE1D0))
#define FLASH_USERREG0        (*((vu32 *)0x400FE1E0))
#define FLASH_USERREG1        (*((vu32 *)0x400FE1E4))
#define FLASH_USERREG2        (*((vu32 *)0x400FE1E8))
#define FLASH_USERREG3        (*((vu32 *)0x400FE1EC))
#define FLASH_FMPRE0          (*((vu32 *)0x400FE200))
#define FLASH_FMPRE1          (*((vu32 *)0x400FE204))
#define FLASH_FMPRE2          (*((vu32 *)0x400FE208))
#define FLASH_FMPRE3          (*((vu32 *)0x400FE20C))
#define FLASH_FMPPE0          (*((vu32 *)0x400FE400))
#define FLASH_FMPPE1          (*((vu32 *)0x400FE404))
#define FLASH_FMPPE2          (*((vu32 *)0x400FE408))
#define FLASH_FMPPE3          (*((vu32 *)0x400FE40C))
//*****************************************************************************
//
// The following are defines for the system control register addresses.
// 地址测试正确
//*****************************************************************************
typedef struct
{
	vu32 DID[2];          // 器件标识寄存器 0
	vu32 DC0;             // SARM FLASH SIZE 寄存器
	vu32 DC[5];           // DC[0]偏移量0x0c no use(空) DC[1]~DC[4]对应DC1~DC4
	u32  EMPTY1[4];       // 地址填充
	vu32 PBORCTL;         // 掉电复位控制
	vu32 LDOPCTL;         // LDO输出电压控制
	u32  EMPTY2[2];       // 地址填充
	vu32 SRCR[3];         // 软件复位控制Software Reset Control reg
	u32  EMPTY3;          // 地址填充
	vu32 RIS;             // 原始中断状态
	vu32 IMC;             // 中断屏蔽控制
	vu32 MISC;            // 屏蔽后的中断状态和清除
	vu32 RESC;            // 复位源
	vu32 RCC;             // 运行模式时时钟配置寄存器1
	vu32 PLLCFG;          // XTAL到PLL切换系数 READ ONLY
	u32  EMPTY4;          // 地址填充
	vu32 GPIOHSCTL;       // GPIO 高速控制
	vu32 RCC2;            // 运行模式时时钟配置寄存器2 扩展
	u32  EMPTY5[35];       // 地址填充
	vu32 RCGC[3];         // 运行模式时钟选通寄存器
	u32  EMPTY6;          // 地址填充
	vu32 SCGC[3];         // 睡眠模式时钟选通寄存器
	u32  EMPTY7;          // 地址填充
	vu32 DCGC[3];         // 深度睡眠模式时钟选通寄存器
	u32  EMPTY8[6];       // 地址填充
	vu32 DSLPCLKCFG;      // 深度睡眠模式时钟配置
	u32  EMPTY9[2];       // 地址填充
	vu32 CLKVCLR;         // 时钟校准清除寄存器
	u32  EMPTY10[3];      // 地址填充		    LM3S2139无效
	vu32 LDOARST;         // LDO 复位控制寄存器 LM3S2139无效
	u32  EMPTY11[31];     // 地址填充
	vu32 USER[2];         // NV User Register
}SYSCTL_Typedef;
#define SYSCTL                ((SYSCTL_Typedef*)SYSCTL_BASE)
#define SYSCTL_DID0           (*((vu32 *)0x400FE000))
#define SYSCTL_DID1           (*((vu32 *)0x400FE004))
#define SYSCTL_DC0            (*((vu32 *)0x400FE008))
#define SYSCTL_DC1            (*((vu32 *)0x400FE010))
#define SYSCTL_DC2            (*((vu32 *)0x400FE014))
#define SYSCTL_DC3            (*((vu32 *)0x400FE018))
#define SYSCTL_DC4            (*((vu32 *)0x400FE01C))
#define SYSCTL_PBORCTL        (*((vu32 *)0x400FE030))
#define SYSCTL_LDOPCTL        (*((vu32 *)0x400FE034))
#define SYSCTL_SRCR0          (*((vu32 *)0x400FE040))
#define SYSCTL_SRCR1          (*((vu32 *)0x400FE044))
#define SYSCTL_SRCR2          (*((vu32 *)0x400FE048))
#define SYSCTL_RIS            (*((vu32 *)0x400FE050))
#define SYSCTL_IMC            (*((vu32 *)0x400FE054))
#define SYSCTL_MISC           (*((vu32 *)0x400FE058))
#define SYSCTL_RESC           (*((vu32 *)0x400FE05C))
#define SYSCTL_RCC            (*((vu32 *)0x400FE060))
#define SYSCTL_PLLCFG         (*((vu32 *)0x400FE064))
#define SYSCTL_GPIOHSCTL      (*((vu32 *)0x400FE06C))  //
#define SYSCTL_RCC2           (*((vu32 *)0x400FE070))
#define SYSCTL_RCGC0          (*((vu32 *)0x400FE100))
#define SYSCTL_RCGC1          (*((vu32 *)0x400FE104))
#define SYSCTL_RCGC2          (*((vu32 *)0x400FE108))
#define SYSCTL_SCGC0          (*((vu32 *)0x400FE110))
#define SYSCTL_SCGC1          (*((vu32 *)0x400FE114))
#define SYSCTL_SCGC2          (*((vu32 *)0x400FE118))
#define SYSCTL_DCGC0          (*((vu32 *)0x400FE120))
#define SYSCTL_DCGC1          (*((vu32 *)0x400FE124))
#define SYSCTL_DCGC2          (*((vu32 *)0x400FE128))
#define SYSCTL_DSLPCLKCFG     (*((vu32 *)0x400FE144))
#define SYSCTL_CLKVCLR        (*((vu32 *)0x400FE150))
#define SYSCTL_LDOARST        (*((vu32 *)0x400FE160))
#define SYSCTL_USER0          (*((vu32 *)0x400FE1E0))
#define SYSCTL_USER1          (*((vu32 *)0x400FE1E4))
/****************************************************************************
*
* The following are defines for the NVIC register addresses.
* 地址验证通过
****************************************************************************/
typedef struct 
{ 
	u32  EMPYT1;           //地址填充
	vu32 INT_TYPE;         //中断控制类型寄存器 只读 CPU支持的中断数目 =32*(寄存器值+1)
	u32  EMPTY2[2];        //地址填充
	vu32 ST_CTRL;          //系统时钟节拍控制和状态寄存器 包含溢出标志 时钟源选择 中断允许 启动定时器 位
	vu32 ST_RELOAD;        //系统定时器重置寄存器
	vu32 ST_CURRENT;       //系统定时器当前寄存器
	vu32 ST_CAL;           //系统定时器校准寄存器
	u32  EMPTY3[56];       //地址填充
	vu32 EN[2];            //中断使能设置寄存器
	u32  EMPTY4[30];       //地址填充
	vu32 DIS[2];           //清除中断使能寄存器
	u32  EMPTY5[30];       //地址填充
	vu32 PEND[2];          //中断挂起设置寄存器(可软件置位申请中断)
	u32  EMPTY6[30];       //地址填充
	vu32 UNPEND[2];        //清除已挂起的中断
	u32  EMPTY7[30];       //地址填充
	vu32 ACTIVE[2];        //中断激活状态寄存器
	u32  EMPTY8[62];       //地址填充
	vu32 PRI[12];          //设备中断优先级寄存器 每个寄存器包含4个中断的优先级 每个中断包含8bit位 
	u32  EMPTY9[564];      //地址填充
	vu32 CPUID;            //CPU 基址寄存器 包含CPU ID和REVISION
	vu32 INT_CTRL;         //中断控制状态寄存器 软件申请系统中断 中断ID<16
	vu32 VTABLE;           //中断向量表偏移量寄存器 默任(CODE) VTABLE.TBLBASE=1(RAM)
	vu32 APINT;            //应用中断与复位控制寄存器 包括大小端(ENDIANESS),优先级分组,系统服务请求位,清除有效向量位
	vu32 SYS_CTRL;         //系统控制寄存器 包含WFE唤醒模式,深度睡眠请求位,退出线程睡眠请求位
	vu32 CFG_CTRL;         //配置控制寄存器 包含除0陷阱位
	vu32 SYS_PRI[3];       //系统中断优先级寄存器 3个WORD 控制12个系统中断的优先级 4~15  
	vu32 SYS_HND_CTRL;     //系统处理器控制和状态寄存器 包含系统MPU,Bus,Usage三个中断使能控制位 和系统中断状态位
	vu32 FAULT_STAT;       //可配置故障状态寄存器 包括存储器管理故障寄存器(0~7),总线故障状态寄存器(8~15),使用故障状态寄存器(16~31) 
	vu32 HFAULT_STAT;      //硬故障状态寄存器
	vu32 DEBUG_STAT;       //调试故障状态寄存器
	vu32 MM_ADDR;          //存储器管理故障地址寄存器 存储器故障地址
	vu32 FAULT_ADDR;       //总线故障地址寄存器       保存总线故障的地址
	u32  EMPTY10[21];      //地址填充
	vu32 MPU_TYPE;         //MPU类型寄存器 Memory Protect unit 包含MPU的信息 只读
	vu32 MPU_CTRL;         //MPU控制寄存器 包含MPU使能位
	vu32 MPU_NUMBER;       //MPU区号寄存器 
	vu32 MPU_BASE;         //MPU基址寄存器
	vu32 MPU_ATTR;         //MPU区域属性和大小寄存器
	u32  EMPTY11[19];      //地址填充
	vu32 DBG_CTRL;         //
	vu32 DBG_XFER;         //
	vu32 DBG_DATA;         //
	vu32 DBG_INT;          //
	u32  EMPTY12[64];      //地址填充
	vu32 SW_TRIG;          //软件触发中断寄存器 对设备中断有效(16~63)
}NVIC_Typedef;
#define NVIC                  ((NVIC_Typedef*)NVIC_BASE)
#define NVIC_INT_TYPE         (*((vu32 *)0xE000E004))
#define NVIC_ST_CTRL          (*((vu32 *)0xE000E010))
#define NVIC_ST_RELOAD        (*((vu32 *)0xE000E014))
#define NVIC_ST_CURRENT       (*((vu32 *)0xE000E018))
#define NVIC_ST_CAL           (*((vu32 *)0xE000E01C))
#define NVIC_EN0              (*((vu32 *)0xE000E100))
#define NVIC_EN1              (*((vu32 *)0xE000E104))
#define NVIC_DIS0             (*((vu32 *)0xE000E180))
#define NVIC_DIS1             (*((vu32 *)0xE000E184))
#define NVIC_PEND0            (*((vu32 *)0xE000E200))
#define NVIC_PEND1            (*((vu32 *)0xE000E204))
#define NVIC_UNPEND0          (*((vu32 *)0xE000E280))
#define NVIC_UNPEND1          (*((vu32 *)0xE000E284))
#define NVIC_ACTIVE0          (*((vu32 *)0xE000E300))
#define NVIC_ACTIVE1          (*((vu32 *)0xE000E304))
#define NVIC_PRI0             (*((vu32 *)0xE000E400))
#define NVIC_PRI1             (*((vu32 *)0xE000E404))
#define NVIC_PRI2             (*((vu32 *)0xE000E408))
#define NVIC_PRI3             (*((vu32 *)0xE000E40C))
#define NVIC_PRI4             (*((vu32 *)0xE000E410))
#define NVIC_PRI5             (*((vu32 *)0xE000E414))
#define NVIC_PRI6             (*((vu32 *)0xE000E418))
#define NVIC_PRI7             (*((vu32 *)0xE000E41C))
#define NVIC_PRI8             (*((vu32 *)0xE000E420))
#define NVIC_PRI9             (*((vu32 *)0xE000E424))
#define NVIC_PRI10            (*((vu32 *)0xE000E428))
#define NVIC_PRI11            (*((vu32 *)0xE000E42C)) 
#define NVIC_CPUID            (*((vu32 *)0xE000ED00))
#define NVIC_INT_CTRL         (*((vu32 *)0xE000ED04))
#define NVIC_VTABLE           (*((vu32 *)0xE000ED08))
#define NVIC_APINT            (*((vu32 *)0xE000ED0C))
#define NVIC_SYS_CTRL         (*((vu32 *)0xE000ED10))
#define NVIC_CFG_CTRL         (*((vu32 *)0xE000ED14))
#define NVIC_SYS_PRI1         (*((vu32 *)0xE000ED18))
#define NVIC_SYS_PRI2         (*((vu32 *)0xE000ED1C))
#define NVIC_SYS_PRI3         (*((vu32 *)0xE000ED20))
#define NVIC_SYS_HND_CTRL     (*((vu32 *)0xE000ED24))
#define NVIC_FAULT_STAT       (*((vu32 *)0xE000ED28))
#define NVIC_HFAULT_STAT      (*((vu32 *)0xE000ED2C))
#define NVIC_DEBUG_STAT       (*((vu32 *)0xE000ED30))
#define NVIC_MM_ADDR          (*((vu32 *)0xE000ED34))
#define NVIC_FAULT_ADDR       (*((vu32 *)0xE000ED38))
#define NVIC_MPU_TYPE         (*((vu32 *)0xE000ED90))
#define NVIC_MPU_CTRL         (*((vu32 *)0xE000ED94))
#define NVIC_MPU_NUMBER       (*((vu32 *)0xE000ED98))
#define NVIC_MPU_BASE         (*((vu32 *)0xE000ED9C))
#define NVIC_MPU_ATTR         (*((vu32 *)0xE000EDA0))
#define NVIC_DBG_CTRL         (*((vu32 *)0xE000EDF0))
#define NVIC_DBG_XFER         (*((vu32 *)0xE000EDF4))
#define NVIC_DBG_DATA         (*((vu32 *)0xE000EDF8))
#define NVIC_DBG_INT          (*((vu32 *)0xE000EDFC))
#define NVIC_SW_TRIG          (*((vu32 *)0xE000EF00))

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_CTL register.
//
//*****************************************************************************
#define WDT_CTL_RESEN           0x00000002  // Enable reset output
#define WDT_CTL_INTEN           0x00000001  // Enable the WDT counter and int

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_ISR, WDT_RIS, and
// WDT_MIS registers.
//
//*****************************************************************************
#define WDT_INT_TIMEOUT         0x00000001  // Watchdog timer expired

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_TEST register.
//
//*****************************************************************************
#define WDT_TEST_STALL          0x00000100  // Watchdog stall enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_LOCK register.
//
//*****************************************************************************
#define WDT_LOCK_M              0xFFFFFFFF  // Watchdog Lock.
#define WDT_LOCK_UNLOCK         0x1ACCE551  // Unlocks the watchdog timer
#define WDT_LOCK_LOCKED         0x00000001  // Watchdog timer is locked
#define WDT_LOCK_UNLOCKED       0x00000000  // Watchdog timer is unlocked

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
//*****************************************************************************
#define WDT_LOAD_M              0xFFFFFFFF  // Watchdog Load Value.
#define WDT_LOAD_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
//*****************************************************************************
#define WDT_VALUE_M             0xFFFFFFFF  // Watchdog Value.
#define WDT_VALUE_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
//*****************************************************************************
#define WDT_ICR_M               0xFFFFFFFF  // Watchdog Interrupt Clear.
#define WDT_ICR_S               0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
//*****************************************************************************
#define WDT_RIS_WDTRIS          0x00000001  // Watchdog Raw Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
//*****************************************************************************
#define WDT_MIS_WDTMIS          0x00000001  // Watchdog Masked Interrupt
                                            // Status.

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the WDT_TEST
// register.
//
//*****************************************************************************
#define WDT_TEST_STALL_EN       0x00000100  // Watchdog stall enable

//*****************************************************************************
//
// The following are deprecated defines for the reset values for the WDT
// registers.
//
//*****************************************************************************
#define WDT_RV_VALUE            0xFFFFFFFF  // Current value register
#define WDT_RV_LOAD             0xFFFFFFFF  // Load register
#define WDT_RV_PCellID1         0x000000F0
#define WDT_RV_PCellID3         0x000000B1
#define WDT_RV_PeriphID1        0x00000018
#define WDT_RV_PeriphID2        0x00000018
#define WDT_RV_PCellID0         0x0000000D
#define WDT_RV_PCellID2         0x00000005
#define WDT_RV_PeriphID0        0x00000005
#define WDT_RV_PeriphID3        0x00000001
#define WDT_RV_PeriphID5        0x00000000
#define WDT_RV_RIS              0x00000000  // Raw interrupt status register
#define WDT_RV_CTL              0x00000000  // Control register
#define WDT_RV_PeriphID4        0x00000000
#define WDT_RV_PeriphID6        0x00000000
#define WDT_RV_PeriphID7        0x00000000
#define WDT_RV_LOCK             0x00000000  // Lock register
#define WDT_RV_MIS              0x00000000  // Masked interrupt status register

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_LOCK register.
//
//*****************************************************************************
#define GPIO_LOCK_M             0xFFFFFFFF  // GPIO Lock.
#define GPIO_LOCK_UNLOCKED      0x00000000  // unlocked
#define GPIO_LOCK_LOCKED        0x00000001  // locked
#define GPIO_LOCK_KEY           0x1ACCE551  // Unlocks the GPIO_CR register

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate.
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase.
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity.
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select.
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_FRF_TI          0x00000010  // Texas Intruments Synchronous
                                            // Serial Frame Format
#define SSI_CR0_FRF_NMW         0x00000020  // MICROWIRE Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select.
#define SSI_CR0_DSS_4           0x00000003  // 4-bit data
#define SSI_CR0_DSS_5           0x00000004  // 5-bit data
#define SSI_CR0_DSS_6           0x00000005  // 6-bit data
#define SSI_CR0_DSS_7           0x00000006  // 7-bit data
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR0_DSS_9           0x00000008  // 9-bit data
#define SSI_CR0_DSS_10          0x00000009  // 10-bit data
#define SSI_CR0_DSS_11          0x0000000A  // 11-bit data
#define SSI_CR0_DSS_12          0x0000000B  // 12-bit data
#define SSI_CR0_DSS_13          0x0000000C  // 13-bit data
#define SSI_CR0_DSS_14          0x0000000D  // 14-bit data
#define SSI_CR0_DSS_15          0x0000000E  // 15-bit data
#define SSI_CR0_DSS_16          0x0000000F  // 16-bit data
#define SSI_CR0_SCR_S           8

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
#define SSI_CR1_SOD             0x00000008  // SSI Slave Mode Output Disable.
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select.
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable.
#define SSI_CR1_LBM             0x00000001  // SSI Loopback Mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
#define SSI_DR_DATA_M           0x0000FFFF  // SSI Receive/Transmit Data.
#define SSI_DR_DATA_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit.
#define SSI_SR_RFF              0x00000008  // SSI Receive FIFO Full.
#define SSI_SR_RNE              0x00000004  // SSI Receive FIFO Not Empty.
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full.
#define SSI_SR_TFE              0x00000001  // SSI Transmit FIFO Empty.

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor.
#define SSI_CPSR_CPSDVSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
#define SSI_IM_TXIM             0x00000008  // SSI Transmit FIFO Interrupt
                                            // Mask.
#define SSI_IM_RXIM             0x00000004  // SSI Receive FIFO Interrupt Mask.
#define SSI_IM_RTIM             0x00000002  // SSI Receive Time-Out Interrupt
                                            // Mask.
#define SSI_IM_RORIM            0x00000001  // SSI Receive Overrun Interrupt
                                            // Mask.

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
#define SSI_RIS_TXRIS           0x00000008  // SSI Transmit FIFO Raw Interrupt
                                            // Status.
#define SSI_RIS_RXRIS           0x00000004  // SSI Receive FIFO Raw Interrupt
                                            // Status.
#define SSI_RIS_RTRIS           0x00000002  // SSI Receive Time-Out Raw
                                            // Interrupt Status.
#define SSI_RIS_RORRIS          0x00000001  // SSI Receive Overrun Raw
                                            // Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
#define SSI_MIS_TXMIS           0x00000008  // SSI Transmit FIFO Masked
                                            // Interrupt Status.
#define SSI_MIS_RXMIS           0x00000004  // SSI Receive FIFO Masked
                                            // Interrupt Status.
#define SSI_MIS_RTMIS           0x00000002  // SSI Receive Time-Out Masked
                                            // Interrupt Status.
#define SSI_MIS_RORMIS          0x00000001  // SSI Receive Overrun Masked
                                            // Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
#define SSI_ICR_RTIC            0x00000002  // SSI Receive Time-Out Interrupt
                                            // Clear.
#define SSI_ICR_RORIC           0x00000001  // SSI Receive Overrun Interrupt
                                            // Clear.

//*****************************************************************************
//
// The following are defines for the Data Register bits
//
//*****************************************************************************
#define UART_DR_OE              0x00000800  // Overrun Error
#define UART_DR_BE              0x00000400  // Break Error
#define UART_DR_PE              0x00000200  // Parity Error
#define UART_DR_FE              0x00000100  // Framing Error
#define UART_DR_DATA_M          0x000000FF  // Data Transmitted or Received.
#define UART_DR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the Receive Status Register bits
//
//*****************************************************************************
#define UART_RSR_OE             0x00000008  // Overrun Error
#define UART_RSR_BE             0x00000004  // Break Error
#define UART_RSR_PE             0x00000002  // Parity Error
#define UART_RSR_FE             0x00000001  // Framing Error

//*****************************************************************************
//
// The following are defines for the Flag Register bits
//
//*****************************************************************************
#define UART_FR_TXFE            0x00000080  // TX FIFO Empty
#define UART_FR_RXFF            0x00000040  // RX FIFO Full
#define UART_FR_TXFF            0x00000020  // TX FIFO Full
#define UART_FR_RXFE            0x00000010  // RX FIFO Empty
#define UART_FR_BUSY            0x00000008  // UART Busy

//*****************************************************************************
//
// The following are defines for the Integer baud-rate divisor
//
//*****************************************************************************
#define UART_IBRD_DIVINT_M      0x0000FFFF  // Integer Baud-Rate Divisor.
#define UART_IBRD_DIVINT_S      0

//*****************************************************************************
//
// The following are defines for the Fractional baud-rate divisor
//
//*****************************************************************************
#define UART_FBRD_DIVFRAC_M     0x0000003F  // Fractional Baud-Rate Divisor.
#define UART_FBRD_DIVFRAC_S     0

//*****************************************************************************
//
// The following are defines for the Control Register bits
//
//*****************************************************************************
#define UART_CTL_RXE            0x00000200  // Receive Enable
#define UART_CTL_TXE            0x00000100  // Transmit Enable
#define UART_CTL_LBE            0x00000080  // Loopback Enable
#define UART_CTL_SIRLP          0x00000004  // SIR (IrDA) Low Power Enable
#define UART_CTL_SIREN          0x00000002  // SIR (IrDA) Enable
#define UART_CTL_UARTEN         0x00000001  // UART Enable

//*****************************************************************************
//
// The following are defines for the Interrupt FIFO Level Select Register bits
//
//*****************************************************************************
#define UART_IFLS_RX_M          0x00000038  // RX FIFO Level Interrupt Mask
#define UART_IFLS_RX1_8         0x00000000  // 1/8 Full
#define UART_IFLS_RX2_8         0x00000008  // 1/4 Full
#define UART_IFLS_RX4_8         0x00000010  // 1/2 Full
#define UART_IFLS_RX6_8         0x00000018  // 3/4 Full
#define UART_IFLS_RX7_8         0x00000020  // 7/8 Full
#define UART_IFLS_TX_M          0x00000007  // TX FIFO Level Interrupt Mask
#define UART_IFLS_TX1_8         0x00000000  // 1/8 Full
#define UART_IFLS_TX2_8         0x00000001  // 1/4 Full
#define UART_IFLS_TX4_8         0x00000002  // 1/2 Full
#define UART_IFLS_TX6_8         0x00000003  // 3/4 Full
#define UART_IFLS_TX7_8         0x00000004  // 7/8 Full

//*****************************************************************************
//
// The following are defines for the Interrupt Mask Set/Clear Register bits
//
//*****************************************************************************
#define UART_IM_OEIM            0x00000400  // Overrun Error Interrupt Mask
#define UART_IM_BEIM            0x00000200  // Break Error Interrupt Mask
#define UART_IM_PEIM            0x00000100  // Parity Error Interrupt Mask
#define UART_IM_FEIM            0x00000080  // Framing Error Interrupt Mask
#define UART_IM_RTIM            0x00000040  // Receive Timeout Interrupt Mask
#define UART_IM_TXIM            0x00000020  // Transmit Interrupt Mask
#define UART_IM_RXIM            0x00000010  // Receive Interrupt Mask

//*****************************************************************************
//
// The following are defines for the Raw Interrupt Status Register
//
//*****************************************************************************
#define UART_RIS_OERIS          0x00000400  // Overrun Error Interrupt Status
#define UART_RIS_BERIS          0x00000200  // Break Error Interrupt Status
#define UART_RIS_PERIS          0x00000100  // Parity Error Interrupt Status
#define UART_RIS_FERIS          0x00000080  // Framing Error Interrupt Status
#define UART_RIS_RTRIS          0x00000040  // Receive Timeout Interrupt Status
#define UART_RIS_TXRIS          0x00000020  // Transmit Interrupt Status
#define UART_RIS_RXRIS          0x00000010  // Receive Interrupt Status

//*****************************************************************************
//
// The following are defines for the Masked Interrupt Status Register
//
//*****************************************************************************
#define UART_MIS_OEMIS          0x00000400  // Overrun Error Interrupt Status
#define UART_MIS_BEMIS          0x00000200  // Break Error Interrupt Status
#define UART_MIS_PEMIS          0x00000100  // Parity Error Interrupt Status
#define UART_MIS_FEMIS          0x00000080  // Framing Error Interrupt Status
#define UART_MIS_RTMIS          0x00000040  // Receive Timeout Interrupt Status
#define UART_MIS_TXMIS          0x00000020  // Transmit Interrupt Status
#define UART_MIS_RXMIS          0x00000010  // Receive Interrupt Status

//*****************************************************************************
//
// The following are defines for the Interrupt Clear Register bits
//
//*****************************************************************************
#define UART_ICR_OEIC           0x00000400  // Overrun Error Interrupt Clear
#define UART_ICR_BEIC           0x00000200  // Break Error Interrupt Clear
#define UART_ICR_PEIC           0x00000100  // Parity Error Interrupt Clear
#define UART_ICR_FEIC           0x00000080  // Framing Error Interrupt Clear
#define UART_ICR_RTIC           0x00000040  // Receive Timeout Interrupt Clear
#define UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
#define UART_ECR_DATA_M         0x000000FF  // Error Clear.
#define UART_ECR_DATA_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
#define UART_LCRH_SPS           0x00000080  // UART Stick Parity Select.
#define UART_LCRH_WLEN_M        0x00000060  // UART Word Length.
#define UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define UART_LCRH_WLEN_8        0x00000060  // 8 bits
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs.
#define UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select.
#define UART_LCRH_EPS           0x00000004  // UART Even Parity Select.
#define UART_LCRH_PEN           0x00000002  // UART Parity Enable.
#define UART_LCRH_BRK           0x00000001  // UART Send Break.

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
#define UART_ILPR_ILPDVSR_M     0x000000FF  // IrDA Low-Power Divisor.
#define UART_ILPR_ILPDVSR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
#define UART_DMACTL_DMAERR      0x00000004  // DMA on Error.
#define UART_DMACTL_TXDMAE      0x00000002  // Transmit DMA Enable.
#define UART_DMACTL_RXDMAE      0x00000001  // Receive DMA Enable.


//*****************************************************************************
//
// The following are deprecated defines for the Data Register bits
//
//*****************************************************************************
#define UART_DR_DATA_MASK       0x000000FF  // UART data

//*****************************************************************************
//
// The following are deprecated defines for the Integer baud-rate divisor
//
//*****************************************************************************
#define UART_IBRD_DIVINT_MASK   0x0000FFFF  // Integer baud-rate divisor

//*****************************************************************************
//
// The following are deprecated defines for the Fractional baud-rate divisor
//
//*****************************************************************************
#define UART_FBRD_DIVFRAC_MASK  0x0000003F  // Fractional baud-rate divisor

//*****************************************************************************
//
// The following are deprecated defines for the Line Control Register High bits
//
//*****************************************************************************
#define UART_LCR_H_SPS          0x00000080  // Stick Parity Select
#define UART_LCR_H_WLEN         0x00000060  // Word length
#define UART_LCR_H_WLEN_5       0x00000000  // 5 bit data
#define UART_LCR_H_WLEN_6       0x00000020  // 6 bit data
#define UART_LCR_H_WLEN_7       0x00000040  // 7 bit data
#define UART_LCR_H_WLEN_8       0x00000060  // 8 bit data
#define UART_LCR_H_FEN          0x00000010  // Enable FIFO
#define UART_LCR_H_STP2         0x00000008  // Two Stop Bits Select
#define UART_LCR_H_EPS          0x00000004  // Even Parity Select
#define UART_LCR_H_PEN          0x00000002  // Parity Enable
#define UART_LCR_H_BRK          0x00000001  // Send Break

//*****************************************************************************
//
// The following are deprecated defines for the Interrupt FIFO Level Select
// Register bits
//
//*****************************************************************************
#define UART_IFLS_RX_MASK       0x00000038  // RX FIFO level mask
#define UART_IFLS_TX_MASK       0x00000007  // TX FIFO level mask

//*****************************************************************************
//
// The following are deprecated defines for the Interrupt Clear Register bits
//
//*****************************************************************************
#define UART_RSR_ANY            (UART_RSR_OE | UART_RSR_BE | UART_RSR_PE | \
                                 UART_RSR_FE)

//*****************************************************************************
//
// The following are deprecated defines for the Reset Values for UART
// Registers.
//
//*****************************************************************************
#define UART_RV_CTL             0x00000300
#define UART_RV_PCellID1        0x000000F0
#define UART_RV_PCellID3        0x000000B1
#define UART_RV_FR              0x00000090
#define UART_RV_PeriphID2       0x00000018
#define UART_RV_IFLS            0x00000012
#define UART_RV_PeriphID0       0x00000011
#define UART_RV_PCellID0        0x0000000D
#define UART_RV_PCellID2        0x00000005
#define UART_RV_PeriphID3       0x00000001
#define UART_RV_PeriphID4       0x00000000
#define UART_RV_LCR_H           0x00000000
#define UART_RV_PeriphID6       0x00000000
#define UART_RV_DR              0x00000000
#define UART_RV_RSR             0x00000000
#define UART_RV_ECR             0x00000000
#define UART_RV_PeriphID5       0x00000000
#define UART_RV_RIS             0x00000000
#define UART_RV_FBRD            0x00000000
#define UART_RV_IM              0x00000000
#define UART_RV_MIS             0x00000000
#define UART_RV_ICR             0x00000000
#define UART_RV_PeriphID1       0x00000000
#define UART_RV_PeriphID7       0x00000000
#define UART_RV_IBRD            0x00000000

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
//*****************************************************************************
#define I2C_MSA_SA_M            0x000000FE  // I2C Slave Address.
#define I2C_MSA_RS              0x00000001  // Receive not send.
#define I2C_MSA_SA_S            1

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
//*****************************************************************************
#define I2C_SOAR_OAR_M          0x0000007F  // I2C Slave Own Address.
#define I2C_SOAR_OAR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
//*****************************************************************************
#define I2C_SCSR_FBR            0x00000004  // First Byte Received.
#define I2C_SCSR_TREQ           0x00000002  // Transmit Request.
#define I2C_SCSR_DA             0x00000001  // Device Active.
#define I2C_SCSR_RREQ           0x00000001  // Receive Request.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
//*****************************************************************************
#define I2C_MCS_BUSBSY          0x00000040  // Bus Busy.
#define I2C_MCS_IDLE            0x00000020  // I2C Idle.
#define I2C_MCS_ARBLST          0x00000010  // Arbitration Lost.
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable.
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data.
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address.
#define I2C_MCS_STOP            0x00000004  // Generate STOP.
#define I2C_MCS_START           0x00000002  // Generate START.
#define I2C_MCS_ERROR           0x00000002  // Error.
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable.
#define I2C_MCS_BUSY            0x00000001  // I2C Busy.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
//*****************************************************************************
#define I2C_SDR_DATA_M          0x000000FF  // Data for Transfer.
#define I2C_SDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
//*****************************************************************************
#define I2C_MDR_DATA_M          0x000000FF  // Data Transferred.
#define I2C_MDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
//*****************************************************************************
#define I2C_MTPR_TPR_M          0x000000FF  // SCL Clock Period.
#define I2C_MTPR_TPR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
//*****************************************************************************
#define I2C_SIMR_IM             0x00000001  // Data Interrupt Mask.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
#define I2C_SRIS_RIS            0x00000001  // Data Raw Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
#define I2C_MIMR_IM             0x00000001  // Interrupt Mask.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
#define I2C_MRIS_RIS            0x00000001  // Raw Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
//*****************************************************************************
#define I2C_SMIS_MIS            0x00000001  // Data Masked Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
//*****************************************************************************
#define I2C_SICR_IC             0x00000001  // Data Interrupt Clear.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
#define I2C_MMIS_MIS            0x00000001  // Masked Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
#define I2C_MICR_IC             0x00000001  // Interrupt Clear.

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
#define I2C_MCR_SFE             0x00000020  // I2C Slave Function Enable.
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable.
#define I2C_MCR_LPBK            0x00000001  // I2C Loopback.

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_CFG register.
//
//*****************************************************************************
#define TIMER_CFG_M             0x00000007  // GPTM Configuration.
#define TIMER_CFG_16_BIT        0x00000004  // Two 16 bit timers
#define TIMER_CFG_32_BIT_RTC    0x00000001  // 32 bit RTC
#define TIMER_CFG_32_BIT_TIMER  0x00000000  // 32 bit timer

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_CTL register.
//
//*****************************************************************************
#define TIMER_CTL_TBPWML        0x00004000  // TimerB PWM output level invert
#define TIMER_CTL_TBOTE         0x00002000  // TimerB output trigger enable
#define TIMER_CTL_TBEVENT_POS   0x00000000  // TimerB event mode - pos edge
#define TIMER_CTL_TBEVENT_NEG   0x00000400  // TimerB event mode - neg edge
#define TIMER_CTL_TBEVENT_BOTH  0x00000C00  // TimerB event mode - both edges
#define TIMER_CTL_TBEVENT_M     0x00000C00  // GPTM TimerB Event Mode.
#define TIMER_CTL_TBSTALL       0x00000200  // TimerB stall enable
#define TIMER_CTL_TBEN          0x00000100  // TimerB enable
#define TIMER_CTL_TAPWML        0x00000040  // TimerA PWM output level invert
#define TIMER_CTL_TAOTE         0x00000020  // TimerA output trigger enable
#define TIMER_CTL_RTCEN         0x00000010  // RTC counter enable
#define TIMER_CTL_TAEVENT_M     0x0000000C  // GPTM TimerA Event Mode.
#define TIMER_CTL_TAEVENT_POS   0x00000000  // TimerA event mode - pos edge
#define TIMER_CTL_TAEVENT_NEG   0x00000004  // TimerA event mode - neg edge
#define TIMER_CTL_TAEVENT_BOTH  0x0000000C  // TimerA event mode - both edges
#define TIMER_CTL_TASTALL       0x00000002  // TimerA stall enable
#define TIMER_CTL_TAEN          0x00000001  // TimerA enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_IMR register.
//
//*****************************************************************************
#define TIMER_IMR_CBEIM         0x00000400  // CaptureB event interrupt mask
#define TIMER_IMR_CBMIM         0x00000200  // CaptureB match interrupt mask
#define TIMER_IMR_TBTOIM        0x00000100  // TimerB time out interrupt mask
#define TIMER_IMR_RTCIM         0x00000008  // RTC interrupt mask
#define TIMER_IMR_CAEIM         0x00000004  // CaptureA event interrupt mask
#define TIMER_IMR_CAMIM         0x00000002  // CaptureA match interrupt mask
#define TIMER_IMR_TATOIM        0x00000001  // TimerA time out interrupt mask

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_RIS register.
//
//*****************************************************************************
#define TIMER_RIS_CBERIS        0x00000400  // CaptureB event raw int status
#define TIMER_RIS_CBMRIS        0x00000200  // CaptureB match raw int status
#define TIMER_RIS_TBTORIS       0x00000100  // TimerB time out raw int status
#define TIMER_RIS_RTCRIS        0x00000008  // RTC raw int status
#define TIMER_RIS_CAERIS        0x00000004  // CaptureA event raw int status
#define TIMER_RIS_CAMRIS        0x00000002  // CaptureA match raw int status
#define TIMER_RIS_TATORIS       0x00000001  // TimerA time out raw int status

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_ICR register.
//
//*****************************************************************************
#define TIMER_ICR_CBECINT       0x00000400  // CaptureB event interrupt clear
#define TIMER_ICR_CBMCINT       0x00000200  // CaptureB match interrupt clear
#define TIMER_ICR_TBTOCINT      0x00000100  // TimerB time out interrupt clear
#define TIMER_ICR_RTCCINT       0x00000008  // RTC interrupt clear
#define TIMER_ICR_CAECINT       0x00000004  // CaptureA event interrupt clear
#define TIMER_ICR_CAMCINT       0x00000002  // CaptureA match interrupt clear
#define TIMER_ICR_TATOCINT      0x00000001  // TimerA time out interrupt clear

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_TAILR register.
//
//*****************************************************************************
#define TIMER_TAILR_TAILRH_M    0xFFFF0000  // GPTM TimerA Interval Load
                                            // Register High.
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low.
#define TIMER_TAILR_TAILRH_S    16
#define TIMER_TAILR_TAILRL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_TBILR register.
//
//*****************************************************************************
#define TIMER_TBILR_TBILRL_M    0x0000FFFF  // GPTM TimerB Interval Load
                                            // Register.
#define TIMER_TBILR_TBILRL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_TAMATCHR register.
//
//*****************************************************************************
#define TIMER_TAMATCHR_TAMRH_M  0xFFFF0000  // GPTM TimerA Match Register High.
#define TIMER_TAMATCHR_TAMRL_M  0x0000FFFF  // GPTM TimerA Match Register Low.
#define TIMER_TAMATCHR_TAMRH_S  16
#define TIMER_TAMATCHR_TAMRL_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_TBMATCHR register.
//
//*****************************************************************************
#define TIMER_TBMATCHR_TBMRL_M  0x0000FFFF  // GPTM TimerB Match Register Low.
#define TIMER_TBMATCHR_TBMRL_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_TAR register.
//
//*****************************************************************************
#define TIMER_TAR_TARH_M        0xFFFF0000  // GPTM TimerA Register High.
#define TIMER_TAR_TARL_M        0x0000FFFF  // GPTM TimerA Register Low.
#define TIMER_TAR_TARH_S        16
#define TIMER_TAR_TARL_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_TBR register.
//
//*****************************************************************************
#define TIMER_TBR_TBRL_M        0x0000FFFF  // GPTM TimerB.
#define TIMER_TBR_TBRL_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
//*****************************************************************************
#define TIMER_TAMR_TAAMS        0x00000008  // GPTM TimerA Alternate Mode
                                            // Select.
#define TIMER_TAMR_TACMR        0x00000004  // GPTM TimerA Capture Mode.
#define TIMER_TAMR_TAMR_M       0x00000003  // GPTM TimerA Mode.
#define TIMER_TAMR_TAMR_1_SHOT  0x00000001  // One-Shot Timer mode.
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode.
#define TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
//*****************************************************************************
#define TIMER_TBMR_TBAMS        0x00000008  // GPTM TimerB Alternate Mode
                                            // Select.
#define TIMER_TBMR_TBCMR        0x00000004  // GPTM TimerB Capture Mode.
#define TIMER_TBMR_TBMR_M       0x00000003  // GPTM TimerB Mode.
#define TIMER_TBMR_TBMR_1_SHOT  0x00000001  // One-Shot Timer mode.
#define TIMER_TBMR_TBMR_PERIOD  0x00000002  // Periodic Timer mode.
#define TIMER_TBMR_TBMR_CAP     0x00000003  // Capture mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_MIS register.
//
//*****************************************************************************
#define TIMER_MIS_CBEMIS        0x00000400  // GPTM CaptureB Event Masked
                                            // Interrupt.
#define TIMER_MIS_CBMMIS        0x00000200  // GPTM CaptureB Match Masked
                                            // Interrupt.
#define TIMER_MIS_TBTOMIS       0x00000100  // GPTM TimerB Time-Out Masked
                                            // Interrupt.
#define TIMER_MIS_RTCMIS        0x00000008  // GPTM RTC Masked Interrupt.
#define TIMER_MIS_CAEMIS        0x00000004  // GPTM CaptureA Event Masked
                                            // Interrupt.
#define TIMER_MIS_CAMMIS        0x00000002  // GPTM CaptureA Match Masked
                                            // Interrupt.
#define TIMER_MIS_TATOMIS       0x00000001  // GPTM TimerA Time-Out Masked
                                            // Interrupt.

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPR register.
//
//*****************************************************************************
#define TIMER_TAPR_TAPSR_M      0x000000FF  // GPTM TimerA Prescale.
#define TIMER_TAPR_TAPSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
//*****************************************************************************
#define TIMER_TBPR_TBPSR_M      0x000000FF  // GPTM TimerB Prescale.
#define TIMER_TBPR_TBPSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
//*****************************************************************************
#define TIMER_TAPMR_TAPSMR_M    0x000000FF  // GPTM TimerA Prescale Match.
#define TIMER_TAPMR_TAPSMR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
//*****************************************************************************
#define TIMER_TBPMR_TBPSMR_M    0x000000FF  // GPTM TimerB Prescale Match.
#define TIMER_TBPMR_TBPSMR_S    0

//*****************************************************************************
//
// The following are deprecated defines for the reset values of the timer
// registers.
//
//*****************************************************************************
#define TIMER_RV_TAILR          0xFFFFFFFF  // TimerA interval load reg RV
#define TIMER_RV_TAR            0xFFFFFFFF  // TimerA register RV
#define TIMER_RV_TAMATCHR       0xFFFFFFFF  // TimerA match register RV
#define TIMER_RV_TBILR          0x0000FFFF  // TimerB interval load reg RV
#define TIMER_RV_TBMATCHR       0x0000FFFF  // TimerB match register RV
#define TIMER_RV_TBR            0x0000FFFF  // TimerB register RV
#define TIMER_RV_TAPR           0x00000000  // TimerA prescale register RV
#define TIMER_RV_CFG            0x00000000  // Configuration register RV
#define TIMER_RV_TBPMR          0x00000000  // TimerB prescale match regi RV
#define TIMER_RV_TAPMR          0x00000000  // TimerA prescale match reg RV
#define TIMER_RV_CTL            0x00000000  // Control register RV
#define TIMER_RV_ICR            0x00000000  // Interrupt clear register RV
#define TIMER_RV_TBMR           0x00000000  // TimerB mode register RV
#define TIMER_RV_MIS            0x00000000  // Masked interrupt status reg RV
#define TIMER_RV_RIS            0x00000000  // Interrupt status register RV
#define TIMER_RV_TBPR           0x00000000  // TimerB prescale register RV
#define TIMER_RV_IMR            0x00000000  // Interrupt mask register RV
#define TIMER_RV_TAMR           0x00000000  // TimerA mode register RV

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_CFG
// register.
//
//*****************************************************************************
#define TIMER_CFG_CFG_MSK       0x00000007  // Configuration options mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TnMR
// register.
//
//*****************************************************************************
#define TIMER_TNMR_TNAMS        0x00000008  // Alternate mode select
#define TIMER_TNMR_TNCMR        0x00000004  // Capture mode - count or time
#define TIMER_TNMR_TNTMR_MSK    0x00000003  // Timer mode mask
#define TIMER_TNMR_TNTMR_1_SHOT 0x00000001  // Mode - one shot
#define TIMER_TNMR_TNTMR_PERIOD 0x00000002  // Mode - periodic
#define TIMER_TNMR_TNTMR_CAP    0x00000003  // Mode - capture

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_CTL
// register.
//
//*****************************************************************************
#define TIMER_CTL_TBEVENT_MSK   0x00000C00  // TimerB event mode mask
#define TIMER_CTL_TAEVENT_MSK   0x0000000C  // TimerA event mode mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_MIS
// register.
//
//*****************************************************************************
#define TIMER_RIS_CBEMIS        0x00000400  // CaptureB event masked int status
#define TIMER_RIS_CBMMIS        0x00000200  // CaptureB match masked int status
#define TIMER_RIS_TBTOMIS       0x00000100  // TimerB time out masked int stat
#define TIMER_RIS_RTCMIS        0x00000008  // RTC masked int status
#define TIMER_RIS_CAEMIS        0x00000004  // CaptureA event masked int status
#define TIMER_RIS_CAMMIS        0x00000002  // CaptureA match masked int status
#define TIMER_RIS_TATOMIS       0x00000001  // TimerA time out masked int stat

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TAILR
// register.
//
//*****************************************************************************
#define TIMER_TAILR_TAILRH      0xFFFF0000  // TimerB load val in 32 bit mode
#define TIMER_TAILR_TAILRL      0x0000FFFF  // TimerA interval load value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TBILR
// register.
//
//*****************************************************************************
#define TIMER_TBILR_TBILRL      0x0000FFFF  // TimerB interval load value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// TIMER_TAMATCHR register.
//
//*****************************************************************************
#define TIMER_TAMATCHR_TAMRH    0xFFFF0000  // TimerB match val in 32 bit mode
#define TIMER_TAMATCHR_TAMRL    0x0000FFFF  // TimerA match value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// TIMER_TBMATCHR register.
//
//*****************************************************************************
#define TIMER_TBMATCHR_TBMRL    0x0000FFFF  // TimerB match load value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TnPR
// register.
//
//*****************************************************************************
#define TIMER_TNPR_TNPSR        0x000000FF  // TimerN prescale value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TnPMR
// register.
//
//*****************************************************************************
#define TIMER_TNPMR_TNPSMR      0x000000FF  // TimerN prescale match value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TAR
// register.
//
//*****************************************************************************
#define TIMER_TAR_TARH          0xFFFF0000  // TimerB val in 32 bit mode
#define TIMER_TAR_TARL          0x0000FFFF  // TimerA value

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the TIMER_TBR
// register.
//
//*****************************************************************************
#define TIMER_TBR_TBRL          0x0000FFFF  // TimerB value

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_ACTSS register.
//
//*****************************************************************************
#define ADC_ACTSS_ASEN3         0x00000008  // Sample sequence 3 enable
#define ADC_ACTSS_ASEN2         0x00000004  // Sample sequence 2 enable
#define ADC_ACTSS_ASEN1         0x00000002  // Sample sequence 1 enable
#define ADC_ACTSS_ASEN0         0x00000001  // Sample sequence 0 enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_RIS register.
//
//*****************************************************************************
#define ADC_RIS_INR3            0x00000008  // Sample sequence 3 interrupt
#define ADC_RIS_INR2            0x00000004  // Sample sequence 2 interrupt
#define ADC_RIS_INR1            0x00000002  // Sample sequence 1 interrupt
#define ADC_RIS_INR0            0x00000001  // Sample sequence 0 interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_IM register.
//
//*****************************************************************************
#define ADC_IM_MASK3            0x00000008  // Sample sequence 3 mask
#define ADC_IM_MASK2            0x00000004  // Sample sequence 2 mask
#define ADC_IM_MASK1            0x00000002  // Sample sequence 1 mask
#define ADC_IM_MASK0            0x00000001  // Sample sequence 0 mask

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_ISC register.
//
//*****************************************************************************
#define ADC_ISC_IN3             0x00000008  // Sample sequence 3 interrupt
#define ADC_ISC_IN2             0x00000004  // Sample sequence 2 interrupt
#define ADC_ISC_IN1             0x00000002  // Sample sequence 1 interrupt
#define ADC_ISC_IN0             0x00000001  // Sample sequence 0 interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_OSTAT register.
//
//*****************************************************************************
#define ADC_OSTAT_OV3           0x00000008  // Sample sequence 3 overflow
#define ADC_OSTAT_OV2           0x00000004  // Sample sequence 2 overflow
#define ADC_OSTAT_OV1           0x00000002  // Sample sequence 1 overflow
#define ADC_OSTAT_OV0           0x00000001  // Sample sequence 0 overflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_EMUX register.
//
//*****************************************************************************
#define ADC_EMUX_EM3_M          0x0000F000  // Event mux 3 mask
#define ADC_EMUX_EM3_PROCESSOR  0x00000000  // Processor event
#define ADC_EMUX_EM3_COMP0      0x00001000  // Analog comparator 0 event
#define ADC_EMUX_EM3_COMP1      0x00002000  // Analog comparator 1 event
#define ADC_EMUX_EM3_COMP2      0x00003000  // Analog comparator 2 event
#define ADC_EMUX_EM3_EXTERNAL   0x00004000  // External event
#define ADC_EMUX_EM3_TIMER      0x00005000  // Timer event
#define ADC_EMUX_EM3_PWM0       0x00006000  // PWM0 event
#define ADC_EMUX_EM3_PWM1       0x00007000  // PWM1 event
#define ADC_EMUX_EM3_PWM2       0x00008000  // PWM2 event
#define ADC_EMUX_EM3_ALWAYS     0x0000F000  // Always event
#define ADC_EMUX_EM2_M          0x00000F00  // Event mux 2 mask
#define ADC_EMUX_EM2_PROCESSOR  0x00000000  // Processor event
#define ADC_EMUX_EM2_COMP0      0x00000100  // Analog comparator 0 event
#define ADC_EMUX_EM2_COMP1      0x00000200  // Analog comparator 1 event
#define ADC_EMUX_EM2_COMP2      0x00000300  // Analog comparator 2 event
#define ADC_EMUX_EM2_EXTERNAL   0x00000400  // External event
#define ADC_EMUX_EM2_TIMER      0x00000500  // Timer event
#define ADC_EMUX_EM2_PWM0       0x00000600  // PWM0 event
#define ADC_EMUX_EM2_PWM1       0x00000700  // PWM1 event
#define ADC_EMUX_EM2_PWM2       0x00000800  // PWM2 event
#define ADC_EMUX_EM2_ALWAYS     0x00000F00  // Always event
#define ADC_EMUX_EM1_M          0x000000F0  // Event mux 1 mask
#define ADC_EMUX_EM1_PROCESSOR  0x00000000  // Processor event
#define ADC_EMUX_EM1_COMP0      0x00000010  // Analog comparator 0 event
#define ADC_EMUX_EM1_COMP1      0x00000020  // Analog comparator 1 event
#define ADC_EMUX_EM1_COMP2      0x00000030  // Analog comparator 2 event
#define ADC_EMUX_EM1_EXTERNAL   0x00000040  // External event
#define ADC_EMUX_EM1_TIMER      0x00000050  // Timer event
#define ADC_EMUX_EM1_PWM0       0x00000060  // PWM0 event
#define ADC_EMUX_EM1_PWM1       0x00000070  // PWM1 event
#define ADC_EMUX_EM1_PWM2       0x00000080  // PWM2 event
#define ADC_EMUX_EM1_ALWAYS     0x000000F0  // Always event
#define ADC_EMUX_EM0_M          0x0000000F  // Event mux 0 mask
#define ADC_EMUX_EM0_PROCESSOR  0x00000000  // Processor event
#define ADC_EMUX_EM0_COMP0      0x00000001  // Analog comparator 0 event
#define ADC_EMUX_EM0_COMP1      0x00000002  // Analog comparator 1 event
#define ADC_EMUX_EM0_COMP2      0x00000003  // Analog comparator 2 event
#define ADC_EMUX_EM0_EXTERNAL   0x00000004  // External event
#define ADC_EMUX_EM0_TIMER      0x00000005  // Timer event
#define ADC_EMUX_EM0_PWM0       0x00000006  // PWM0 event
#define ADC_EMUX_EM0_PWM1       0x00000007  // PWM1 event
#define ADC_EMUX_EM0_PWM2       0x00000008  // PWM2 event
#define ADC_EMUX_EM0_ALWAYS     0x0000000F  // Always event

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_USTAT register.
//
//*****************************************************************************
#define ADC_USTAT_UV3           0x00000008  // Sample sequence 3 underflow
#define ADC_USTAT_UV2           0x00000004  // Sample sequence 2 underflow
#define ADC_USTAT_UV1           0x00000002  // Sample sequence 1 underflow
#define ADC_USTAT_UV0           0x00000001  // Sample sequence 0 underflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_SSPRI register.
//
//*****************************************************************************
#define ADC_SSPRI_SS3_M         0x00003000  // Sequencer 3 priority mask
#define ADC_SSPRI_SS3_1ST       0x00000000  // First priority
#define ADC_SSPRI_SS3_2ND       0x00001000  // Second priority
#define ADC_SSPRI_SS3_3RD       0x00002000  // Third priority
#define ADC_SSPRI_SS3_4TH       0x00003000  // Fourth priority
#define ADC_SSPRI_SS2_M         0x00000300  // Sequencer 2 priority mask
#define ADC_SSPRI_SS2_1ST       0x00000000  // First priority
#define ADC_SSPRI_SS2_2ND       0x00000100  // Second priority
#define ADC_SSPRI_SS2_3RD       0x00000200  // Third priority
#define ADC_SSPRI_SS2_4TH       0x00000300  // Fourth priority
#define ADC_SSPRI_SS1_M         0x00000030  // Sequencer 1 priority mask
#define ADC_SSPRI_SS1_1ST       0x00000000  // First priority
#define ADC_SSPRI_SS1_2ND       0x00000010  // Second priority
#define ADC_SSPRI_SS1_3RD       0x00000020  // Third priority
#define ADC_SSPRI_SS1_4TH       0x00000030  // Fourth priority
#define ADC_SSPRI_SS0_M         0x00000003  // Sequencer 0 priority mask
#define ADC_SSPRI_SS0_1ST       0x00000000  // First priority
#define ADC_SSPRI_SS0_2ND       0x00000001  // Second priority
#define ADC_SSPRI_SS0_3RD       0x00000002  // Third priority
#define ADC_SSPRI_SS0_4TH       0x00000003  // Fourth priority

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_PSSI register.
//
//*****************************************************************************
#define ADC_PSSI_SS3            0x00000008  // Trigger sample sequencer 3
#define ADC_PSSI_SS2            0x00000004  // Trigger sample sequencer 2
#define ADC_PSSI_SS1            0x00000002  // Trigger sample sequencer 1
#define ADC_PSSI_SS0            0x00000001  // Trigger sample sequencer 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_SAC register.
//
//*****************************************************************************
#define ADC_SAC_AVG_M           0x00000007  // Hardware Averaging Control.
#define ADC_SAC_AVG_64X         0x00000006  // 64x hardware oversampling
#define ADC_SAC_AVG_32X         0x00000005  // 32x hardware oversampling
#define ADC_SAC_AVG_16X         0x00000004  // 16x hardware oversampling
#define ADC_SAC_AVG_8X          0x00000003  // 8x hardware oversampling
#define ADC_SAC_AVG_4X          0x00000002  // 4x hardware oversampling
#define ADC_SAC_AVG_2X          0x00000001  // 2x hardware oversampling
#define ADC_SAC_AVG_OFF         0x00000000  // No hardware oversampling

//*****************************************************************************
//
// The following are defines for the the interpretation of the data in the
// SSFIFOx when the ADC TMLB is enabled.
//
//*****************************************************************************
#define ADC_SSFIFO_TMLB_CNT_M   0x000003C0  // Continuous Sample Counter.
#define ADC_SSFIFO_TMLB_CONT    0x00000020  // Continuation Sample Indicator.
#define ADC_SSFIFO_TMLB_DIFF    0x00000010  // Differential Sample Indicator.
#define ADC_SSFIFO_TMLB_TS      0x00000008  // Temp Sensor Sample Indicator.
#define ADC_SSFIFO_TMLB_MUX_M   0x00000007  // Analog Input Indicator.
#define ADC_SSFIFO_TMLB_CNT_S   6           // Sample counter shift
#define ADC_SSFIFO_TMLB_MUX_S   0           // Input channel number shift

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_TMLB register.
//
//*****************************************************************************
#define ADC_TMLB_LB             0x00000001  // Loopback control signals

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX0 register.
//
//*****************************************************************************
#define ADC_SSMUX0_MUX7_M       0x70000000  // 8th Sample Input Select.
#define ADC_SSMUX0_MUX6_M       0x07000000  // 7th Sample Input Select.
#define ADC_SSMUX0_MUX5_M       0x00700000  // 6th Sample Input Select.
#define ADC_SSMUX0_MUX4_M       0x00070000  // 5th Sample Input Select.
#define ADC_SSMUX0_MUX3_M       0x00007000  // 4th Sample Input Select.
#define ADC_SSMUX0_MUX2_M       0x00000700  // 3rd Sample Input Select.
#define ADC_SSMUX0_MUX1_M       0x00000070  // 2nd Sample Input Select.
#define ADC_SSMUX0_MUX0_M       0x00000007  // 1st Sample Input Select.
#define ADC_SSMUX0_MUX7_S       28
#define ADC_SSMUX0_MUX6_S       24
#define ADC_SSMUX0_MUX5_S       20
#define ADC_SSMUX0_MUX4_S       16
#define ADC_SSMUX0_MUX3_S       12
#define ADC_SSMUX0_MUX2_S       8
#define ADC_SSMUX0_MUX1_S       4
#define ADC_SSMUX0_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL0 register.
//
//*****************************************************************************
#define ADC_SSCTL0_TS7          0x80000000  // 8th Sample Temp Sensor Select.
#define ADC_SSCTL0_IE7          0x40000000  // 8th Sample Interrupt Enable.
#define ADC_SSCTL0_END7         0x20000000  // 8th Sample is End of Sequence.
#define ADC_SSCTL0_D7           0x10000000  // 8th Sample Diff Input Select.
#define ADC_SSCTL0_TS6          0x08000000  // 7th Sample Temp Sensor Select.
#define ADC_SSCTL0_IE6          0x04000000  // 7th Sample Interrupt Enable.
#define ADC_SSCTL0_END6         0x02000000  // 7th Sample is End of Sequence.
#define ADC_SSCTL0_D6           0x01000000  // 7th Sample Diff Input Select.
#define ADC_SSCTL0_TS5          0x00800000  // 6th Sample Temp Sensor Select.
#define ADC_SSCTL0_IE5          0x00400000  // 6th Sample Interrupt Enable.
#define ADC_SSCTL0_END5         0x00200000  // 6th Sample is End of Sequence.
#define ADC_SSCTL0_D5           0x00100000  // 6th Sample Diff Input Select.
#define ADC_SSCTL0_TS4          0x00080000  // 5th Sample Temp Sensor Select.
#define ADC_SSCTL0_IE4          0x00040000  // 5th Sample Interrupt Enable.
#define ADC_SSCTL0_END4         0x00020000  // 5th Sample is End of Sequence.
#define ADC_SSCTL0_D4           0x00010000  // 5th Sample Diff Input Select.
#define ADC_SSCTL0_TS3          0x00008000  // 4th Sample Temp Sensor Select.
#define ADC_SSCTL0_IE3          0x00004000  // 4th Sample Interrupt Enable.
#define ADC_SSCTL0_END3         0x00002000  // 4th Sample is End of Sequence.
#define ADC_SSCTL0_D3           0x00001000  // 4th Sample Diff Input Select.
#define ADC_SSCTL0_TS2          0x00000800  // 3rd Sample Temp Sensor Select.
#define ADC_SSCTL0_IE2          0x00000400  // 3rd Sample Interrupt Enable.
#define ADC_SSCTL0_END2         0x00000200  // 3rd Sample is End of Sequence.
#define ADC_SSCTL0_D2           0x00000100  // 3rd Sample Diff Input Select.
#define ADC_SSCTL0_TS1          0x00000080  // 2nd Sample Temp Sensor Select.
#define ADC_SSCTL0_IE1          0x00000040  // 2nd Sample Interrupt Enable.
#define ADC_SSCTL0_END1         0x00000020  // 2nd Sample is End of Sequence.
#define ADC_SSCTL0_D1           0x00000010  // 2nd Sample Diff Input Select.
#define ADC_SSCTL0_TS0          0x00000008  // 1st Sample Temp Sensor Select.
#define ADC_SSCTL0_IE0          0x00000004  // 1st Sample Interrupt Enable.
#define ADC_SSCTL0_END0         0x00000002  // 1st Sample is End of Sequence.
#define ADC_SSCTL0_D0           0x00000001  // 1st Sample Diff Input Select.

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO0 register.
//
//*****************************************************************************
#define ADC_SSFIFO0_DATA_M      0x000003FF  // Conversion Result Data.
#define ADC_SSFIFO0_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT0 register.
//
//*****************************************************************************
#define ADC_SSFSTAT0_FULL       0x00001000  // FIFO Full.
#define ADC_SSFSTAT0_EMPTY      0x00000100  // FIFO Empty.
#define ADC_SSFSTAT0_HPTR_M     0x000000F0  // FIFO Head Pointer.
#define ADC_SSFSTAT0_TPTR_M     0x0000000F  // FIFO Tail Pointer.
#define ADC_SSFSTAT0_HPTR_S     4
#define ADC_SSFSTAT0_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX1 register.
//
//*****************************************************************************
#define ADC_SSMUX1_MUX3_M       0x00007000  // 4th Sample Input Select.
#define ADC_SSMUX1_MUX2_M       0x00000700  // 3rd Sample Input Select.
#define ADC_SSMUX1_MUX1_M       0x00000070  // 2nd Sample Input Select.
#define ADC_SSMUX1_MUX0_M       0x00000007  // 1st Sample Input Select.
#define ADC_SSMUX1_MUX3_S       12
#define ADC_SSMUX1_MUX2_S       8
#define ADC_SSMUX1_MUX1_S       4
#define ADC_SSMUX1_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL1 register.
//
//*****************************************************************************
#define ADC_SSCTL1_TS3          0x00008000  // 4th Sample Temp Sensor Select.
#define ADC_SSCTL1_IE3          0x00004000  // 4th Sample Interrupt Enable.
#define ADC_SSCTL1_END3         0x00002000  // 4th Sample is End of Sequence.
#define ADC_SSCTL1_D3           0x00001000  // 4th Sample Diff Input Select.
#define ADC_SSCTL1_TS2          0x00000800  // 3rd Sample Temp Sensor Select.
#define ADC_SSCTL1_IE2          0x00000400  // 3rd Sample Interrupt Enable.
#define ADC_SSCTL1_END2         0x00000200  // 3rd Sample is End of Sequence.
#define ADC_SSCTL1_D2           0x00000100  // 3rd Sample Diff Input Select.
#define ADC_SSCTL1_TS1          0x00000080  // 2nd Sample Temp Sensor Select.
#define ADC_SSCTL1_IE1          0x00000040  // 2nd Sample Interrupt Enable.
#define ADC_SSCTL1_END1         0x00000020  // 2nd Sample is End of Sequence.
#define ADC_SSCTL1_D1           0x00000010  // 2nd Sample Diff Input Select.
#define ADC_SSCTL1_TS0          0x00000008  // 1st Sample Temp Sensor Select.
#define ADC_SSCTL1_IE0          0x00000004  // 1st Sample Interrupt Enable.
#define ADC_SSCTL1_END0         0x00000002  // 1st Sample is End of Sequence.
#define ADC_SSCTL1_D0           0x00000001  // 1st Sample Diff Input Select.

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO1 register.
//
//*****************************************************************************
#define ADC_SSFIFO1_DATA_M      0x000003FF  // Conversion Result Data.
#define ADC_SSFIFO1_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT1 register.
//
//*****************************************************************************
#define ADC_SSFSTAT1_FULL       0x00001000  // FIFO Full.
#define ADC_SSFSTAT1_EMPTY      0x00000100  // FIFO Empty.
#define ADC_SSFSTAT1_HPTR_M     0x000000F0  // FIFO Head Pointer.
#define ADC_SSFSTAT1_TPTR_M     0x0000000F  // FIFO Tail Pointer.
#define ADC_SSFSTAT1_HPTR_S     4
#define ADC_SSFSTAT1_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX2 register.
//
//*****************************************************************************
#define ADC_SSMUX2_MUX3_M       0x00007000  // 4th Sample Input Select.
#define ADC_SSMUX2_MUX2_M       0x00000700  // 3rd Sample Input Select.
#define ADC_SSMUX2_MUX1_M       0x00000070  // 2nd Sample Input Select.
#define ADC_SSMUX2_MUX0_M       0x00000007  // 1st Sample Input Select.
#define ADC_SSMUX2_MUX3_S       12
#define ADC_SSMUX2_MUX2_S       8
#define ADC_SSMUX2_MUX1_S       4
#define ADC_SSMUX2_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL2 register.
//
//*****************************************************************************
#define ADC_SSCTL2_TS3          0x00008000  // 4th Sample Temp Sensor Select.
#define ADC_SSCTL2_IE3          0x00004000  // 4th Sample Interrupt Enable.
#define ADC_SSCTL2_END3         0x00002000  // 4th Sample is End of Sequence.
#define ADC_SSCTL2_D3           0x00001000  // 4th Sample Diff Input Select.
#define ADC_SSCTL2_TS2          0x00000800  // 3rd Sample Temp Sensor Select.
#define ADC_SSCTL2_IE2          0x00000400  // 3rd Sample Interrupt Enable.
#define ADC_SSCTL2_END2         0x00000200  // 3rd Sample is End of Sequence.
#define ADC_SSCTL2_D2           0x00000100  // 3rd Sample Diff Input Select.
#define ADC_SSCTL2_TS1          0x00000080  // 2nd Sample Temp Sensor Select.
#define ADC_SSCTL2_IE1          0x00000040  // 2nd Sample Interrupt Enable.
#define ADC_SSCTL2_END1         0x00000020  // 2nd Sample is End of Sequence.
#define ADC_SSCTL2_D1           0x00000010  // 2nd Sample Diff Input Select.
#define ADC_SSCTL2_TS0          0x00000008  // 1st Sample Temp Sensor Select.
#define ADC_SSCTL2_IE0          0x00000004  // 1st Sample Interrupt Enable.
#define ADC_SSCTL2_END0         0x00000002  // 1st Sample is End of Sequence.
#define ADC_SSCTL2_D0           0x00000001  // 1st Sample Diff Input Select.

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO2 register.
//
//*****************************************************************************
#define ADC_SSFIFO2_DATA_M      0x000003FF  // Conversion Result Data.
#define ADC_SSFIFO2_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT2 register.
//
//*****************************************************************************
#define ADC_SSFSTAT2_FULL       0x00001000  // FIFO Full.
#define ADC_SSFSTAT2_EMPTY      0x00000100  // FIFO Empty.
#define ADC_SSFSTAT2_HPTR_M     0x000000F0  // FIFO Head Pointer.
#define ADC_SSFSTAT2_TPTR_M     0x0000000F  // FIFO Tail Pointer.
#define ADC_SSFSTAT2_HPTR_S     4
#define ADC_SSFSTAT2_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX3 register.
//
//*****************************************************************************
#define ADC_SSMUX3_MUX0_M       0x00000007  // 1st Sample Input Select.
#define ADC_SSMUX3_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL3 register.
//
//*****************************************************************************
#define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select.
#define ADC_SSCTL3_IE0          0x00000004  // 1st Sample Interrupt Enable.
#define ADC_SSCTL3_END0         0x00000002  // 1st Sample is End of Sequence.
#define ADC_SSCTL3_D0           0x00000001  // 1st Sample Diff Input Select.

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO3 register.
//
//*****************************************************************************
#define ADC_SSFIFO3_DATA_M      0x000003FF  // Conversion Result Data.
#define ADC_SSFIFO3_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT3 register.
//
//*****************************************************************************
#define ADC_SSFSTAT3_FULL       0x00001000  // FIFO Full.
#define ADC_SSFSTAT3_EMPTY      0x00000100  // FIFO Empty.
#define ADC_SSFSTAT3_HPTR_M     0x000000F0  // FIFO Head Pointer.
#define ADC_SSFSTAT3_TPTR_M     0x0000000F  // FIFO Tail Pointer.
#define ADC_SSFSTAT3_HPTR_S     4
#define ADC_SSFSTAT3_TPTR_S     0

//*****************************************************************************
//
// The following are deprecated defines for the ADC sequence register offsets.
//
//*****************************************************************************
#define ADC_O_SEQ               0x00000040  // Offset to the first sequence
#define ADC_O_SEQ_STEP          0x00000020  // Increment to the next sequence
#define ADC_O_X_SSFSTAT         0x0000000C  // FIFO status register
#define ADC_O_X_SSFIFO          0x00000008  // Result FIFO register
#define ADC_O_X_SSCTL           0x00000004  // Sample sequence control register
#define ADC_O_X_SSMUX           0x00000000  // Multiplexer select register

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the ADC_EMUX
// register.
//
//*****************************************************************************
#define ADC_EMUX_EM3_MASK       0x0000F000  // Event mux 3 mask
#define ADC_EMUX_EM2_MASK       0x00000F00  // Event mux 2 mask
#define ADC_EMUX_EM1_MASK       0x000000F0  // Event mux 1 mask
#define ADC_EMUX_EM0_MASK       0x0000000F  // Event mux 0 mask
#define ADC_EMUX_EM3_SHIFT      12          // The shift for the fourth event
#define ADC_EMUX_EM2_SHIFT      8           // The shift for the third event
#define ADC_EMUX_EM1_SHIFT      4           // The shift for the second event
#define ADC_EMUX_EM0_SHIFT      0           // The shift for the first event

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the ADC_SSPRI
// register.
//
//*****************************************************************************
#define ADC_SSPRI_SS3_MASK      0x00003000  // Sequencer 3 priority mask
#define ADC_SSPRI_SS2_MASK      0x00000300  // Sequencer 2 priority mask
#define ADC_SSPRI_SS1_MASK      0x00000030  // Sequencer 1 priority mask
#define ADC_SSPRI_SS0_MASK      0x00000003  // Sequencer 0 priority mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the ADC_SSMUX0,
// ADC_SSMUX1, ADC_SSMUX2, and ADC_SSMUX3 registers. Not all fields are present
// in all registers.
//
//*****************************************************************************
#define ADC_SSMUX_MUX7_MASK     0x70000000  // 8th mux select mask
#define ADC_SSMUX_MUX6_MASK     0x07000000  // 7th mux select mask
#define ADC_SSMUX_MUX5_MASK     0x00700000  // 6th mux select mask
#define ADC_SSMUX_MUX4_MASK     0x00070000  // 5th mux select mask
#define ADC_SSMUX_MUX3_MASK     0x00007000  // 4th mux select mask
#define ADC_SSMUX_MUX2_MASK     0x00000700  // 3rd mux select mask
#define ADC_SSMUX_MUX1_MASK     0x00000070  // 2nd mux select mask
#define ADC_SSMUX_MUX0_MASK     0x00000007  // 1st mux select mask
#define ADC_SSMUX_MUX7_SHIFT    28
#define ADC_SSMUX_MUX6_SHIFT    24
#define ADC_SSMUX_MUX5_SHIFT    20
#define ADC_SSMUX_MUX4_SHIFT    16
#define ADC_SSMUX_MUX3_SHIFT    12
#define ADC_SSMUX_MUX2_SHIFT    8
#define ADC_SSMUX_MUX1_SHIFT    4
#define ADC_SSMUX_MUX0_SHIFT    0

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the ADC_SSCTL0,
// ADC_SSCTL1, ADC_SSCTL2, and ADC_SSCTL3 registers. Not all fields are present
// in all registers.
//
//*****************************************************************************
#define ADC_SSCTL_TS7           0x80000000  // 8th temperature sensor select
#define ADC_SSCTL_IE7           0x40000000  // 8th interrupt enable
#define ADC_SSCTL_END7          0x20000000  // 8th sequence end select
#define ADC_SSCTL_D7            0x10000000  // 8th differential select
#define ADC_SSCTL_TS6           0x08000000  // 7th temperature sensor select
#define ADC_SSCTL_IE6           0x04000000  // 7th interrupt enable
#define ADC_SSCTL_END6          0x02000000  // 7th sequence end select
#define ADC_SSCTL_D6            0x01000000  // 7th differential select
#define ADC_SSCTL_TS5           0x00800000  // 6th temperature sensor select
#define ADC_SSCTL_IE5           0x00400000  // 6th interrupt enable
#define ADC_SSCTL_END5          0x00200000  // 6th sequence end select
#define ADC_SSCTL_D5            0x00100000  // 6th differential select
#define ADC_SSCTL_TS4           0x00080000  // 5th temperature sensor select
#define ADC_SSCTL_IE4           0x00040000  // 5th interrupt enable
#define ADC_SSCTL_END4          0x00020000  // 5th sequence end select
#define ADC_SSCTL_D4            0x00010000  // 5th differential select
#define ADC_SSCTL_TS3           0x00008000  // 4th temperature sensor select
#define ADC_SSCTL_IE3           0x00004000  // 4th interrupt enable
#define ADC_SSCTL_END3          0x00002000  // 4th sequence end select
#define ADC_SSCTL_D3            0x00001000  // 4th differential select
#define ADC_SSCTL_TS2           0x00000800  // 3rd temperature sensor select
#define ADC_SSCTL_IE2           0x00000400  // 3rd interrupt enable
#define ADC_SSCTL_END2          0x00000200  // 3rd sequence end select
#define ADC_SSCTL_D2            0x00000100  // 3rd differential select
#define ADC_SSCTL_TS1           0x00000080  // 2nd temperature sensor select
#define ADC_SSCTL_IE1           0x00000040  // 2nd interrupt enable
#define ADC_SSCTL_END1          0x00000020  // 2nd sequence end select
#define ADC_SSCTL_D1            0x00000010  // 2nd differential select
#define ADC_SSCTL_TS0           0x00000008  // 1st temperature sensor select
#define ADC_SSCTL_IE0           0x00000004  // 1st interrupt enable
#define ADC_SSCTL_END0          0x00000002  // 1st sequence end select
#define ADC_SSCTL_D0            0x00000001  // 1st differential select

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the ADC_SSFIFO0,
// ADC_SSFIFO1, ADC_SSFIFO2, and ADC_SSFIFO3 registers.
//
//*****************************************************************************
#define ADC_SSFIFO_DATA_MASK    0x000003FF  // Sample data
#define ADC_SSFIFO_DATA_SHIFT   0

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the ADC_SSFSTAT0,
// ADC_SSFSTAT1, ADC_SSFSTAT2, and ADC_SSFSTAT3 registers.
//
//*****************************************************************************
#define ADC_SSFSTAT_FULL        0x00001000  // FIFO is full
#define ADC_SSFSTAT_EMPTY       0x00000100  // FIFO is empty
#define ADC_SSFSTAT_HPTR        0x000000F0  // FIFO head pointer
#define ADC_SSFSTAT_TPTR        0x0000000F  // FIFO tail pointer

//*****************************************************************************
//
// The following are deprecated defines for the the interpretation of the data
// in the SSFIFOx when the ADC TMLB is enabled.
//
//*****************************************************************************
#define ADC_TMLB_CNT_M          0x000003C0  // Continuous Sample Counter.
#define ADC_TMLB_CONT           0x00000020  // Continuation Sample Indicator.
#define ADC_TMLB_DIFF           0x00000010  // Differential Sample Indicator.
#define ADC_TMLB_TS             0x00000008  // Temp Sensor Sample Indicator.
#define ADC_TMLB_MUX_M          0x00000007  // Analog Input Indicator.
#define ADC_TMLB_CNT_S          6           // Sample counter shift
#define ADC_TMLB_MUX_S          0           // Input channel number shift

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the loopback ADC
// data.
//
//*****************************************************************************
#define ADC_LB_CNT_MASK         0x000003C0  // Sample counter mask
#define ADC_LB_CONT             0x00000020  // Continuation sample
#define ADC_LB_DIFF             0x00000010  // Differential sample
#define ADC_LB_TS               0x00000008  // Temperature sensor sample
#define ADC_LB_MUX_MASK         0x00000007  // Input channel number mask
#define ADC_LB_CNT_SHIFT        6           // Sample counter shift
#define ADC_LB_MUX_SHIFT        0           // Input channel number shift

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACMIS register.
//
//*****************************************************************************
#define COMP_ACMIS_IN2          0x00000004  // Comparator 2 Masked Interrupt
                                            // Status.
#define COMP_ACMIS_IN1          0x00000002  // Comparator 1 Masked Interrupt
                                            // Status.
#define COMP_ACMIS_IN0          0x00000001  // Comparator 0 Masked Interrupt
                                            // Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACRIS register.
//
//*****************************************************************************
#define COMP_ACRIS_IN2          0x00000004  // Comparator 2 Interrupt Status.
#define COMP_ACRIS_IN1          0x00000002  // Comparator 1 Interrupt Status.
#define COMP_ACRIS_IN0          0x00000001  // Comparator 0 Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACINTEN register.
//
//*****************************************************************************
#define COMP_ACINTEN_IN2        0x00000004  // Comparator 2 Interrupt Enable.
#define COMP_ACINTEN_IN1        0x00000002  // Comparator 1 Interrupt Enable.
#define COMP_ACINTEN_IN0        0x00000001  // Comparator 0 Interrupt Enable.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACREFCTL
// register.
//
//*****************************************************************************
#define COMP_ACREFCTL_EN        0x00000200  // Resistor Ladder Enable.
#define COMP_ACREFCTL_RNG       0x00000100  // Resistor Ladder Range.
#define COMP_ACREFCTL_VREF_M    0x0000000F  // Resistor Ladder Voltage Ref.
#define COMP_ACREFCTL_VREF_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT0 register.
//
//*****************************************************************************
#define COMP_ACSTAT0_OVAL       0x00000002  // Comparator Output Value.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL0 register.
//
//*****************************************************************************
#define COMP_ACCTL0_TOEN        0x00000800  // Trigger Output Enable.
#define COMP_ACCTL0_ASRCP_M     0x00000600  // Analog Source Positive.
#define COMP_ACCTL0_ASRCP_PIN   0x00000000  // Pin value
#define COMP_ACCTL0_ASRCP_PIN0  0x00000200  // Pin value of C0+
#define COMP_ACCTL0_ASRCP_REF   0x00000400  // Internal voltage reference
#define COMP_ACCTL0_TSLVAL      0x00000080  // Trigger Sense Level Value.
#define COMP_ACCTL0_TSEN_M      0x00000060  // Trigger Sense.
#define COMP_ACCTL0_TSEN_LEVEL  0x00000000  // Level sense, see TSLVAL
#define COMP_ACCTL0_TSEN_FALL   0x00000020  // Falling edge
#define COMP_ACCTL0_TSEN_RISE   0x00000040  // Rising edge
#define COMP_ACCTL0_TSEN_BOTH   0x00000060  // Either edge
#define COMP_ACCTL0_ISLVAL      0x00000010  // Interrupt Sense Level Value.
#define COMP_ACCTL0_ISEN_M      0x0000000C  // Interrupt Sense.
#define COMP_ACCTL0_ISEN_LEVEL  0x00000000  // Level sense, see ISLVAL
#define COMP_ACCTL0_ISEN_FALL   0x00000004  // Falling edge
#define COMP_ACCTL0_ISEN_RISE   0x00000008  // Rising edge
#define COMP_ACCTL0_ISEN_BOTH   0x0000000C  // Either edge
#define COMP_ACCTL0_CINV        0x00000002  // Comparator Output Invert.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT1 register.
//
//*****************************************************************************
#define COMP_ACSTAT1_OVAL       0x00000002  // Comparator Output Value.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL1 register.
//
//*****************************************************************************
#define COMP_ACCTL1_TOEN        0x00000800  // Trigger Output Enable.
#define COMP_ACCTL1_ASRCP_M     0x00000600  // Analog Source Positive.
#define COMP_ACCTL1_ASRCP_PIN   0x00000000  // Pin value
#define COMP_ACCTL1_ASRCP_PIN0  0x00000200  // Pin value of C0+
#define COMP_ACCTL1_ASRCP_REF   0x00000400  // Internal voltage reference
#define COMP_ACCTL1_TSLVAL      0x00000080  // Trigger Sense Level Value.
#define COMP_ACCTL1_TSEN_M      0x00000060  // Trigger Sense.
#define COMP_ACCTL1_TSEN_LEVEL  0x00000000  // Level sense, see TSLVAL
#define COMP_ACCTL1_TSEN_FALL   0x00000020  // Falling edge
#define COMP_ACCTL1_TSEN_RISE   0x00000040  // Rising edge
#define COMP_ACCTL1_TSEN_BOTH   0x00000060  // Either edge
#define COMP_ACCTL1_ISLVAL      0x00000010  // Interrupt Sense Level Value.
#define COMP_ACCTL1_ISEN_M      0x0000000C  // Interrupt Sense.
#define COMP_ACCTL1_ISEN_LEVEL  0x00000000  // Level sense, see ISLVAL
#define COMP_ACCTL1_ISEN_FALL   0x00000004  // Falling edge
#define COMP_ACCTL1_ISEN_RISE   0x00000008  // Rising edge
#define COMP_ACCTL1_ISEN_BOTH   0x0000000C  // Either edge
#define COMP_ACCTL1_CINV        0x00000002  // Comparator Output Invert.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT2 register.
//
//*****************************************************************************
#define COMP_ACSTAT2_OVAL       0x00000002  // Comparator Output Value.

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL2 register.
//
//*****************************************************************************
#define COMP_ACCTL2_TOEN        0x00000800  // Trigger Output Enable.
#define COMP_ACCTL2_ASRCP_M     0x00000600  // Analog Source Positive.
#define COMP_ACCTL2_ASRCP_PIN   0x00000000  // Pin value
#define COMP_ACCTL2_ASRCP_PIN0  0x00000200  // Pin value of C0+
#define COMP_ACCTL2_ASRCP_REF   0x00000400  // Internal voltage reference
#define COMP_ACCTL2_TSLVAL      0x00000080  // Trigger Sense Level Value.
#define COMP_ACCTL2_TSEN_M      0x00000060  // Trigger Sense.
#define COMP_ACCTL2_TSEN_LEVEL  0x00000000  // Level sense, see TSLVAL
#define COMP_ACCTL2_TSEN_FALL   0x00000020  // Falling edge
#define COMP_ACCTL2_TSEN_RISE   0x00000040  // Rising edge
#define COMP_ACCTL2_TSEN_BOTH   0x00000060  // Either edge
#define COMP_ACCTL2_ISLVAL      0x00000010  // Interrupt Sense Level Value.
#define COMP_ACCTL2_ISEN_M      0x0000000C  // Interrupt Sense.
#define COMP_ACCTL2_ISEN_LEVEL  0x00000000  // Level sense, see ISLVAL
#define COMP_ACCTL2_ISEN_FALL   0x00000004  // Falling edge
#define COMP_ACCTL2_ISEN_RISE   0x00000008  // Rising edge
#define COMP_ACCTL2_ISEN_BOTH   0x0000000C  // Either edge
#define COMP_ACCTL2_CINV        0x00000002  // Comparator Output Invert.

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_CTL register.
//
//*****************************************************************************
#define CAN_CTL_TEST            0x00000080  // Test mode enable
#define CAN_CTL_CCE             0x00000040  // Configuration change enable
#define CAN_CTL_DAR             0x00000020  // Disable automatic retransmission
#define CAN_CTL_EIE             0x00000008  // Error interrupt enable
#define CAN_CTL_SIE             0x00000004  // Status change interrupt enable
#define CAN_CTL_IE              0x00000002  // Module interrupt enable
#define CAN_CTL_INIT            0x00000001  // Initialization

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_STS register.
//
//*****************************************************************************
#define CAN_STS_BOFF            0x00000080  // Bus Off status
#define CAN_STS_EWARN           0x00000040  // Error Warning status
#define CAN_STS_EPASS           0x00000020  // Error Passive status
#define CAN_STS_RXOK            0x00000010  // Received Message Successful
#define CAN_STS_TXOK            0x00000008  // Transmitted Message Successful
#define CAN_STS_LEC_M           0x00000007  // Last Error Code
#define CAN_STS_LEC_NONE        0x00000000  // No error
#define CAN_STS_LEC_STUFF       0x00000001  // Stuff error
#define CAN_STS_LEC_FORM        0x00000002  // Form(at) error
#define CAN_STS_LEC_ACK         0x00000003  // Ack error
#define CAN_STS_LEC_BIT1        0x00000004  // Bit 1 error
#define CAN_STS_LEC_BIT0        0x00000005  // Bit 0 error
#define CAN_STS_LEC_CRC         0x00000006  // CRC error
#define CAN_STS_LEC_NOEVENT     0x00000007  // Unused

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_ERR register.
//
//*****************************************************************************
#define CAN_ERR_RP              0x00008000  // Receive error passive status
#define CAN_ERR_REC_M           0x00007F00  // Receive Error Counter.
#define CAN_ERR_TEC_M           0x000000FF  // Transmit Error Counter.
#define CAN_ERR_REC_S           8           // Receive error counter bit pos
#define CAN_ERR_TEC_S           0           // Transmit error counter bit pos

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_BIT register.
//
//*****************************************************************************
#define CAN_BIT_TSEG2_M         0x00007000  // Time Segment after Sample Point.
#define CAN_BIT_TSEG1_M         0x00000F00  // Time Segment Before Sample
                                            // Point.
#define CAN_BIT_SJW_M           0x000000C0  // (Re)Synchronization Jump Width.
#define CAN_BIT_BRP_M           0x0000003F  // Baud Rate Prescalar.
#define CAN_BIT_TSEG2_S         12
#define CAN_BIT_TSEG1_S         8
#define CAN_BIT_SJW_S           6
#define CAN_BIT_BRP_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_INT register.
//
//*****************************************************************************
#define CAN_INT_INTID_M         0x0000FFFF  // Interrupt Identifier.
#define CAN_INT_INTID_NONE      0x00000000  // No Interrupt Pending
#define CAN_INT_INTID_STATUS    0x00008000  // Status Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_TST register.
//
//*****************************************************************************
#define CAN_TST_RX              0x00000080  // CAN_RX pin status
#define CAN_TST_TX_M            0x00000060  // Overide control of CAN_TX pin
#define CAN_TST_TX_CANCTL       0x00000000  // CAN core controls CAN_TX
#define CAN_TST_TX_SAMPLE       0x00000020  // Sample Point on CAN_TX
#define CAN_TST_TX_DOMINANT     0x00000040  // Dominant value on CAN_TX
#define CAN_TST_TX_RECESSIVE    0x00000060  // Recessive value on CAN_TX
#define CAN_TST_LBACK           0x00000010  // Loop back mode
#define CAN_TST_SILENT          0x00000008  // Silent mode
#define CAN_TST_BASIC           0x00000004  // Basic mode

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_BRPE register.
//
//*****************************************************************************
#define CAN_BRPE_BRPE_M         0x0000000F  // Baud Rate Prescalar Extension.
#define CAN_BRPE_BRPE_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_TXRQ1 register.
//
//*****************************************************************************
#define CAN_TXRQ1_TXRQST_M      0x0000FFFF  // Transmission Request Bits.
#define CAN_TXRQ1_TXRQST_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_TXRQ2 register.
//
//*****************************************************************************
#define CAN_TXRQ2_TXRQST_M      0x0000FFFF  // Transmission Request Bits.
#define CAN_TXRQ2_TXRQST_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_NWDA1 register.
//
//*****************************************************************************
#define CAN_NWDA1_NEWDAT_M      0x0000FFFF  // New Data Bits.
#define CAN_NWDA1_NEWDAT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_NWDA2 register.
//
//*****************************************************************************
#define CAN_NWDA2_NEWDAT_M      0x0000FFFF  // New Data Bits.
#define CAN_NWDA2_NEWDAT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CRQ register.
//
//*****************************************************************************
#define CAN_IF1CRQ_BUSY         0x00008000  // Busy Flag.
#define CAN_IF1CRQ_MNUM_M       0x0000003F  // Message Number.
#define CAN_IF1CRQ_MNUM_RSVD    0x00000000  // 0 is not a valid message number;
                                            // it is interpreted as 0x20, or
                                            // object 32.

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CMSK register.
//
//*****************************************************************************
#define CAN_IF1CMSK_WRNRD       0x00000080  // Write, Not Read.
#define CAN_IF1CMSK_MASK        0x00000040  // Access Mask Bits.
#define CAN_IF1CMSK_ARB         0x00000020  // Access Arbitration Bits.
#define CAN_IF1CMSK_CONTROL     0x00000010  // Access Control Bits.
#define CAN_IF1CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit.
#define CAN_IF1CMSK_NEWDAT      0x00000004  // Access New Data.
#define CAN_IF1CMSK_TXRQST      0x00000004  // Access Transmission Request.
#define CAN_IF1CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3.
#define CAN_IF1CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7.

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK1 register.
//
//*****************************************************************************
#define CAN_IF1MSK1_IDMSK_M     0x0000FFFF  // Identifier Mask.
#define CAN_IF1MSK1_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK2 register.
//
//*****************************************************************************
#define CAN_IF1MSK2_MXTD        0x00008000  // Mask Extended Identifier.
#define CAN_IF1MSK2_MDIR        0x00004000  // Mask Message Direction.
#define CAN_IF1MSK2_IDMSK_M     0x00001FFF  // Identifier Mask.
#define CAN_IF1MSK2_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB1 register.
//
//*****************************************************************************
#define CAN_IF1ARB1_ID_M        0x0000FFFF  // Message Identifier.
#define CAN_IF1ARB1_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB2 register.
//
//*****************************************************************************
#define CAN_IF1ARB2_MSGVAL      0x00008000  // Message Valid.
#define CAN_IF1ARB2_XTD         0x00004000  // Extended Identifier.
#define CAN_IF1ARB2_DIR         0x00002000  // Message Direction.
#define CAN_IF1ARB2_ID_M        0x00001FFF  // Message Identifier.
#define CAN_IF1ARB2_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MCTL register.
//
//*****************************************************************************
#define CAN_IF1MCTL_NEWDAT      0x00008000  // New Data.
#define CAN_IF1MCTL_MSGLST      0x00004000  // Message Lost.
#define CAN_IF1MCTL_INTPND      0x00002000  // Interrupt Pending.
#define CAN_IF1MCTL_UMASK       0x00001000  // Use Acceptance Mask.
#define CAN_IF1MCTL_TXIE        0x00000800  // Transmit Interrupt Enable.
#define CAN_IF1MCTL_RXIE        0x00000400  // Receive Interrupt Enable.
#define CAN_IF1MCTL_RMTEN       0x00000200  // Remote Enable.
#define CAN_IF1MCTL_TXRQST      0x00000100  // Transmit Request.
#define CAN_IF1MCTL_EOB         0x00000080  // End of Buffer.
#define CAN_IF1MCTL_DLC_M       0x0000000F  // Data Length Code.
#define CAN_IF1MCTL_DLC_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA1 register.
//
//*****************************************************************************
#define CAN_IF1DA1_DATA_M       0x0000FFFF  // Data.
#define CAN_IF1DA1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA2 register.
//
//*****************************************************************************
#define CAN_IF1DA2_DATA_M       0x0000FFFF  // Data.
#define CAN_IF1DA2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB1 register.
//
//*****************************************************************************
#define CAN_IF1DB1_DATA_M       0x0000FFFF  // Data.
#define CAN_IF1DB1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB2 register.
//
//*****************************************************************************
#define CAN_IF1DB2_DATA_M       0x0000FFFF  // Data.
#define CAN_IF1DB2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CRQ register.
//
//*****************************************************************************
#define CAN_IF2CRQ_BUSY         0x00008000  // Busy Flag.
#define CAN_IF2CRQ_MNUM_M       0x0000003F  // Message Number.
#define CAN_IF2CRQ_MNUM_RSVD    0x00000000  // 0 is not a valid message number;
                                            // it is interpreted as 0x20, or
                                            // object 32.

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CMSK register.
//
//*****************************************************************************
#define CAN_IF2CMSK_WRNRD       0x00000080  // Write, Not Read.
#define CAN_IF2CMSK_MASK        0x00000040  // Access Mask Bits.
#define CAN_IF2CMSK_ARB         0x00000020  // Access Arbitration Bits.
#define CAN_IF2CMSK_CONTROL     0x00000010  // Access Control Bits.
#define CAN_IF2CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit.
#define CAN_IF2CMSK_NEWDAT      0x00000004  // Access New Data.
#define CAN_IF2CMSK_TXRQST      0x00000004  // Access Transmission Request.
#define CAN_IF2CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3.
#define CAN_IF2CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7.

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK1 register.
//
//*****************************************************************************
#define CAN_IF2MSK1_IDMSK_M     0x0000FFFF  // Identifier Mask.
#define CAN_IF2MSK1_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK2 register.
//
//*****************************************************************************
#define CAN_IF2MSK2_MXTD        0x00008000  // Mask Extended Identifier.
#define CAN_IF2MSK2_MDIR        0x00004000  // Mask Message Direction.
#define CAN_IF2MSK2_IDMSK_M     0x00001FFF  // Identifier Mask.
#define CAN_IF2MSK2_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB1 register.
//
//*****************************************************************************
#define CAN_IF2ARB1_ID_M        0x0000FFFF  // Message Identifier.
#define CAN_IF2ARB1_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB2 register.
//
//*****************************************************************************
#define CAN_IF2ARB2_MSGVAL      0x00008000  // Message Valid.
#define CAN_IF2ARB2_XTD         0x00004000  // Extended Identifier.
#define CAN_IF2ARB2_DIR         0x00002000  // Message Direction.
#define CAN_IF2ARB2_ID_M        0x00001FFF  // Message Identifier.
#define CAN_IF2ARB2_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MCTL register.
//
//*****************************************************************************
#define CAN_IF2MCTL_NEWDAT      0x00008000  // New Data.
#define CAN_IF2MCTL_MSGLST      0x00004000  // Message Lost.
#define CAN_IF2MCTL_INTPND      0x00002000  // Interrupt Pending.
#define CAN_IF2MCTL_UMASK       0x00001000  // Use Acceptance Mask.
#define CAN_IF2MCTL_TXIE        0x00000800  // Transmit Interrupt Enable.
#define CAN_IF2MCTL_RXIE        0x00000400  // Receive Interrupt Enable.
#define CAN_IF2MCTL_RMTEN       0x00000200  // Remote Enable.
#define CAN_IF2MCTL_TXRQST      0x00000100  // Transmit Request.
#define CAN_IF2MCTL_EOB         0x00000080  // End of Buffer.
#define CAN_IF2MCTL_DLC_M       0x0000000F  // Data Length Code.
#define CAN_IF2MCTL_DLC_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA1 register.
//
//*****************************************************************************
#define CAN_IF2DA1_DATA_M       0x0000FFFF  // Data.
#define CAN_IF2DA1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA2 register.
//
//*****************************************************************************
#define CAN_IF2DA2_DATA_M       0x0000FFFF  // Data.
#define CAN_IF2DA2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB1 register.
//
//*****************************************************************************
#define CAN_IF2DB1_DATA_M       0x0000FFFF  // Data.
#define CAN_IF2DB1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB2 register.
//
//*****************************************************************************
#define CAN_IF2DB2_DATA_M       0x0000FFFF  // Data.
#define CAN_IF2DB2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1INT register.
//
//*****************************************************************************
#define CAN_MSG1INT_INTPND_M    0x0000FFFF  // Interrupt Pending Bits.
#define CAN_MSG1INT_INTPND_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2INT register.
//
//*****************************************************************************
#define CAN_MSG2INT_INTPND_M    0x0000FFFF  // Interrupt Pending Bits.
#define CAN_MSG2INT_INTPND_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1VAL register.
//
//*****************************************************************************
#define CAN_MSG1VAL_MSGVAL_M    0x0000FFFF  // Message Valid Bits.
#define CAN_MSG1VAL_MSGVAL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2VAL register.
//
//*****************************************************************************
#define CAN_MSG2VAL_MSGVAL_M    0x0000FFFF  // Message Valid Bits.
#define CAN_MSG2VAL_MSGVAL_S    0

//*****************************************************************************
//
// The following are deprecated defines for the CAN register offsets.
//
//*****************************************************************************
#define CAN_O_MSGINT1           0x00000140  // Intr. Pending in Msg Obj 1 reg.
#define CAN_O_MSGINT2           0x00000144  // Intr. Pending in Msg Obj 2 reg.
#define CAN_O_MSGVAL1           0x00000160  // Message Valid in Msg Obj 1 reg.
#define CAN_O_MSGVAL2           0x00000164  // Message Valid in Msg Obj 2 reg.

//*****************************************************************************
//
// The following are deprecated defines for the reset values of the can
// registers.
//
//*****************************************************************************
#define CAN_RV_IF1MSK2          0x0000FFFF
#define CAN_RV_IF1MSK1          0x0000FFFF
#define CAN_RV_IF2MSK1          0x0000FFFF
#define CAN_RV_IF2MSK2          0x0000FFFF
#define CAN_RV_BIT              0x00002301
#define CAN_RV_CTL              0x00000001
#define CAN_RV_IF1CRQ           0x00000001
#define CAN_RV_IF2CRQ           0x00000001
#define CAN_RV_TXRQ2            0x00000000
#define CAN_RV_IF2DB1           0x00000000
#define CAN_RV_INT              0x00000000
#define CAN_RV_IF1DB2           0x00000000
#define CAN_RV_BRPE             0x00000000
#define CAN_RV_IF2DA2           0x00000000
#define CAN_RV_MSGVAL2          0x00000000
#define CAN_RV_TXRQ1            0x00000000
#define CAN_RV_IF1MCTL          0x00000000
#define CAN_RV_IF1DB1           0x00000000
#define CAN_RV_STS              0x00000000
#define CAN_RV_MSGINT1          0x00000000
#define CAN_RV_IF1DA2           0x00000000
#define CAN_RV_TST              0x00000000
#define CAN_RV_IF1ARB1          0x00000000
#define CAN_RV_IF1ARB2          0x00000000
#define CAN_RV_NWDA2            0x00000000
#define CAN_RV_IF2CMSK          0x00000000
#define CAN_RV_NWDA1            0x00000000
#define CAN_RV_IF1DA1           0x00000000
#define CAN_RV_IF2DA1           0x00000000
#define CAN_RV_IF2MCTL          0x00000000
#define CAN_RV_MSGVAL1          0x00000000
#define CAN_RV_IF1CMSK          0x00000000
#define CAN_RV_ERR              0x00000000
#define CAN_RV_IF2ARB2          0x00000000
#define CAN_RV_MSGINT2          0x00000000
#define CAN_RV_IF2ARB1          0x00000000
#define CAN_RV_IF2DB2           0x00000000

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_STS
// register.
//
//*****************************************************************************
#define CAN_STS_LEC_MSK         0x00000007  // Last Error Code

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_ERR
// register.
//
//*****************************************************************************
#define CAN_ERR_REC_MASK        0x00007F00  // Receive error counter status
#define CAN_ERR_TEC_MASK        0x000000FF  // Transmit error counter status
#define CAN_ERR_REC_SHIFT       8           // Receive error counter bit pos
#define CAN_ERR_TEC_SHIFT       0           // Transmit error counter bit pos

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_BIT
// register.
//
//*****************************************************************************
#define CAN_BIT_TSEG2           0x00007000  // Time segment after sample point
#define CAN_BIT_TSEG1           0x00000F00  // Time segment before sample point
#define CAN_BIT_SJW             0x000000C0  // (Re)Synchronization jump width
#define CAN_BIT_BRP             0x0000003F  // Baud rate prescaler

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_INT
// register.
//
//*****************************************************************************
#define CAN_INT_INTID_MSK       0x0000FFFF  // Interrupt Identifier

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_TST
// register.
//
//*****************************************************************************
#define CAN_TST_TX_MSK          0x00000060  // Overide control of CAN_TX pin

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_BRPE
// register.
//
//*****************************************************************************
#define CAN_BRPE_BRPE           0x0000000F  // Baud rate prescaler extension

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1CRQ
// and CAN_IF1CRQ registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFCRQ_BUSY          0x00008000  // Busy flag status
#define CAN_IFCRQ_MNUM_MSK      0x0000003F  // Message Number

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1CMSK
// and CAN_IF2CMSK registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFCMSK_WRNRD        0x00000080  // Write, not Read
#define CAN_IFCMSK_MASK         0x00000040  // Access Mask Bits
#define CAN_IFCMSK_ARB          0x00000020  // Access Arbitration Bits
#define CAN_IFCMSK_CONTROL      0x00000010  // Access Control Bits
#define CAN_IFCMSK_CLRINTPND    0x00000008  // Clear interrupt pending Bit
#define CAN_IFCMSK_TXRQST       0x00000004  // Access Tx request bit (WRNRD=1)
#define CAN_IFCMSK_NEWDAT       0x00000004  // Access New Data bit (WRNRD=0)
#define CAN_IFCMSK_DATAA        0x00000002  // DataA access - bytes 0 to 3
#define CAN_IFCMSK_DATAB        0x00000001  // DataB access - bytes 4 to 7

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1MSK1
// and CAN_IF2MSK1 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFMSK1_MSK          0x0000FFFF  // Identifier Mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1MSK2
// and CAN_IF2MSK2 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFMSK2_MXTD         0x00008000  // Mask extended identifier
#define CAN_IFMSK2_MDIR         0x00004000  // Mask message direction
#define CAN_IFMSK2_MSK          0x00001FFF  // Mask identifier

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1ARB1
// and CAN_IF2ARB1 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFARB1_ID           0x0000FFFF  // Identifier

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1ARB2
// and CAN_IF2ARB2 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFARB2_MSGVAL       0x00008000  // Message valid
#define CAN_IFARB2_XTD          0x00004000  // Extended identifier
#define CAN_IFARB2_DIR          0x00002000  // Message direction
#define CAN_IFARB2_ID           0x00001FFF  // Message identifier

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1MCTL
// and CAN_IF2MCTL registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFMCTL_NEWDAT       0x00008000  // New Data
#define CAN_IFMCTL_MSGLST       0x00004000  // Message lost
#define CAN_IFMCTL_INTPND       0x00002000  // Interrupt pending
#define CAN_IFMCTL_UMASK        0x00001000  // Use acceptance mask
#define CAN_IFMCTL_TXIE         0x00000800  // Transmit interrupt enable
#define CAN_IFMCTL_RXIE         0x00000400  // Receive interrupt enable
#define CAN_IFMCTL_RMTEN        0x00000200  // Remote enable
#define CAN_IFMCTL_TXRQST       0x00000100  // Transmit request
#define CAN_IFMCTL_EOB          0x00000080  // End of buffer
#define CAN_IFMCTL_DLC          0x0000000F  // Data length code

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1DA1
// and CAN_IF2DA1 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFDA1_DATA          0x0000FFFF  // Data - bytes 1 and 0

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1DA2
// and CAN_IF2DA2 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFDA2_DATA          0x0000FFFF  // Data - bytes 3 and 2

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1DB1
// and CAN_IF2DB1 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFDB1_DATA          0x0000FFFF  // Data - bytes 5 and 4

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_IF1DB2
// and CAN_IF2DB2 registers.
// Note: All bits may not be available in all registers
//
//*****************************************************************************
#define CAN_IFDB2_DATA          0x0000FFFF  // Data - bytes 7 and 6

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_TXRQ1
// register.
//
//*****************************************************************************
#define CAN_TXRQ1_TXRQST        0x0000FFFF  // Transmission Request Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_TXRQ2
// register.
//
//*****************************************************************************
#define CAN_TXRQ2_TXRQST        0x0000FFFF  // Transmission Request Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_NWDA1
// register.
//
//*****************************************************************************
#define CAN_NWDA1_NEWDATA       0x0000FFFF  // New Data Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_NWDA2
// register.
//
//*****************************************************************************
#define CAN_NWDA2_NEWDATA       0x0000FFFF  // New Data Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_MSGINT1
// register.
//
//*****************************************************************************
#define CAN_MSGINT1_INTPND      0x0000FFFF  // Interrupt Pending Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_MSGINT2
// register.
//
//*****************************************************************************
#define CAN_MSGINT2_INTPND      0x0000FFFF  // Interrupt Pending Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_MSGVAL1
// register.
//
//*****************************************************************************
#define CAN_MSGVAL1_MSGVAL      0x0000FFFF  // Message Valid Bits

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the CAN_MSGVAL2
// register.
//
//*****************************************************************************
#define CAN_MSGVAL2_MSGVAL      0x0000FFFF  // Message Valid Bits

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC register.
//
//*****************************************************************************
#define FLASH_FMC_WRKEY_M       0xFFFF0000  // FLASH write key mask
#define FLASH_FMC_WRKEY         0xA4420000  // FLASH write key
#define FLASH_FMC_COMT          0x00000008  // Commit user register
#define FLASH_FMC_MERASE        0x00000004  // Mass erase FLASH
#define FLASH_FMC_ERASE         0x00000002  // Erase FLASH page
#define FLASH_FMC_WRITE         0x00000001  // Write FLASH word
#define FLASH_FMC_WRKEY_S       16

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCRIS register.
//
//*****************************************************************************
#define FLASH_FCRIS_PRIS        0x00000002  // Programming Raw Interrupt
                                            // Status.
#define FLASH_FCRIS_ARIS        0x00000001  // Access Raw Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCIM register.
//
//*****************************************************************************
#define FLASH_FCIM_PMASK        0x00000002  // Programming Interrupt Mask.
#define FLASH_FCIM_AMASK        0x00000001  // Access Interrupt Mask.

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMIS register.
//
//*****************************************************************************
#define FLASH_FCMISC_PMISC      0x00000002  // Programming Masked Interrupt
                                            // Status and Clear.
#define FLASH_FCMISC_AMISC      0x00000001  // Access Masked Interrupt Status
                                            // and Clear.

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE and
// FLASH_FMPPE registers.
//
//*****************************************************************************
#define FLASH_FMP_BLOCK_31      0x80000000  // Enable for block 31
#define FLASH_FMP_BLOCK_30      0x40000000  // Enable for block 30
#define FLASH_FMP_BLOCK_29      0x20000000  // Enable for block 29
#define FLASH_FMP_BLOCK_28      0x10000000  // Enable for block 28
#define FLASH_FMP_BLOCK_27      0x08000000  // Enable for block 27
#define FLASH_FMP_BLOCK_26      0x04000000  // Enable for block 26
#define FLASH_FMP_BLOCK_25      0x02000000  // Enable for block 25
#define FLASH_FMP_BLOCK_24      0x01000000  // Enable for block 24
#define FLASH_FMP_BLOCK_23      0x00800000  // Enable for block 23
#define FLASH_FMP_BLOCK_22      0x00400000  // Enable for block 22
#define FLASH_FMP_BLOCK_21      0x00200000  // Enable for block 21
#define FLASH_FMP_BLOCK_20      0x00100000  // Enable for block 20
#define FLASH_FMP_BLOCK_19      0x00080000  // Enable for block 19
#define FLASH_FMP_BLOCK_18      0x00040000  // Enable for block 18
#define FLASH_FMP_BLOCK_17      0x00020000  // Enable for block 17
#define FLASH_FMP_BLOCK_16      0x00010000  // Enable for block 16
#define FLASH_FMP_BLOCK_15      0x00008000  // Enable for block 15
#define FLASH_FMP_BLOCK_14      0x00004000  // Enable for block 14
#define FLASH_FMP_BLOCK_13      0x00002000  // Enable for block 13
#define FLASH_FMP_BLOCK_12      0x00001000  // Enable for block 12
#define FLASH_FMP_BLOCK_11      0x00000800  // Enable for block 11
#define FLASH_FMP_BLOCK_10      0x00000400  // Enable for block 10
#define FLASH_FMP_BLOCK_9       0x00000200  // Enable for block 9
#define FLASH_FMP_BLOCK_8       0x00000100  // Enable for block 8
#define FLASH_FMP_BLOCK_7       0x00000080  // Enable for block 7
#define FLASH_FMP_BLOCK_6       0x00000040  // Enable for block 6
#define FLASH_FMP_BLOCK_5       0x00000020  // Enable for block 5
#define FLASH_FMP_BLOCK_4       0x00000010  // Enable for block 4
#define FLASH_FMP_BLOCK_3       0x00000008  // Enable for block 3
#define FLASH_FMP_BLOCK_2       0x00000004  // Enable for block 2
#define FLASH_FMP_BLOCK_1       0x00000002  // Enable for block 1
#define FLASH_FMP_BLOCK_0       0x00000001  // Enable for block 0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USECRL register.
//
//*****************************************************************************
#define FLASH_USECRL_M          0x000000FF  // Microsecond Reload Value.
#define FLASH_USECRL_S          0

//*****************************************************************************
//
// The following are defines for the erase size of the FLASH block that is
// erased by an erase operation, and the protect size is the size of the FLASH
// block that is protected by each protection register.
//
//*****************************************************************************
#define FLASH_PROTECT_SIZE      0x00000800
#define FLASH_ERASE_SIZE        0x00000400

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMA register.
//
//*****************************************************************************
#define FLASH_FMA_OFFSET_M      0x0003FFFF  // Address Offset.
#define FLASH_FMA_OFFSET_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMD register.
//
//*****************************************************************************
#define FLASH_FMD_DATA_M        0xFFFFFFFF  // Data Value.
#define FLASH_FMD_DATA_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERDBG register.
//
//*****************************************************************************
#define FLASH_USERDBG_NW        0x80000000  // User Debug Not Written.
#define FLASH_USERDBG_DATA_M    0x7FFFFFFC  // User Data.
#define FLASH_USERDBG_DBG1      0x00000002  // Debug Control 1.
#define FLASH_USERDBG_DBG0      0x00000001  // Debug Control 0.
#define FLASH_USERDBG_DATA_S    2

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG0 register.
//
//*****************************************************************************
#define FLASH_USERREG0_NW       0x80000000  // Not Written.
#define FLASH_USERREG0_DATA_M   0x7FFFFFFF  // User Data.
#define FLASH_USERREG0_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG1 register.
//
//*****************************************************************************
#define FLASH_USERREG1_NW       0x80000000  // Not Written.
#define FLASH_USERREG1_DATA_M   0x7FFFFFFF  // User Data.
#define FLASH_USERREG1_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_RMCTL register.
//
//*****************************************************************************
#define FLASH_RMCTL_BA          0x00000001  // Boot Alias.

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_RMVER register.
//
//*****************************************************************************
#define FLASH_RMVER_CONT_M      0xFF000000  // ROM Contents.
#define FLASH_RMVER_CONT_LM     0x00000000  // Boot Loader & DriverLib
#define FLASH_RMVER_SIZE_M      0x00FF0000  // ROM Size.
#define FLASH_RMVER_SIZE_11K    0x00000000  // 11KB Size
#define FLASH_RMVER_VER_M       0x0000FF00  // ROM Version.
#define FLASH_RMVER_REV_M       0x000000FF  // ROM Revision.
#define FLASH_RMVER_VER_S       8
#define FLASH_RMVER_REV_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG2 register.
//
//*****************************************************************************
#define FLASH_USERREG2_NW       0x80000000  // Not Written.
#define FLASH_USERREG2_DATA_M   0x7FFFFFFF  // User Data.
#define FLASH_USERREG2_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG3 register.
//
//*****************************************************************************
#define FLASH_USERREG3_NW       0x80000000  // Not Written.
#define FLASH_USERREG3_DATA_M   0x7FFFFFFF  // User Data.
#define FLASH_USERREG3_DATA_S   0

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the FLASH_FMC
// register.
//
//*****************************************************************************
#define FLASH_FMC_WRKEY_MASK    0xFFFF0000  // FLASH write key mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the FLASH_FCRIS
// register.
//
//*****************************************************************************
#define FLASH_FCRIS_PROGRAM     0x00000002  // Programming status
#define FLASH_FCRIS_ACCESS      0x00000001  // Invalid access status

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the FLASH_FCIM
// register.
//
//*****************************************************************************
#define FLASH_FCIM_PROGRAM      0x00000002  // Programming mask
#define FLASH_FCIM_ACCESS       0x00000001  // Invalid access mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the FLASH_FMIS
// register.
//
//*****************************************************************************
#define FLASH_FCMISC_PROGRAM    0x00000002  // Programming status
#define FLASH_FCMISC_ACCESS     0x00000001  // Invalid access status

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the FLASH_USECRL
// register.
//
//*****************************************************************************
#define FLASH_USECRL_MASK       0x000000FF  // Clock per uSec
#define FLASH_USECRL_SHIFT      0

//*****************************************************************************
//
// The following are defines for the erase size of the FLASH block that is
// erased by an erase operation, and the protect size is the size of the FLASH
// block that is protected by each protection register.
//
//*****************************************************************************
#define FLASH_PROTECT_SIZE      0x00000800
#define FLASH_ERASE_SIZE        0x00000400

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
//*****************************************************************************
#define SYSCTL_DID0_VER_M       0x70000000  // DID0 version mask
#define SYSCTL_DID0_VER_0       0x00000000  // DID0 version 0
#define SYSCTL_DID0_VER_1       0x10000000  // DID0 version 1
#define SYSCTL_DID0_CLASS_M     0x00FF0000  // Device Class
#define SYSCTL_DID0_CLASS_SANDSTORM \
                                0x00000000  // Sandstorm-class Device
#define SYSCTL_DID0_CLASS_FURY  0x00010000  // Fury-class Device
#define SYSCTL_DID0_CLASS_DUSTDEVIL \
                                0x00030000  // DustDevil-class Device
#define SYSCTL_DID0_MAJ_M       0x0000FF00  // Major revision mask
#define SYSCTL_DID0_MAJ_REVA    0x00000000  // Revision A (initial device)
#define SYSCTL_DID0_MAJ_REVB    0x00000100  // Revision B (first base layer
                                            // revision)
#define SYSCTL_DID0_MAJ_REVC    0x00000200  // Revision C (second base layer
                                            // revision)
#define SYSCTL_DID0_MIN_M       0x000000FF  // Minor revision mask
#define SYSCTL_DID0_MIN_0       0x00000000  // Minor revision 0
#define SYSCTL_DID0_MIN_1       0x00000001  // Minor revision 1
#define SYSCTL_DID0_MIN_2       0x00000002  // Minor revision 2
#define SYSCTL_DID0_MIN_3       0x00000003  // Minor revision 3
#define SYSCTL_DID0_MIN_4       0x00000004  // Minor revision 4
#define SYSCTL_DID0_MIN_5       0x00000005  // Minor revision 5

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID1 register.
//
//*****************************************************************************
#define SYSCTL_DID1_VER_M       0xF0000000  // DID1 Version.
#define SYSCTL_DID1_VER_0       0x00000000  // Initial DID1 register format
                                            // definition, indicating a
                                            // Stellaris LM3Snnn device.
#define SYSCTL_DID1_VER_1       0x10000000  // First revision of the DID1
                                            // register format, indicating a
                                            // Stellaris Fury-class device.
#define SYSCTL_DID1_FAM_M       0x0F000000  // Family.
#define SYSCTL_DID1_FAM_STELLARIS \
                                0x00000000  // Stellaris family of
                                            // microcontollers, that is, all
                                            // devices with external part
                                            // numbers starting with LM3S.
#define SYSCTL_DID1_PRTNO_M     0x00FF0000  // Part number mask
#define SYSCTL_DID1_PRTNO_101   0x00010000  // LM3S101
#define SYSCTL_DID1_PRTNO_102   0x00020000  // LM3S102
#define SYSCTL_DID1_PRTNO_300   0x00190000  // LM3S300
#define SYSCTL_DID1_PRTNO_301   0x00110000  // LM3S301
#define SYSCTL_DID1_PRTNO_308   0x001A0000  // LM3S308
#define SYSCTL_DID1_PRTNO_310   0x00120000  // LM3S310
#define SYSCTL_DID1_PRTNO_315   0x00130000  // LM3S315
#define SYSCTL_DID1_PRTNO_316   0x00140000  // LM3S316
#define SYSCTL_DID1_PRTNO_317   0x00170000  // LM3S317
#define SYSCTL_DID1_PRTNO_328   0x00150000  // LM3S328
#define SYSCTL_DID1_PRTNO_600   0x002A0000  // LM3S600
#define SYSCTL_DID1_PRTNO_601   0x00210000  // LM3S601
#define SYSCTL_DID1_PRTNO_608   0x002B0000  // LM3S608
#define SYSCTL_DID1_PRTNO_610   0x00220000  // LM3S610
#define SYSCTL_DID1_PRTNO_611   0x00230000  // LM3S611
#define SYSCTL_DID1_PRTNO_612   0x00240000  // LM3S612
#define SYSCTL_DID1_PRTNO_613   0x00250000  // LM3S613
#define SYSCTL_DID1_PRTNO_615   0x00260000  // LM3S615
#define SYSCTL_DID1_PRTNO_617   0x00280000  // LM3S617
#define SYSCTL_DID1_PRTNO_618   0x00290000  // LM3S618
#define SYSCTL_DID1_PRTNO_628   0x00270000  // LM3S628
#define SYSCTL_DID1_PRTNO_800   0x00380000  // LM3S800
#define SYSCTL_DID1_PRTNO_801   0x00310000  // LM3S801
#define SYSCTL_DID1_PRTNO_808   0x00390000  // LM3S808
#define SYSCTL_DID1_PRTNO_811   0x00320000  // LM3S811
#define SYSCTL_DID1_PRTNO_812   0x00330000  // LM3S812
#define SYSCTL_DID1_PRTNO_815   0x00340000  // LM3S815
#define SYSCTL_DID1_PRTNO_817   0x00360000  // LM3S817
#define SYSCTL_DID1_PRTNO_818   0x00370000  // LM3S818
#define SYSCTL_DID1_PRTNO_828   0x00350000  // LM3S828
#define SYSCTL_DID1_PRTNO_1110  0x00BF0000  // LM3S1110
#define SYSCTL_DID1_PRTNO_1133  0x00C30000  // LM3S1133
#define SYSCTL_DID1_PRTNO_1138  0x00C50000  // LM3S1138
#define SYSCTL_DID1_PRTNO_1150  0x00C10000  // LM3S1150
#define SYSCTL_DID1_PRTNO_1162  0x00C40000  // LM3S1162
#define SYSCTL_DID1_PRTNO_1165  0x00C20000  // LM3S1165
#define SYSCTL_DID1_PRTNO_1332  0x00C60000  // LM3S1332
#define SYSCTL_DID1_PRTNO_1435  0x00BC0000  // LM3S1435
#define SYSCTL_DID1_PRTNO_1439  0x00BA0000  // LM3S1439
#define SYSCTL_DID1_PRTNO_1512  0x00BB0000  // LM3S1512
#define SYSCTL_DID1_PRTNO_1538  0x00C70000  // LM3S1538
#define SYSCTL_DID1_PRTNO_1601  0x00DB0000  // LM3S1601
#define SYSCTL_DID1_PRTNO_1607  0x00060000  // LM3S1607
#define SYSCTL_DID1_PRTNO_1608  0x00DA0000  // LM3S1608
#define SYSCTL_DID1_PRTNO_1620  0x00C00000  // LM3S1620
#define SYSCTL_DID1_PRTNO_1625  0x00030000  // LM3S1625
#define SYSCTL_DID1_PRTNO_1626  0x00040000  // LM3S1626
#define SYSCTL_DID1_PRTNO_1627  0x00050000  // LM3S1627
#define SYSCTL_DID1_PRTNO_1635  0x00B30000  // LM3S1635
#define SYSCTL_DID1_PRTNO_1637  0x00BD0000  // LM3S1637
#define SYSCTL_DID1_PRTNO_1751  0x00B90000  // LM3S1751
#define SYSCTL_DID1_PRTNO_1776  0x00100000  // LM3S1776
#define SYSCTL_DID1_PRTNO_1850  0x00B40000  // LM3S1850
#define SYSCTL_DID1_PRTNO_1911  0x00DD0000  // LM3S1911
#define SYSCTL_DID1_PRTNO_1918  0x00DC0000  // LM3S1918
#define SYSCTL_DID1_PRTNO_1937  0x00B70000  // LM3S1937
#define SYSCTL_DID1_PRTNO_1958  0x00BE0000  // LM3S1958
#define SYSCTL_DID1_PRTNO_1960  0x00B50000  // LM3S1960
#define SYSCTL_DID1_PRTNO_1968  0x00B80000  // LM3S1968
#define SYSCTL_DID1_PRTNO_2110  0x00510000  // LM3S2110
#define SYSCTL_DID1_PRTNO_2139  0x00840000  // LM3S2139
#define SYSCTL_DID1_PRTNO_2276  0x00390000  // LM3S2276
#define SYSCTL_DID1_PRTNO_2410  0x00A20000  // LM3S2410
#define SYSCTL_DID1_PRTNO_2412  0x00590000  // LM3S2412
#define SYSCTL_DID1_PRTNO_2432  0x00560000  // LM3S2432
#define SYSCTL_DID1_PRTNO_2533  0x005A0000  // LM3S2533
#define SYSCTL_DID1_PRTNO_2601  0x00E10000  // LM3S2601
#define SYSCTL_DID1_PRTNO_2608  0x00E00000  // LM3S2608
#define SYSCTL_DID1_PRTNO_2616  0x00330000  // LM3S2616
#define SYSCTL_DID1_PRTNO_2620  0x00570000  // LM3S2620
#define SYSCTL_DID1_PRTNO_2637  0x00850000  // LM3S2637
#define SYSCTL_DID1_PRTNO_2651  0x00530000  // LM3S2651
#define SYSCTL_DID1_PRTNO_2671  0x00800000  // LM3S2671
#define SYSCTL_DID1_PRTNO_2678  0x00500000  // LM3S2678
#define SYSCTL_DID1_PRTNO_2730  0x00A40000  // LM3S2730
#define SYSCTL_DID1_PRTNO_2739  0x00520000  // LM3S2739
#define SYSCTL_DID1_PRTNO_2776  0x003A0000  // LM3S2776
#define SYSCTL_DID1_PRTNO_2911  0x00E30000  // LM3S2911
#define SYSCTL_DID1_PRTNO_2918  0x00E20000  // LM3S2918
#define SYSCTL_DID1_PRTNO_2939  0x00540000  // LM3S2939
#define SYSCTL_DID1_PRTNO_2948  0x008F0000  // LM3S2948
#define SYSCTL_DID1_PRTNO_2950  0x00580000  // LM3S2950
#define SYSCTL_DID1_PRTNO_2965  0x00550000  // LM3S2965
#define SYSCTL_DID1_PRTNO_3651  0x00430000  // LM3S3651
#define SYSCTL_DID1_PRTNO_3739  0x00440000  // LM3S3739
#define SYSCTL_DID1_PRTNO_3748  0x00490000  // LM3S3748
#define SYSCTL_DID1_PRTNO_3749  0x00450000  // LM3S3749
#define SYSCTL_DID1_PRTNO_3759  0x00460000  // LM3S3759
#define SYSCTL_DID1_PRTNO_3768  0x00480000  // LM3S3768
#define SYSCTL_DID1_PRTNO_5632  0x00810000  // LM3S5632
#define SYSCTL_DID1_PRTNO_5652  0x008A0000  // LM3S5652
#define SYSCTL_DID1_PRTNO_5662  0x00910000  // LM3S5662
#define SYSCTL_DID1_PRTNO_5732  0x00960000  // LM3S5732
#define SYSCTL_DID1_PRTNO_5737  0x00970000  // LM3S5737
#define SYSCTL_DID1_PRTNO_5739  0x00A00000  // LM3S5739
#define SYSCTL_DID1_PRTNO_5747  0x00990000  // LM3S5747
#define SYSCTL_DID1_PRTNO_5749  0x00A70000  // LM3S5749
#define SYSCTL_DID1_PRTNO_5752  0x009A0000  // LM3S5752
#define SYSCTL_DID1_PRTNO_5757  0x009B0000  // LM3S5757
#define SYSCTL_DID1_PRTNO_5762  0x009C0000  // LM3S5762
#define SYSCTL_DID1_PRTNO_5767  0x009D0000  // LM3S5767
#define SYSCTL_DID1_PRTNO_5768  0x00A90000  // LM3S5768
#define SYSCTL_DID1_PRTNO_5769  0x00A80000  // LM3S5769
#define SYSCTL_DID1_PRTNO_6100  0x00A10000  // LM3S6100
#define SYSCTL_DID1_PRTNO_6110  0x00740000  // LM3S6110
#define SYSCTL_DID1_PRTNO_6420  0x00A50000  // LM3S6420
#define SYSCTL_DID1_PRTNO_6422  0x00820000  // LM3S6422
#define SYSCTL_DID1_PRTNO_6432  0x00750000  // LM3S6432
#define SYSCTL_DID1_PRTNO_6537  0x00760000  // LM3S6537
#define SYSCTL_DID1_PRTNO_6610  0x00710000  // LM3S6610
#define SYSCTL_DID1_PRTNO_6611  0x00E70000  // LM3S6611
#define SYSCTL_DID1_PRTNO_6618  0x00E60000  // LM3S6618
#define SYSCTL_DID1_PRTNO_6633  0x00830000  // LM3S6633
#define SYSCTL_DID1_PRTNO_6637  0x008B0000  // LM3S6637
#define SYSCTL_DID1_PRTNO_6730  0x00A30000  // LM3S6730
#define SYSCTL_DID1_PRTNO_6753  0x00770000  // LM3S6753
#define SYSCTL_DID1_PRTNO_6911  0x00E90000  // LM3S6911
#define SYSCTL_DID1_PRTNO_6918  0x00E80000  // LM3S6918
#define SYSCTL_DID1_PRTNO_6938  0x00890000  // LM3S6938
#define SYSCTL_DID1_PRTNO_6950  0x00720000  // LM3S6950
#define SYSCTL_DID1_PRTNO_6952  0x00780000  // LM3S6952
#define SYSCTL_DID1_PRTNO_6965  0x00730000  // LM3S6965
#define SYSCTL_DID1_PRTNO_8530  0x00640000  // LM3S8530
#define SYSCTL_DID1_PRTNO_8538  0x008E0000  // LM3S8538
#define SYSCTL_DID1_PRTNO_8630  0x00610000  // LM3S8630
#define SYSCTL_DID1_PRTNO_8730  0x00630000  // LM3S8730
#define SYSCTL_DID1_PRTNO_8733  0x008D0000  // LM3S8733
#define SYSCTL_DID1_PRTNO_8738  0x00860000  // LM3S8738
#define SYSCTL_DID1_PRTNO_8930  0x00650000  // LM3S8930
#define SYSCTL_DID1_PRTNO_8933  0x008C0000  // LM3S8933
#define SYSCTL_DID1_PRTNO_8938  0x00880000  // LM3S8938
#define SYSCTL_DID1_PRTNO_8962  0x00A60000  // LM3S8962
#define SYSCTL_DID1_PRTNO_8970  0x00620000  // LM3S8970
#define SYSCTL_DID1_PRTNO_8971  0x00D70000  // LM3S8971
#define SYSCTL_DID1_PINCNT_M    0x0000E000  // Package Pin Count.
#define SYSCTL_DID1_PINCNT_28   0x00000000  // 28 pin package
#define SYSCTL_DID1_PINCNT_48   0x00002000  // 48 pin package
#define SYSCTL_DID1_PINCNT_100  0x00004000  // 100 pin package
#define SYSCTL_DID1_PINCNT_64   0x00006000  // 64 pin package
#define SYSCTL_DID1_TEMP_M      0x000000E0  // Temperature range mask
#define SYSCTL_DID1_TEMP_C      0x00000000  // Commercial temp range (0..70C)
#define SYSCTL_DID1_TEMP_I      0x00000020  // Industrial temp range (-40..85C)
#define SYSCTL_DID1_TEMP_E      0x00000040  // Extended temperature range (-40C
                                            // to 105C)
#define SYSCTL_DID1_PKG_M       0x00000018  // Package Type.
#define SYSCTL_DID1_PKG_28SOIC  0x00000000  // 28-pin SOIC
#define SYSCTL_DID1_PKG_48QFP   0x00000008  // 48-pin QFP
#define SYSCTL_DID1_PKG_BGA     0x00000010  // BGA package
#define SYSCTL_DID1_ROHS        0x00000004  // Part is RoHS compliant
#define SYSCTL_DID1_QUAL_M      0x00000003  // Qualification status mask
#define SYSCTL_DID1_QUAL_ES     0x00000000  // Engineering sample (unqualified)
#define SYSCTL_DID1_QUAL_PP     0x00000001  // Pilot production (unqualified)
#define SYSCTL_DID1_QUAL_FQ     0x00000002  // Fully qualified
#define SYSCTL_DID1_PRTNO_S     16          // Part number shift

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC0 register.
//
//*****************************************************************************
#define SYSCTL_DC0_SRAMSZ_M     0xFFFF0000  // SRAM size mask
#define SYSCTL_DC0_SRAMSZ_2KB   0x00070000  // 2 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_4KB   0x000F0000  // 4 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_8KB   0x001F0000  // 8 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_16KB  0x003F0000  // 16 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_32KB  0x007F0000  // 32 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_64KB  0x00FF0000  // 64 KB of SRAM
#define SYSCTL_DC0_FLASHSZ_M    0x0000FFFF  // Flash size mask
#define SYSCTL_DC0_FLASHSZ_8KB  0x00000003  // 8 KB of flash
#define SYSCTL_DC0_FLASHSZ_16KB 0x00000007  // 16 KB of flash
#define SYSCTL_DC0_FLASHSZ_32KB 0x0000000F  // 32 KB of flash
#define SYSCTL_DC0_FLASHSZ_64KB 0x0000001F  // 64 KB of flash
#define SYSCTL_DC0_FLASHSZ_96KB 0x0000002F  // 96 KB of flash
#define SYSCTL_DC0_FLASHSZ_128K 0x0000003F  // 128 KB of flash
#define SYSCTL_DC0_FLASHSZ_256K 0x0000007F  // 256 KB of flash
#define SYSCTL_DC0_SRAMSZ_S     16          // SRAM size shift
#define SYSCTL_DC0_FLASHSZ_S    0           // Flash size shift

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC1 register.
//
//*****************************************************************************
#define SYSCTL_DC1_CAN2         0x04000000  // CAN2 module present
#define SYSCTL_DC1_CAN1         0x02000000  // CAN1 module present
#define SYSCTL_DC1_CAN0         0x01000000  // CAN0 module present
#define SYSCTL_DC1_PWM          0x00100000  // PWM module present
#define SYSCTL_DC1_ADC          0x00010000  // ADC module present
#define SYSCTL_DC1_MINSYSDIV_M  0x0000F000  // System Clock Divider.
#define SYSCTL_DC1_MINSYSDIV_50 0x00003000  // Specifies a 50-MHz CPU clock
                                            // with a PLL divider of 4.
#define SYSCTL_DC1_MINSYSDIV_25 0x00007000  // Specifies a 25-MHz clock with a
                                            // PLL divider of 8.
#define SYSCTL_DC1_MINSYSDIV_20 0x00009000  // Specifies a 20-MHz clock with a
                                            // PLL divider of 10.
#define SYSCTL_DC1_ADCSPD_M     0x00000F00  // ADC speed mask
#define SYSCTL_DC1_ADCSPD_125K  0x00000000  // 125Ksps ADC
#define SYSCTL_DC1_ADCSPD_250K  0x00000100  // 250Ksps ADC
#define SYSCTL_DC1_ADCSPD_500K  0x00000200  // 500Ksps ADC
#define SYSCTL_DC1_ADCSPD_1M    0x00000300  // 1Msps ADC
#define SYSCTL_DC1_MPU          0x00000080  // Cortex M3 MPU present
#define SYSCTL_DC1_HIB          0x00000040  // Hibernation module present
#define SYSCTL_DC1_TEMP         0x00000020  // Temperature sensor present
#define SYSCTL_DC1_PLL          0x00000010  // PLL present
#define SYSCTL_DC1_WDT          0x00000008  // Watchdog Timer Present.
#define SYSCTL_DC1_SWO          0x00000004  // Serial wire output present
#define SYSCTL_DC1_SWD          0x00000002  // Serial wire debug present
#define SYSCTL_DC1_JTAG         0x00000001  // JTAG debug present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC2 register.
//
//*****************************************************************************
#define SYSCTL_DC2_COMP2        0x04000000  // Analog comparator 2 present
#define SYSCTL_DC2_COMP1        0x02000000  // Analog comparator 1 present
#define SYSCTL_DC2_COMP0        0x01000000  // Analog comparator 0 present
#define SYSCTL_DC2_TIMER3       0x00080000  // Timer 3 present
#define SYSCTL_DC2_TIMER2       0x00040000  // Timer 2 present
#define SYSCTL_DC2_TIMER1       0x00020000  // Timer 1 present
#define SYSCTL_DC2_TIMER0       0x00010000  // Timer 0 present
#define SYSCTL_DC2_I2C1         0x00004000  // I2C 1 present
#define SYSCTL_DC2_I2C0         0x00001000  // I2C 0 present
#define SYSCTL_DC2_QEI1         0x00000200  // QEI 1 present
#define SYSCTL_DC2_QEI0         0x00000100  // QEI 0 present
#define SYSCTL_DC2_SSI1         0x00000020  // SSI 1 present
#define SYSCTL_DC2_SSI0         0x00000010  // SSI 0 present
#define SYSCTL_DC2_UART2        0x00000004  // UART 2 present
#define SYSCTL_DC2_UART1        0x00000002  // UART 1 present
#define SYSCTL_DC2_UART0        0x00000001  // UART 0 present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC3 register.
//
//*****************************************************************************
#define SYSCTL_DC3_32KHZ        0x80000000  // 32KHz Pin Present.
#define SYSCTL_DC3_CCP5         0x20000000  // CCP5 pin present
#define SYSCTL_DC3_CCP4         0x10000000  // CCP4 pin present
#define SYSCTL_DC3_CCP3         0x08000000  // CCP3 pin present
#define SYSCTL_DC3_CCP2         0x04000000  // CCP2 pin present
#define SYSCTL_DC3_CCP1         0x02000000  // CCP1 pin present
#define SYSCTL_DC3_CCP0         0x01000000  // CCP0 pin present
#define SYSCTL_DC3_ADC7         0x00800000  // ADC7 pin present
#define SYSCTL_DC3_ADC6         0x00400000  // ADC6 pin present
#define SYSCTL_DC3_ADC5         0x00200000  // ADC5 pin present
#define SYSCTL_DC3_ADC4         0x00100000  // ADC4 pin present
#define SYSCTL_DC3_ADC3         0x00080000  // ADC3 pin present
#define SYSCTL_DC3_ADC2         0x00040000  // ADC2 pin present
#define SYSCTL_DC3_ADC1         0x00020000  // ADC1 pin present
#define SYSCTL_DC3_ADC0         0x00010000  // ADC0 pin present
#define SYSCTL_DC3_PWMFAULT     0x00008000  // PWM Fault Pin Present.
#define SYSCTL_DC3_C2O          0x00004000  // C2o pin present
#define SYSCTL_DC3_C2PLUS       0x00002000  // C2+ pin present
#define SYSCTL_DC3_C2MINUS      0x00001000  // C2- pin present
#define SYSCTL_DC3_C1O          0x00000800  // C1o pin present
#define SYSCTL_DC3_C1PLUS       0x00000400  // C1+ pin present
#define SYSCTL_DC3_C1MINUS      0x00000200  // C1- pin present
#define SYSCTL_DC3_C0O          0x00000100  // C0o pin present
#define SYSCTL_DC3_C0PLUS       0x00000080  // C0+ pin present
#define SYSCTL_DC3_C0MINUS      0x00000040  // C0- pin present
#define SYSCTL_DC3_PWM5         0x00000020  // PWM5 pin present
#define SYSCTL_DC3_PWM4         0x00000010  // PWM4 pin present
#define SYSCTL_DC3_PWM3         0x00000008  // PWM3 pin present
#define SYSCTL_DC3_PWM2         0x00000004  // PWM2 pin present
#define SYSCTL_DC3_PWM1         0x00000002  // PWM1 pin present
#define SYSCTL_DC3_PWM0         0x00000001  // PWM0 pin present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC4 register.
//
//*****************************************************************************
#define SYSCTL_DC4_ETH          0x50000000  // Ethernet present
#define SYSCTL_DC4_EPHY0        0x40000000  // Ethernet PHY0 Present.
#define SYSCTL_DC4_EMAC0        0x10000000  // Ethernet MAC0 Present.
#define SYSCTL_DC4_E1588        0x01000000  // 1588 Capable.
#define SYSCTL_DC4_CCP7         0x00008000  // CCP7 Pin Present.
#define SYSCTL_DC4_CCP6         0x00004000  // CCP6 Pin Present.
#define SYSCTL_DC4_UDMA         0x00002000  // Micro-DMA is present.
#define SYSCTL_DC4_ROM          0x00001000  // Internal Code ROM is present.
#define SYSCTL_DC4_GPIOH        0x00000080  // GPIO port H present
#define SYSCTL_DC4_GPIOG        0x00000040  // GPIO port G present
#define SYSCTL_DC4_GPIOF        0x00000020  // GPIO port F present
#define SYSCTL_DC4_GPIOE        0x00000010  // GPIO port E present
#define SYSCTL_DC4_GPIOD        0x00000008  // GPIO port D present
#define SYSCTL_DC4_GPIOC        0x00000004  // GPIO port C present
#define SYSCTL_DC4_GPIOB        0x00000002  // GPIO port B present
#define SYSCTL_DC4_GPIOA        0x00000001  // GPIO port A present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PBORCTL register.
//
//*****************************************************************************
#define SYSCTL_PBORCTL_BORTIM_M 0x0000FFFC  // BOR Time Delay.
#define SYSCTL_PBORCTL_BORIOR   0x00000002  // BOR interrupt or reset
#define SYSCTL_PBORCTL_BORWT    0x00000001  // BOR wait and check for noise
#define SYSCTL_PBORCTL_BORTIM_S 2

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDOPCTL register.
//
//*****************************************************************************
#define SYSCTL_LDOPCTL_M        0x0000003F  // LDO Output Voltage.
#define SYSCTL_LDOPCTL_2_55V    0x0000001F  // LDO output of 2.55V
#define SYSCTL_LDOPCTL_2_60V    0x0000001E  // LDO output of 2.60V
#define SYSCTL_LDOPCTL_2_65V    0x0000001D  // LDO output of 2.65V
#define SYSCTL_LDOPCTL_2_70V    0x0000001C  // LDO output of 2.70V
#define SYSCTL_LDOPCTL_2_75V    0x0000001B  // LDO output of 2.75V
#define SYSCTL_LDOPCTL_2_25V    0x00000005  // LDO output of 2.25V
#define SYSCTL_LDOPCTL_2_30V    0x00000004  // LDO output of 2.30V
#define SYSCTL_LDOPCTL_2_35V    0x00000003  // LDO output of 2.35V
#define SYSCTL_LDOPCTL_2_40V    0x00000002  // LDO output of 2.40V
#define SYSCTL_LDOPCTL_2_45V    0x00000001  // LDO output of 2.45V
#define SYSCTL_LDOPCTL_2_50V    0x00000000  // LDO output of 2.50V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESC register.
//
//*****************************************************************************
#define SYSCTL_RESC_MOSCFAIL    0x00010000  // MOSC Failure Reset.
#define SYSCTL_RESC_LDO         0x00000020  // LDO power OK lost reset
#define SYSCTL_RESC_SW          0x00000010  // Software reset
#define SYSCTL_RESC_WDT         0x00000008  // Watchdog Timer Reset.
#define SYSCTL_RESC_BOR         0x00000004  // Brown-out reset
#define SYSCTL_RESC_POR         0x00000002  // Power on reset
#define SYSCTL_RESC_EXT         0x00000001  // External reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC register.
//
//*****************************************************************************
#define SYSCTL_RCC_ACG          0x08000000  // 自动时钟门控
#define SYSCTL_RCC_SYSDIV_M     0x07800000  // System Clock Divisor.
#define SYSCTL_RCC_SYSDIV_2     0x00800000  // System clock /2
#define SYSCTL_RCC_SYSDIV_3     0x01000000  // System clock /3
#define SYSCTL_RCC_SYSDIV_4     0x01800000  // System clock /4
#define SYSCTL_RCC_SYSDIV_5     0x02000000  // System clock /5
#define SYSCTL_RCC_SYSDIV_6     0x02800000  // System clock /6
#define SYSCTL_RCC_SYSDIV_7     0x03000000  // System clock /7
#define SYSCTL_RCC_SYSDIV_8     0x03800000  // System clock /8
#define SYSCTL_RCC_SYSDIV_9     0x04000000  // System clock /9
#define SYSCTL_RCC_SYSDIV_10    0x04800000  // System clock /10
#define SYSCTL_RCC_SYSDIV_11    0x05000000  // System clock /11
#define SYSCTL_RCC_SYSDIV_12    0x05800000  // System clock /12
#define SYSCTL_RCC_SYSDIV_13    0x06000000  // System clock /13
#define SYSCTL_RCC_SYSDIV_14    0x06800000  // System clock /14
#define SYSCTL_RCC_SYSDIV_15    0x07000000  // System clock /15
#define SYSCTL_RCC_SYSDIV_16    0x07800000  // System clock /16
#define SYSCTL_RCC_USESYSDIV    0x00400000  // Enable System Clock Divider.
#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor.
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM clock divider
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // PWM clock /2
#define SYSCTL_RCC_PWMDIV_4     0x00020000  // PWM clock /4
#define SYSCTL_RCC_PWMDIV_8     0x00040000  // PWM clock /8
#define SYSCTL_RCC_PWMDIV_16    0x00060000  // PWM clock /16
#define SYSCTL_RCC_PWMDIV_32    0x00080000  // PWM clock /32
#define SYSCTL_RCC_PWMDIV_64    0x000A0000  // PWM clock /64
#define SYSCTL_RCC_PWRDN        0x00002000  // PLL power down
#define SYSCTL_RCC_OEN          0x00001000  // PLL Output Enable.
#define SYSCTL_RCC_BYPASS       0x00000800  // PLL bypass
#define SYSCTL_RCC_XTAL_M       0x000007C0  // Crystal attached to main osc
#define SYSCTL_RCC_XTAL_1MHZ    0x00000000  // Using a 1MHz crystal
#define SYSCTL_RCC_XTAL_1_84MHZ 0x00000040  // Using a 1.8432MHz crystal
#define SYSCTL_RCC_XTAL_2MHZ    0x00000080  // Using a 2MHz crystal
#define SYSCTL_RCC_XTAL_2_45MHZ 0x000000C0  // Using a 2.4576MHz crystal
#define SYSCTL_RCC_XTAL_3_57MHZ 0x00000100  // Using a 3.579545MHz crystal
#define SYSCTL_RCC_XTAL_3_68MHZ 0x00000140  // Using a 3.6864MHz crystal
#define SYSCTL_RCC_XTAL_4MHZ    0x00000180  // Using a 4MHz crystal
#define SYSCTL_RCC_XTAL_4_09MHZ 0x000001C0  // Using a 4.096MHz crystal
#define SYSCTL_RCC_XTAL_4_91MHZ 0x00000200  // Using a 4.9152MHz crystal
#define SYSCTL_RCC_XTAL_5MHZ    0x00000240  // Using a 5MHz crystal
#define SYSCTL_RCC_XTAL_5_12MHZ 0x00000280  // Using a 5.12MHz crystal
#define SYSCTL_RCC_XTAL_6MHZ    0x000002C0  // Using a 6MHz crystal
#define SYSCTL_RCC_XTAL_6_14MHZ 0x00000300  // Using a 6.144MHz crystal
#define SYSCTL_RCC_XTAL_7_37MHZ 0x00000340  // Using a 7.3728MHz crystal
#define SYSCTL_RCC_XTAL_8MHZ    0x00000380  // Using a 8MHz crystal
#define SYSCTL_RCC_XTAL_8_19MHZ 0x000003C0  // Using a 8.192MHz crystal
#define SYSCTL_RCC_XTAL_10MHZ   0x00000400  // 10.0 MHz (USB)
#define SYSCTL_RCC_XTAL_12MHZ   0x00000440  // 12.0 MHz (USB)
#define SYSCTL_RCC_XTAL_12_2MHZ 0x00000480  // 12.288 MHz
#define SYSCTL_RCC_XTAL_13_5MHZ 0x000004C0  // 13.56 MHz
#define SYSCTL_RCC_XTAL_14_3MHZ 0x00000500  // 14.31818 MHz
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  // 16.0 MHz (USB)
#define SYSCTL_RCC_XTAL_16_3MHZ 0x00000580  // 16.384 MHz
#define SYSCTL_RCC_PLLVER       0x00000400  // PLL verification timer enable
#define SYSCTL_RCC_OSCSRC_M     0x00000030  // Oscillator input select
#define SYSCTL_RCC_OSCSRC_MAIN  0x00000000  // Use the main oscillator
#define SYSCTL_RCC_OSCSRC_INT   0x00000010  // Use the internal oscillator
#define SYSCTL_RCC_OSCSRC_INT4  0x00000020  // Use the internal oscillator / 4
#define SYSCTL_RCC_OSCSRC_30    0x00000030  // 30 KHz internal oscillator
#define SYSCTL_RCC_IOSCVER      0x00000008  // Int. osc. verification timer en
#define SYSCTL_RCC_MOSCVER      0x00000004  // Main osc. verification timer en
#define SYSCTL_RCC_IOSCDIS      0x00000002  // Internal oscillator disable
#define SYSCTL_RCC_MOSCDIS      0x00000001  // Main oscillator disable
#define SYSCTL_RCC_SYSDIV_S     23          // Shift to the SYSDIV field
#define SYSCTL_RCC_PWMDIV_S     17          // Shift to the PWMDIV field
#define SYSCTL_RCC_XTAL_S       6           // Shift to the XTAL field
#define SYSCTL_RCC_OSCSRC_S     4           // Shift to the OSCSRC field

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLCFG register.
//
//*****************************************************************************
#define SYSCTL_PLLCFG_OD_M      0x0000C000  // Output divider
#define SYSCTL_PLLCFG_OD_1      0x00000000  // Output divider is 1
#define SYSCTL_PLLCFG_OD_2      0x00004000  // Output divider is 2
#define SYSCTL_PLLCFG_OD_4      0x00008000  // Output divider is 4
#define SYSCTL_PLLCFG_F_M       0x00003FE0  // PLL F Value.
#define SYSCTL_PLLCFG_R_M       0x0000001F  // PLL R Value.
#define SYSCTL_PLLCFG_F_S       5
#define SYSCTL_PLLCFG_R_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC2 register.
//
//*****************************************************************************
#define SYSCTL_RCC2_USERCC2     0x80000000  // 使用 RCC2
#define SYSCTL_RCC2_SYSDIV2_M   0x1F800000  // 系统时钟分频屏蔽位
#define SYSCTL_RCC2_SYSDIV2_2   0x00800000  // 系统时钟分频 PLL/2
#define SYSCTL_RCC2_SYSDIV2_3   0x01000000  // 系统时钟分频 PLL/3
#define SYSCTL_RCC2_SYSDIV2_4   0x01800000  // 系统时钟分频 PLL/4
#define SYSCTL_RCC2_SYSDIV2_5   0x02000000  // 系统时钟分频 PLL/5
#define SYSCTL_RCC2_SYSDIV2_6   0x02800000  // 系统时钟分频 PLL/6
#define SYSCTL_RCC2_SYSDIV2_7   0x03000000  // 系统时钟分频 PLL/7
#define SYSCTL_RCC2_SYSDIV2_8   0x03800000  // 系统时钟分频 PLL/8
#define SYSCTL_RCC2_SYSDIV2_9   0x04000000  // 系统时钟分频 PLL/9
#define SYSCTL_RCC2_SYSDIV2_10  0x04800000  // 系统时钟分频 PLL/10
#define SYSCTL_RCC2_SYSDIV2_11  0x05000000  // 系统时钟分频 PLL/11
#define SYSCTL_RCC2_SYSDIV2_12  0x05800000  // 系统时钟分频 PLL/12
#define SYSCTL_RCC2_SYSDIV2_13  0x06000000  // 系统时钟分频 PLL/13
#define SYSCTL_RCC2_SYSDIV2_14  0x06800000  // 系统时钟分频 PLL/14
#define SYSCTL_RCC2_SYSDIV2_15  0x07000000  // 系统时钟分频 PLL/15
#define SYSCTL_RCC2_SYSDIV2_16  0x07800000  // 系统时钟分频 PLL/16
#define SYSCTL_RCC2_SYSDIV2_17  0x08000000  // 系统时钟分频 PLL/17
#define SYSCTL_RCC2_SYSDIV2_18  0x08800000  // 系统时钟分频 PLL/18
#define SYSCTL_RCC2_SYSDIV2_19  0x09000000  // 系统时钟分频 PLL/19
#define SYSCTL_RCC2_SYSDIV2_20  0x09800000  // 系统时钟分频 PLL /20
#define SYSCTL_RCC2_SYSDIV2_21  0x0A000000  // 系统时钟分频 PLL /21
#define SYSCTL_RCC2_SYSDIV2_22  0x0A800000  // 系统时钟分频 PLL /22
#define SYSCTL_RCC2_SYSDIV2_23  0x0B000000  // 系统时钟分频 PLL /23
#define SYSCTL_RCC2_SYSDIV2_24  0x0B800000  // 系统时钟分频 PLL /24
#define SYSCTL_RCC2_SYSDIV2_25  0x0C000000  // 系统时钟分频 PLL /25
#define SYSCTL_RCC2_SYSDIV2_26  0x0C800000  // 系统时钟分频 PLL /26
#define SYSCTL_RCC2_SYSDIV2_27  0x0D000000  // 系统时钟分频 PLL /27
#define SYSCTL_RCC2_SYSDIV2_28  0x0D800000  // 系统时钟分频 PLL /28
#define SYSCTL_RCC2_SYSDIV2_29  0x0E000000  // 系统时钟分频 PLL /29
#define SYSCTL_RCC2_SYSDIV2_30  0x0E800000  // 系统时钟分频 PLL /30
#define SYSCTL_RCC2_SYSDIV2_31  0x0F000000  // 系统时钟分频 PLL /31
#define SYSCTL_RCC2_SYSDIV2_32  0x0F800000  // 系统时钟分频 PLL /32
#define SYSCTL_RCC2_SYSDIV2_33  0x10000000  // 系统时钟分频 PLL /33
#define SYSCTL_RCC2_SYSDIV2_34  0x10800000  // 系统时钟分频 PLL /34
#define SYSCTL_RCC2_SYSDIV2_35  0x11000000  // 系统时钟分频 PLL /35
#define SYSCTL_RCC2_SYSDIV2_36  0x11800000  // 系统时钟分频 PLL /36
#define SYSCTL_RCC2_SYSDIV2_37  0x12000000  // 系统时钟分频 PLL /37
#define SYSCTL_RCC2_SYSDIV2_38  0x12800000  // 系统时钟分频 PLL /38
#define SYSCTL_RCC2_SYSDIV2_39  0x13000000  // 系统时钟分频 PLL /39
#define SYSCTL_RCC2_SYSDIV2_40  0x13800000  // 系统时钟分频 PLL /40
#define SYSCTL_RCC2_SYSDIV2_41  0x14000000  // 系统时钟分频 PLL /41
#define SYSCTL_RCC2_SYSDIV2_42  0x14800000  // 系统时钟分频 PLL /42
#define SYSCTL_RCC2_SYSDIV2_43  0x15000000  // 系统时钟分频 PLL /43
#define SYSCTL_RCC2_SYSDIV2_44  0x15800000  // 系统时钟分频 PLL /44
#define SYSCTL_RCC2_SYSDIV2_45  0x16000000  // 系统时钟分频 PLL /45
#define SYSCTL_RCC2_SYSDIV2_46  0x16800000  // 系统时钟分频 PLL /46
#define SYSCTL_RCC2_SYSDIV2_47  0x17000000  // 系统时钟分频 PLL /47
#define SYSCTL_RCC2_SYSDIV2_48  0x17800000  // 系统时钟分频 PLL /48
#define SYSCTL_RCC2_SYSDIV2_49  0x18000000  // 系统时钟分频 PLL /49
#define SYSCTL_RCC2_SYSDIV2_50  0x18800000  // 系统时钟分频 PLL /50
#define SYSCTL_RCC2_SYSDIV2_51  0x19000000  // 系统时钟分频 PLL /51
#define SYSCTL_RCC2_SYSDIV2_52  0x19800000  // 系统时钟分频 PLL /52
#define SYSCTL_RCC2_SYSDIV2_53  0x1A000000  // 系统时钟分频 PLL /53
#define SYSCTL_RCC2_SYSDIV2_54  0x1A800000  // 系统时钟分频 PLL /54
#define SYSCTL_RCC2_SYSDIV2_55  0x1B000000  // 系统时钟分频 PLL /55
#define SYSCTL_RCC2_SYSDIV2_56  0x1B800000  // 系统时钟分频 PLL /56
#define SYSCTL_RCC2_SYSDIV2_57  0x1C000000  // 系统时钟分频 PLL /57
#define SYSCTL_RCC2_SYSDIV2_58  0x1C800000  // 系统时钟分频 PLL /58
#define SYSCTL_RCC2_SYSDIV2_59  0x1D000000  // 系统时钟分频 PLL /59
#define SYSCTL_RCC2_SYSDIV2_60  0x1D800000  // 系统时钟分频 PLL /60
#define SYSCTL_RCC2_SYSDIV2_61  0x1E000000  // 系统时钟分频 PLL /61
#define SYSCTL_RCC2_SYSDIV2_62  0x1E800000  // 系统时钟分频 PLL /62
#define SYSCTL_RCC2_SYSDIV2_63  0x1F000000  // 系统时钟分频 PLL /63
#define SYSCTL_RCC2_SYSDIV2_64  0x1F800000  // 系统时钟分频 PLL /64
#define SYSCTL_RCC2_USBPWRDN    0x00004000  // Power-Down USB PLL.
#define SYSCTL_RCC2_PWRDN2      0x00002000  // PLL power down
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL bypass
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  // System Clock Source.
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // Use the main oscillator
#define SYSCTL_RCC2_OSCSRC2_IO  0x00000010  // Use the internal oscillator
#define SYSCTL_RCC2_OSCSRC2_IO4 0x00000020  // Use the internal oscillator / 4
#define SYSCTL_RCC2_OSCSRC2_30  0x00000030  // Use the 30 KHz internal osc.
#define SYSCTL_RCC2_OSCSRC2_32  0x00000070  // Use the 32 KHz external osc.
#define SYSCTL_RCC2_SYSDIV2_S   23          // Shift to the SYSDIV2 field

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPCLKCFG
// register.
//
//*****************************************************************************
#define SYSCTL_DSLPCLKCFG_D_M   0x1F800000  // Divider Field Override.
#define SYSCTL_DSLPCLKCFG_D_2   0x00800000  // System clock /2
#define SYSCTL_DSLPCLKCFG_D_3   0x01000000  // System clock /3
#define SYSCTL_DSLPCLKCFG_D_4   0x01800000  // System clock /4
#define SYSCTL_DSLPCLKCFG_D_5   0x02000000  // System clock /5
#define SYSCTL_DSLPCLKCFG_D_6   0x02800000  // System clock /6
#define SYSCTL_DSLPCLKCFG_D_7   0x03000000  // System clock /7
#define SYSCTL_DSLPCLKCFG_D_8   0x03800000  // System clock /8
#define SYSCTL_DSLPCLKCFG_D_9   0x04000000  // System clock /9
#define SYSCTL_DSLPCLKCFG_D_10  0x04800000  // System clock /10
#define SYSCTL_DSLPCLKCFG_D_11  0x05000000  // System clock /11
#define SYSCTL_DSLPCLKCFG_D_12  0x05800000  // System clock /12
#define SYSCTL_DSLPCLKCFG_D_13  0x06000000  // System clock /13
#define SYSCTL_DSLPCLKCFG_D_14  0x06800000  // System clock /14
#define SYSCTL_DSLPCLKCFG_D_15  0x07000000  // System clock /15
#define SYSCTL_DSLPCLKCFG_D_16  0x07800000  // System clock /16
#define SYSCTL_DSLPCLKCFG_D_17  0x08000000  // System clock /17
#define SYSCTL_DSLPCLKCFG_D_18  0x08800000  // System clock /18
#define SYSCTL_DSLPCLKCFG_D_19  0x09000000  // System clock /19
#define SYSCTL_DSLPCLKCFG_D_20  0x09800000  // System clock /20
#define SYSCTL_DSLPCLKCFG_D_21  0x0A000000  // System clock /21
#define SYSCTL_DSLPCLKCFG_D_22  0x0A800000  // System clock /22
#define SYSCTL_DSLPCLKCFG_D_23  0x0B000000  // System clock /23
#define SYSCTL_DSLPCLKCFG_D_24  0x0B800000  // System clock /24
#define SYSCTL_DSLPCLKCFG_D_25  0x0C000000  // System clock /25
#define SYSCTL_DSLPCLKCFG_D_26  0x0C800000  // System clock /26
#define SYSCTL_DSLPCLKCFG_D_27  0x0D000000  // System clock /27
#define SYSCTL_DSLPCLKCFG_D_28  0x0D800000  // System clock /28
#define SYSCTL_DSLPCLKCFG_D_29  0x0E000000  // System clock /29
#define SYSCTL_DSLPCLKCFG_D_30  0x0E800000  // System clock /30
#define SYSCTL_DSLPCLKCFG_D_31  0x0F000000  // System clock /31
#define SYSCTL_DSLPCLKCFG_D_32  0x0F800000  // System clock /32
#define SYSCTL_DSLPCLKCFG_D_33  0x10000000  // System clock /33
#define SYSCTL_DSLPCLKCFG_D_34  0x10800000  // System clock /34
#define SYSCTL_DSLPCLKCFG_D_35  0x11000000  // System clock /35
#define SYSCTL_DSLPCLKCFG_D_36  0x11800000  // System clock /36
#define SYSCTL_DSLPCLKCFG_D_37  0x12000000  // System clock /37
#define SYSCTL_DSLPCLKCFG_D_38  0x12800000  // System clock /38
#define SYSCTL_DSLPCLKCFG_D_39  0x13000000  // System clock /39
#define SYSCTL_DSLPCLKCFG_D_40  0x13800000  // System clock /40
#define SYSCTL_DSLPCLKCFG_D_41  0x14000000  // System clock /41
#define SYSCTL_DSLPCLKCFG_D_42  0x14800000  // System clock /42
#define SYSCTL_DSLPCLKCFG_D_43  0x15000000  // System clock /43
#define SYSCTL_DSLPCLKCFG_D_44  0x15800000  // System clock /44
#define SYSCTL_DSLPCLKCFG_D_45  0x16000000  // System clock /45
#define SYSCTL_DSLPCLKCFG_D_46  0x16800000  // System clock /46
#define SYSCTL_DSLPCLKCFG_D_47  0x17000000  // System clock /47
#define SYSCTL_DSLPCLKCFG_D_48  0x17800000  // System clock /48
#define SYSCTL_DSLPCLKCFG_D_49  0x18000000  // System clock /49
#define SYSCTL_DSLPCLKCFG_D_50  0x18800000  // System clock /50
#define SYSCTL_DSLPCLKCFG_D_51  0x19000000  // System clock /51
#define SYSCTL_DSLPCLKCFG_D_52  0x19800000  // System clock /52
#define SYSCTL_DSLPCLKCFG_D_53  0x1A000000  // System clock /53
#define SYSCTL_DSLPCLKCFG_D_54  0x1A800000  // System clock /54
#define SYSCTL_DSLPCLKCFG_D_55  0x1B000000  // System clock /55
#define SYSCTL_DSLPCLKCFG_D_56  0x1B800000  // System clock /56
#define SYSCTL_DSLPCLKCFG_D_57  0x1C000000  // System clock /57
#define SYSCTL_DSLPCLKCFG_D_58  0x1C800000  // System clock /58
#define SYSCTL_DSLPCLKCFG_D_59  0x1D000000  // System clock /59
#define SYSCTL_DSLPCLKCFG_D_60  0x1D800000  // System clock /60
#define SYSCTL_DSLPCLKCFG_D_61  0x1E000000  // System clock /61
#define SYSCTL_DSLPCLKCFG_D_62  0x1E800000  // System clock /62
#define SYSCTL_DSLPCLKCFG_D_63  0x1F000000  // System clock /63
#define SYSCTL_DSLPCLKCFG_D_64  0x1F800000  // System clock /64
#define SYSCTL_DSLPCLKCFG_O_M   0x00000070  // Clock Source.
#define SYSCTL_DSLPCLKCFG_O_IGN 0x00000000  // Do not override
#define SYSCTL_DSLPCLKCFG_O_IO  0x00000010  // Use the internal oscillator
#define SYSCTL_DSLPCLKCFG_O_30  0x00000030  // Use the 30 KHz internal osc.
#define SYSCTL_DSLPCLKCFG_O_32  0x00000070  // Use the 32 KHz external osc.
#define SYSCTL_DSLPCLKCFG_IOSC  0x00000001  // IOSC Clock Source.
#define SYSCTL_DSLPCLKCFG_D_S   23

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_CLKVCLR register.
//
//*****************************************************************************
#define SYSCTL_CLKVCLR_VERCLR   0x00000001  // Clock Verification Clear.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDOARST register.
//
//*****************************************************************************
#define SYSCTL_LDOARST_LDOARST  0x00000001  // LDO Reset.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR0 register.
//
//*****************************************************************************
#define SYSCTL_SRCR0_CAN2       0x04000000  // CAN2 Reset Control.
#define SYSCTL_SRCR0_CAN1       0x02000000  // CAN1 Reset Control.
#define SYSCTL_SRCR0_CAN0       0x01000000  // CAN0 Reset Control.
#define SYSCTL_SRCR0_PWM        0x00100000  // PWM Reset Control.
#define SYSCTL_SRCR0_ADC        0x00010000  // ADC0 Reset Control.
#define SYSCTL_SRCR0_HIB        0x00000040  // HIB Reset Control.
#define SYSCTL_SRCR0_WDT        0x00000008  // WDT Reset Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR1 register.
//
//*****************************************************************************
#define SYSCTL_SRCR1_COMP2      0x04000000  // Analog Comp 2 Reset Control.
#define SYSCTL_SRCR1_COMP1      0x02000000  // Analog Comp 1 Reset Control.
#define SYSCTL_SRCR1_COMP0      0x01000000  // Analog Comp 0 Reset Control.
#define SYSCTL_SRCR1_TIMER3     0x00080000  // Timer 3 Reset Control.
#define SYSCTL_SRCR1_TIMER2     0x00040000  // Timer 2 Reset Control.
#define SYSCTL_SRCR1_TIMER1     0x00020000  // Timer 1 Reset Control.
#define SYSCTL_SRCR1_TIMER0     0x00010000  // Timer 0 Reset Control.
#define SYSCTL_SRCR1_I2C1       0x00004000  // I2C1 Reset Control.
#define SYSCTL_SRCR1_I2C0       0x00001000  // I2C0 Reset Control.
#define SYSCTL_SRCR1_QEI1       0x00000200  // QEI1 Reset Control.
#define SYSCTL_SRCR1_QEI0       0x00000100  // QEI0 Reset Control.
#define SYSCTL_SRCR1_SSI1       0x00000020  // SSI1 Reset Control.
#define SYSCTL_SRCR1_SSI0       0x00000010  // SSI0 Reset Control.
#define SYSCTL_SRCR1_UART2      0x00000004  // UART2 Reset Control.
#define SYSCTL_SRCR1_UART1      0x00000002  // UART1 Reset Control.
#define SYSCTL_SRCR1_UART0      0x00000001  // UART0 Reset Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR2 register.
//
//*****************************************************************************
#define SYSCTL_SRCR2_EPHY0      0x40000000  // PHY0 Reset Control.
#define SYSCTL_SRCR2_EMAC0      0x10000000  // MAC0 Reset Control.
#define SYSCTL_SRCR2_USB0       0x00010000  // USB0 Reset Control.
#define SYSCTL_SRCR2_UDMA       0x00002000  // UDMA Reset Control.
#define SYSCTL_SRCR2_GPIOH      0x00000080  // Port H Reset Control.
#define SYSCTL_SRCR2_GPIOG      0x00000040  // Port G Reset Control.
#define SYSCTL_SRCR2_GPIOF      0x00000020  // Port F Reset Control.
#define SYSCTL_SRCR2_GPIOE      0x00000010  // Port E Reset Control.
#define SYSCTL_SRCR2_GPIOD      0x00000008  // Port D Reset Control.
#define SYSCTL_SRCR2_GPIOC      0x00000004  // Port C Reset Control.
#define SYSCTL_SRCR2_GPIOB      0x00000002  // Port B Reset Control.
#define SYSCTL_SRCR2_GPIOA      0x00000001  // Port A Reset Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RIS register.
//
//*****************************************************************************
#define SYSCTL_RIS_MOSCPUPRIS   0x00000100  // MOSC Power Up Raw Interrupt
                                            // Status.
#define SYSCTL_RIS_USBPLLLRIS   0x00000080  // USB PLL Lock Raw Interrupt
                                            // Status.
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status.
#define SYSCTL_RIS_CLRIS        0x00000020  // Current Limit Raw Interrupt
                                            // Status.
#define SYSCTL_RIS_IOFRIS       0x00000010  // Internal Oscillator Fault Raw
                                            // Interrupt Status.
#define SYSCTL_RIS_MOFRIS       0x00000008  // Main Oscillator Fault Raw
                                            // Interrupt Status.
#define SYSCTL_RIS_LDORIS       0x00000004  // LDO Power Unregulated Raw
                                            // Interrupt Status.
#define SYSCTL_RIS_BORRIS       0x00000002  // Brown-Out Reset Raw Interrupt
                                            // Status.
#define SYSCTL_RIS_PLLFRIS      0x00000001  // PLL Fault Raw Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_IMC register.
//
//*****************************************************************************
#define SYSCTL_IMC_MOSCPUPIM    0x00000100  // MOSC Power Up Interrupt Mask.
#define SYSCTL_IMC_USBPLLLIM    0x00000080  // USB PLL Lock Interrupt Mask.
#define SYSCTL_IMC_PLLLIM       0x00000040  // PLL Lock Interrupt Mask.
#define SYSCTL_IMC_CLIM         0x00000020  // Current Limit Interrupt Mask.
#define SYSCTL_IMC_IOFIM        0x00000010  // Internal Oscillator Fault
                                            // Interrupt Mask.
#define SYSCTL_IMC_MOFIM        0x00000008  // Main Oscillator Fault Interrupt
                                            // Mask.
#define SYSCTL_IMC_LDOIM        0x00000004  // LDO Power Unregulated Interrupt
                                            // Mask.
#define SYSCTL_IMC_BORIM        0x00000002  // Brown-Out Reset Interrupt Mask.
#define SYSCTL_IMC_PLLFIM       0x00000001  // PLL Fault Interrupt Mask.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MISC register.
//
//*****************************************************************************
#define SYSCTL_MISC_MOSCPUPMIS  0x00000100  // MOSC Power Up Masked Interrupt
                                            // Status.
#define SYSCTL_MISC_USBPLLLMIS  0x00000080  // USB PLL Lock Masked Interrupt
                                            // Status.
#define SYSCTL_MISC_PLLLMIS     0x00000040  // PLL Lock Masked Interrupt
                                            // Status.
#define SYSCTL_MISC_CLMIS       0x00000020  // Current Limit Masked Interrupt
                                            // Status.
#define SYSCTL_MISC_IOFMIS      0x00000010  // Internal Oscillator Fault Masked
                                            // Interrupt Status.
#define SYSCTL_MISC_MOFMIS      0x00000008  // Main Oscillator Fault Masked
                                            // Interrupt Status.
#define SYSCTL_MISC_LDOMIS      0x00000004  // LDO Power Unregulated Masked
                                            // Interrupt Status.
#define SYSCTL_MISC_BORMIS      0x00000002  // BOR Masked Interrupt Status.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC0 register.
//
//*****************************************************************************
#define SYSCTL_RCGC0_CAN2       0x04000000  // CAN2 Clock Gating Control.
#define SYSCTL_RCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control.
#define SYSCTL_RCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control.
#define SYSCTL_RCGC0_PWM        0x00100000  // PWM Clock Gating Control.
#define SYSCTL_RCGC0_ADC        0x00010000  // ADC0 Clock Gating Control.
#define SYSCTL_RCGC0_ADCSPD_M   0x00000F00  // ADC Sample Speed.
#define SYSCTL_RCGC0_ADCSPD125K 0x00000000  // 125K samples/second
#define SYSCTL_RCGC0_ADCSPD250K 0x00000100  // 250K samples/second
#define SYSCTL_RCGC0_ADCSPD500K 0x00000200  // 500K samples/second
#define SYSCTL_RCGC0_ADCSPD1M   0x00000300  // 1M samples/second
#define SYSCTL_RCGC0_HIB        0x00000040  // HIB Clock Gating Control.
#define SYSCTL_RCGC0_WDT        0x00000008  // WDT Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC1 register.
//
//*****************************************************************************
#define SYSCTL_RCGC1_COMP2      0x04000000  // Analog Comparator 2 Clock
                                            // Gating.
#define SYSCTL_RCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock
                                            // Gating.
#define SYSCTL_RCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock
                                            // Gating.
#define SYSCTL_RCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control.
#define SYSCTL_RCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control.
#define SYSCTL_RCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control.
#define SYSCTL_RCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control.
#define SYSCTL_RCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control.
#define SYSCTL_RCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control.
#define SYSCTL_RCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control.
#define SYSCTL_RCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control.
#define SYSCTL_RCGC1_SSI1       0x00000020  // SSI1 Clock Gating Control.
#define SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control.
#define SYSCTL_RCGC1_UART2      0x00000004  // UART2 Clock Gating Control.
#define SYSCTL_RCGC1_UART1      0x00000002  // UART1 Clock Gating Control.
#define SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC2 register.
//
//*****************************************************************************
#define SYSCTL_RCGC2_EPHY0      0x40000000  // PHY0 Clock Gating Control.
#define SYSCTL_RCGC2_EMAC0      0x10000000  // MAC0 Clock Gating Control.
#define SYSCTL_RCGC2_USB0       0x00010000  // USB0 Clock Gating Control.
#define SYSCTL_RCGC2_UDMA       0x00002000  // UDMA Clock Gating Control.
#define SYSCTL_RCGC2_GPIOH      0x00000080  // Port H Clock Gating Control.
#define SYSCTL_RCGC2_GPIOG      0x00000040  // Port G Clock Gating Control.
#define SYSCTL_RCGC2_GPIOF      0x00000020  // Port F Clock Gating Control.
#define SYSCTL_RCGC2_GPIOE      0x00000010  // Port E Clock Gating Control.
#define SYSCTL_RCGC2_GPIOD      0x00000008  // Port D Clock Gating Control.
#define SYSCTL_RCGC2_GPIOC      0x00000004  // Port C Clock Gating Control.
#define SYSCTL_RCGC2_GPIOB      0x00000002  // Port B Clock Gating Control.
#define SYSCTL_RCGC2_GPIOA      0x00000001  // Port A Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC0 register.
//
//*****************************************************************************
#define SYSCTL_SCGC0_CAN2       0x04000000  // CAN2 Clock Gating Control.
#define SYSCTL_SCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control.
#define SYSCTL_SCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control.
#define SYSCTL_SCGC0_PWM        0x00100000  // PWM Clock Gating Control.
#define SYSCTL_SCGC0_ADC        0x00010000  // ADC0 Clock Gating Control.
#define SYSCTL_SCGC0_ADCSPD_M   0x00000F00  // ADC Sample Speed.
#define SYSCTL_SCGC0_ADCSPD125K 0x00000000  // 125K samples/second
#define SYSCTL_SCGC0_ADCSPD250K 0x00000100  // 250K samples/second
#define SYSCTL_SCGC0_ADCSPD500K 0x00000200  // 500K samples/second
#define SYSCTL_SCGC0_ADCSPD1M   0x00000300  // 1M samples/second
#define SYSCTL_SCGC0_HIB        0x00000040  // HIB Clock Gating Control.
#define SYSCTL_SCGC0_WDT        0x00000008  // WDT Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC1 register.
//
//*****************************************************************************
#define SYSCTL_SCGC1_COMP2      0x04000000  // Analog Comparator 2 Clock
                                            // Gating.
#define SYSCTL_SCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock
                                            // Gating.
#define SYSCTL_SCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock
                                            // Gating.
#define SYSCTL_SCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control.
#define SYSCTL_SCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control.
#define SYSCTL_SCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control.
#define SYSCTL_SCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control.
#define SYSCTL_SCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control.
#define SYSCTL_SCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control.
#define SYSCTL_SCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control.
#define SYSCTL_SCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control.
#define SYSCTL_SCGC1_SSI1       0x00000020  // SSI1 Clock Gating Control.
#define SYSCTL_SCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control.
#define SYSCTL_SCGC1_UART2      0x00000004  // UART2 Clock Gating Control.
#define SYSCTL_SCGC1_UART1      0x00000002  // UART1 Clock Gating Control.
#define SYSCTL_SCGC1_UART0      0x00000001  // UART0 Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC2 register.
//
//*****************************************************************************
#define SYSCTL_SCGC2_EPHY0      0x40000000  // PHY0 Clock Gating Control.
#define SYSCTL_SCGC2_EMAC0      0x10000000  // MAC0 Clock Gating Control.
#define SYSCTL_SCGC2_USB0       0x00010000  // USB0 Clock Gating Control.
#define SYSCTL_SCGC2_UDMA       0x00002000  // UDMA Clock Gating Control.
#define SYSCTL_SCGC2_GPIOH      0x00000080  // Port H Clock Gating Control.
#define SYSCTL_SCGC2_GPIOG      0x00000040  // Port G Clock Gating Control.
#define SYSCTL_SCGC2_GPIOF      0x00000020  // Port F Clock Gating Control.
#define SYSCTL_SCGC2_GPIOE      0x00000010  // Port E Clock Gating Control.
#define SYSCTL_SCGC2_GPIOD      0x00000008  // Port D Clock Gating Control.
#define SYSCTL_SCGC2_GPIOC      0x00000004  // Port C Clock Gating Control.
#define SYSCTL_SCGC2_GPIOB      0x00000002  // Port B Clock Gating Control.
#define SYSCTL_SCGC2_GPIOA      0x00000001  // Port A Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC0 register.
//
//*****************************************************************************
#define SYSCTL_DCGC0_CAN2       0x04000000  // CAN2 Clock Gating Control.
#define SYSCTL_DCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control.
#define SYSCTL_DCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control.
#define SYSCTL_DCGC0_PWM        0x00100000  // PWM Clock Gating Control.
#define SYSCTL_DCGC0_ADC        0x00010000  // ADC0 Clock Gating Control.
#define SYSCTL_DCGC0_ADCSPD_M   0x00000F00  // ADC Sample Speed.
#define SYSCTL_DCGC0_ADCSPD125K 0x00000000  // 125K samples/second
#define SYSCTL_DCGC0_ADCSPD250K 0x00000100  // 250K samples/second
#define SYSCTL_DCGC0_ADCSPD500K 0x00000200  // 500K samples/second
#define SYSCTL_DCGC0_ADCSPD1M   0x00000300  // 1M samples/second
#define SYSCTL_DCGC0_HIB        0x00000040  // HIB Clock Gating Control.
#define SYSCTL_DCGC0_WDT        0x00000008  // WDT Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC1 register.
//
//*****************************************************************************
#define SYSCTL_DCGC1_COMP2      0x04000000  // Analog Comparator 2 Clock
                                            // Gating.
#define SYSCTL_DCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock
                                            // Gating.
#define SYSCTL_DCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock
                                            // Gating.
#define SYSCTL_DCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control.
#define SYSCTL_DCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control.
#define SYSCTL_DCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control.
#define SYSCTL_DCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control.
#define SYSCTL_DCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control.
#define SYSCTL_DCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control.
#define SYSCTL_DCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control.
#define SYSCTL_DCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control.
#define SYSCTL_DCGC1_SSI1       0x00000020  // SSI1 Clock Gating Control.
#define SYSCTL_DCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control.
#define SYSCTL_DCGC1_UART2      0x00000004  // UART2 Clock Gating Control.
#define SYSCTL_DCGC1_UART1      0x00000002  // UART1 Clock Gating Control.
#define SYSCTL_DCGC1_UART0      0x00000001  // UART0 Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC2 register.
//
//*****************************************************************************
#define SYSCTL_DCGC2_EPHY0      0x40000000  // PHY0 Clock Gating Control.
#define SYSCTL_DCGC2_EMAC0      0x10000000  // MAC0 Clock Gating Control.
#define SYSCTL_DCGC2_USB0       0x00010000  // USB0 Clock Gating Control.
#define SYSCTL_DCGC2_UDMA       0x00002000  // UDMA Clock Gating Control.
#define SYSCTL_DCGC2_GPIOH      0x00000080  // Port H Clock Gating Control.
#define SYSCTL_DCGC2_GPIOG      0x00000040  // Port G Clock Gating Control.
#define SYSCTL_DCGC2_GPIOF      0x00000020  // Port F Clock Gating Control.
#define SYSCTL_DCGC2_GPIOE      0x00000010  // Port E Clock Gating Control.
#define SYSCTL_DCGC2_GPIOD      0x00000008  // Port D Clock Gating Control.
#define SYSCTL_DCGC2_GPIOC      0x00000004  // Port C Clock Gating Control.
#define SYSCTL_DCGC2_GPIOB      0x00000002  // Port B Clock Gating Control.
#define SYSCTL_DCGC2_GPIOA      0x00000001  // Port A Clock Gating Control.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC5 register.
//
//*****************************************************************************
#define SYSCTL_DC5_PWMFAULT3    0x08000000  // PWM Fault 3 Pin Present.
#define SYSCTL_DC5_PWMFAULT2    0x04000000  // PWM Fault 2 Pin Present.
#define SYSCTL_DC5_PWMFAULT1    0x02000000  // PWM Fault 1 Pin Present.
#define SYSCTL_DC5_PWMFAULT0    0x01000000  // PWM Fault 0 Pin Present.
#define SYSCTL_DC5_PWMEFLT      0x00200000  // PWM Extended Fault feature is
                                            // active.
#define SYSCTL_DC5_PWMESYNC     0x00100000  // PWM Extended SYNC feature is
                                            // active.
#define SYSCTL_DC5_PWM7         0x00000080  // PWM7 Pin Present.
#define SYSCTL_DC5_PWM6         0x00000040  // PWM6 Pin Present.
#define SYSCTL_DC5_PWM5         0x00000020  // PWM5 Pin Present.
#define SYSCTL_DC5_PWM4         0x00000010  // PWM4 Pin Present.
#define SYSCTL_DC5_PWM3         0x00000008  // PWM3 Pin Present.
#define SYSCTL_DC5_PWM2         0x00000004  // PWM2 Pin Present.
#define SYSCTL_DC5_PWM1         0x00000002  // PWM1 Pin Present.
#define SYSCTL_DC5_PWM0         0x00000001  // PWM0 Pin Present.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC6 register.
//
//*****************************************************************************
#define SYSCTL_DC6_USB0_M       0x00000003  // This specifies that USB0 is
                                            // present and its capability.
#define SYSCTL_DC6_USB0_HOSTDEV 0x00000002  // USB is DEVICE or HOST
#define SYSCTL_DC6_USB0_OTG     0x00000003  // USB is OTG

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_GPIOHSCTL
// register.
//
//*****************************************************************************
#define SYSCTL_GPIOHSCTL_PORTH  0x00000080  // Port H High-Speed.
#define SYSCTL_GPIOHSCTL_PORTG  0x00000040  // Port G High-Speed.
#define SYSCTL_GPIOHSCTL_PORTF  0x00000020  // Port F High-Speed.
#define SYSCTL_GPIOHSCTL_PORTE  0x00000010  // Port E High-Speed.
#define SYSCTL_GPIOHSCTL_PORTD  0x00000008  // Port D High-Speed.
#define SYSCTL_GPIOHSCTL_PORTC  0x00000004  // Port C High-Speed.
#define SYSCTL_GPIOHSCTL_PORTB  0x00000002  // Port B High-Speed.
#define SYSCTL_GPIOHSCTL_PORTA  0x00000001  // Port A High-Speed.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MOSCCTL register.
//
//*****************************************************************************
#define SYSCTL_MOSCCTL_CVAL     0x00000001  // Clock Validation for MOSC.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC7 register.
//
//*****************************************************************************
#define SYSCTL_DC7_SSI1_TX      0x02000000  // SSI1 TX on uDMA Ch25.
#define SYSCTL_DC7_SSI1_RX      0x01000000  // SSI1 RX on uDMA Ch24.
#define SYSCTL_DC7_UART1_TX     0x00800000  // UART1 TX on uDMA Ch23.
#define SYSCTL_DC7_UART1_RX     0x00400000  // UART1 RX on uDMA Ch22.
#define SYSCTL_DC7_SSI0_TX      0x00000800  // SSI0 TX on uDMA Ch11.
#define SYSCTL_DC7_SSI0_RX      0x00000400  // SSI0 RX on uDMA Ch10.
#define SYSCTL_DC7_UART0_TX     0x00000200  // UART0 TX on uDMA Ch9.
#define SYSCTL_DC7_UART0_RX     0x00000100  // UART0 RX on uDMA Ch8.
#define SYSCTL_DC7_USB_EP3_TX   0x00000020  // USB EP3 TX on uDMA Ch5.
#define SYSCTL_DC7_USB_EP3_RX   0x00000010  // USB EP3 RX on uDMA Ch4.
#define SYSCTL_DC7_USB_EP2_TX   0x00000008  // USB EP2 TX on uDMA Ch3.
#define SYSCTL_DC7_USB_EP2_RX   0x00000004  // USB EP2 RX on uDMA Ch2.
#define SYSCTL_DC7_USB_EP1_TX   0x00000002  // USB EP1 TX on uDMA Ch1.
#define SYSCTL_DC7_USB_EP1_RX   0x00000001  // USB EP1 RX on uDMA Ch0.


//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_DID0
// register.
//
//*****************************************************************************
#define SYSCTL_DID0_VER_MASK    0x70000000  // DID0 version mask
#define SYSCTL_DID0_CLASS_MASK  0x00FF0000  // Device Class
#define SYSCTL_DID0_MAJ_MASK    0x0000FF00  // Major revision mask
#define SYSCTL_DID0_MAJ_A       0x00000000  // Major revision A
#define SYSCTL_DID0_MAJ_B       0x00000100  // Major revision B
#define SYSCTL_DID0_MAJ_C       0x00000200  // Major revision C
#define SYSCTL_DID0_MIN_MASK    0x000000FF  // Minor revision mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_DID1
// register.
//
//*****************************************************************************
#define SYSCTL_DID1_VER_MASK    0xF0000000  // Register version mask
#define SYSCTL_DID1_FAM_MASK    0x0F000000  // Family mask
#define SYSCTL_DID1_FAM_S       0x00000000  // Stellaris family
#define SYSCTL_DID1_PRTNO_MASK  0x00FF0000  // Part number mask
#define SYSCTL_DID1_PINCNT_MASK 0x0000E000  // Pin count
#define SYSCTL_DID1_TEMP_MASK   0x000000E0  // Temperature range mask
#define SYSCTL_DID1_PKG_MASK    0x00000018  // Package mask
#define SYSCTL_DID1_QUAL_MASK   0x00000003  // Qualification status mask
#define SYSCTL_DID1_PRTNO_SHIFT 16

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_DC0
// register.
//
//*****************************************************************************
#define SYSCTL_DC0_SRAMSZ_MASK  0xFFFF0000  // SRAM size mask
#define SYSCTL_DC0_FLASHSZ_MASK 0x0000FFFF  // Flash size mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_DC1
// register.
//
//*****************************************************************************
#define SYSCTL_DC1_SYSDIV_MASK  0x0000F000  // Minimum system divider mask
#define SYSCTL_DC1_ADCSPD_MASK  0x00000F00  // ADC speed mask
#define SYSCTL_DC1_WDOG         0x00000008  // Watchdog present

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_DC2
// register.
//
//*****************************************************************************
#define SYSCTL_DC2_I2C          0x00001000  // I2C present
#define SYSCTL_DC2_QEI          0x00000100  // QEI present
#define SYSCTL_DC2_SSI          0x00000010  // SSI present

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_DC3
// register.
//
//*****************************************************************************
#define SYSCTL_DC3_MC_FAULT0    0x00008000  // MC0 fault pin present

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// SYSCTL_PBORCTL register.
//
//*****************************************************************************
#define SYSCTL_PBORCTL_BOR_MASK 0x0000FFFC  // BOR wait timer
#define SYSCTL_PBORCTL_BOR_SH   2

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// SYSCTL_LDOPCTL register.
//
//*****************************************************************************
#define SYSCTL_LDOPCTL_MASK     0x0000003F  // Voltage adjust mask

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_SRCR0,
// SYSCTL_RCGC0, SYSCTL_SCGC0, and SYSCTL_DCGC0 registers.
//
//*****************************************************************************
#define SYSCTL_SET0_CAN2        0x04000000  // CAN 2 module
#define SYSCTL_SET0_CAN1        0x02000000  // CAN 1 module
#define SYSCTL_SET0_CAN0        0x01000000  // CAN 0 module
#define SYSCTL_SET0_PWM         0x00100000  // PWM module
#define SYSCTL_SET0_ADC         0x00010000  // ADC module
#define SYSCTL_SET0_ADCSPD_MASK 0x00000F00  // ADC speed mask
#define SYSCTL_SET0_ADCSPD_125K 0x00000000  // 125Ksps ADC
#define SYSCTL_SET0_ADCSPD_250K 0x00000100  // 250Ksps ADC
#define SYSCTL_SET0_ADCSPD_500K 0x00000200  // 500Ksps ADC
#define SYSCTL_SET0_ADCSPD_1M   0x00000300  // 1Msps ADC
#define SYSCTL_SET0_HIB         0x00000040  // Hibernation module
#define SYSCTL_SET0_WDOG        0x00000008  // Watchdog module

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_SRCR1,
// SYSCTL_RCGC1, SYSCTL_SCGC1, and SYSCTL_DCGC1 registers.
//
//*****************************************************************************
#define SYSCTL_SET1_COMP2       0x04000000  // Analog comparator module 2
#define SYSCTL_SET1_COMP1       0x02000000  // Analog comparator module 1
#define SYSCTL_SET1_COMP0       0x01000000  // Analog comparator module 0
#define SYSCTL_SET1_TIMER3      0x00080000  // Timer module 3
#define SYSCTL_SET1_TIMER2      0x00040000  // Timer module 2
#define SYSCTL_SET1_TIMER1      0x00020000  // Timer module 1
#define SYSCTL_SET1_TIMER0      0x00010000  // Timer module 0
#define SYSCTL_SET1_I2C1        0x00002000  // I2C module 1
#define SYSCTL_SET1_I2C0        0x00001000  // I2C module 0
#define SYSCTL_SET1_I2C         0x00001000  // I2C module
#define SYSCTL_SET1_QEI1        0x00000200  // QEI module 1
#define SYSCTL_SET1_QEI         0x00000100  // QEI module
#define SYSCTL_SET1_QEI0        0x00000100  // QEI module 0
#define SYSCTL_SET1_SSI1        0x00000020  // SSI module 1
#define SYSCTL_SET1_SSI0        0x00000010  // SSI module 0
#define SYSCTL_SET1_SSI         0x00000010  // SSI module
#define SYSCTL_SET1_UART2       0x00000004  // UART module 2
#define SYSCTL_SET1_UART1       0x00000002  // UART module 1
#define SYSCTL_SET1_UART0       0x00000001  // UART module 0

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_SRCR2,
// SYSCTL_RCGC2, SYSCTL_SCGC2, and SYSCTL_DCGC2 registers.
//
//*****************************************************************************
#define SYSCTL_SET2_ETH         0x50000000  // ETH module
#define SYSCTL_SET2_GPIOH       0x00000080  // GPIO H module
#define SYSCTL_SET2_GPIOG       0x00000040  // GPIO G module
#define SYSCTL_SET2_GPIOF       0x00000020  // GPIO F module
#define SYSCTL_SET2_GPIOE       0x00000010  // GPIO E module
#define SYSCTL_SET2_GPIOD       0x00000008  // GPIO D module
#define SYSCTL_SET2_GPIOC       0x00000004  // GPIO C module
#define SYSCTL_SET2_GPIOB       0x00000002  // GPIO B module
#define SYSCTL_SET2_GPIOA       0x00000001  // GIPO A module

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_RIS,
// SYSCTL_IMC, and SYSCTL_IMS registers.
//
//*****************************************************************************
#define SYSCTL_INT_PLL_LOCK     0x00000040  // PLL lock interrupt
#define SYSCTL_INT_CUR_LIMIT    0x00000020  // Current limit interrupt
#define SYSCTL_INT_IOSC_FAIL    0x00000010  // Internal oscillator failure int
#define SYSCTL_INT_MOSC_FAIL    0x00000008  // Main oscillator failure int
#define SYSCTL_INT_POR          0x00000004  // Power on reset interrupt
#define SYSCTL_INT_BOR          0x00000002  // Brown out interrupt
#define SYSCTL_INT_PLL_FAIL     0x00000001  // PLL failure interrupt

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_RESC
// register.
//
//*****************************************************************************
#define SYSCTL_RESC_WDOG        0x00000008  // Watchdog reset

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_RCC
// register.
//
//*****************************************************************************
#define SYSCTL_RCC_SYSDIV_MASK  0x07800000  // System clock divider
#define SYSCTL_RCC_USE_SYSDIV   0x00400000  // Use sytem clock divider
#define SYSCTL_RCC_USE_PWMDIV   0x00100000  // Use PWM clock divider
#define SYSCTL_RCC_PWMDIV_MASK  0x000E0000  // PWM clock divider
#define SYSCTL_RCC_OE           0x00001000  // PLL output enable
#define SYSCTL_RCC_XTAL_3_68MHz 0x00000140  // Using a 3.6864 MHz crystal
#define SYSCTL_RCC_XTAL_4MHz    0x00000180  // Using a 4 MHz crystal
#define SYSCTL_RCC_XTAL_MASK    0x000003C0  // Crystal attached to main osc
#define SYSCTL_RCC_OSCSRC_MASK  0x00000030  // Oscillator input select
#define SYSCTL_RCC_SYSDIV_SHIFT 23          // Shift to the SYSDIV field
#define SYSCTL_RCC_PWMDIV_SHIFT 17          // Shift to the PWMDIV field
#define SYSCTL_RCC_XTAL_SHIFT   6           // Shift to the XTAL field
#define SYSCTL_RCC_OSCSRC_SHIFT 4           // Shift to the OSCSRC field

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_PLLCFG
// register.
//
//*****************************************************************************
#define SYSCTL_PLLCFG_OD_MASK   0x0000C000  // Output divider
#define SYSCTL_PLLCFG_F_MASK    0x00003FE0  // PLL multiplier
#define SYSCTL_PLLCFG_R_MASK    0x0000001F  // Input predivider
#define SYSCTL_PLLCFG_F_SHIFT   5
#define SYSCTL_PLLCFG_R_SHIFT   0

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the SYSCTL_RCC2
// register.
//
//*****************************************************************************
#define SYSCTL_RCC2_SYSDIV2_MSK 0x1F800000  // System clock divider
#define SYSCTL_RCC2_OSCSRC2_MSK 0x00000070  // Oscillator input select

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// SYSCTL_DSLPCLKCFG register.
//
//*****************************************************************************
#define SYSCTL_DSLPCLKCFG_D_MSK 0x1F800000  // Deep sleep system clock override
#define SYSCTL_DSLPCLKCFG_O_MSK 0x00000070  // Deep sleep oscillator override

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// SYSCTL_CLKVCLR register.
//
//*****************************************************************************
#define SYSCTL_CLKVCLR_CLR      0x00000001  // Clear clock verification fault

//*****************************************************************************
//
// The following are deprecated defines for the bit fields in the
// SYSCTL_LDOARST register.
//
//*****************************************************************************
#define SYSCTL_LDOARST_ARST     0x00000001  // Allow LDO to reset device

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_INT_TYPE register.
//
//*****************************************************************************
#define NVIC_INT_TYPE_LINES_M   0x0000001F  // Number of interrupt lines (x32)
#define NVIC_INT_TYPE_LINES_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CTRL register.
//
//*****************************************************************************
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_RELOAD register.
//
//*****************************************************************************
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value
#define NVIC_ST_RELOAD_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CURRENT
// register.
//
//*****************************************************************************
#define NVIC_ST_CURRENT_M       0x00FFFFFF  // Counter current value
#define NVIC_ST_CURRENT_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CAL register.
//
//*****************************************************************************
#define NVIC_ST_CAL_NOREF       0x80000000  // No reference clock
#define NVIC_ST_CAL_SKEW        0x40000000  // Clock skew
#define NVIC_ST_CAL_ONEMS_M     0x00FFFFFF  // 1ms reference value
#define NVIC_ST_CAL_ONEMS_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN0 register.
//
//*****************************************************************************
#define NVIC_EN0_INT31          0x80000000  // Interrupt 31 enable
#define NVIC_EN0_INT30          0x40000000  // Interrupt 30 enable
#define NVIC_EN0_INT29          0x20000000  // Interrupt 29 enable
#define NVIC_EN0_INT28          0x10000000  // Interrupt 28 enable
#define NVIC_EN0_INT27          0x08000000  // Interrupt 27 enable
#define NVIC_EN0_INT26          0x04000000  // Interrupt 26 enable
#define NVIC_EN0_INT25          0x02000000  // Interrupt 25 enable
#define NVIC_EN0_INT24          0x01000000  // Interrupt 24 enable
#define NVIC_EN0_INT23          0x00800000  // Interrupt 23 enable
#define NVIC_EN0_INT22          0x00400000  // Interrupt 22 enable
#define NVIC_EN0_INT21          0x00200000  // Interrupt 21 enable
#define NVIC_EN0_INT20          0x00100000  // Interrupt 20 enable
#define NVIC_EN0_INT19          0x00080000  // Interrupt 19 enable
#define NVIC_EN0_INT18          0x00040000  // Interrupt 18 enable
#define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable
#define NVIC_EN0_INT16          0x00010000  // Interrupt 16 enable
#define NVIC_EN0_INT15          0x00008000  // Interrupt 15 enable
#define NVIC_EN0_INT14          0x00004000  // Interrupt 14 enable
#define NVIC_EN0_INT13          0x00002000  // Interrupt 13 enable
#define NVIC_EN0_INT12          0x00001000  // Interrupt 12 enable
#define NVIC_EN0_INT11          0x00000800  // Interrupt 11 enable
#define NVIC_EN0_INT10          0x00000400  // Interrupt 10 enable
#define NVIC_EN0_INT9           0x00000200  // Interrupt 9 enable
#define NVIC_EN0_INT8           0x00000100  // Interrupt 8 enable
#define NVIC_EN0_INT7           0x00000080  // Interrupt 7 enable
#define NVIC_EN0_INT6           0x00000040  // Interrupt 6 enable
#define NVIC_EN0_INT5           0x00000020  // Interrupt 5 enable
#define NVIC_EN0_INT4           0x00000010  // Interrupt 4 enable
#define NVIC_EN0_INT3           0x00000008  // Interrupt 3 enable
#define NVIC_EN0_INT2           0x00000004  // Interrupt 2 enable
#define NVIC_EN0_INT1           0x00000002  // Interrupt 1 enable
#define NVIC_EN0_INT0           0x00000001  // Interrupt 0 enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN1 register.
//
//*****************************************************************************
#define NVIC_EN1_INT59          0x08000000  // Interrupt 59 enable
#define NVIC_EN1_INT58          0x04000000  // Interrupt 58 enable
#define NVIC_EN1_INT57          0x02000000  // Interrupt 57 enable
#define NVIC_EN1_INT56          0x01000000  // Interrupt 56 enable
#define NVIC_EN1_INT55          0x00800000  // Interrupt 55 enable
#define NVIC_EN1_INT54          0x00400000  // Interrupt 54 enable
#define NVIC_EN1_INT53          0x00200000  // Interrupt 53 enable
#define NVIC_EN1_INT52          0x00100000  // Interrupt 52 enable
#define NVIC_EN1_INT51          0x00080000  // Interrupt 51 enable
#define NVIC_EN1_INT50          0x00040000  // Interrupt 50 enable
#define NVIC_EN1_INT49          0x00020000  // Interrupt 49 enable
#define NVIC_EN1_INT48          0x00010000  // Interrupt 48 enable
#define NVIC_EN1_INT47          0x00008000  // Interrupt 47 enable
#define NVIC_EN1_INT46          0x00004000  // Interrupt 46 enable
#define NVIC_EN1_INT45          0x00002000  // Interrupt 45 enable
#define NVIC_EN1_INT44          0x00001000  // Interrupt 44 enable
#define NVIC_EN1_INT43          0x00000800  // Interrupt 43 enable
#define NVIC_EN1_INT42          0x00000400  // Interrupt 42 enable
#define NVIC_EN1_INT41          0x00000200  // Interrupt 41 enable
#define NVIC_EN1_INT40          0x00000100  // Interrupt 40 enable
#define NVIC_EN1_INT39          0x00000080  // Interrupt 39 enable
#define NVIC_EN1_INT38          0x00000040  // Interrupt 38 enable
#define NVIC_EN1_INT37          0x00000020  // Interrupt 37 enable
#define NVIC_EN1_INT36          0x00000010  // Interrupt 36 enable
#define NVIC_EN1_INT35          0x00000008  // Interrupt 35 enable
#define NVIC_EN1_INT34          0x00000004  // Interrupt 34 enable
#define NVIC_EN1_INT33          0x00000002  // Interrupt 33 enable
#define NVIC_EN1_INT32          0x00000001  // Interrupt 32 enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS0 register.
//
//*****************************************************************************
#define NVIC_DIS0_INT31         0x80000000  // Interrupt 31 disable
#define NVIC_DIS0_INT30         0x40000000  // Interrupt 30 disable
#define NVIC_DIS0_INT29         0x20000000  // Interrupt 29 disable
#define NVIC_DIS0_INT28         0x10000000  // Interrupt 28 disable
#define NVIC_DIS0_INT27         0x08000000  // Interrupt 27 disable
#define NVIC_DIS0_INT26         0x04000000  // Interrupt 26 disable
#define NVIC_DIS0_INT25         0x02000000  // Interrupt 25 disable
#define NVIC_DIS0_INT24         0x01000000  // Interrupt 24 disable
#define NVIC_DIS0_INT23         0x00800000  // Interrupt 23 disable
#define NVIC_DIS0_INT22         0x00400000  // Interrupt 22 disable
#define NVIC_DIS0_INT21         0x00200000  // Interrupt 21 disable
#define NVIC_DIS0_INT20         0x00100000  // Interrupt 20 disable
#define NVIC_DIS0_INT19         0x00080000  // Interrupt 19 disable
#define NVIC_DIS0_INT18         0x00040000  // Interrupt 18 disable
#define NVIC_DIS0_INT17         0x00020000  // Interrupt 17 disable
#define NVIC_DIS0_INT16         0x00010000  // Interrupt 16 disable
#define NVIC_DIS0_INT15         0x00008000  // Interrupt 15 disable
#define NVIC_DIS0_INT14         0x00004000  // Interrupt 14 disable
#define NVIC_DIS0_INT13         0x00002000  // Interrupt 13 disable
#define NVIC_DIS0_INT12         0x00001000  // Interrupt 12 disable
#define NVIC_DIS0_INT11         0x00000800  // Interrupt 11 disable
#define NVIC_DIS0_INT10         0x00000400  // Interrupt 10 disable
#define NVIC_DIS0_INT9          0x00000200  // Interrupt 9 disable
#define NVIC_DIS0_INT8          0x00000100  // Interrupt 8 disable
#define NVIC_DIS0_INT7          0x00000080  // Interrupt 7 disable
#define NVIC_DIS0_INT6          0x00000040  // Interrupt 6 disable
#define NVIC_DIS0_INT5          0x00000020  // Interrupt 5 disable
#define NVIC_DIS0_INT4          0x00000010  // Interrupt 4 disable
#define NVIC_DIS0_INT3          0x00000008  // Interrupt 3 disable
#define NVIC_DIS0_INT2          0x00000004  // Interrupt 2 disable
#define NVIC_DIS0_INT1          0x00000002  // Interrupt 1 disable
#define NVIC_DIS0_INT0          0x00000001  // Interrupt 0 disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS1 register.
//
//*****************************************************************************
#define NVIC_DIS1_INT59         0x08000000  // Interrupt 59 disable
#define NVIC_DIS1_INT58         0x04000000  // Interrupt 58 disable
#define NVIC_DIS1_INT57         0x02000000  // Interrupt 57 disable
#define NVIC_DIS1_INT56         0x01000000  // Interrupt 56 disable
#define NVIC_DIS1_INT55         0x00800000  // Interrupt 55 disable
#define NVIC_DIS1_INT54         0x00400000  // Interrupt 54 disable
#define NVIC_DIS1_INT53         0x00200000  // Interrupt 53 disable
#define NVIC_DIS1_INT52         0x00100000  // Interrupt 52 disable
#define NVIC_DIS1_INT51         0x00080000  // Interrupt 51 disable
#define NVIC_DIS1_INT50         0x00040000  // Interrupt 50 disable
#define NVIC_DIS1_INT49         0x00020000  // Interrupt 49 disable
#define NVIC_DIS1_INT48         0x00010000  // Interrupt 48 disable
#define NVIC_DIS1_INT47         0x00008000  // Interrupt 47 disable
#define NVIC_DIS1_INT46         0x00004000  // Interrupt 46 disable
#define NVIC_DIS1_INT45         0x00002000  // Interrupt 45 disable
#define NVIC_DIS1_INT44         0x00001000  // Interrupt 44 disable
#define NVIC_DIS1_INT43         0x00000800  // Interrupt 43 disable
#define NVIC_DIS1_INT42         0x00000400  // Interrupt 42 disable
#define NVIC_DIS1_INT41         0x00000200  // Interrupt 41 disable
#define NVIC_DIS1_INT40         0x00000100  // Interrupt 40 disable
#define NVIC_DIS1_INT39         0x00000080  // Interrupt 39 disable
#define NVIC_DIS1_INT38         0x00000040  // Interrupt 38 disable
#define NVIC_DIS1_INT37         0x00000020  // Interrupt 37 disable
#define NVIC_DIS1_INT36         0x00000010  // Interrupt 36 disable
#define NVIC_DIS1_INT35         0x00000008  // Interrupt 35 disable
#define NVIC_DIS1_INT34         0x00000004  // Interrupt 34 disable
#define NVIC_DIS1_INT33         0x00000002  // Interrupt 33 disable
#define NVIC_DIS1_INT32         0x00000001  // Interrupt 32 disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND0 register.
//
//*****************************************************************************
#define NVIC_PEND0_INT31        0x80000000  // Interrupt 31 pend
#define NVIC_PEND0_INT30        0x40000000  // Interrupt 30 pend
#define NVIC_PEND0_INT29        0x20000000  // Interrupt 29 pend
#define NVIC_PEND0_INT28        0x10000000  // Interrupt 28 pend
#define NVIC_PEND0_INT27        0x08000000  // Interrupt 27 pend
#define NVIC_PEND0_INT26        0x04000000  // Interrupt 26 pend
#define NVIC_PEND0_INT25        0x02000000  // Interrupt 25 pend
#define NVIC_PEND0_INT24        0x01000000  // Interrupt 24 pend
#define NVIC_PEND0_INT23        0x00800000  // Interrupt 23 pend
#define NVIC_PEND0_INT22        0x00400000  // Interrupt 22 pend
#define NVIC_PEND0_INT21        0x00200000  // Interrupt 21 pend
#define NVIC_PEND0_INT20        0x00100000  // Interrupt 20 pend
#define NVIC_PEND0_INT19        0x00080000  // Interrupt 19 pend
#define NVIC_PEND0_INT18        0x00040000  // Interrupt 18 pend
#define NVIC_PEND0_INT17        0x00020000  // Interrupt 17 pend
#define NVIC_PEND0_INT16        0x00010000  // Interrupt 16 pend
#define NVIC_PEND0_INT15        0x00008000  // Interrupt 15 pend
#define NVIC_PEND0_INT14        0x00004000  // Interrupt 14 pend
#define NVIC_PEND0_INT13        0x00002000  // Interrupt 13 pend
#define NVIC_PEND0_INT12        0x00001000  // Interrupt 12 pend
#define NVIC_PEND0_INT11        0x00000800  // Interrupt 11 pend
#define NVIC_PEND0_INT10        0x00000400  // Interrupt 10 pend
#define NVIC_PEND0_INT9         0x00000200  // Interrupt 9 pend
#define NVIC_PEND0_INT8         0x00000100  // Interrupt 8 pend
#define NVIC_PEND0_INT7         0x00000080  // Interrupt 7 pend
#define NVIC_PEND0_INT6         0x00000040  // Interrupt 6 pend
#define NVIC_PEND0_INT5         0x00000020  // Interrupt 5 pend
#define NVIC_PEND0_INT4         0x00000010  // Interrupt 4 pend
#define NVIC_PEND0_INT3         0x00000008  // Interrupt 3 pend
#define NVIC_PEND0_INT2         0x00000004  // Interrupt 2 pend
#define NVIC_PEND0_INT1         0x00000002  // Interrupt 1 pend
#define NVIC_PEND0_INT0         0x00000001  // Interrupt 0 pend

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND1 register.
//
//*****************************************************************************
#define NVIC_PEND1_INT59        0x08000000  // Interrupt 59 pend
#define NVIC_PEND1_INT58        0x04000000  // Interrupt 58 pend
#define NVIC_PEND1_INT57        0x02000000  // Interrupt 57 pend
#define NVIC_PEND1_INT56        0x01000000  // Interrupt 56 pend
#define NVIC_PEND1_INT55        0x00800000  // Interrupt 55 pend
#define NVIC_PEND1_INT54        0x00400000  // Interrupt 54 pend
#define NVIC_PEND1_INT53        0x00200000  // Interrupt 53 pend
#define NVIC_PEND1_INT52        0x00100000  // Interrupt 52 pend
#define NVIC_PEND1_INT51        0x00080000  // Interrupt 51 pend
#define NVIC_PEND1_INT50        0x00040000  // Interrupt 50 pend
#define NVIC_PEND1_INT49        0x00020000  // Interrupt 49 pend
#define NVIC_PEND1_INT48        0x00010000  // Interrupt 48 pend
#define NVIC_PEND1_INT47        0x00008000  // Interrupt 47 pend
#define NVIC_PEND1_INT46        0x00004000  // Interrupt 46 pend
#define NVIC_PEND1_INT45        0x00002000  // Interrupt 45 pend
#define NVIC_PEND1_INT44        0x00001000  // Interrupt 44 pend
#define NVIC_PEND1_INT43        0x00000800  // Interrupt 43 pend
#define NVIC_PEND1_INT42        0x00000400  // Interrupt 42 pend
#define NVIC_PEND1_INT41        0x00000200  // Interrupt 41 pend
#define NVIC_PEND1_INT40        0x00000100  // Interrupt 40 pend
#define NVIC_PEND1_INT39        0x00000080  // Interrupt 39 pend
#define NVIC_PEND1_INT38        0x00000040  // Interrupt 38 pend
#define NVIC_PEND1_INT37        0x00000020  // Interrupt 37 pend
#define NVIC_PEND1_INT36        0x00000010  // Interrupt 36 pend
#define NVIC_PEND1_INT35        0x00000008  // Interrupt 35 pend
#define NVIC_PEND1_INT34        0x00000004  // Interrupt 34 pend
#define NVIC_PEND1_INT33        0x00000002  // Interrupt 33 pend
#define NVIC_PEND1_INT32        0x00000001  // Interrupt 32 pend

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND0 register.
//
//*****************************************************************************
#define NVIC_UNPEND0_INT31      0x80000000  // Interrupt 31 unpend
#define NVIC_UNPEND0_INT30      0x40000000  // Interrupt 30 unpend
#define NVIC_UNPEND0_INT29      0x20000000  // Interrupt 29 unpend
#define NVIC_UNPEND0_INT28      0x10000000  // Interrupt 28 unpend
#define NVIC_UNPEND0_INT27      0x08000000  // Interrupt 27 unpend
#define NVIC_UNPEND0_INT26      0x04000000  // Interrupt 26 unpend
#define NVIC_UNPEND0_INT25      0x02000000  // Interrupt 25 unpend
#define NVIC_UNPEND0_INT24      0x01000000  // Interrupt 24 unpend
#define NVIC_UNPEND0_INT23      0x00800000  // Interrupt 23 unpend
#define NVIC_UNPEND0_INT22      0x00400000  // Interrupt 22 unpend
#define NVIC_UNPEND0_INT21      0x00200000  // Interrupt 21 unpend
#define NVIC_UNPEND0_INT20      0x00100000  // Interrupt 20 unpend
#define NVIC_UNPEND0_INT19      0x00080000  // Interrupt 19 unpend
#define NVIC_UNPEND0_INT18      0x00040000  // Interrupt 18 unpend
#define NVIC_UNPEND0_INT17      0x00020000  // Interrupt 17 unpend
#define NVIC_UNPEND0_INT16      0x00010000  // Interrupt 16 unpend
#define NVIC_UNPEND0_INT15      0x00008000  // Interrupt 15 unpend
#define NVIC_UNPEND0_INT14      0x00004000  // Interrupt 14 unpend
#define NVIC_UNPEND0_INT13      0x00002000  // Interrupt 13 unpend
#define NVIC_UNPEND0_INT12      0x00001000  // Interrupt 12 unpend
#define NVIC_UNPEND0_INT11      0x00000800  // Interrupt 11 unpend
#define NVIC_UNPEND0_INT10      0x00000400  // Interrupt 10 unpend
#define NVIC_UNPEND0_INT9       0x00000200  // Interrupt 9 unpend
#define NVIC_UNPEND0_INT8       0x00000100  // Interrupt 8 unpend
#define NVIC_UNPEND0_INT7       0x00000080  // Interrupt 7 unpend
#define NVIC_UNPEND0_INT6       0x00000040  // Interrupt 6 unpend
#define NVIC_UNPEND0_INT5       0x00000020  // Interrupt 5 unpend
#define NVIC_UNPEND0_INT4       0x00000010  // Interrupt 4 unpend
#define NVIC_UNPEND0_INT3       0x00000008  // Interrupt 3 unpend
#define NVIC_UNPEND0_INT2       0x00000004  // Interrupt 2 unpend
#define NVIC_UNPEND0_INT1       0x00000002  // Interrupt 1 unpend
#define NVIC_UNPEND0_INT0       0x00000001  // Interrupt 0 unpend

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND1 register.
//
//*****************************************************************************
#define NVIC_UNPEND1_INT59      0x08000000  // Interrupt 59 unpend
#define NVIC_UNPEND1_INT58      0x04000000  // Interrupt 58 unpend
#define NVIC_UNPEND1_INT57      0x02000000  // Interrupt 57 unpend
#define NVIC_UNPEND1_INT56      0x01000000  // Interrupt 56 unpend
#define NVIC_UNPEND1_INT55      0x00800000  // Interrupt 55 unpend
#define NVIC_UNPEND1_INT54      0x00400000  // Interrupt 54 unpend
#define NVIC_UNPEND1_INT53      0x00200000  // Interrupt 53 unpend
#define NVIC_UNPEND1_INT52      0x00100000  // Interrupt 52 unpend
#define NVIC_UNPEND1_INT51      0x00080000  // Interrupt 51 unpend
#define NVIC_UNPEND1_INT50      0x00040000  // Interrupt 50 unpend
#define NVIC_UNPEND1_INT49      0x00020000  // Interrupt 49 unpend
#define NVIC_UNPEND1_INT48      0x00010000  // Interrupt 48 unpend
#define NVIC_UNPEND1_INT47      0x00008000  // Interrupt 47 unpend
#define NVIC_UNPEND1_INT46      0x00004000  // Interrupt 46 unpend
#define NVIC_UNPEND1_INT45      0x00002000  // Interrupt 45 unpend
#define NVIC_UNPEND1_INT44      0x00001000  // Interrupt 44 unpend
#define NVIC_UNPEND1_INT43      0x00000800  // Interrupt 43 unpend
#define NVIC_UNPEND1_INT42      0x00000400  // Interrupt 42 unpend
#define NVIC_UNPEND1_INT41      0x00000200  // Interrupt 41 unpend
#define NVIC_UNPEND1_INT40      0x00000100  // Interrupt 40 unpend
#define NVIC_UNPEND1_INT39      0x00000080  // Interrupt 39 unpend
#define NVIC_UNPEND1_INT38      0x00000040  // Interrupt 38 unpend
#define NVIC_UNPEND1_INT37      0x00000020  // Interrupt 37 unpend
#define NVIC_UNPEND1_INT36      0x00000010  // Interrupt 36 unpend
#define NVIC_UNPEND1_INT35      0x00000008  // Interrupt 35 unpend
#define NVIC_UNPEND1_INT34      0x00000004  // Interrupt 34 unpend
#define NVIC_UNPEND1_INT33      0x00000002  // Interrupt 33 unpend
#define NVIC_UNPEND1_INT32      0x00000001  // Interrupt 32 unpend

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE0 register.
//
//*****************************************************************************
#define NVIC_ACTIVE0_INT31      0x80000000  // Interrupt 31 active
#define NVIC_ACTIVE0_INT30      0x40000000  // Interrupt 30 active
#define NVIC_ACTIVE0_INT29      0x20000000  // Interrupt 29 active
#define NVIC_ACTIVE0_INT28      0x10000000  // Interrupt 28 active
#define NVIC_ACTIVE0_INT27      0x08000000  // Interrupt 27 active
#define NVIC_ACTIVE0_INT26      0x04000000  // Interrupt 26 active
#define NVIC_ACTIVE0_INT25      0x02000000  // Interrupt 25 active
#define NVIC_ACTIVE0_INT24      0x01000000  // Interrupt 24 active
#define NVIC_ACTIVE0_INT23      0x00800000  // Interrupt 23 active
#define NVIC_ACTIVE0_INT22      0x00400000  // Interrupt 22 active
#define NVIC_ACTIVE0_INT21      0x00200000  // Interrupt 21 active
#define NVIC_ACTIVE0_INT20      0x00100000  // Interrupt 20 active
#define NVIC_ACTIVE0_INT19      0x00080000  // Interrupt 19 active
#define NVIC_ACTIVE0_INT18      0x00040000  // Interrupt 18 active
#define NVIC_ACTIVE0_INT17      0x00020000  // Interrupt 17 active
#define NVIC_ACTIVE0_INT16      0x00010000  // Interrupt 16 active
#define NVIC_ACTIVE0_INT15      0x00008000  // Interrupt 15 active
#define NVIC_ACTIVE0_INT14      0x00004000  // Interrupt 14 active
#define NVIC_ACTIVE0_INT13      0x00002000  // Interrupt 13 active
#define NVIC_ACTIVE0_INT12      0x00001000  // Interrupt 12 active
#define NVIC_ACTIVE0_INT11      0x00000800  // Interrupt 11 active
#define NVIC_ACTIVE0_INT10      0x00000400  // Interrupt 10 active
#define NVIC_ACTIVE0_INT9       0x00000200  // Interrupt 9 active
#define NVIC_ACTIVE0_INT8       0x00000100  // Interrupt 8 active
#define NVIC_ACTIVE0_INT7       0x00000080  // Interrupt 7 active
#define NVIC_ACTIVE0_INT6       0x00000040  // Interrupt 6 active
#define NVIC_ACTIVE0_INT5       0x00000020  // Interrupt 5 active
#define NVIC_ACTIVE0_INT4       0x00000010  // Interrupt 4 active
#define NVIC_ACTIVE0_INT3       0x00000008  // Interrupt 3 active
#define NVIC_ACTIVE0_INT2       0x00000004  // Interrupt 2 active
#define NVIC_ACTIVE0_INT1       0x00000002  // Interrupt 1 active
#define NVIC_ACTIVE0_INT0       0x00000001  // Interrupt 0 active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE1 register.
//
//*****************************************************************************
#define NVIC_ACTIVE1_INT59      0x08000000  // Interrupt 59 active
#define NVIC_ACTIVE1_INT58      0x04000000  // Interrupt 58 active
#define NVIC_ACTIVE1_INT57      0x02000000  // Interrupt 57 active
#define NVIC_ACTIVE1_INT56      0x01000000  // Interrupt 56 active
#define NVIC_ACTIVE1_INT55      0x00800000  // Interrupt 55 active
#define NVIC_ACTIVE1_INT54      0x00400000  // Interrupt 54 active
#define NVIC_ACTIVE1_INT53      0x00200000  // Interrupt 53 active
#define NVIC_ACTIVE1_INT52      0x00100000  // Interrupt 52 active
#define NVIC_ACTIVE1_INT51      0x00080000  // Interrupt 51 active
#define NVIC_ACTIVE1_INT50      0x00040000  // Interrupt 50 active
#define NVIC_ACTIVE1_INT49      0x00020000  // Interrupt 49 active
#define NVIC_ACTIVE1_INT48      0x00010000  // Interrupt 48 active
#define NVIC_ACTIVE1_INT47      0x00008000  // Interrupt 47 active
#define NVIC_ACTIVE1_INT46      0x00004000  // Interrupt 46 active
#define NVIC_ACTIVE1_INT45      0x00002000  // Interrupt 45 active
#define NVIC_ACTIVE1_INT44      0x00001000  // Interrupt 44 active
#define NVIC_ACTIVE1_INT43      0x00000800  // Interrupt 43 active
#define NVIC_ACTIVE1_INT42      0x00000400  // Interrupt 42 active
#define NVIC_ACTIVE1_INT41      0x00000200  // Interrupt 41 active
#define NVIC_ACTIVE1_INT40      0x00000100  // Interrupt 40 active
#define NVIC_ACTIVE1_INT39      0x00000080  // Interrupt 39 active
#define NVIC_ACTIVE1_INT38      0x00000040  // Interrupt 38 active
#define NVIC_ACTIVE1_INT37      0x00000020  // Interrupt 37 active
#define NVIC_ACTIVE1_INT36      0x00000010  // Interrupt 36 active
#define NVIC_ACTIVE1_INT35      0x00000008  // Interrupt 35 active
#define NVIC_ACTIVE1_INT34      0x00000004  // Interrupt 34 active
#define NVIC_ACTIVE1_INT33      0x00000002  // Interrupt 33 active
#define NVIC_ACTIVE1_INT32      0x00000001  // Interrupt 32 active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI0 register.
//
//*****************************************************************************
#define NVIC_PRI0_INT3_M        0xFF000000  // Interrupt 3 priority mask
#define NVIC_PRI0_INT2_M        0x00FF0000  // Interrupt 2 priority mask
#define NVIC_PRI0_INT1_M        0x0000FF00  // Interrupt 1 priority mask
#define NVIC_PRI0_INT0_M        0x000000FF  // Interrupt 0 priority mask
#define NVIC_PRI0_INT3_S        24
#define NVIC_PRI0_INT2_S        16
#define NVIC_PRI0_INT1_S        8
#define NVIC_PRI0_INT0_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI1 register.
//
//*****************************************************************************
#define NVIC_PRI1_INT7_M        0xFF000000  // Interrupt 7 priority mask
#define NVIC_PRI1_INT6_M        0x00FF0000  // Interrupt 6 priority mask
#define NVIC_PRI1_INT5_M        0x0000FF00  // Interrupt 5 priority mask
#define NVIC_PRI1_INT4_M        0x000000FF  // Interrupt 4 priority mask
#define NVIC_PRI1_INT7_S        24
#define NVIC_PRI1_INT6_S        16
#define NVIC_PRI1_INT5_S        8
#define NVIC_PRI1_INT4_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI2 register.
//
//*****************************************************************************
#define NVIC_PRI2_INT11_M       0xFF000000  // Interrupt 11 priority mask
#define NVIC_PRI2_INT10_M       0x00FF0000  // Interrupt 10 priority mask
#define NVIC_PRI2_INT9_M        0x0000FF00  // Interrupt 9 priority mask
#define NVIC_PRI2_INT8_M        0x000000FF  // Interrupt 8 priority mask
#define NVIC_PRI2_INT11_S       24
#define NVIC_PRI2_INT10_S       16
#define NVIC_PRI2_INT9_S        8
#define NVIC_PRI2_INT8_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI3 register.
//
//*****************************************************************************
#define NVIC_PRI3_INT15_M       0xFF000000  // Interrupt 15 priority mask
#define NVIC_PRI3_INT14_M       0x00FF0000  // Interrupt 14 priority mask
#define NVIC_PRI3_INT13_M       0x0000FF00  // Interrupt 13 priority mask
#define NVIC_PRI3_INT12_M       0x000000FF  // Interrupt 12 priority mask
#define NVIC_PRI3_INT15_S       24
#define NVIC_PRI3_INT14_S       16
#define NVIC_PRI3_INT13_S       8
#define NVIC_PRI3_INT12_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI4 register.
//
//*****************************************************************************
#define NVIC_PRI4_INT19_M       0xFF000000  // Interrupt 19 priority mask
#define NVIC_PRI4_INT18_M       0x00FF0000  // Interrupt 18 priority mask
#define NVIC_PRI4_INT17_M       0x0000FF00  // Interrupt 17 priority mask
#define NVIC_PRI4_INT16_M       0x000000FF  // Interrupt 16 priority mask
#define NVIC_PRI4_INT19_S       24
#define NVIC_PRI4_INT18_S       16
#define NVIC_PRI4_INT17_S       8
#define NVIC_PRI4_INT16_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI5 register.
//
//*****************************************************************************
#define NVIC_PRI5_INT23_M       0xFF000000  // Interrupt 23 priority mask
#define NVIC_PRI5_INT22_M       0x00FF0000  // Interrupt 22 priority mask
#define NVIC_PRI5_INT21_M       0x0000FF00  // Interrupt 21 priority mask
#define NVIC_PRI5_INT20_M       0x000000FF  // Interrupt 20 priority mask
#define NVIC_PRI5_INT23_S       24
#define NVIC_PRI5_INT22_S       16
#define NVIC_PRI5_INT21_S       8
#define NVIC_PRI5_INT20_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI6 register.
//
//*****************************************************************************
#define NVIC_PRI6_INT27_M       0xFF000000  // Interrupt 27 priority mask
#define NVIC_PRI6_INT26_M       0x00FF0000  // Interrupt 26 priority mask
#define NVIC_PRI6_INT25_M       0x0000FF00  // Interrupt 25 priority mask
#define NVIC_PRI6_INT24_M       0x000000FF  // Interrupt 24 priority mask
#define NVIC_PRI6_INT27_S       24
#define NVIC_PRI6_INT26_S       16
#define NVIC_PRI6_INT25_S       8
#define NVIC_PRI6_INT24_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI7 register.
//
//*****************************************************************************
#define NVIC_PRI7_INT31_M       0xFF000000  // Interrupt 31 priority mask
#define NVIC_PRI7_INT30_M       0x00FF0000  // Interrupt 30 priority mask
#define NVIC_PRI7_INT29_M       0x0000FF00  // Interrupt 29 priority mask
#define NVIC_PRI7_INT28_M       0x000000FF  // Interrupt 28 priority mask
#define NVIC_PRI7_INT31_S       24
#define NVIC_PRI7_INT30_S       16
#define NVIC_PRI7_INT29_S       8
#define NVIC_PRI7_INT28_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI8 register.
//
//*****************************************************************************
#define NVIC_PRI8_INT35_M       0xFF000000  // Interrupt 35 priority mask
#define NVIC_PRI8_INT34_M       0x00FF0000  // Interrupt 34 priority mask
#define NVIC_PRI8_INT33_M       0x0000FF00  // Interrupt 33 priority mask
#define NVIC_PRI8_INT32_M       0x000000FF  // Interrupt 32 priority mask
#define NVIC_PRI8_INT35_S       24
#define NVIC_PRI8_INT34_S       16
#define NVIC_PRI8_INT33_S       8
#define NVIC_PRI8_INT32_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI9 register.
//
//*****************************************************************************
#define NVIC_PRI9_INT39_M       0xFF000000  // Interrupt 39 priority mask
#define NVIC_PRI9_INT38_M       0x00FF0000  // Interrupt 38 priority mask
#define NVIC_PRI9_INT37_M       0x0000FF00  // Interrupt 37 priority mask
#define NVIC_PRI9_INT36_M       0x000000FF  // Interrupt 36 priority mask
#define NVIC_PRI9_INT39_S       24
#define NVIC_PRI9_INT38_S       16
#define NVIC_PRI9_INT37_S       8
#define NVIC_PRI9_INT36_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI10 register.
//
//*****************************************************************************
#define NVIC_PRI10_INT43_M      0xFF000000  // Interrupt 43 priority mask
#define NVIC_PRI10_INT42_M      0x00FF0000  // Interrupt 42 priority mask
#define NVIC_PRI10_INT41_M      0x0000FF00  // Interrupt 41 priority mask
#define NVIC_PRI10_INT40_M      0x000000FF  // Interrupt 40 priority mask
#define NVIC_PRI10_INT43_S      24
#define NVIC_PRI10_INT42_S      16
#define NVIC_PRI10_INT41_S      8
#define NVIC_PRI10_INT40_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPUID register.
//
//*****************************************************************************
#define NVIC_CPUID_IMP_M        0xFF000000  // Implementer
#define NVIC_CPUID_VAR_M        0x00F00000  // Variant
#define NVIC_CPUID_PARTNO_M     0x0000FFF0  // Processor part number
#define NVIC_CPUID_REV_M        0x0000000F  // Revision

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_INT_CTRL register.
//
//*****************************************************************************
#define NVIC_INT_CTRL_NMI_SET   0x80000000  // Pend a NMI
#define NVIC_INT_CTRL_PEND_SV   0x10000000  // Pend a PendSV
#define NVIC_INT_CTRL_UNPEND_SV 0x08000000  // Unpend a PendSV
#define NVIC_INT_CTRL_ISR_PRE   0x00800000  // Debug interrupt handling
#define NVIC_INT_CTRL_ISR_PEND  0x00400000  // Debug interrupt pending
#define NVIC_INT_CTRL_VEC_PEN_M 0x003FF000  // Highest pending exception
#define NVIC_INT_CTRL_RET_BASE  0x00000800  // Return to base
#define NVIC_INT_CTRL_VEC_ACT_M 0x000003FF  // Current active exception
#define NVIC_INT_CTRL_VEC_PEN_S 12
#define NVIC_INT_CTRL_VEC_ACT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_VTABLE register.
//
//*****************************************************************************
#define NVIC_VTABLE_BASE        0x20000000  // Vector table base
#define NVIC_VTABLE_OFFSET_M    0x1FFFFF00  // Vector table offset
#define NVIC_VTABLE_OFFSET_S    8

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_APINT register.
//
//*****************************************************************************
#define NVIC_APINT_VECTKEY_M    0xFFFF0000  // Vector key mask
#define NVIC_APINT_VECTKEY      0x05FA0000  // Vector key
#define NVIC_APINT_ENDIANESS    0x00008000  // Data endianess
#define NVIC_APINT_PRIGROUP_M   0x00000700  // Priority group
#define NVIC_APINT_PRIGROUP_0_8 0x00000700  // Priority group 0.8 split
#define NVIC_APINT_PRIGROUP_1_7 0x00000600  // Priority group 1.7 split
#define NVIC_APINT_PRIGROUP_2_6 0x00000500  // Priority group 2.6 split
#define NVIC_APINT_PRIGROUP_3_5 0x00000400  // Priority group 3.5 split
#define NVIC_APINT_PRIGROUP_4_4 0x00000300  // Priority group 4.4 split
#define NVIC_APINT_PRIGROUP_5_3 0x00000200  // Priority group 5.3 split
#define NVIC_APINT_PRIGROUP_6_2 0x00000100  // Priority group 6.2 split
#define NVIC_APINT_PRIGROUP_7_1 0x00000000  // Priority group 7.1 split
#define NVIC_APINT_SYSRESETREQ  0x00000004  // System reset request
#define NVIC_APINT_VECT_CLR_ACT 0x00000002  // Clear active NMI/fault info
#define NVIC_APINT_VECT_RESET   0x00000001  // System reset

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_CTRL register.
//
//*****************************************************************************
#define NVIC_SYS_CTRL_SEVONPEND 0x00000010  // Wakeup on pend
#define NVIC_SYS_CTRL_SLEEPDEEP 0x00000004  // Deep sleep enable
#define NVIC_SYS_CTRL_SLEEPEXIT 0x00000002  // Sleep on ISR exit

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CFG_CTRL register.
//
//*****************************************************************************
#define NVIC_CFG_CTRL_BFHFNMIGN 0x00000100  // Ignore bus fault in NMI/fault
#define NVIC_CFG_CTRL_DIV0      0x00000010  // Trap on divide by 0
#define NVIC_CFG_CTRL_UNALIGNED 0x00000008  // Trap on unaligned access
#define NVIC_CFG_CTRL_DEEP_PEND 0x00000004  // Allow deep interrupt trigger
#define NVIC_CFG_CTRL_MAIN_PEND 0x00000002  // Allow main interrupt trigger
#define NVIC_CFG_CTRL_BASE_THR  0x00000001  // Thread state control

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI1 register.
//
//*****************************************************************************
#define NVIC_SYS_PRI1_RES_M     0xFF000000  // Priority of reserved handler
#define NVIC_SYS_PRI1_USAGE_M   0x00FF0000  // Priority of usage fault handler
#define NVIC_SYS_PRI1_BUS_M     0x0000FF00  // Priority of bus fault handler
#define NVIC_SYS_PRI1_MEM_M     0x000000FF  // Priority of mem manage handler
#define NVIC_SYS_PRI1_USAGE_S   16
#define NVIC_SYS_PRI1_BUS_S     8
#define NVIC_SYS_PRI1_MEM_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI2 register.
//
//*****************************************************************************
#define NVIC_SYS_PRI2_SVC_M     0xFF000000  // Priority of SVCall handler
#define NVIC_SYS_PRI2_RES_M     0x00FFFFFF  // Priority of reserved handlers
#define NVIC_SYS_PRI2_SVC_S     24

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI3 register.
//
//*****************************************************************************
#define NVIC_SYS_PRI3_TICK_M    0xFF000000  // Priority of Sys Tick handler
#define NVIC_SYS_PRI3_PENDSV_M  0x00FF0000  // Priority of PendSV handler
#define NVIC_SYS_PRI3_RES_M     0x0000FF00  // Priority of reserved handler
#define NVIC_SYS_PRI3_DEBUG_M   0x000000FF  // Priority of debug handler
#define NVIC_SYS_PRI3_TICK_S    24
#define NVIC_SYS_PRI3_PENDSV_S  16
#define NVIC_SYS_PRI3_DEBUG_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_HND_CTRL
// register.
//
//*****************************************************************************
#define NVIC_SYS_HND_CTRL_USAGE 0x00040000  // Usage fault enable
#define NVIC_SYS_HND_CTRL_BUS   0x00020000  // Bus fault enable
#define NVIC_SYS_HND_CTRL_MEM   0x00010000  // Mem manage fault enable
#define NVIC_SYS_HND_CTRL_SVC   0x00008000  // SVCall is pended
#define NVIC_SYS_HND_CTRL_BUSP  0x00004000  // Bus fault is pended
#define NVIC_SYS_HND_CTRL_TICK  0x00000800  // Sys tick is active
#define NVIC_SYS_HND_CTRL_PNDSV 0x00000400  // PendSV is active
#define NVIC_SYS_HND_CTRL_MON   0x00000100  // Monitor is active
#define NVIC_SYS_HND_CTRL_SVCA  0x00000080  // SVCall is active
#define NVIC_SYS_HND_CTRL_USGA  0x00000008  // Usage fault is active
#define NVIC_SYS_HND_CTRL_BUSA  0x00000002  // Bus fault is active
#define NVIC_SYS_HND_CTRL_MEMA  0x00000001  // Mem manage is active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_STAT
// register.
//
//*****************************************************************************
#define NVIC_FAULT_STAT_DIV0    0x02000000  // Divide by zero fault
#define NVIC_FAULT_STAT_UNALIGN 0x01000000  // Unaligned access fault
#define NVIC_FAULT_STAT_NOCP    0x00080000  // No coprocessor fault
#define NVIC_FAULT_STAT_INVPC   0x00040000  // Invalid PC fault
#define NVIC_FAULT_STAT_INVSTAT 0x00020000  // Invalid state fault
#define NVIC_FAULT_STAT_UNDEF   0x00010000  // Undefined instruction fault
#define NVIC_FAULT_STAT_BFARV   0x00008000  // BFAR is valid
#define NVIC_FAULT_STAT_BSTKE   0x00001000  // Stack bus fault
#define NVIC_FAULT_STAT_BUSTKE  0x00000800  // Unstack bus fault
#define NVIC_FAULT_STAT_IMPRE   0x00000400  // Imprecise data bus error
#define NVIC_FAULT_STAT_PRECISE 0x00000200  // Precise data bus error
#define NVIC_FAULT_STAT_IBUS    0x00000100  // Instruction bus fault
#define NVIC_FAULT_STAT_MMARV   0x00000080  // MMAR is valid
#define NVIC_FAULT_STAT_MSTKE   0x00000010  // Stack access violation
#define NVIC_FAULT_STAT_MUSTKE  0x00000008  // Unstack access violation
#define NVIC_FAULT_STAT_DERR    0x00000002  // Data access violation
#define NVIC_FAULT_STAT_IERR    0x00000001  // Instruction access violation

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_HFAULT_STAT
// register.
//
//*****************************************************************************
#define NVIC_HFAULT_STAT_DBG    0x80000000  // Debug event
#define NVIC_HFAULT_STAT_FORCED 0x40000000  // Cannot execute fault handler
#define NVIC_HFAULT_STAT_VECT   0x00000002  // Vector table read fault

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DEBUG_STAT
// register.
//
//*****************************************************************************
#define NVIC_DEBUG_STAT_EXTRNL  0x00000010  // EDBGRQ asserted
#define NVIC_DEBUG_STAT_VCATCH  0x00000008  // Vector catch
#define NVIC_DEBUG_STAT_DWTTRAP 0x00000004  // DWT match
#define NVIC_DEBUG_STAT_BKPT    0x00000002  // Breakpoint instruction
#define NVIC_DEBUG_STAT_HALTED  0x00000001  // Halt request

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MM_ADDR register.
//
//*****************************************************************************
#define NVIC_MM_ADDR_M          0xFFFFFFFF  // Data fault address
#define NVIC_MM_ADDR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_ADDR
// register.
//
//*****************************************************************************
#define NVIC_FAULT_ADDR_M       0xFFFFFFFF  // Data bus fault address
#define NVIC_FAULT_ADDR_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_TYPE register.
//
//*****************************************************************************
#define NVIC_MPU_TYPE_IREGION_M 0x00FF0000  // Number of I regions
#define NVIC_MPU_TYPE_DREGION_M 0x0000FF00  // Number of D regions
#define NVIC_MPU_TYPE_SEPARATE  0x00000001  // Separate or unified MPU
#define NVIC_MPU_TYPE_IREGION_S 16
#define NVIC_MPU_TYPE_DREGION_S 8

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_CTRL register.
//
//*****************************************************************************
#define NVIC_MPU_CTRL_PRIVDEFEN 0x00000004  // MPU default region in priv mode
#define NVIC_MPU_CTRL_HFNMIENA  0x00000002  // MPU enabled during faults
#define NVIC_MPU_CTRL_ENABLE    0x00000001  // MPU enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_NUMBER
// register.
//
//*****************************************************************************
#define NVIC_MPU_NUMBER_M       0x000000FF  // MPU region to access
#define NVIC_MPU_NUMBER_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE register.
//
//*****************************************************************************
#define NVIC_MPU_BASE_ADDR_M    0xFFFFFFE0  // Base address mask
#define NVIC_MPU_BASE_VALID     0x00000010  // Region number valid
#define NVIC_MPU_BASE_REGION_M  0x0000000F  // Region number
#define NVIC_MPU_BASE_ADDR_S    8
#define NVIC_MPU_BASE_REGION_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR register.
//
//*****************************************************************************
#define NVIC_MPU_ATTR_M         0xFFFF0000  // Attributes
#define NVIC_MPU_ATTR_AP_NO_NO  0x00000000  // prv: no access, usr: no access
#define NVIC_MPU_ATTR_BUFFRABLE 0x00010000  // Bufferable  可缓冲
#define NVIC_MPU_ATTR_CACHEABLE 0x00020000  // Cacheable   可高速缓存
#define NVIC_MPU_ATTR_SHAREABLE 0x00040000  // Shareable   可共享
#define NVIC_MPU_ATTR_TEX_M     0x00380000  // Type extension mask	类型扩展域
#define NVIC_MPU_ATTR_AP_RW_NO  0x01000000  // prv: rw, usr: none      特权:读写 用户:不能访问
#define NVIC_MPU_ATTR_AP_RW_RO  0x02000000  // prv: rw, usr: read-only 特权:读写 用户:只读
#define NVIC_MPU_ATTR_AP_RW_RW  0x03000000  // prv: rw, usr: rw		   特权:读写 用户:读写
#define NVIC_MPU_ATTR_AP_RO_NO  0x05000000  // prv: ro, usr: none	   特权:只读 用户:不能访问
#define NVIC_MPU_ATTR_AP_RO_RO  0x06000000  // prv: ro, usr: ro		   特权:只读 用户:只读
#define NVIC_MPU_ATTR_AP_M      0x07000000  // Access permissions mask 访问许可域
#define NVIC_MPU_ATTR_XN        0x10000000  // Execute disable		   指令访问 取指允许
#define NVIC_MPU_ATTR_SRD_M     0x0000FF00  // Sub-region disable mask
#define NVIC_MPU_ATTR_SRD_0     0x00000100  // Sub-region 0 disable
#define NVIC_MPU_ATTR_SRD_1     0x00000200  // Sub-region 1 disable
#define NVIC_MPU_ATTR_SRD_2     0x00000400  // Sub-region 2 disable
#define NVIC_MPU_ATTR_SRD_3     0x00000800  // Sub-region 3 disable
#define NVIC_MPU_ATTR_SRD_4     0x00001000  // Sub-region 4 disable
#define NVIC_MPU_ATTR_SRD_5     0x00002000  // Sub-region 5 disable
#define NVIC_MPU_ATTR_SRD_6     0x00004000  // Sub-region 6 disable
#define NVIC_MPU_ATTR_SRD_7     0x00008000  // Sub-region 7 disable
#define NVIC_MPU_ATTR_SIZE_M    0x0000003E  // Region size mask
#define NVIC_MPU_ATTR_SIZE_32B  0x00000008  // Region size 32 bytes
#define NVIC_MPU_ATTR_SIZE_64B  0x0000000A  // Region size 64 bytes
#define NVIC_MPU_ATTR_SIZE_128B 0x0000000C  // Region size 128 bytes
#define NVIC_MPU_ATTR_SIZE_256B 0x0000000E  // Region size 256 bytes
#define NVIC_MPU_ATTR_SIZE_512B 0x00000010  // Region size 512 bytes
#define NVIC_MPU_ATTR_SIZE_1K   0x00000012  // Region size 1 Kbytes
#define NVIC_MPU_ATTR_SIZE_2K   0x00000014  // Region size 2 Kbytes
#define NVIC_MPU_ATTR_SIZE_4K   0x00000016  // Region size 4 Kbytes
#define NVIC_MPU_ATTR_SIZE_8K   0x00000018  // Region size 8 Kbytes
#define NVIC_MPU_ATTR_SIZE_16K  0x0000001A  // Region size 16 Kbytes
#define NVIC_MPU_ATTR_SIZE_32K  0x0000001C  // Region size 32 Kbytes
#define NVIC_MPU_ATTR_SIZE_64K  0x0000001E  // Region size 64 Kbytes
#define NVIC_MPU_ATTR_SIZE_128K 0x00000020  // Region size 128 Kbytes
#define NVIC_MPU_ATTR_SIZE_256K 0x00000022  // Region size 256 Kbytes
#define NVIC_MPU_ATTR_SIZE_512K 0x00000024  // Region size 512 Kbytes
#define NVIC_MPU_ATTR_SIZE_1M   0x00000026  // Region size 1 Mbytes
#define NVIC_MPU_ATTR_SIZE_2M   0x00000028  // Region size 2 Mbytes
#define NVIC_MPU_ATTR_SIZE_4M   0x0000002A  // Region size 4 Mbytes
#define NVIC_MPU_ATTR_SIZE_8M   0x0000002C  // Region size 8 Mbytes
#define NVIC_MPU_ATTR_SIZE_16M  0x0000002E  // Region size 16 Mbytes
#define NVIC_MPU_ATTR_SIZE_32M  0x00000030  // Region size 32 Mbytes
#define NVIC_MPU_ATTR_SIZE_64M  0x00000032  // Region size 64 Mbytes
#define NVIC_MPU_ATTR_SIZE_128M 0x00000034  // Region size 128 Mbytes
#define NVIC_MPU_ATTR_SIZE_256M 0x00000036  // Region size 256 Mbytes
#define NVIC_MPU_ATTR_SIZE_512M 0x00000038  // Region size 512 Mbytes
#define NVIC_MPU_ATTR_SIZE_1G   0x0000003A  // Region size 1 Gbytes
#define NVIC_MPU_ATTR_SIZE_2G   0x0000003C  // Region size 2 Gbytes
#define NVIC_MPU_ATTR_SIZE_4G   0x0000003E  // Region size 4 Gbytes
#define NVIC_MPU_ATTR_ENABLE    0x00000001  // Region enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_CTRL register.
//
//*****************************************************************************
#define NVIC_DBG_CTRL_DBGKEY_M  0xFFFF0000  // Debug key mask
#define NVIC_DBG_CTRL_DBGKEY    0xA05F0000  // Debug key
#define NVIC_DBG_CTRL_S_RESET_ST \
                                0x02000000  // Core has reset since last read
#define NVIC_DBG_CTRL_S_RETIRE_ST \
                                0x01000000  // Core has executed insruction
                                            // since last read
#define NVIC_DBG_CTRL_S_LOCKUP  0x00080000  // Core is locked up
#define NVIC_DBG_CTRL_S_SLEEP   0x00040000  // Core is sleeping
#define NVIC_DBG_CTRL_S_HALT    0x00020000  // Core status on halt
#define NVIC_DBG_CTRL_S_REGRDY  0x00010000  // Register read/write available
#define NVIC_DBG_CTRL_C_SNAPSTALL \
                                0x00000020  // Breaks a stalled load/store
#define NVIC_DBG_CTRL_C_MASKINT 0x00000008  // Mask interrupts when stepping
#define NVIC_DBG_CTRL_C_STEP    0x00000004  // Step the core
#define NVIC_DBG_CTRL_C_HALT    0x00000002  // Halt the core
#define NVIC_DBG_CTRL_C_DEBUGEN 0x00000001  // Enable debug

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_XFER register.
//
//*****************************************************************************
#define NVIC_DBG_XFER_REG_WNR   0x00010000  // Write or not read
#define NVIC_DBG_XFER_REG_SEL_M 0x0000001F  // Register
#define NVIC_DBG_XFER_REG_CFBP  0x00000014  // Control/Fault/BasePri/PriMask
#define NVIC_DBG_XFER_REG_DSP   0x00000013  // Deep SP
#define NVIC_DBG_XFER_REG_PSP   0x00000012  // Process SP
#define NVIC_DBG_XFER_REG_MSP   0x00000011  // Main SP
#define NVIC_DBG_XFER_REG_FLAGS 0x00000010  // xPSR/Flags register
#define NVIC_DBG_XFER_REG_R15   0x0000000F  // Register R15
#define NVIC_DBG_XFER_REG_R14   0x0000000E  // Register R14
#define NVIC_DBG_XFER_REG_R13   0x0000000D  // Register R13
#define NVIC_DBG_XFER_REG_R12   0x0000000C  // Register R12
#define NVIC_DBG_XFER_REG_R11   0x0000000B  // Register R11
#define NVIC_DBG_XFER_REG_R10   0x0000000A  // Register R10
#define NVIC_DBG_XFER_REG_R9    0x00000009  // Register R9
#define NVIC_DBG_XFER_REG_R8    0x00000008  // Register R8
#define NVIC_DBG_XFER_REG_R7    0x00000007  // Register R7
#define NVIC_DBG_XFER_REG_R6    0x00000006  // Register R6
#define NVIC_DBG_XFER_REG_R5    0x00000005  // Register R5
#define NVIC_DBG_XFER_REG_R4    0x00000004  // Register R4
#define NVIC_DBG_XFER_REG_R3    0x00000003  // Register R3
#define NVIC_DBG_XFER_REG_R2    0x00000002  // Register R2
#define NVIC_DBG_XFER_REG_R1    0x00000001  // Register R1
#define NVIC_DBG_XFER_REG_R0    0x00000000  // Register R0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_DATA register.
//
//*****************************************************************************
#define NVIC_DBG_DATA_M         0xFFFFFFFF  // Data temporary cache
#define NVIC_DBG_DATA_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_INT register.
//
//*****************************************************************************
#define NVIC_DBG_INT_HARDERR    0x00000400  // Debug trap on hard fault
#define NVIC_DBG_INT_INTERR     0x00000200  // Debug trap on interrupt errors
#define NVIC_DBG_INT_BUSERR     0x00000100  // Debug trap on bus error
#define NVIC_DBG_INT_STATERR    0x00000080  // Debug trap on usage fault state
#define NVIC_DBG_INT_CHKERR     0x00000040  // Debug trap on usage fault check
#define NVIC_DBG_INT_NOCPERR    0x00000020  // Debug trap on coprocessor error
#define NVIC_DBG_INT_MMERR      0x00000010  // Debug trap on mem manage fault
#define NVIC_DBG_INT_RESET      0x00000008  // Core reset status
#define NVIC_DBG_INT_RSTPENDCLR 0x00000004  // Clear pending core reset
#define NVIC_DBG_INT_RSTPENDING 0x00000002  // Core reset is pending
#define NVIC_DBG_INT_RSTVCATCH  0x00000001  // Reset vector catch

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SW_TRIG register.
//
//*****************************************************************************
#define NVIC_SW_TRIG_INTID_M    0x000003FF  // Interrupt to trigger
#define NVIC_SW_TRIG_INTID_S    0

#endif // __LM3S2139_H__
