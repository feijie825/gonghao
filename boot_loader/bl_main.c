//*****************************************************************************
//
// bl_main.c - The file holds the main control loop of the boot loader.
//
// Copyright (c) 2006-2008 Luminary Micro, Inc.  All rights reserved.
// 
// Software License Agreement
// 
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
// 
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  You may not combine
// this software with "viral" open-source software in order to form a larger
// program.  Any use in violation of the foregoing restrictions may subject
// the user to criminal sanctions under applicable laws, as well as to civil
// liability for the breach of the terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 3416 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************

#include "bl_commands.h"
#include "bl_config.h"
#include "bl_decrypt.h"
#include "bl_i2c.h"
#include "bl_packet.h"
#include "bl_ssi.h"
#include "bl_uart.h"
#include "LM3S2139.h"
#include "lm3s21xx_lib.h"
#include "disp.h"
unsigned char Disp_Sts;      //显示状态 bit7 显示状态
//*****************************************************************************
//
// Make sure that the application start address is a multiple of 1024 bytes.
//
//*****************************************************************************
#if (APP_START_ADDRESS & 0x3ff)
#error ERROR: APP_START_ADDRESS must be a multiple of 1024 bytes!
#endif

//*****************************************************************************
//
// Make sure that the flash reserved space is a multiple of 1024 bytes.
//
//*****************************************************************************
#if (FLASH_RSVD_SPACE & 0x3ff)
#error ERROR: FLASH_RSVD_SPACE must be a multiple of 1024 bytes!
#endif

//*****************************************************************************
//
//! \addtogroup boot_loader_api
//! @{
//
//*****************************************************************************
#if defined(I2C_ENABLE_UPDATE) || defined(SSI_ENABLE_UPDATE) || \
    defined(UART_ENABLE_UPDATE) || defined(DOXYGEN)

//*****************************************************************************
//
// A prototype for the function (in the startup code) for calling the
// application.
//
//*****************************************************************************
extern void CallApplication(unsigned long ulBase);

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(unsigned long ulCount);

//*****************************************************************************
//
// Holds the current status of the last command that was issued to the boot
// loader.
//
//*****************************************************************************
unsigned char g_ucStatus;

//*****************************************************************************
//
// This holds the current remaining size in bytes to be downloaded.
//
//*****************************************************************************
unsigned long g_ulTransferSize;

//*****************************************************************************
//
// This holds the current address that is being written to during a download
// command.
//
//*****************************************************************************
unsigned long g_ulTransferAddress;

//*****************************************************************************
//
// This is the data buffer used during transfers to the boot loader.
//
//*****************************************************************************
unsigned long g_pulDataBuffer[BUFFER_SIZE];

unsigned long BOOT_EN[BOOT_EN_SIZE];

//*****************************************************************************
//
// This is an specially aligned buffer pointer to g_pulDataBuffer to make
// copying to the buffer simpler.  It must be offset to end on an address that
// ends with 3.
//
//*****************************************************************************
unsigned char *g_pucDataBuffer;

unsigned char Board_Id;
unsigned char BootLoader_En; //引导区更新使能命令
volatile unsigned int  Timer_1ms;
volatile unsigned char Timer_Uart;

//*****************************************************************************
//
// Converts a word from big endian to little endian.  This macro uses compiler-
// specific constructs to perform an inline insertion of the "rev" instruction,
// which performs the byte swap directly.
//
//*****************************************************************************
#if defined(ewarm)
#include <intrinsics.h>
#define SwapWord(x)             __REV(x)
#endif
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
#define SwapWord(x) __extension__                                \
        ({                                                       \
             register unsigned long __ret, __inp = x;            \
             __asm__("rev %0, %1" : "=r" (__ret) : "r" (__inp)); \
             __ret;                                              \
        })
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
#define SwapWord(x)             __rev(x)
#endif
//*********************取设备所在DCx**************************************
#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 8) & 0xf)

//*********************取设备所在位*******************************************
#define SYSCTL_PERIPH_MASK(a)   ( 1<< ((a) & 0xff))
//*****************************************************************************
//
//! Handles the SysTick interrupt.
//!
//! This function is called when the SysTick interrupt occurs.  It simply
//! keeps a running count of interrupts, used as a time basis for the BOOTP and
//! TFTP protocols.
//!
//! This function is contained in <tt>bl_enet.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Increment the tick count.
    //
    Timer_1ms++;
    Timer_Uart++;
    if((Timer_1ms % 0x100) == 0)                            //512ms
     GPIOPinWrite(GPIOF,WDI_MC,~GPIOPinRead(GPIOF,WDI_MC)); //复位外部硬件看门狗 ;
}
//*****************************************************************************
//
//! Configures the microcontroller.
//!
//! This function configures the peripherals and GPIOs of the microcontroller,
//! preparing it for use by the boot loader.  The interface that has been
//! selected as the update port will be configured, and auto-baud will be
//! performed if required.
//!
//! This function is contained in <tt>bl_main.c</tt>.
//!
//! \return None.
// 初始化设备
//*****************************************************************************
void
ConfigureDevice(void)
{
#ifdef UART_ENABLE_UPDATE
    unsigned long ulProcRatio;
#endif

#ifdef CRYSTAL_FREQ
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it.
    //
    SYSCTL->RCC &= ~(SYSCTL_RCC_MOSCDIS);
    Delay(50000);
    SYSCTL->RCC = ((SYSCTL->RCC & (~SYSCTL_RCC_OSCSRC_M)) | SYSCTL_RCC_OSCSRC_MAIN);

    //
    // Set the flash programming time based on the specified crystal frequency.
    //
    FLASH->USECRL = ((CRYSTAL_FREQ + 999999) / 1000000) - 1;
#else
    //
    // Set the flash to program at 16 MHz since that is just beyond the fastest
    // that we could run.
    //
    FLASH->USECRL = 15;
#endif

#ifdef I2C_ENABLE_UPDATE
    //
    // Enable the clocks to the I2C and GPIO modules.
    //
    SYSCTL->RCGC[2] |= SYSCTL_RCGC2_GPIOB;   //I2C WDI
    SYSCTL->RCGC[1] |= SYSCTL_RCGC1_I2C0;

    //
    // Configure the GPIO pins for hardware control, open drain with pull-up,
    // and enable them.
    //
    GPIOB->AFSEL = (1 << 7) | I2C_PINS;
    GPIOB->DEN = (1 << 7) | I2C_PINS;
    GPIOB->ODR = I2C_PINS;

    //
    // Enable the I2C Slave Mode.
    //
    HWREG(I2C0_MASTER_BASE + I2C_O_MCR) = I2C_MCR_MFE | I2C_MCR_SFE;

    //
    // Setup the I2C Slave Address.
    //
    HWREG(I2C0_SLAVE_BASE + I2C_O_SOAR) = I2C_SLAVE_ADDR;

    //
    // Enable the I2C Slave Device on the I2C bus.
    //
    HWREG(I2C0_SLAVE_BASE + I2C_O_SCSR) = I2C_SCSR_DA;
#endif

#ifdef SSI_ENABLE_UPDATE
    //
    // Enable the clocks to the SSI and GPIO modules.
    //
    SYSCTL>RCGC[2] = SYSCTL_RCGC2_GPIOA;
    SYSCTL>RCGC[1] = SYSCTL_RCGC1_SSI0;

    //
    // Make the pin be peripheral controlled.
    //
    GPIOA->AFSEL |= SSI_PINS;
    GPIOA->DEN |= SSI_PINS;

    //
    // Set the SSI protocol to Motorola with default clock high and data
    // valid on the rising edge.
    //
    SSI0->CR0 = (SSI_CR0_SPH | SSI_CR0_SPO |
                (DATA_BITS_SSI - 1));

    //
    // Enable the SSI interface in slave mode.
    //
    SSI0->CR1 = SSI_CR1_MS | SSI_CR1_SSE;
#endif

    Init_Ssi();		//初始化SSI 控制显示

    Init_Gpio();        //参数I/O口
    Reset_HD7279();     //复位显示
    Disp_Boot();        //按方式0译码 显示引导状态
    Disp_Sts=0x80;      //显示状态为显示
    Delay(5000);        //

    // 2010.3.25 张力阵添加 系统中断
    // Setup SysTick.
    // 系统频率 8mHz 系统晶振频率
    NVIC->ST_RELOAD = ((CRYSTAL_FREQ /1000)*SYS_TIME) - 1;
    
    NVIC->ST_CTRL = (NVIC_ST_CTRL_CLK_SRC |
                     NVIC_ST_CTRL_INTEN |
                     NVIC_ST_CTRL_ENABLE);

#ifdef UART_ENABLE_UPDATE
    //
    // Enable the the clocks to the UART and GPIO modules.
    //
#ifdef BOOT_UART1	 //判断是否通过UART1升级
    SYSCTL->RCGC[2] |= SYSCTL_RCGC2_GPIOD;
    SYSCTL->RCGC[1] |= SYSCTL_RCGC1_UART1;
#else				 //通过UART0 升级
    SYSCTL->RCGC[2] |= SYSCTL_RCGC2_GPIOA;
    SYSCTL->RCGC[1] |= SYSCTL_RCGC1_UART0;
#endif
    //
    // Keep attempting to sync until we are successful.
    //
#ifdef UART_AUTOBAUD
    while(UARTAutoBaud(&ulProcRatio) < 0)
    {
    }
#else
    ulProcRatio = UART_BAUD_RATIO(UART_FIXED_BAUDRATE);
#endif

#ifdef BOOT_UART1
    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOD->AFSEL |= UART_PINS1;  //PA0 PA1
    //
    // Set the pin type.
    //
    GPIOD->DEN |= UART_PINS1;
    //
    // Set the baud rate.
    //
    UART1->IBRD = ulProcRatio >> 6;
    UART1->FBRD = ulProcRatio & UART_FBRD_DIVFRAC_M;

    //
    // Set data length, parity, and number of stop bits to 8-N-1.
    //
    UART1->LCRH = UART_LCRH_WLEN_8 | UART_LCRH_FEN;

    //
    // Enable RX, TX, and the UART.
    //
    UART1->CTL = (UART_CTL_UARTEN | 
                  UART_CTL_TXE |
                  UART_CTL_RXE);

#else
    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOA->AFSEL |= UART_PINS0;  //PA0 PA1
    //
    // Set the pin type.
    //
    GPIOA->DEN |= UART_PINS0;
    //
    // Set the baud rate.
    //
    UART0->IBRD = ulProcRatio >> 6;
    UART0->FBRD = ulProcRatio & UART_FBRD_DIVFRAC_M;

    //
    // Set data length, parity, and number of stop bits to 8-N-1.
    //
    UART0->LCRH = UART_LCRH_WLEN_8 | UART_LCRH_FEN;

    //
    // Enable RX, TX, and the UART.
    //
    UART0->CTL = (UART_CTL_UARTEN | 
                  UART_CTL_TXE |
                  UART_CTL_RXE);

#endif
#ifdef UART_AUTOBAUD
    //
    // Need to ack in the UART case to hold it up while we get things set up.
    //
	Delay(20000);
    AckPacket();
#endif
#endif
}

//*****************************************************************************
//
//! This function performs the update on the selected port.
//!
//! This function is called directly by the boot loader or it is called as a
//! result of an update request from the application.
//!
//! This function is contained in <tt>bl_main.c</tt>.
//!
//! \return Never returns.
//
//*****************************************************************************
void
Updater(void)
{
    unsigned long ulSize, ulTemp, ulFlashSize;

    g_pucDataBuffer = ((unsigned char *)g_pulDataBuffer) + 3;

    g_ulTransferAddress = 0xffffffff;
    Timer_Uart=0;                  //串口定时器清零
    BootLoader_En=0;               //默认禁止引导区更新
    while(1)
     {
      ulSize = sizeof(g_pulDataBuffer) - 3;
      if(ReceivePacket(g_pucDataBuffer, &ulSize) != 0) //接收包
       continue;
      switch(g_pucDataBuffer[0])
       {
        case COMMAND_PING: 
         {//0x20 ECHO 计算机查询命令 
          g_ucStatus = COMMAND_RET_SUCCESS;
          AckPacket();
          break;
         }
        case COMMAND_DOWNLOAD:
         {//0x21 传输程序偏移地址和程序长度
          g_ucStatus = COMMAND_RET_SUCCESS;
          do
           {
            if(ulSize != 9)
             {
              g_ucStatus = COMMAND_RET_INVALID_CMD;
              break;
             }
            g_ulTransferAddress = SwapWord(g_pulDataBuffer[1]);	 //起始地址
            g_ulTransferSize = SwapWord(g_pulDataBuffer[2]);
            ulFlashSize = (((SYSCTL->DC0 & SYSCTL_DC0_FLASHSZ_M) + 1) << 11);
#ifdef FLASH_RSVD_SPACE
            if((ulFlashSize - FLASH_RSVD_SPACE) != g_ulTransferAddress)
             {
              ulFlashSize -= FLASH_RSVD_SPACE;
             }
#endif
            if( 

#ifdef ENABLE_BL_UPDATE	 //是否使能引导区更新
               (g_ulTransferAddress != 0) &&
#endif
#ifdef FLASH_RSVD_SPACE
               (g_ulTransferAddress !=(ulFlashSize - FLASH_RSVD_SPACE)) &&
#endif
               ((!BootLoader_En)||(g_ulTransferAddress!=0))&&  //不允许引导 时 地址为0
               (g_ulTransferAddress != APP_START_ADDRESS) ||
               ((g_ulTransferAddress + g_ulTransferSize) > ulFlashSize) ||
               ((g_ulTransferAddress & 3) != 0))
                { //地址或长度不合法
                 g_ucStatus = COMMAND_RET_INVALID_ADR;
                 break;
                }
#ifndef FLASH_CODE_PROTECTION
            ulFlashSize = g_ulTransferAddress + g_ulTransferSize;
#endif

            FLASH->FCMISC = FLASH_FCMISC_AMISC;

            for(ulTemp = g_ulTransferAddress; ulTemp < ulFlashSize;
                ulTemp += 0x400)
             {
              FLASH->FMA = ulTemp;
              FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;
              while(FLASH->FMC & FLASH_FMC_ERASE)
               {
               }
             }
            
            if(FLASH->FCRIS & FLASH_FCRIS_ARIS)
             {
              g_ucStatus = COMMAND_RET_FLASH_FAIL;
             }
           }
          while(0);

          if(g_ucStatus != COMMAND_RET_SUCCESS)
           {
            g_ulTransferSize = 0;
           }
          AckPacket();
          break;
         }
        case COMMAND_RUN:
         {//0x22 +4字节 程序地址 程序运行
          AckPacket();
          if(ulSize != 5)
           {
            g_ucStatus = COMMAND_RET_INVALID_CMD;
            //
            // This packet has been handled.
            //
            break;
           }
          
          //
          // Get the address to which control should be transferred.
          //
          g_ulTransferAddress = SwapWord(g_pulDataBuffer[1]);
          
          //
          // This determines the size of the flash available on the
          // device in use.
          //
          ulFlashSize = ((SYSCTL->DC0 & SYSCTL_DC0_FLASHSZ_M + 1) << 11);
          
          //
          // Test if the transfer address is valid for this device.
          //
          if(g_ulTransferAddress >= ulFlashSize)
           {
            //
            // Indicate that an invalid address was specified.
            //
            g_ucStatus = COMMAND_RET_INVALID_ADR;
           
            //
            // This packet has been handled.
            //
            break;
           }
          
          //
          // Make sure that the ACK packet has been sent.
          //
          FlushData();
          
          //
          // Reset and disable the peripherals used by the boot loader.
          //
          SYSCTL->RCGC[1] = 0;
          SYSCTL->RCGC[2] = 0;
          SYSCTL->SRCR[1] = (SYSCTL_SRCR1_I2C0 | 
                             SYSCTL_SRCR1_SSI0 |
                             SYSCTL_SRCR1_UART0|
                             SYSCTL_SRCR1_UART1);
          SYSCTL->SRCR[2] = SYSCTL_SRCR2_GPIOA | 
                            SYSCTL_SRCR2_GPIOB |
                            SYSCTL_SRCR2_GPIOD; 
          SYSCTL->SRCR[1] = 0;
          SYSCTL->SRCR[2] = 0;
          
          //
          // Branch to the specified address.  This should never return.
          // If it does, very bad things will likely happen since it is
          // likely that the copy of the boot loader in SRAM will have
          // been overwritten.
          //
          ((void (*)(void))g_ulTransferAddress)();
          
          //
          // In case this ever does return and the boot loader is still
          // intact, simply reset the device.
          //
          NVIC->APINT = (NVIC_APINT_VECTKEY |
                         NVIC_APINT_SYSRESETREQ);
          
          //
          // The microcontroller should have reset, so this should
          // never be reached.  Just in case, loop forever.
          //
          while(1)
           {
           }
         }
        //
        // This command just returns the status of the last command that
        // was sent.
        //
        case COMMAND_GET_STATUS:
         {//0x23 获取状态
          //
          // Acknowledge that this command was received correctly.  This
          // does not indicate success, just that the command was
          // received.
          //
          AckPacket();
         
          //
          // Return the status to the updater.
          //
          SendPacket(&g_ucStatus, 1);
         
          //
          // Go back and wait for a new command.
          //
          break;
         }
        //
        // This command is sent to transfer data to the device following
        // a download command.
        //
        case COMMAND_SEND_DATA:
         {//0x24 下载数据
          //
          // Until determined otherwise, the command status is success.
          //
          g_ucStatus = COMMAND_RET_SUCCESS;
         
          //
          // If this is overwriting the boot loader then the application
          // has already been erased so now erase the boot loader.
          //
          if(g_ulTransferAddress == 0)
           {
            //
            // Clear the flash access interrupt.
            //
            FLASH->FCMISC = FLASH_FCMISC_AMISC;
            
            //
            // Erase the application before the boot loader.
            //
            for(ulTemp = 0; ulTemp < APP_START_ADDRESS;
                ulTemp += 0x400)
             {
              //
              // Erase this block.
              //
              FLASH->FMA = ulTemp;
              FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;
             
              //
              // Wait until this block has been erased.
              //
              while(FLASH->FMC & FLASH_FMC_ERASE)
               {
               }
             }
            //
            // Return an error if an access violation occurred.
            //
            if(FLASH->FCRIS & FLASH_FCRIS_ARIS)
             {
              //
              // Setting g_ulTransferSize to zero makes
              // COMMAND_SEND_DATA fail to accept any more data.
              //
              g_ulTransferSize = 0;
             
              //
              // Indicate that the flash erase failed.
              //
              g_ucStatus = COMMAND_RET_FLASH_FAIL;
             }
           }
          //
          // Take one byte off for the command.
          //
          ulSize = ulSize - 1;
         
          //
          // Check if there are any more bytes to receive.
          //
          if(g_ulTransferSize >= ulSize)
           {
            //
            // This function is a stub to show where to insert a
            // function to decrypt the data as it is received.
            //
#ifdef ENABLE_DECRYPTION
            DecryptData(g_pucDataBuffer + 1, ulSize);
#endif
            //
            // Clear the flash access interrupt.
            //
            FLASH->FCMISC = FLASH_FCMISC_AMISC;

            //
            // Loop over the words to program.
            //
            for(ulTemp = 0; ulTemp < ((ulSize + 3) & ~3); ulTemp += 4)
             {
              //
              // Program the next word.
              //
              FLASH->FMA = g_ulTransferAddress + ulTemp;
              FLASH->FMD = g_pulDataBuffer[(ulTemp >> 2) + 1];
              FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;
            
              //
              // Wait until the word has been programmed.
              //
              while(FLASH->FMC & FLASH_FMC_WRITE)
               {
               }
             }
            //
            // Return an error if an access violation occurred.
            //
            if(FLASH->FCRIS & FLASH_FCRIS_ARIS)
             {
              //
              // Indicate that the flash programming failed.
              //
              g_ucStatus = COMMAND_RET_FLASH_FAIL;
             }
            else
             {
              //
              // Now update the address to program.
              //
              g_ulTransferSize -= ulSize;
              g_ulTransferAddress += ulSize;
             }
           }
          else
           {
             //
             // This indicates that too much data is being sent to the
             // device.
             //
             g_ucStatus = COMMAND_RET_INVALID_ADR;
           }
          //
          // Acknowledge that this command was received correctly.  This
          // does not indicate success, just that the command was
          // received.
          //
          Disp_Sts++;
          if(Disp_Sts&0x04)    //每4条指令改变一次显示
           {
            if(Disp_Sts&0x80)  //判断当前是否显示
             {
              Disp_Sts=0x00;   //清除显示标志
              Disp_Blank();    //清除显示
             }
            else
             {
              Disp_Sts=0x80;
              Disp_Boot();
             }
           } 	 	 			
          AckPacket();
          //
          // Go back and wait for a new command.
          //
          break;
         }
        //
        // This command is used to reset the device.
        //
        case COMMAND_RESET:
         {//0x25 复位
          //
          // Send out a one-byte ACK to ensure the byte goes back to the
          // host before we reset everything.
          //
          AckPacket();
          //
          // Make sure that the ACK packet has been sent.
          //
          FlushData();
          //
          // Perform a software reset request.  This will cause the
          // microcontroller to reset; no further code will be executed.
          //
          NVIC->APINT = (NVIC_APINT_VECTKEY |
                         NVIC_APINT_SYSRESETREQ);
          //
          // The microcontroller should have reset, so this should never
          // be reached.  Just in case, loop forever.
          //
          while(1)
           {
           }
         }
        case COMMAND_BOOTID:
         {//设置引导响应ID
          if(ulSize != 2)
           break;
          BOOT_EN[2]=g_pucDataBuffer[1];  //更新引导响应ID 
          break;				
         }
        case COMMAND_BOOTLOADER:
         {//引导区更新使能命令
          if(ulSize != 2)
           break;
          BootLoader_En=g_pucDataBuffer[1]; //更新引导响应ID  	 
          break;				
         }	
        //
        // Just acknowledge the command and set the error to indicate that
        // a bad command was sent.
        //
        default:
         {
          //
          // Acknowledge that this command was received correctly.  This
          // does not indicate success, just that the command was
          // received.
          //
          AckPacket();
          //
          // Indicate that a bad comand was sent.
          //
          if(Board_Id==BOOT_EN[2])    //只有板号设定的表位回送数据
           g_ucStatus = COMMAND_RET_UNKNOWN_CMD;
         
          //
          // Go back and wait for a new command.
          //
          break;
         }
       }
     }
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif

