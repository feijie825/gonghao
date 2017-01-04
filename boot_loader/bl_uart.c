//*****************************************************************************
//
// bl_uart.c - Functions to transfer data via the UART port.
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
extern volatile unsigned long BOOT_EN[];
extern volatile unsigned char Timer_Uart;

#include "lm3s2139.h"
#include "bl_config.h"
#include "bl_uart.h"

//*****************************************************************************
//
//! \addtogroup boot_loader_api
//! @{
//
//*****************************************************************************
#if defined(UART_ENABLE_UPDATE) || defined(DOXYGEN)

//*****************************************************************************
//
//! Sends data over the UART port.
//!
//! \param pucData is the buffer containing the data to write out to the UART
//! port.
//! \param ulSize is the number of bytes provided in \e pucData buffer that
//! will be written out to the UART port.
//!
//! This function sends \e ulSize bytes of data from the buffer pointed to by
//! \e pucData via the UART port.
//!
//! This function is contained in <tt>bl_uart.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
extern unsigned char Board_Id;
void
UARTSend(const unsigned char *pucData, unsigned long ulSize)
{
    if(Board_Id!=BOOT_EN[2])    //只有板号设定的表位回送数据
	 return;

    //
    // Transmit the number of bytes requested on the UART port.
    //
    while(ulSize--)
    {
#ifdef BOOT_UART1         //是否通过UART1引导
        //
        // Make sure that the transmit FIFO is not full.
        //
        while((UART1->FR & UART_FR_TXFF))
        {
        }

        //
        // Send out the next byte.
        //
        UART1->DR = *pucData++;
#else
        //
        // Make sure that the transmit FIFO is not full.
        //
        while((UART0->FR & UART_FR_TXFF))
        {
        }

        //
        // Send out the next byte.
        //
        UART0->DR = *pucData++;
#endif
    }
    //
    // Wait until the UART is done transmitting.
    //
    UARTFlush();
}

//*****************************************************************************
//
//! Waits until all data has been transmitted by the UART port.
//!
//! This function waits until all data written to the UART port has been
//! transmitted.
//!
//! This function is contained in <tt>bl_uart.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
void
UARTFlush(void)
{
#ifdef BOOT_UART1         //是否通过UART1引导
    //
    // Wait for the UART FIFO to empty and then wait for the shifter to get the
    // bytes out the port.
    //
    while(!(UART1->FR & UART_FR_TXFE))
    {
    }

    //
    // Wait for the FIFO to not be busy so that the shifter completes.
    //
    while((UART1->FR & UART_FR_BUSY))
    {
    }
#else
    //
    // Wait for the UART FIFO to empty and then wait for the shifter to get the
    // bytes out the port.
    //
    while(!(UART0->FR & UART_FR_TXFE))
    {
    }

    //
    // Wait for the FIFO to not be busy so that the shifter completes.
    //
    while((UART0->FR & UART_FR_BUSY))
    {
    }
#endif
}

//*****************************************************************************
//
//! Receives data over the UART port.
//!
//! \param pucData is the buffer to read data into from the UART port.
//! \param ulSize is the number of bytes provided in the \e pucData buffer that
//! should be written with data from the UART port.
//!
//! This function reads back \e ulSize bytes of data from the UART port, into
//! the buffer that is pointed to by \e pucData.  This function will not return
//! until \e ulSize number of bytes have been received.
//!
//! This function is contained in <tt>bl_uart.c</tt>.
//!
//! \return None.
// 2010.3.25 张力阵修改 添加超时处理
//接收成功 返回 真 不成功返回0
//*****************************************************************************
unsigned char UARTReceive(unsigned char *pucData, unsigned long ulSize)
{
    //
    // Send out the number of bytes requested.
    //
    Timer_Uart=0;                              //接收前清除超时定时器
    for(;;)
     {
#ifdef BOOT_UART1         //是否通过UART1引导
      //
      // Wait for the FIFO to not be empty.
      //
      while((UART1->FR & UART_FR_RXFE))         //等待数据接收
       {
        if(Timer_Uart>100)                      //100ms收不到数据
         return(0);                             //回主程序	
       }
      //
      // Receive a byte from the UART.
      //
      *pucData++ = UART1->DR;
#else
      //
      // Wait for the FIFO to not be empty.
      //
      while((UART0->FR & UART_FR_RXFE))         //等待数据接收
       {
        if(Timer_Uart>100)                      //100ms收不到数据
         return(0);                             //回主程序	
       }
      //
      // Receive a byte from the UART.
      //
      *pucData++ = UART0->DR;
#endif
      Timer_Uart=0;                            //接收成功 重置定时器
      ulSize--;
      if(ulSize==0) 
       return(0xFF);                           //
     }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
