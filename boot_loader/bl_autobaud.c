//*****************************************************************************
//
// bl_autobaud.c - Automatic baud rate detection code.
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

#include "LM3S2139.h"
#include "bl_config.h"
#include "bl_uart.h"

extern unsigned long CPUcpsid(void);
extern unsigned long CPUcpsie(void);

//*****************************************************************************
//
// If using auto-baud, make sure that the data buffer is large enough.
//
//*****************************************************************************
#if defined(UART_ENABLE_UPDATE) && defined(UART_AUTOBAUD) && (BUFFER_SIZE < 20)
#error ERROR: BUFFER_SIZE must be >= 20!
#endif

//*****************************************************************************
//
//! \addtogroup boot_loader_api
//! @{
//
//*****************************************************************************
#if defined(UART_ENABLE_UPDATE) && defined(UART_AUTOBAUD) || defined(DOXYGEN)

//*****************************************************************************
//
// This define holds the multiplier for the pulse detection algorithm.  The
// value is used to generate a fractional difference detection of
// 1 / PULSE_DETECTION_MULT.
//
//*****************************************************************************
#define PULSE_DETECTION_MULT    3

//*****************************************************************************
//
// This define holds the minimum number of edges to successfully sync to a
// pattern of 2 bytes.
//
//*****************************************************************************
#define MIN_EDGE_COUNT          18

//*****************************************************************************
//
// This global holds the number of edges that have been stored in the global
// buffer g_pulDataBuffer.
//
//*****************************************************************************
static volatile unsigned long g_ulTickIndex;

//*****************************************************************************
//
// The data buffer that is used for receiving packets is used to hold the edge
// times during auto-baud.  The buffer is not used for receiving packets while
// auto-baud is in progress, so this does not present problems.
//
//*****************************************************************************
extern unsigned long g_pulDataBuffer[];

//*****************************************************************************
//
//! Handles the UART Rx GPIO interrupt.
//!
//! When an edge is detected on the UART Rx pin, this function is called to
//! save the time of the edge.  These times are later used to determine the
//! ratio of the UART baud rate to the processor clock rate.
//!
//! This function is contained in <tt>bl_autobaud.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
void GPIOIntHandler(void)
{
    unsigned long ulTemp;

#ifdef BOOT_UART1         //是否通过UART1引导
    GPIOD->ICR = UART_RX1;
#else
    //
    // Clear the GPIO interrupt source.
    //
    GPIOA->ICR = UART_RX0;
#endif
    //
    // While we still have space in our buffer, store the current system tick
    // count and return from interrupt.
    //
    if(g_ulTickIndex < 20)
    {
        ulTemp = NVIC->ST_CURRENT;
        g_pulDataBuffer[g_ulTickIndex++] = ulTemp;
    }
}

//*****************************************************************************
//
//! Performs auto-baud on the UART port.
//!
//! \param pulRatio is the ratio of the processor's crystal frequency to the
//! baud rate being used by the UART port for communications.
//!
//! This function attempts to synchronize to the updater program that is trying
//! to communicate with the boot loader.  The UART port is monitored for edges
//! using interrupts.  Once enough edges are detected, the boot loader
//! determines the ratio of baud rate and crystal frequency needed to program
//! the UART.
//!
//! This function is contained in <tt>bl_autobaud.c</tt>.
//!
//! \return Returns a value of 0 to indicate that this call successfully
//! synchronized with the other device communicating over the UART, and a
//! negative value to indicate that this function did not successfully
//! synchronize with the other UART device.
//
//*****************************************************************************
int
UARTAutoBaud(unsigned long *pulRatio)
{
    long lPulse, lValidPulses, lTemp, lTotal;
    volatile long lDelay;

    //
    // Configure and enable SysTick.  Set the reload value to the maximum;
    // there are only 24 bits in the register but loading 32 bits of ones is
    // more efficient.
    //
    NVIC->ST_RELOAD = 0xffffffff;
    NVIC->ST_CTRL = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
    //
    // Reset the counters that control the pulse detection.
    //
    lValidPulses = 0;
    lTotal = 0;
    g_ulTickIndex = 0;

#ifdef BOOT_UART1         //是否通过UART1引导
    //
    // Set the pad(s) for standard push-pull operation.
    //
    GPIOD->PUR |= UART_PINS1;
    GPIOD->DEN |= UART_PINS1;

    //
    // Interrupt on both edges.
    //
    GPIOD->IBE |= UART_RX1;

    //
    // Clear out all of the gpio interrupts in this register.
    //
    GPIOD->ICR = UART_RX1;

    //
    // Enable the GPIO pin corresponding to the UART RX pin.
    //
    GPIOD->IM |= UART_PINS1;


    //
    // 张力阵 2008.11.12添加
    //
    CPUcpsie();	   //开中断 2010.3.10张力阵添加
    GPIOD->DIR |= UART_TX1;
    GPIOD->DR2R |= UART_TX1;
    GPIOD->DATA[UART_TX1] = UART_TX1;
    //
    // Enable GPIOd Interrupt.
    //
    NVIC->EN[0] = (1<<(INT_GPIOD-16));

    //
    // Wait for MIN_EDGE_COUNT to pass to collect enough edges.
    //
    while(g_ulTickIndex < MIN_EDGE_COUNT)
    {
    }

    //
    // Disable GPIOD Interrupt.
    //
    NVIC->DIS[0] = (1<<(INT_GPIOD-16));
#else
    //
    // Set the pad(s) for standard push-pull operation.
    //
    GPIOA->PUR |= UART_PINS0;
    GPIOA->DEN |= UART_PINS0;

    //
    // Interrupt on both edges.
    //
    GPIOA->IBE |= UART_RX0;

    //
    // Clear out all of the gpio interrupts in this register.
    //
    GPIOA->ICR = UART_RX0;

    //
    // Enable the GPIO pin corresponding to the UART RX pin.
    //
    GPIOA->IM |= UART_PINS0;


    //
    // 张力阵 2008.11.12添加
    //
    CPUcpsie();	   //开中断 2010.3.10张力阵添加
    GPIOA->DIR |= UART_TX0;
    GPIOA->DR2R |= UART_TX0;
    GPIOA->DATA[UART_TX0] = UART_TX0;
    //
    // Enable GPIOA Interrupt.
    //
    NVIC->EN[0] = 1;

    //
    // Wait for MIN_EDGE_COUNT to pass to collect enough edges.
    //
    while(g_ulTickIndex < MIN_EDGE_COUNT)
    {
    }

    //
    // Disable GPIOA Interrupt.
    //
    NVIC->DIS[0] = 1;
#endif
    //
    // Calculate the pulse widths from the array of tick times.
    //
    for(lPulse = 0; lPulse < (MIN_EDGE_COUNT - 1); lPulse++)
    {
        lTemp = (((long)g_pulDataBuffer[lPulse] -
                  (long)g_pulDataBuffer[lPulse + 1]) & 0x00ffffff);
        g_pulDataBuffer[lPulse] = lTemp;
    }

    //
    // This loops handles checking for consecutive pulses that have pulse
    // widths that are within an acceptable margin.
    //
    for(lPulse = 0; lPulse < (MIN_EDGE_COUNT - 1); lPulse++)
    {
        //
        // Calculate the absolute difference between two consecutive pulses.
        //
        lTemp = (long)g_pulDataBuffer[lPulse];
        lTemp -= (long)g_pulDataBuffer[lPulse + 1];
        if(lTemp < 0)
        {
            lTemp *= -1;
        }

        //
        // This pulse detection code uses the following algorithm:
        // If the following is true then we have consecutive acceptable pulses
        // abs(Pulse[n] - Pulse[n + 1]) < Pulse[n + 1] / PULSE_DETECTION_MULT
        // or
        // PULSE_DETECTION_MULT * abs(Pulse[n] - Pulse[n + 1]) < Pulse[n + 1]
        //
        if((lTemp * PULSE_DETECTION_MULT) < (long)g_pulDataBuffer[lPulse + 1])
        {
            lTotal += (long)g_pulDataBuffer[lPulse];
            lValidPulses++;
        }
        else
        {
            lValidPulses = 0;
            lTotal = 0;
        }

        //
        // Once we have 7 pulses calculate the ratio needed to program the
        // UART.
        //
        if(lValidPulses == 7)
        {
            //
            // Add in the last pulse and calculate the ratio.
            //
            lTotal += (long)g_pulDataBuffer[lPulse];
            *pulRatio = lTotal >> 1;

            //
            // Wait for at least 2 UART clocks since we only wait for 18 of 20
            // that are coming from the host.  If we don't wait, we can turn
            // on the UART while the last two pulses come down.
            //
            for(lDelay = lTotal; lDelay; lDelay--)
            {
            }

            //
            // Indicate a successful auto baud operation.
            //
            return(0);
        }
    }

    //
    // Automatic baud rate detection failed.
    //
    return(-1);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
