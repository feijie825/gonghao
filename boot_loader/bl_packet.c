//*****************************************************************************
//
// bl_packet.c - Packet handler functions used by the boot loader.
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
#include "bl_i2c.h"
#include "bl_packet.h"
#include "bl_ssi.h"
#include "bl_uart.h"

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
// The packet that is sent to acknowledge a received packet.
//
//*****************************************************************************
static const unsigned char g_pucACK[2] = { 0, COMMAND_ACK };

//*****************************************************************************
//
// The packet that is sent to not-acknowledge a received packet.
//
//*****************************************************************************
static const unsigned char g_pucNAK[2] = { 0, COMMAND_NAK };

//*****************************************************************************
//
//! Calculates an 8-bit checksum
//!
//! \param pucData is a pointer to an array of 8-bit data of size ulSize.
//! \param ulSize is the size of the array that will run through the checksum
//! algorithm.
//!
//! This function simply calculates an 8-bit checksum on the data passed in.
//!
//! This function is contained in <tt>bl_packet.c</tt>.
//!
//! \return Returns the calculated checksum.
//
//*****************************************************************************
unsigned long
CheckSum(const unsigned char *pucData, unsigned long ulSize)
{
    unsigned long ulCheckSum;

    //
    // Initialize the checksum to zero.
    //
    ulCheckSum = 0;

    //
    // Add up all the bytes, do not do anything for an overflow.
    //
    while(ulSize--)
    {
        ulCheckSum += *pucData++;
    }

    //
    // Return the caculated check sum.
    //
    return(ulCheckSum & 0xff);
}

//*****************************************************************************
//
//! Sends an Acknowledge packet.
//!
//! This function is called to acknowledge that a packet has been received by
//! the microcontroller.
//!
//! This function is contained in <tt>bl_packet.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
void
AckPacket(void)
{
    //
    // ACK/NAK packets are the only ones with no size.
    //
    SendData(g_pucACK, 2);
}

//*****************************************************************************
//
//! Sends a no-acknowledge packet.
//!
//! This function is called when an invalid packet has been received by the
//! microcontroller, indicating that it should be retransmitted.
//!
//! This function is contained in <tt>bl_packet.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
void
NakPacket(void)
{
    //
    // ACK/NAK packets are the only ones with no size.
    //
    SendData(g_pucNAK, 2);
}

//*****************************************************************************
//
//! Receives a data packet.
//!
//! \param pucData is the location to store the data that is sent to the boot
//! loader.
//! \param pulSize is the number of bytes returned in the pucData buffer that
//! was provided.
//!
//! This function receives a packet of data from specified transfer function.
//!
//! This function is contained in <tt>bl_packet.c</tt>.
//!
//! \return Returns zero to indicate success while any non-zero value indicates
//! a failure.
//
//*****************************************************************************
int ReceivePacket(unsigned char *pucData, unsigned long *pulSize)
{
    unsigned long ulSize, ulCheckSum;
    unsigned char Sts;
    ulSize = 0;
    for(;;)
     {
      Sts=ReceiveData((unsigned char *)&ulSize, 1);   //数据长度字节
      if((Sts==0)||                                   //判断是否为接收超时
         (ulSize <3)||                                //包长度小于3 包长度错误
         (ulSize == 0xCC))                            //包长度为0xCC 	
       continue;                                      //等待
      else
       break;
     } 
    ulSize -= 2;                                      //减去长度和校验和 实际数据长度
    Sts=ReceiveData((unsigned char *)&ulCheckSum, 1); //接收校验和字节
    if(Sts==0)                                        //判断是否为接收超时
     return(-1);                                      //返回错误 回主程序                             	
    if(*pulSize >= ulSize)                            //判断 需要接收的数据长度>=包实际长度
     {                                                //
      Sts=ReceiveData(pucData, ulSize);               //接收包数据
      if((Sts==0)||                                   //判断是否为接收超时
      	(CheckSum(pucData, ulSize)!= (ulCheckSum & 0xff))) //检测校验和是否正确
       {                                              //校验和错误 返回错误
        NakPacket();
        return(-1);
       }
     }
    else                                               //包太长 舍弃
     {
      for(;;)
       {
        Sts==ReceiveData(pucData, 1);
        ulSize--;
        if((Sts==0)||                                  //判断是否为接收超时
           (ulSize==0))                                //判断是否接收完毕
        return(-1);
       }
     }
    *pulSize = ulSize;
    return(0);
}

//*****************************************************************************
//
//! Sends a data packet.
//!
//! \param pucData is the location of the data to be sent.
//! \param ulSize is the number of bytes to send.
//!
//! This function sends the data provided in the \e pucData parameter in the
//! packet format used by the boot loader.  The caller only needs to specify
//! the buffer with the data that needs to be transferred.  This function
//! addresses all other packet formatting issues.
//!
//! This function is contained in <tt>bl_packet.c</tt>.
//!
//! \return Returns zero to indicate success while any non-zero value indicates
//! a failure.
//
//*****************************************************************************
int SendPacket(unsigned char *pucData, unsigned long ulSize)
{
    unsigned long ulTemp;
    unsigned char Sts=0;
    //
    // Caculate the checksum to be sent out with the data.
    //
    ulTemp = CheckSum(pucData, ulSize);

    //
    // Need to include the size and checksum bytes in the packet.
    //
    ulSize += 2;

    //
    // Send out the size followed by the data.
    //
    SendData((unsigned char *)&ulSize, 1);
    SendData((unsigned char *)&ulTemp, 1);
    SendData(pucData, ulSize - 2);

    //
    // Wait for a non zero byte.
    //
    ulTemp = 0;
    Sts==ReceiveData((unsigned char *)&ulTemp, 1);	 //等待响应
    if((Sts==0)||                                    //判断是否接收超时
       (ulTemp != COMMAND_ACK))                      //不为响应命令
     return(-1);  
    //
    // This packet was sent and received successfully.
    //
    return(0);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
