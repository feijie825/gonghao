#line 1 "bl_packet.c"




























#line 1 "bl_commands.h"









































































































































































































































#line 30 "bl_packet.c"
#line 1 "bl_config.h"









































































































































































































































































































































































































#line 31 "bl_packet.c"
#line 1 "bl_i2c.h"


























































extern void I2CSend(unsigned char const *pucData, unsigned long ulSize);
extern void I2CReceive(unsigned char *pucData, unsigned long ulSize);
extern void I2CFlush(void);












#line 32 "bl_packet.c"
#line 1 "bl_packet.h"




































extern int ReceivePacket(unsigned char *pucData, unsigned long *pulSize);
extern int SendPacket(unsigned char *pucData, unsigned long ulSize);
extern void AckPacket(void);

#line 33 "bl_packet.c"
#line 1 "bl_ssi.h"
















































































extern void SSISend(unsigned char const *pucData, unsigned long ulSize);
extern void SSIReceive(unsigned char *pucData, unsigned long ulSize);
extern void SSIFlush(void);












#line 34 "bl_packet.c"
#line 1 "bl_uart.h"



































































extern void UARTSend(const unsigned char *pucData, unsigned long ulSize);
extern void UARTReceive(unsigned char *pucData, unsigned long ulSize);
extern void UARTFlush(void);
extern int UARTAutoBaud(unsigned long *pulRatio);












#line 35 "bl_packet.c"















static const unsigned char g_pucACK[2] = { 0, 0xcc };






static const unsigned char g_pucNAK[2] = { 0, 0x33 };
















unsigned long
CheckSum(const unsigned char *pucData, unsigned long ulSize)
{
    unsigned long ulCheckSum;

    
    
    
    ulCheckSum = 0;

    
    
    
    while(ulSize--)
    {
        ulCheckSum += *pucData++;
    }

    
    
    
    return(ulCheckSum & 0xff);
}













void
AckPacket(void)
{
    
    
    
    UARTSend(g_pucACK, 2);
}













void
NakPacket(void)
{
    
    
    
    UARTSend(g_pucNAK, 2);
}


















int
ReceivePacket(unsigned char *pucData, unsigned long *pulSize)
{
    unsigned long ulSize, ulCheckSum;

    ulSize = 0;
    while((ulSize == 0)||(ulSize == 0xCC))          
     UARTReceive((unsigned char *)&ulSize, 1);	    
    ulSize -= 2;								    
    UARTReceive((unsigned char *)&ulCheckSum, 1);   
    if(*pulSize >= ulSize)							
     {
      UARTReceive(pucData, ulSize);					
      if(CheckSum(pucData, ulSize) != (ulCheckSum & 0xff))
       {											
        NakPacket();
        return(-1);
       }
     }
    else
     {
        while(ulSize--)
         UARTReceive(pucData, 1);
        return(-1);
     }
    *pulSize = ulSize;
    return(0);
}



















int
SendPacket(unsigned char *pucData, unsigned long ulSize)
{
    unsigned long ulTemp;

    
    
    
    ulTemp = CheckSum(pucData, ulSize);

    
    
    
    ulSize += 2;

    
    
    
    UARTSend((unsigned char *)&ulSize, 1);
    UARTSend((unsigned char *)&ulTemp, 1);
    UARTSend(pucData, ulSize - 2);

    
    
    
    ulTemp = 0;
    while(ulTemp == 0)
    {
        UARTReceive((unsigned char *)&ulTemp, 1);	 
    }

    
    
    
    
    if(ulTemp != 0xcc)
    {
        return(-1);
    }

    
    
    
    return(0);
}






