#line 1 "LM3S21xx_gpio.c"




 
#line 1 "LM3S2139.h"








 

#line 1 "LM3S21xx_types.h"




 









typedef unsigned char tBoolean;

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
typedef unsigned int   BOOL;

typedef unsigned long   u32;
typedef unsigned short  u16;
typedef unsigned char   u8;

typedef unsigned long  const uc32;   
typedef unsigned short const uc16;   
typedef unsigned char  const uc8;    

typedef signed long   s32;
typedef signed short  s16;
typedef signed char   s8;

typedef signed long  const sc32;   
typedef signed short const sc16;   
typedef signed char  const sc8;    


typedef volatile unsigned long   vu32;
typedef volatile unsigned short  vu16;
typedef volatile unsigned char   vu8;

typedef volatile unsigned long  const vuc32;   
typedef volatile unsigned short const vuc16;   
typedef volatile unsigned char  const vuc8;    

typedef volatile signed long   vs32;
typedef volatile signed short  vs16;
typedef volatile signed char   vs8;

typedef volatile signed long  const vsc32;   
typedef volatile signed short const vsc16;   
typedef volatile signed char  const vsc8;    


























#line 95 "LM3S21xx_types.h"






























#line 131 "LM3S21xx_types.h"










































#line 180 "LM3S21xx_types.h"

#line 14 "LM3S2139.h"




 
#line 28 "LM3S2139.h"



 
#line 80 "LM3S2139.h"































#line 165 "LM3S2139.h"















typedef struct
{
	vu32 LOAD;                
	vu32 VALUE;               
	vu32 CTL;                 
	vu32 ICR;                 
	vu32 RIS;                 
	vu32 MIS;                 
	u32  EMPTY1[256];         
	vu32 TEST;                
	u32  EMPTY[505];          
	vu32 LOCK;                
}WDG_Typedef;
#line 203 "LM3S2139.h"





#line 216 "LM3S2139.h"






#line 234 "LM3S2139.h"






typedef struct
{
	vu32 DATA[256];             
	vu32 DIR;                   
	vu32 IS;                    
	vu32 IBE;                   
	vu32 IEV;                   
	vu32 IM;                    
	vu32 RIS;                   
	vu32 MIS;                   
	vu32 ICR;                   
	vu32 AFSEL;                 
	vu32 EMPTY2[55];            
	vu32 DR2R;                  
	vu32 DR4R;                  
	vu32 DR8R;                  
	vu32 ODR;                   
	vu32 PUR;                   
	vu32 PDR;                   
	vu32 SLR;                   
	vu32 DEN;                   
	vu32 LOCK;                  
	vu32 CR;                    
	vu32 AMSEL;                 
}GPIO_Typedef;
#line 273 "LM3S2139.h"

#line 296 "LM3S2139.h"






#line 323 "LM3S2139.h"






#line 350 "LM3S2139.h"






#line 377 "LM3S2139.h"






#line 404 "LM3S2139.h"






typedef struct
{
	vu32 CR0;                   
	vu32 CR1;                   
	vu32 DR;                    
	vu32 SR;                    
	vu32 CPSR;                  
	vu32 IM;                    
	vu32 RIS;                   
	vu32 MIS;                   
	vu32 ICR;                   
	vu32 DMACTL;                
}SSI_Typedef;


#line 435 "LM3S2139.h"






typedef struct
{
	vu32 DR;                    
	vu32 RSR;                   
	u32  EMPTY1[4];             
	vu32 FR;                    
	u32  EMPTY2;                
	vu32 ILPR;                  
	vu32 IBRD;                  
	vu32 FBRD;                  
	vu32 LCRH;                  
	vu32 CTL;                   
	vu32 IFLS;                  
	vu32 IM;                    
	vu32 RIS;                   
	vu32 MIS;                   
	vu32 ICR;                   
	vu32 DMACTL;                
}UART_Typedef;
#line 476 "LM3S2139.h"





#line 496 "LM3S2139.h"






#line 515 "LM3S2139.h"






#line 537 "LM3S2139.h"






#line 559 "LM3S2139.h"






#line 581 "LM3S2139.h"






#line 608 "LM3S2139.h"






#line 635 "LM3S2139.h"






#line 662 "LM3S2139.h"






#line 689 "LM3S2139.h"






typedef struct
{
	vu32 CFG;                   
	vu32 TAMR;                  
	vu32 TBMR;                  
	vu32 CTL;                   
	u32  EMPTY1[2];             
	vu32 IMR;                   
	vu32 RIS;                   
	vu32 MIS;                   
	vu32 ICR;                   
	vu32 TAILR;                 
	vu32 TBILR;                 
	vu32 TAMATCHR;              
	vu32 TBMATCHR;              
	vu32 TAPR;                  
	vu32 TBPR;                  
	vu32 TAPMR;                 
	vu32 TBPMR;                 
	vu32 TAR;                   
	vu32 TBR;                   
}TIMER_Typedef;




#line 739 "LM3S2139.h"





#line 762 "LM3S2139.h"






#line 786 "LM3S2139.h"






#line 810 "LM3S2139.h"





 
typedef struct
{
	vu32 SSMUX;                
	vu32 SSCTL;                
	vu32 SSFIFO;               
	vu32 SSFSTAT;              
	u32  EMPTY[4];             
}ADC_SEQ;
typedef struct
{
	vu32 ACTSS;                 
	vu32 RIS;                   
	vu32 IM;                    
	vu32 ISC;                   
	vu32 OSTAT;                 
	vu32 EMUX;                  
	vu32 USTAT;                 
	u32  EMPTY1;                
	vu32 SSPRI;                 
	u32  EMPTY2;                
	vu32 PSSI;                  
	u32  EMPTY3;                
	vu32 SAC;                   
	u32  EMPTY4[3];             
	ADC_SEQ SEQ[4];             
	u32  EMPTY8[16];            
	vu32 TMLB;                  
}ADC_Typedef;
#line 872 "LM3S2139.h"






#line 888 "LM3S2139.h"






typedef struct
{
	vu32 CRQ;           
	vu32 CMSK;          
	vu32 MSK1;          
	vu32 MSK2;          
	vu32 ARB1;          
	vu32 ARB2;          
	vu32 MCTL;          
	vu32 DA1;           
	vu32 DA2;           
	vu32 DB1;           
	vu32 DB2;           
	vu32 EMPTY[13];	    
}CAN_INF;             
typedef struct
{
	vu32 CTL;              
	vu32 STS;              
	vu32 ERR;              
	vu32 BIT;              
	vu32 INT;              
	vu32 TST;              
	vu32 BRPE;             
	vu32 EMPTY1;           
	CAN_INF INF[2];        
	vu32 EMPTY2[8];        
	vu32 TXRQ1;            
	vu32 TXRQ2;            
	vu32 EMPTY3[6];        
	vu32 NWDA1;            
	vu32 NWDA2;            
	vu32 EMPTY4[6];        
	vu32 MSG1INT;          
	vu32 MSG2INT;          
	vu32 EMPTY5[6];        
	vu32 MSG1VAL;          
	vu32 MSG2VAL;          
}CAN_Typedef;
#line 971 "LM3S2139.h"


 
#line 1011 "LM3S2139.h"





 
typedef struct
{
	vu32 FMA;                   
	vu32 FMD;                   
	vu32 FMC;                   
	vu32 FCRIS;                 
	vu32 FCIM;                  
	vu32 FCMISC;                
	u32  EMPTY1[1078];          
	vu32 RMCTL;                 
	vu32 RMVER;                 
	u32  EMPTY2[14];            
	vu32 FMPRE0;                
	vu32 FMPPE0;                
	u32  EMPTY3[2];             
	vu32 USECRL;                
	u32  EMPTY4[35];            
	vu32 USERDBG;               
	u32  EMPTY5[3];             
	vu32 USERREG[4];            
	u32  EMPTY6[4];             
	vu32 FMPRE[4];              
	u32  EMPTY7[124];           
	vu32 FMPPE[4];              
}FLASH_Typedef;
#line 1067 "LM3S2139.h"





typedef struct
{
	vu32 DID[2];          
	vu32 DC0;             
	vu32 DC[5];           
	u32  EMPTY1[4];       
	vu32 PBORCTL;         
	vu32 LDOPCTL;         
	u32  EMPTY2[2];       
	vu32 SRCR[3];         
	u32  EMPTY3;          
	vu32 RIS;             
	vu32 IMC;             
	vu32 MISC;            
	vu32 RESC;            
	vu32 RCC;             
	vu32 PLLCFG;          
	u32  EMPTY4;          
	vu32 GPIOHSCTL;       
	vu32 RCC2;            
	u32  EMPTY5[35];       
	vu32 RCGC[3];         
	u32  EMPTY6;          
	vu32 SCGC[3];         
	u32  EMPTY7;          
	vu32 DCGC[3];         
	u32  EMPTY8[6];       
	vu32 DSLPCLKCFG;      
	u32  EMPTY9[2];       
	vu32 CLKVCLR;         
	u32  EMPTY10[3];      
	vu32 LDOARST;         
	u32  EMPTY11[31];     
	vu32 USER[2];         
}SYSCTL_Typedef;
#line 1142 "LM3S2139.h"




 
typedef struct 
{ 
	u32  EMPYT1;           
	vu32 INT_TYPE;         
	u32  EMPTY2[2];        
	vu32 ST_CTRL;          
	vu32 ST_RELOAD;        
	vu32 ST_CURRENT;       
	vu32 ST_CAL;           
	u32  EMPTY3[56];       
	vu32 EN[2];            
	u32  EMPTY4[30];       
	vu32 DIS[2];           
	u32  EMPTY5[30];       
	vu32 PEND[2];          
	u32  EMPTY6[30];       
	vu32 UNPEND[2];        
	u32  EMPTY7[30];       
	vu32 ACTIVE[2];        
	u32  EMPTY8[62];       
	vu32 PRI[12];          
	u32  EMPTY9[564];      
	vu32 CPUID;            
	vu32 INT_CTRL;         
	vu32 VTABLE;           
	vu32 APINT;            
	vu32 SYS_CTRL;         
	vu32 CFG_CTRL;         
	vu32 SYS_PRI[3];       
	vu32 SYS_HND_CTRL;     
	vu32 FAULT_STAT;       
	vu32 HFAULT_STAT;      
	vu32 DEBUG_STAT;       
	vu32 MM_ADDR;          
	vu32 FAULT_ADDR;       
	u32  EMPTY10[21];      
	vu32 MPU_TYPE;         
	vu32 MPU_CTRL;         
	vu32 MPU_NUMBER;       
	vu32 MPU_BASE;         
	vu32 MPU_ATTR;         
	u32  EMPTY11[19];      
	vu32 DBG_CTRL;         
	vu32 DBG_XFER;         
	vu32 DBG_DATA;         
	vu32 DBG_INT;          
	u32  EMPTY12[64];      
	vu32 SW_TRIG;          
}NVIC_Typedef;
#line 1249 "LM3S2139.h"







































































                                            















#line 1354 "LM3S2139.h"
















#line 1376 "LM3S2139.h"
                                            
#line 1393 "LM3S2139.h"









                                            



































                                            


                                            

                                            







                                            

                                            

                                            

                                            







                                            

                                            

                                            

                                            







                                            

                                            






#line 1494 "LM3S2139.h"











































#line 1543 "LM3S2139.h"






#line 1561 "LM3S2139.h"






#line 1574 "LM3S2139.h"






#line 1587 "LM3S2139.h"






#line 1600 "LM3S2139.h"






#line 1613 "LM3S2139.h"














#line 1638 "LM3S2139.h"













































#line 1694 "LM3S2139.h"
























#line 1743 "LM3S2139.h"

































#line 1787 "LM3S2139.h"









































































































#line 1909 "LM3S2139.h"






#line 1922 "LM3S2139.h"






#line 1935 "LM3S2139.h"






#line 1948 "LM3S2139.h"







                                            

                                            









                                            












































                                            












                                            












                                            

                                            

                                            


                                            

                                            

                                            







































#line 2107 "LM3S2139.h"















#line 2128 "LM3S2139.h"
















#line 2151 "LM3S2139.h"



























































































































#line 2318 "LM3S2139.h"
















#line 2354 "LM3S2139.h"
















#line 2378 "LM3S2139.h"







#line 2392 "LM3S2139.h"













#line 2421 "LM3S2139.h"






#line 2459 "LM3S2139.h"














#line 2479 "LM3S2139.h"






#line 2493 "LM3S2139.h"






#line 2515 "LM3S2139.h"














#line 2535 "LM3S2139.h"






#line 2549 "LM3S2139.h"






#line 2571 "LM3S2139.h"














#line 2591 "LM3S2139.h"
































#line 2629 "LM3S2139.h"






#line 2641 "LM3S2139.h"







#line 2656 "LM3S2139.h"



















#line 2691 "LM3S2139.h"








#line 2731 "LM3S2139.h"



























#line 2765 "LM3S2139.h"







#line 2779 "LM3S2139.h"







                                            

                                            

                                            










































#line 2851 "LM3S2139.h"













#line 2882 "LM3S2139.h"













#line 2913 "LM3S2139.h"






#line 2926 "LM3S2139.h"






#line 2946 "LM3S2139.h"



















                                            
#line 2972 "LM3S2139.h"















#line 2996 "LM3S2139.h"

















































                                            
                                            






#line 3062 "LM3S2139.h"











































#line 3116 "LM3S2139.h"









































                                            
                                            






#line 3174 "LM3S2139.h"











































#line 3228 "LM3S2139.h"

















































































#line 3346 "LM3S2139.h"








































































#line 3427 "LM3S2139.h"

















































#line 3486 "LM3S2139.h"










































































































#line 3599 "LM3S2139.h"







                                            
















                                            

                                            







#line 3665 "LM3S2139.h"












































































#line 3749 "LM3S2139.h"














































































#line 3839 "LM3S2139.h"
                                            

                                            
#line 3849 "LM3S2139.h"








                                            
                                            

                                            
                                            



                                            
                                            
                                            
#line 4012 "LM3S2139.h"
                                            
#line 4023 "LM3S2139.h"






#line 4046 "LM3S2139.h"






#line 4059 "LM3S2139.h"
                                            

                                            

                                            
#line 4077 "LM3S2139.h"






#line 4099 "LM3S2139.h"






#line 4136 "LM3S2139.h"






#line 4158 "LM3S2139.h"
















#line 4186 "LM3S2139.h"






#line 4199 "LM3S2139.h"






#line 4272 "LM3S2139.h"






#line 4286 "LM3S2139.h"






#line 4367 "LM3S2139.h"







#line 4445 "LM3S2139.h"




















#line 4472 "LM3S2139.h"






#line 4494 "LM3S2139.h"






#line 4512 "LM3S2139.h"







                                            

                                            


                                            

                                            

                                            

                                            

                                            












                                            

                                            

                                            









                                            

                                            

                                            

                                            

                                            

                                            

                                            







#line 4591 "LM3S2139.h"







                                            

                                            

                                            
#line 4616 "LM3S2139.h"






#line 4634 "LM3S2139.h"






#line 4652 "LM3S2139.h"







                                            

                                            

                                            
#line 4677 "LM3S2139.h"






#line 4695 "LM3S2139.h"






#line 4713 "LM3S2139.h"







                                            

                                            

                                            
#line 4738 "LM3S2139.h"






#line 4756 "LM3S2139.h"











                                            

                                            
#line 4778 "LM3S2139.h"







                                            









#line 4803 "LM3S2139.h"













#line 4830 "LM3S2139.h"








#line 4845 "LM3S2139.h"







#line 4861 "LM3S2139.h"





























































#line 4934 "LM3S2139.h"







#line 4960 "LM3S2139.h"







#line 4976 "LM3S2139.h"







#line 4990 "LM3S2139.h"















#line 5018 "LM3S2139.h"

































































































#line 5147 "LM3S2139.h"






#line 5181 "LM3S2139.h"






#line 5219 "LM3S2139.h"






#line 5253 "LM3S2139.h"






#line 5291 "LM3S2139.h"






#line 5325 "LM3S2139.h"






#line 5363 "LM3S2139.h"






#line 5397 "LM3S2139.h"






#line 5435 "LM3S2139.h"






#line 5469 "LM3S2139.h"






#line 5483 "LM3S2139.h"






#line 5497 "LM3S2139.h"






#line 5511 "LM3S2139.h"






#line 5525 "LM3S2139.h"






#line 5539 "LM3S2139.h"






#line 5553 "LM3S2139.h"






#line 5567 "LM3S2139.h"






#line 5581 "LM3S2139.h"






#line 5595 "LM3S2139.h"






#line 5609 "LM3S2139.h"






#line 5623 "LM3S2139.h"
















#line 5649 "LM3S2139.h"















#line 5679 "LM3S2139.h"















#line 5700 "LM3S2139.h"






#line 5713 "LM3S2139.h"















#line 5735 "LM3S2139.h"







#line 5754 "LM3S2139.h"







#line 5778 "LM3S2139.h"





















































































#line 5915 "LM3S2139.h"






#line 5927 "LM3S2139.h"
                                            
#line 5938 "LM3S2139.h"






#line 5967 "LM3S2139.h"














#line 5992 "LM3S2139.h"









#line 7 "LM3S21xx_gpio.c"
#line 1 "LM3S21xx_Lib.h"




 
#line 7 "LM3S21xx_Lib.h"




























#line 1 "LM3S21xx_gpio.h"




 





















#line 35 "LM3S21xx_gpio.h"








































#line 82 "LM3S21xx_gpio.h"






extern void GPIODirModeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulPinIO);
extern u32 GPIODirModeGet(GPIO_Typedef *GPIOx, u8 ucPin);
extern void GPIOIntTypeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulIntType);
extern u32 GPIOIntTypeGet(GPIO_Typedef *GPIOx, u8 ucPin);
extern void GPIOPadConfigSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulStrength,u32 ulPadType);
extern void GPIOPadConfigGet(GPIO_Typedef *GPIOx, u8 ucPin,u32 *pulStrength,u32 *pulPadType);
extern void GPIOPinIntEnable(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinIntDisable(GPIO_Typedef *GPIOx, u8 ucPins);
extern u32 GPIOPinIntStatus(GPIO_Typedef *GPIOx, u8 bMasked);
extern void GPIOPinIntClear(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPortIntRegister(GPIO_Typedef *GPIOx,void (*pfnIntHandler)(void));
extern void GPIOPortIntUnregister(GPIO_Typedef *GPIOx);
extern u32 GPIOPinRead(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinWrite(GPIO_Typedef *GPIOx, u8 ucPins,u8 ucVal);
extern void GPIOPinTypeADC(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeCAN(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeComparator(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeGPIOInput(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeGPIOOutput(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeGPIOOutputOD(GPIO_Typedef *GPIOx,u8 ucPins);
extern void GPIOPinTypeI2C(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypePWM(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeQEI(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeSSI(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeTimer(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeUART(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeUSBDigital(GPIO_Typedef *GPIOx, u8 ucPins);
extern void Init_Gpio(void);          










#line 37 "LM3S21xx_Lib.h"










#line 1 "LM3S21xx_ssi.h"




 


































#line 46 "LM3S21xx_ssi.h"


















extern void SSIConfigSetExpClk(SSI_Typedef *SSIx, u32 ulSSIClk,
                               u32 ulProtocol, u32 ulMode,
                               u32 ulBitRate,u32 ulDataWidth);
extern void SSIDataGet(SSI_Typedef *SSIx, u32 *pulData);
extern u32 SSIDataGetNonBlocking(SSI_Typedef *SSIx,u32 *pulData);
extern void SSIDataPut(SSI_Typedef *SSIx, u32 ulData);
extern u32 SSIDataPutNonBlocking(SSI_Typedef *SSIx, u32 ulData);
extern void SSIDisable(SSI_Typedef *SSIx);
extern void SSIEnable(SSI_Typedef *SSIx);
extern void SSIIntClear(SSI_Typedef *SSIx, u32 ulIntFlags);
extern void SSIIntDisable(SSI_Typedef *SSIx, u32 ulIntFlags);
extern void SSIIntEnable(SSI_Typedef *SSIx, u32 ulIntFlags);
extern void SSIIntRegister(SSI_Typedef *SSIx, void(*pfnHandler)(void));
extern u32 SSIIntStatus(SSI_Typedef *SSIx, u8 bMasked);
extern void SSIIntUnregister(SSI_Typedef *SSIx);
extern void SSIDMAEnable(SSI_Typedef *SSIx, u32 ulDMAFlags);
extern void SSIDMADisable(SSI_Typedef *SSIx, u32 ulDMAFlags);
extern void Init_Ssi(void);    
void SSIDataLen(SSI_Typedef *SSIx,u8 Len);







#line 1 "lm3s21xx_sysctl.h"




 























 

#line 73 "lm3s21xx_sysctl.h"






#line 112 "lm3s21xx_sysctl.h"







#line 130 "lm3s21xx_sysctl.h"
















#line 155 "lm3s21xx_sysctl.h"







#line 168 "lm3s21xx_sysctl.h"

















#line 192 "lm3s21xx_sysctl.h"











#line 299 "lm3s21xx_sysctl.h"






extern void Init_Pll(void);		               
extern u32 SysCtlSRAMSizeGet(void);
extern u32 SysCtlFlashSizeGet(void);
extern u8 SysCtlPinPresent(u32 ulPin);
extern u8 SysCtlPeripheralPresent(u32 ulPeripheral);
extern void SysCtlPeripheralReset(u32 ulPeripheral);
extern void SysCtlPeripheralEnable(u32 ulPeripheral);
extern void SysCtlPeripheralDisable(u32 ulPeripheral);
extern void SysCtlPeripheralSleepEnable(u32 ulPeripheral);
extern void SysCtlPeripheralSleepDisable(u32 ulPeripheral);
extern void SysCtlPeripheralDeepSleepEnable(u32 ulPeripheral);
extern void SysCtlPeripheralDeepSleepDisable(u32 ulPeripheral);
extern void SysCtlPeripheralClockGating(u8 bEnable);
extern void SysCtlIntRegister(void (*pfnHandler)(void));
extern void SysCtlIntUnregister(void);
extern void SysCtlIntEnable(u32 ulInts);
extern void SysCtlIntDisable(u32 ulInts);
extern void SysCtlIntClear(u32 ulInts);
extern u32 SysCtlIntStatus(u8 bMasked);
extern void SysCtlLDOSet(u32 ulVoltage);
extern u32 SysCtlLDOGet(void);
extern void SysCtlLDOConfigSet(u32 ulConfig);
extern void SysCtlReset(void);
extern void SysCtlSleep(void);
extern void SysCtlDeepSleep(void);
extern u32 SysCtlResetCauseGet(void);
extern void SysCtlResetCauseClear(u32 ulCauses);
extern void SysCtlBrownOutConfigSet(u32 ulConfig,u32 ulDelay);
extern void SysCtlDelay(u32 ulCount);
extern void SysCtlClockSet(u32 ulConfig);
extern u32 SysCtlClockGet(void);
extern void SysCtlPWMClockSet(u32 ulConfig);
extern u32 SysCtlPWMClockGet(void);
extern void SysCtlADCSpeedSet(u32 ulSpeed);
extern u32 SysCtlADCSpeedGet(void);
extern void SysCtlIOSCVerificationSet(u8 bEnable);
extern void SysCtlMOSCVerificationSet(u8 bEnable);
extern void SysCtlPLLVerificationSet(u8 bEnable);
extern void SysCtlClkVerificationClear(void);
extern void SysCtlGPIOAHBEnable(u32 ulGPIOPeripheral);
extern void SysCtlGPIOAHBDisable(u32 ulGPIOPeripheral);
extern void SysCtlUSBPLLEnable(void);
extern void SysCtlUSBPLLDisable(void);










#line 92 "LM3S21xx_ssi.h"
#line 99 "LM3S21xx_ssi.h"










#line 49 "LM3S21xx_Lib.h"


#line 53 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_systick.h"




 




















extern void SysTickEnable(void);
extern void SysTickDisable(void);
extern void SysTickIntRegister(void (*pfnHandler)(void));
extern void SysTickIntUnregister(void);
extern void SysTickIntEnable(void);
extern void SysTickIntDisable(void);
extern void SysTickPeriodSet(u32 ulPeriod);
extern u32 SysTickPeriodGet(void);
extern u32 SysTickValueGet(void);
extern void Init_SysTick(void);		










#line 57 "LM3S21xx_Lib.h"














#line 1 "LM3S21xx_debug.h"




 











#line 26 "LM3S21xx_debug.h"







#line 43 "LM3S21xx_debug.h"

#line 73 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_cpu.h"




 




















extern unsigned long CPUcpsid(void);
extern unsigned long CPUcpsie(void);
extern void CPUwfi(void);










#line 77 "LM3S21xx_Lib.h"
#line 8 "LM3S21xx_gpio.c"
#line 9 "LM3S21xx_gpio.c"
#line 1 "disp.h"




 
#line 7 "disp.h"
#line 1 "LM3S21xx_Lib.h"




 
#line 7 "LM3S21xx_Lib.h"




























#line 37 "LM3S21xx_Lib.h"










#line 49 "LM3S21xx_Lib.h"


#line 53 "LM3S21xx_Lib.h"


#line 57 "LM3S21xx_Lib.h"














#line 73 "LM3S21xx_Lib.h"


#line 77 "LM3S21xx_Lib.h"
#line 8 "disp.h"
#line 9 "disp.h"
extern u8        Disp_Buf[8];                    



 
void SysCtlPeripheralEnable(u32 ulPeripheral);


#line 23 "disp.h"

#line 32 "disp.h"

#line 41 "disp.h"


#line 51 "disp.h"


#line 61 "disp.h"












#line 81 "disp.h"








                 






















#line 120 "disp.h"









#line 135 "disp.h"















































#line 212 "disp.h"




#line 223 "disp.h"
                                                
                                                
                                                
                                                
                                                
                                                
                                                
                                                

                                                
                                                
                                                
                                                
                                                
                                                
                                                
                                                

                                                
                                                
                                                
                                                
                                                
                                                
                                                
                                                

                                                

                                                

                                                

                                                



void Reset_HD7279(void);                 
void Disp_Data(u8 Disp_Mode);            
void Disp_Boot(void);                    
void Disp_Blank(void);                   

#line 10 "LM3S21xx_gpio.c"


 
void Init_Gpio(void)
{

    SysCtlPeripheralEnable(((3<<8)|0 ));

    GPIOPinTypeGPIOOutput(((GPIO_Typedef*)0x40004000),            
                          0x00000010);        
    GPIOPinTypeSSI(((GPIO_Typedef*)0x40004000),                   
                   0x00000004|0x00000008|0x00000020);    









                                    

}





 
void GPIODirModeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulPinIO)
{
    ;


    GPIOx->DIR = ((ulPinIO & 1) ? (GPIOx->DIR | ucPins) :          
                                  (GPIOx->DIR & ~(ucPins)));       
    GPIOx->AFSEL = ((ulPinIO & 2) ? (GPIOx->AFSEL | ucPins) :      
                                    (GPIOx->AFSEL &( ~(ucPins)))); 
}






 
u32 GPIODirModeGet(GPIO_Typedef *GPIOx, u8 ucPins)
{
    u32 ulDir, ulAFSEL;

    ;

    ulDir = GPIOx->DIR;
    ulAFSEL = GPIOx->AFSEL;
    return(((ulDir & ucPins) ? 1 : 0) | ((ulAFSEL & ucPins) ? 2 : 0));
}






 
void GPIOIntTypeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulIntType)
{
    ;
    ;



    GPIOx->IBE = ((ulIntType & 1) ?
                                  (GPIOx->IBE | ucPins) :
                                  (GPIOx->IBE & ~(ucPins)));
    GPIOx->IS = ((ulIntType & 2) ?
                                 (GPIOx->IS | ucPins) :
                                 (GPIOx->IS & ~(ucPins)));
    GPIOx->IEV = ((ulIntType & 4) ?
                                  (GPIOx->IEV | ucPins) :
                                  (GPIOx->IEV & ~(ucPins)));
}






 
u32 GPIOIntTypeGet(GPIO_Typedef *GPIOx, u8 ucPins)
{
    u32 ulIBE, ulIS, ulIEV;

    ;

    ulIBE = GPIOx->IBE;
    ulIS = GPIOx->IS;
    ulIEV = GPIOx->IEV;
    return(((ulIBE & ucPins) ? 1 : 0) | ((ulIS & ucPins) ? 2 : 0) |
           ((ulIEV & ucPins) ? 4 : 0));
}







 
void GPIOPadConfigSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulStrength, u32 ulPinType)
{
    ;
    ;



    
#line 132 "LM3S21xx_gpio.c"

    GPIOx->DR2R = ((ulStrength & 1) ?
                                   (GPIOx->DR2R | ucPins) :
                                   (GPIOx->DR2R & ~(ucPins)));
    GPIOx->DR4R = ((ulStrength & 2) ?
                                   (GPIOx->DR4R | ucPins) :
                                   (GPIOx->DR4R & ~(ucPins)));
    GPIOx->DR8R = ((ulStrength & 4) ?
                                   (GPIOx->DR8R | ucPins) :
                                   (GPIOx->DR8R & ~(ucPins)));
    GPIOx->SLR = ((ulStrength & 8) ?
                                  (GPIOx->SLR | ucPins) :
                                  (GPIOx->SLR & ~(ucPins)));

    GPIOx->ODR = ((ulPinType & 1) ?
                                  (GPIOx->ODR | ucPins) :
                                  (GPIOx->ODR & ~(ucPins)));
    GPIOx->PUR = ((ulPinType & 2) ?
                                  (GPIOx->PUR | ucPins) :
                                  (GPIOx->PUR & ~(ucPins)));
    GPIOx->PDR = ((ulPinType & 4) ?
                                  (GPIOx->PDR | ucPins) :
                                  (GPIOx->PDR & ~(ucPins)));
    GPIOx->DEN = ((ulPinType & 8) ?
                                  (GPIOx->DEN | ucPins) :
                                  (GPIOx->DEN & ~(ucPins)));

    
    GPIOx->AMSEL =
        ((ulPinType == 0x00000000) ?
         (GPIOx->AMSEL | ucPins) :
         (GPIOx->AMSEL & ~(ucPins)));
}







 
void GPIOPadConfigGet(GPIO_Typedef *GPIOx, u8 ucPins,u32 *pulStrength, u32 *pulPinType)
{
    u32 ulTemp1, ulTemp2, ulTemp3, ulTemp4;

    ;

    ulTemp1 = GPIOx->DR2R;
    ulTemp2 = GPIOx->DR4R;
    ulTemp3 = GPIOx->DR8R;
    ulTemp4 = GPIOx->SLR;
    *pulStrength = (((ulTemp1 & ucPins) ? 1 : 0) | ((ulTemp2 & ucPins) ? 2 : 0) |
                    ((ulTemp3 & ucPins) ? 4 : 0) | ((ulTemp4 & ucPins) ? 8 : 0));

    ulTemp1 = GPIOx->ODR;
    ulTemp2 = GPIOx->PUR;
    ulTemp3 = GPIOx->PDR;
    ulTemp4 = GPIOx->DEN;
    *pulPinType = (((ulTemp1 & ucPins) ? 1 : 0) | ((ulTemp2 & ucPins) ? 2 : 0) |
                   ((ulTemp3 & ucPins) ? 4 : 0) | ((ulTemp4 & ucPins) ? 8 : 0));
}





 
void GPIOPinIntEnable(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIOx->IM |= ucPins;
}





 
void GPIOPinIntDisable(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIOx->IM &= ~(ucPins);
}






 
u32 GPIOPinIntStatus(GPIO_Typedef *GPIOx, u8 bMasked)
{
    ;
    if(bMasked)
     return(GPIOx->MIS);
    else
     return(GPIOx->RIS);
}





 
void GPIOPinIntClear(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIOx->ICR = ucPins;
}





 
u32 GPIOPinRead(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;

    return(GPIOx->DATA[ucPins]);
}






 
void GPIOPinWrite(GPIO_Typedef *GPIOx, u8 ucPins, u8 ucVal)
{
    ;

    GPIOx->DATA[ucPins] = ucVal;
}






 
void GPIOPinTypeGPIOInput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;

    GPIODirModeSet(GPIOx, ucPins, 0x00000000);

    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}





 
void GPIOPinTypeGPIOOutput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000001);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}





 
void GPIOPinTypeGPIOOutputOD(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000001);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000009);
}





 
void GPIOPinTypeI2C(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x0000000B);
}





 
void GPIOPinTypePWM(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}





 
void GPIOPinTypeQEI(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x0000000A);
}




 
void GPIOPinTypeSSI(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}





 
void GPIOPinTypeTimer(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}




 
void GPIOPinTypeUART(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;
    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}





 
void GPIOPinTypeUSBDigital(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ;

    GPIODirModeSet(GPIOx, ucPins, 0x00000002);
    GPIOPadConfigSet(GPIOx, ucPins, 0x00000001, 0x00000008);
}

