#line 1 "bl_check.c"




























#line 1 "lm3s2139.h"








 

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

#line 14 "lm3s2139.h"




 
#line 28 "lm3s2139.h"



 
#line 80 "lm3s2139.h"































#line 165 "lm3s2139.h"















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
#line 203 "lm3s2139.h"





#line 216 "lm3s2139.h"






#line 234 "lm3s2139.h"






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
#line 273 "lm3s2139.h"

#line 296 "lm3s2139.h"






#line 323 "lm3s2139.h"






#line 350 "lm3s2139.h"






#line 377 "lm3s2139.h"






#line 404 "lm3s2139.h"






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


#line 435 "lm3s2139.h"






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
#line 476 "lm3s2139.h"





#line 496 "lm3s2139.h"






#line 515 "lm3s2139.h"






#line 537 "lm3s2139.h"






#line 559 "lm3s2139.h"






#line 581 "lm3s2139.h"






#line 608 "lm3s2139.h"






#line 635 "lm3s2139.h"






#line 662 "lm3s2139.h"






#line 689 "lm3s2139.h"






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




#line 739 "lm3s2139.h"





#line 762 "lm3s2139.h"






#line 786 "lm3s2139.h"






#line 810 "lm3s2139.h"





 
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
#line 872 "lm3s2139.h"






#line 888 "lm3s2139.h"






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
#line 971 "lm3s2139.h"


 
#line 1011 "lm3s2139.h"





 
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
#line 1067 "lm3s2139.h"





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
#line 1142 "lm3s2139.h"




 
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
#line 1249 "lm3s2139.h"







































































                                            















#line 1354 "lm3s2139.h"
















#line 1376 "lm3s2139.h"
                                            
#line 1393 "lm3s2139.h"









                                            



































                                            


                                            

                                            







                                            

                                            

                                            

                                            







                                            

                                            

                                            

                                            







                                            

                                            






#line 1494 "lm3s2139.h"











































#line 1543 "lm3s2139.h"






#line 1561 "lm3s2139.h"






#line 1574 "lm3s2139.h"






#line 1587 "lm3s2139.h"






#line 1600 "lm3s2139.h"






#line 1613 "lm3s2139.h"














#line 1638 "lm3s2139.h"













































#line 1694 "lm3s2139.h"
























#line 1743 "lm3s2139.h"

































#line 1787 "lm3s2139.h"









































































































#line 1909 "lm3s2139.h"






#line 1922 "lm3s2139.h"






#line 1935 "lm3s2139.h"






#line 1948 "lm3s2139.h"







                                            

                                            









                                            












































                                            












                                            












                                            

                                            

                                            


                                            

                                            

                                            







































#line 2107 "lm3s2139.h"















#line 2128 "lm3s2139.h"
















#line 2151 "lm3s2139.h"



























































































































#line 2318 "lm3s2139.h"
















#line 2354 "lm3s2139.h"
















#line 2378 "lm3s2139.h"







#line 2392 "lm3s2139.h"













#line 2421 "lm3s2139.h"






#line 2459 "lm3s2139.h"














#line 2479 "lm3s2139.h"






#line 2493 "lm3s2139.h"






#line 2515 "lm3s2139.h"














#line 2535 "lm3s2139.h"






#line 2549 "lm3s2139.h"






#line 2571 "lm3s2139.h"














#line 2591 "lm3s2139.h"
































#line 2629 "lm3s2139.h"






#line 2641 "lm3s2139.h"







#line 2656 "lm3s2139.h"



















#line 2691 "lm3s2139.h"








#line 2731 "lm3s2139.h"



























#line 2765 "lm3s2139.h"







#line 2779 "lm3s2139.h"







                                            

                                            

                                            










































#line 2851 "lm3s2139.h"













#line 2882 "lm3s2139.h"













#line 2913 "lm3s2139.h"






#line 2926 "lm3s2139.h"






#line 2946 "lm3s2139.h"



















                                            
#line 2972 "lm3s2139.h"















#line 2996 "lm3s2139.h"

















































                                            
                                            






#line 3062 "lm3s2139.h"











































#line 3116 "lm3s2139.h"









































                                            
                                            






#line 3174 "lm3s2139.h"











































#line 3228 "lm3s2139.h"

















































































#line 3346 "lm3s2139.h"








































































#line 3427 "lm3s2139.h"

















































#line 3486 "lm3s2139.h"










































































































#line 3599 "lm3s2139.h"







                                            
















                                            

                                            







#line 3665 "lm3s2139.h"












































































#line 3749 "lm3s2139.h"














































































#line 3839 "lm3s2139.h"
                                            

                                            
#line 3849 "lm3s2139.h"








                                            
                                            

                                            
                                            



                                            
                                            
                                            
#line 4012 "lm3s2139.h"
                                            
#line 4023 "lm3s2139.h"






#line 4046 "lm3s2139.h"






#line 4059 "lm3s2139.h"
                                            

                                            

                                            
#line 4077 "lm3s2139.h"






#line 4099 "lm3s2139.h"






#line 4136 "lm3s2139.h"






#line 4158 "lm3s2139.h"
















#line 4186 "lm3s2139.h"






#line 4199 "lm3s2139.h"






#line 4272 "lm3s2139.h"






#line 4286 "lm3s2139.h"






#line 4367 "lm3s2139.h"







#line 4445 "lm3s2139.h"




















#line 4472 "lm3s2139.h"






#line 4494 "lm3s2139.h"






#line 4512 "lm3s2139.h"







                                            

                                            


                                            

                                            

                                            

                                            

                                            












                                            

                                            

                                            









                                            

                                            

                                            

                                            

                                            

                                            

                                            







#line 4591 "lm3s2139.h"







                                            

                                            

                                            
#line 4616 "lm3s2139.h"






#line 4634 "lm3s2139.h"






#line 4652 "lm3s2139.h"







                                            

                                            

                                            
#line 4677 "lm3s2139.h"






#line 4695 "lm3s2139.h"






#line 4713 "lm3s2139.h"







                                            

                                            

                                            
#line 4738 "lm3s2139.h"






#line 4756 "lm3s2139.h"











                                            

                                            
#line 4778 "lm3s2139.h"







                                            









#line 4803 "lm3s2139.h"













#line 4830 "lm3s2139.h"








#line 4845 "lm3s2139.h"







#line 4861 "lm3s2139.h"





























































#line 4934 "lm3s2139.h"







#line 4960 "lm3s2139.h"







#line 4976 "lm3s2139.h"







#line 4990 "lm3s2139.h"















#line 5018 "lm3s2139.h"

































































































#line 5147 "lm3s2139.h"






#line 5181 "lm3s2139.h"






#line 5219 "lm3s2139.h"






#line 5253 "lm3s2139.h"






#line 5291 "lm3s2139.h"






#line 5325 "lm3s2139.h"






#line 5363 "lm3s2139.h"






#line 5397 "lm3s2139.h"






#line 5435 "lm3s2139.h"






#line 5469 "lm3s2139.h"






#line 5483 "lm3s2139.h"






#line 5497 "lm3s2139.h"






#line 5511 "lm3s2139.h"






#line 5525 "lm3s2139.h"






#line 5539 "lm3s2139.h"






#line 5553 "lm3s2139.h"






#line 5567 "lm3s2139.h"






#line 5581 "lm3s2139.h"






#line 5595 "lm3s2139.h"






#line 5609 "lm3s2139.h"






#line 5623 "lm3s2139.h"
















#line 5649 "lm3s2139.h"















#line 5679 "lm3s2139.h"















#line 5700 "lm3s2139.h"






#line 5713 "lm3s2139.h"















#line 5735 "lm3s2139.h"







#line 5754 "lm3s2139.h"







#line 5778 "lm3s2139.h"





















































































#line 5915 "lm3s2139.h"






#line 5927 "lm3s2139.h"
                                            
#line 5938 "lm3s2139.h"






#line 5967 "lm3s2139.h"














#line 5992 "lm3s2139.h"









#line 30 "bl_check.c"
#line 1 "bl_check.h"




































extern unsigned long CheckForceUpdate(void);

#line 31 "bl_check.c"
#line 1 "bl_config.h"









































































































































































































































































































































































































#line 32 "bl_check.c"

extern unsigned long BOOT_EN[];
extern unsigned char Board_Id;







extern void Delay(unsigned long ulCount);





















unsigned long CheckForceUpdate(void)
{
    unsigned long *pulApp;


    
    
    
    ((SYSCTL_Typedef*)0x400FE000)->RCGC[2] |= 0x00000004;


	Delay(200);

    ((SYSCTL_Typedef*)0x400FE000)->RCGC[2] |= 0x00000040;
    ((GPIO_Typedef*)0x40026000)->DEN =0xFF;
    Delay(200);
    Board_Id=((GPIO_Typedef*)0x40026000)->DATA[255];   
    Board_Id++;                          
    ((SYSCTL_Typedef*)0x400FE000)->RCGC[2] &= (~0x00000040);
    
    
    
    
    if((BOOT_EN[0]=='E')&&
	   (BOOT_EN[1]=='E'))
     return(1);
    else
     BOOT_EN[2]=Board_Id;                


    pulApp = (unsigned long *)0x00000C00;
    if((pulApp[0] == 0xffffffff) || ((pulApp[0] & 0xfff00000) != 0x20000000) ||
       (pulApp[1] == 0xffffffff) || ((pulApp[1] & 0xfff00001) != 0x00000001))
    {	
        return(1);
    }


    
    
    
    ((GPIO_Typedef*)0x40006000)->DEN |= 1 << 5;
    Delay(20000);
    if(((GPIO_Typedef*)0x40006000)->DATA[(1 << 5)] ==
      (0 << 5))
    {  
        return(1);
    }


    
    
    
    return(0);
}







