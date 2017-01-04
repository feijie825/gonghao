#line 1 "\274\323\303\334.c"




 
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









#line 7 "\274\323\303\334.c"
#line 1 "LM3S21xx_Lib.h"




 
#line 7 "LM3S21xx_Lib.h"


#line 22 "LM3S21xx_Lib.h"





#line 1 "LM3S21xx_can.h"






 

















 
typedef enum
{
    CAN_INT_STS_CAUSE,	   
    CAN_INT_STS_OBJECT	   
}CANIntSts;




 
typedef enum
{
    CAN_STS_CONTROL,    
    CAN_STS_TXREQUEST,  
    CAN_STS_NEWDAT,     
    CAN_STS_MSGVAL      
}CANSts;





 
typedef enum
{
    CAN_INT_ERROR =   8,  
    CAN_INT_STATUS =  4,  
    CAN_INT_MASTER =  2	  
}
CANIntFlags;




 
typedef enum
{
    MSG_OBJ_TYPE_TX,            
    MSG_OBJ_TYPE_TX_REMOTE,     
    MSG_OBJ_TYPE_RX,            
    MSG_OBJ_TYPE_RX_REMOTE,     
    MSG_OBJ_TYPE_RXTX_REMOTE    
}MsgType;




 
typedef enum
{
    CAN_BUS_OFF  = (1<<7), 
    CAN_EWARN    = (1<<6), 
    CAN_EPASS    = (1<<5), 
    CAN_RXOK     = (1<<4), 
    CAN_TXOK     = (1<<3), 
    CAN_ERR_NONE =   0,    
    CAN_ERR_STUFF=   1,    
    CAN_ERR_FORM =   2,    
    CAN_ERR_ACK  =   3,    
    CAN_ERR_BIT1 =   4,    
    CAN_ERR_BIT0 =   5,    
    CAN_ERR_CRC  =   6,    
    CAN_ERR_MASK =   7     
}
CANStatus;







 
typedef enum
{
    CAN_TX_INT_EN   =  (1<<4),		 
    CAN_RX_INT_EN   =  (1<<5),		 
    CAN_EXD_FRM     =  (1<<6),		 
    CAN_ID_FILT_EN  =  (1<<7),		 
    CAN_DIR_FILT_EN =  ((1<<8)|CAN_ID_FILT_EN), 
    CAN_EXT_FILT_EN =  ((1<<9)|CAN_ID_FILT_EN), 
    CAN_RMT_FRM     =  (1<<10),      
    CAN_NEW_DATA    =  (1<<11),		 
    CAN_DATA_LOST   =  (1<<12),		 
    CAN_NO_FLAGS    =    0			 
}CANFlags;


typedef struct
{
    u32  LEC  :3; 
    u32  TxOK :1; 
    u32  RxOK :1; 
    u32  EPass:1; 
    u32  EWarn:1; 
    u32  Boff :1; 
}CAN_STS_S;

typedef union
{
    u8        BYTE;
    CAN_STS_S BIT;
}CAN_STS_U;

typedef struct
{
    u32	 LEN          :4;     
    u32  TX_INT_EN    :1;     
    u32  RX_INT_EN    :1;     
    u32  EXD_ID       :1;     
    u32  ID_FLT_EN    :1;     
    u32  DIR_FLT_EN   :1;     
    u32  EXT_FLT_EN   :1;     
    u32  RMT_FRM      :1;     
    u32  NEW_DATA     :1;     
    u32  DATA_LOST    :1;     
    u32  RESV         :3;     
    u32  IDx          :5;     
}CTL_S;

typedef union
{
    u32  WORD;
    CTL_S BIT;       
}CTL_U;

typedef struct
{
    u32  IDX   :3  ;
    u32  END   :1  ;
    u32  TYPE  :4  ;
    u32  MNUM  :8  ;
    u32  DIR   :1  ;
    u32  CMD   :11 ;
    u32  EXD   :1  ;
    u32  RESV  :2  ;
}ID_S;

typedef union
{
    u32  WORD;
    ID_S BIT;       
}ID_U;
typedef struct
{
    u32  IDX   :3  ;
    u32  END   :1  ;
    u32  TYPE  :4  ;
    u32  MNUM  :8  ;
    u32  DIR   :1  ;
    u32  CMD   :11 ;
    u32  EXD   :1  ;
    u32  RESV  :3  ;
}MASK_S;
typedef union
{
    u32    WORD;
    MASK_S BIT;
}MASK_U;

typedef struct
{
    u16  DA1;
    u16  DA2;
    u16  DB1;
    u16  DB2;
}DATA_S;

typedef union
{
    DATA_S WORD;        
    u8     BYTE[8];	    
}DATA_U;

typedef struct
{
    CTL_U  CTL;         
    ID_U   ID;          
    MASK_U IDMask;      
}CAN_MSG_SET;

typedef struct
{
    CTL_U  CTL;         
    ID_U   ID;          
    DATA_U Data;        
}CAN_MSG;

typedef struct
{
    u8   *BUF;    
    u16  *HEAD;   
    u16  *TAIL;	  
    u8   *STS;    
}CAN_LMSG_PR;






 
typedef struct
{
    u32 Prescaler: 6;    
    u32 SJW      : 2;    
    u32 TSEG1    : 4;    
    u32 TSEG2    : 3;    
}CANBit_Time;




 
typedef union
{
    u32 WORD;
    CANBit_Time BIT;
}CANBit_Timing;











extern void CANBitTimingGet(CAN_Typedef *CANx, CANBit_Timing *pClkParms);
extern void CANBitTimingSet(CAN_Typedef *CANx, CANBit_Timing *pClkParms);
extern void CANDisable(CAN_Typedef *CANx);
extern void CANEnable(CAN_Typedef *CANx);
extern u8   CANErrCntrGet(CAN_Typedef *CANx, u16 *CanErr_Cnt);
extern void Init_CAN(void);
extern void CANIntClear(CAN_Typedef *CANx, u32 ulIntClr);
extern void CANIntDisable(CAN_Typedef *CANx, u32 ulIntFlags);
extern void CANIntEnable(CAN_Typedef *CANx, u32 ulIntFlags);
extern void CANIntRegister(CAN_Typedef *CANx, void (*pfnHandler)(void));
extern u32 CANIntStatus(CAN_Typedef *CANx,CANIntSts eIntStsReg);
extern void CANIntUnregister(CAN_Typedef *CANx);
extern void CANMessageClear(CAN_Typedef *CANx, u32 ulObjID);
extern void CAN_RxMsg_Set(CAN_Typedef *CANx, CAN_MSG_SET *CAN_MSG, MsgType eMsgType);
extern void CAN_Rx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, u8 ClrPending);
extern void CAN_Tx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, MsgType eMsgType);
extern u8 CANRetryGet(CAN_Typedef *CANx);
extern void CANRetrySet(CAN_Typedef *CANx, u8 bAutoRetry);
extern u32 CANStatusGet(CAN_Typedef *CANx, CANSts eStatusReg);
extern u32 CANIntNumberGet(CAN_Typedef *CANx);
extern void CANRegWrite(u32 ulRegAddress, u32 ulRegValue);
extern u32	CANRegRead(u32 ulIntNumber,u32 ulRegAddress);
extern void Clr_MsgRam(CAN_Typedef *CANx);
extern void Set_MsgRam(CAN_Typedef *CANx);
extern void CAN_ECHO(void);               
extern void CAN_TEST(void);


 
extern void SDATA_MSG_OHead_ADD_ONE(void);


 
void CAN_MSG_TX_CANCEL(CAN_Typedef *CANx, u32 ulObjID);


 
void Proc_CAN_REQ_OvTm(void);  



 
void CAN_LDATA_RX_Pr(CAN_MSG *CAN_RX_MSG);


 
void CAN_LDATA_TX_Pr(void);


 
void Proc_Mst_Check(void);
void Proc_LDATA_MSG_IBUF(void);


 
void Proc_SDATA_IBUF(void);


 
void Proc_SDATA_OBUF(void);


 
void SLV_REQ_RETX(void);


 
void SLV_REQ_TX(void);


 
void Proc_CAN_ERR_STS(void);  



 
extern const CAN_LMSG_PR CAN_LMSG_TX_TAB[];


 
extern const CAN_LMSG_PR CAN_LMSG_RX_TAB[];


 
extern const u32 CANBitClkSettings[];


 
extern const CAN_MSG_SET CAN_MSG_SET_TAB[];





















#line 29 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_flash.h"




 





















typedef enum
{
    FlashReadWrite,                         
    FlashReadOnly,                          
    FlashExecuteOnly                        
}
tFlashProtection;






extern u32 FlashUsecGet(void);
extern void FlashUsecSet(u32 ulClocks);
extern s32 FlashErase(u32 ulAddress);
extern s32 FlashProgram(u32 *pulData, u32 ulAddress,u32 ulCount);
extern tFlashProtection FlashProtectGet(u32 ulAddress);
extern s32 FlashProtectSet(u32 ulAddress,tFlashProtection eProtect);
extern s32 FlashProtectSave(void);
extern s32 FlashUserGet(u32 *pulUser0, u32 *pulUser1);
extern s32 FlashUserSet(u32 ulUser0, u32 ulUser1);
extern s32 FlashUserSave(void);
extern void FlashIntRegister(void (*pfnHandler)(void));
extern void FlashIntUnregister(void);
extern void FlashIntEnable(u32 ulIntFlags);
extern void FlashIntDisable(u32 ulIntFlags);
extern u32 FlashIntGetStatus(u8 bMasked);
extern void FlashIntClear(u32 ulIntFlags);










#line 33 "LM3S21xx_Lib.h"


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


#line 1 "LM3S21xx_interrupt.h"




 























#line 37 "LM3S21xx_interrupt.h"






extern u8 IntMasterEnable(void);
extern u8 IntMasterDisable(void);
extern void IntRegister(u32 ulInterrupt, void (*pfnHandler)(void));
extern void IntUnregister(u32 ulInterrupt);
extern void IntPriorityGroupingSet(u32 ulBits);
extern u32 IntPriorityGroupingGet(void);
extern void IntPrioritySet(u32 ulInterrupt,u8 ucPriority);
extern u32 IntPriorityGet(u32 ulInterrupt);
extern void IntEnable(u32 ulInterrupt);
extern void IntDisable(u32 ulInterrupt);
extern void Init_Int(void);      










#line 41 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_mpu.h"




 



































#line 51 "LM3S21xx_mpu.h"

#line 62 "LM3S21xx_mpu.h"










#line 80 "LM3S21xx_mpu.h"






#line 94 "LM3S21xx_mpu.h"














extern void MPUEnable(u32 ulMPUConfig);
extern void MPUDisable(void);
extern u32  MPURegionCountGet(void);
extern void MPURegionEnable(u32 ulRegion);
extern void MPURegionDisable(u32 ulRegion);
extern void MPURegionSet(u32 ulRegion, u32 ulAddr,u32 ulFlags);
extern void MPURegionGet(u32 ulRegion, u32 *pulAddr,u32 *pulFlags);
extern void MPUIntRegister(void (*pfnHandler)(void));
extern void MPUIntUnregister(void);
extern void Init_Mpu(void);   










#line 45 "LM3S21xx_Lib.h"


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


#line 1 "LM3S21xx_timer.h"




 




















#line 40 "LM3S21xx_timer.h"







#line 54 "LM3S21xx_timer.h"

























extern void TimerEnable(TIMER_Typedef *TIMERx, u32 ulTimer);
extern void TimerDisable(TIMER_Typedef *TIMERx, u32 ulTimer);
extern void TimerConfigure(TIMER_Typedef *TIMERx, u32 ulConfig);
extern void TimerControlLevel(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bInvert);
extern void TimerControlTrigger(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bEnable);
extern void TimerControlEvent(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulEvent);
extern void TimerControlStall(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bStall);
extern void TimerRTCEnable(TIMER_Typedef *TIMERx);
extern void TimerRTCDisable(TIMER_Typedef *TIMERx);
extern void TimerPrescaleSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue);
extern u32 TimerPrescaleGet(TIMER_Typedef *TIMERx,u32 ulTimer);
extern void TimerLoadSet(TIMER_Typedef *TIMERx, u32 ulTimer, u32 ulValue);
extern u32 TimerLoadGet(TIMER_Typedef *TIMERx, u32 ulTimer);
extern u32 TimerValueGet(TIMER_Typedef *TIMERx,u32 ulTimer);
extern void TimerMatchSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue);
extern u32 TimerMatchGet(TIMER_Typedef *TIMERx,u32 ulTimer);
extern void TimerIntRegister(TIMER_Typedef *TIMERx, u32 ulTimer, void (*pfnHandler)(void));
extern void TimerIntUnregister(TIMER_Typedef *TIMERx, u32 ulTimer);
extern void TimerIntEnable(TIMER_Typedef *TIMERx, u32 ulIntFlags);
extern void TimerIntDisable(TIMER_Typedef *TIMERx, u32 ulIntFlags);
extern u32 TimerIntStatus(TIMER_Typedef *TIMERx, u8 bMasked);
extern void TimerIntClear(TIMER_Typedef *TIMERx, u32 ulIntFlags);
extern void Init_Timer(void);      








extern void TimerQuiesce(TIMER_Typedef *TIMERx);











#line 61 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_uart.h"




 





















#line 34 "LM3S21xx_uart.h"










#line 58 "LM3S21xx_uart.h"

















































extern void UARTParityModeSet(UART_Typedef *UARTx, u32 ulParity);
extern u32 UARTParityModeGet(UART_Typedef *UARTx);
extern void UARTFIFOLevelSet(UART_Typedef *UARTx, u32 ulTxLevel,u32 ulRxLevel);
extern void UARTFIFOLevelGet(UART_Typedef *UARTx, u32 *pulTxLevel,u32 *pulRxLevel);
extern void UARTConfigSetExpClk(UART_Typedef *UARTx, u32 ulUARTClk,u32 ulBaud, u32 ulConfig);
extern void UARTConfigGetExpClk(UART_Typedef *UARTx, u32 ulUARTClk,u32 *pulBaud,u32 *pulConfig);
extern void UARTEnable(UART_Typedef *UARTx);
extern void UARTDisable(UART_Typedef *UARTx);
extern void UARTEnableSIR(UART_Typedef *UARTx, u8 bLowPower);
extern void UARTDisableSIR(UART_Typedef *UARTx);
extern u8 UARTCharsAvail(UART_Typedef *UARTx);
extern u8 UARTSpaceAvail(UART_Typedef *UARTx);
extern s32 UARTCharGetNonBlocking(UART_Typedef *UARTx);
extern s32 UARTCharGet(UART_Typedef *UARTx);
extern u8 UARTCharPutNonBlocking(UART_Typedef *UARTx,u8 ucData);
extern void UARTCharPut(UART_Typedef *UARTx, u8 ucData);
extern void UARTBreakCtl(UART_Typedef *UARTx, u8 bBreakState);
extern u8 UARTBusy(UART_Typedef *UARTx);
extern void UARTIntRegister(UART_Typedef *UARTx, void(*pfnHandler)(void));
extern void UARTIntUnregister(UART_Typedef *UARTx);
extern void UARTIntEnable(UART_Typedef *UARTx, u32 ulIntFlags);
extern void UARTIntDisable(UART_Typedef *UARTx, u32 ulIntFlags);
extern u32 UARTIntStatus(UART_Typedef *UARTx, u8 bMasked);
extern void UARTIntClear(UART_Typedef *UARTx, u32 ulIntFlags);
extern void UARTDMAEnable(UART_Typedef *UARTx, u32 ulDMAFlags);
extern void UARTDMADisable(UART_Typedef *UARTx, u32 ulDMAFlags);
extern u32 UARTRxErrorGet(UART_Typedef *UARTx);
extern void UARTRxErrorClear(UART_Typedef *UARTx);
extern void Init_Uart(void);              


 
extern void Init_Com(u8 COM);


 
extern void Proc_Com_OBuf(u8 COM);


 
extern void Proc_Com_IBuf(u8 COM);   


 
extern void Proc_Com_Rx_OvTm(u8 COM);







#line 161 "LM3S21xx_uart.h"
#line 170 "LM3S21xx_uart.h"

extern const u32 UART_PORT[];










#line 65 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_watchdog.h"





 
#line 13 "LM3S21xx_watchdog.h"






extern u8 WatchdogRunning(void);
extern void WatchdogEnable(void);
extern void WatchdogResetEnable(void);
extern void WatchdogResetDisable(void);
extern void WatchdogLock(void);
extern void WatchdogUnlock(void);
extern u8 WatchdogLockState(void);
extern void WatchdogReloadSet(u32 ulLoadVal);
extern u32 WatchdogReloadGet(void);
extern u32 WatchdogValueGet(void);
extern void WatchdogIntRegister(void(*pfnHandler)(void));
extern void WatchdogIntUnregister(void);
extern void WatchdogIntEnable(void);
extern u32 WatchdogIntStatus(u8 bMasked);
extern void WatchdogIntClear(void);
extern void WatchdogStallEnable(void);
extern void WatchdogStallDisable(void);

extern void Init_Wdt(void);	   
extern void WDTFeed(void);	   










#line 69 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_debug.h"




 











#line 26 "LM3S21xx_debug.h"







#line 43 "LM3S21xx_debug.h"

#line 73 "LM3S21xx_Lib.h"


#line 1 "LM3S21xx_cpu.h"




 




















extern unsigned long CPUcpsid(void);
extern unsigned long CPUcpsie(void);
extern void CPUwfi(void);










#line 77 "LM3S21xx_Lib.h"
#line 8 "\274\323\303\334.c"
#line 1 "define.h"







 
#line 10 "define.h"
#line 1 "LM3S21xx_Lib.h"




 
#line 7 "LM3S21xx_Lib.h"


#line 22 "LM3S21xx_Lib.h"





#line 29 "LM3S21xx_Lib.h"


#line 33 "LM3S21xx_Lib.h"


#line 37 "LM3S21xx_Lib.h"


#line 41 "LM3S21xx_Lib.h"


#line 45 "LM3S21xx_Lib.h"


#line 49 "LM3S21xx_Lib.h"


#line 53 "LM3S21xx_Lib.h"


#line 57 "LM3S21xx_Lib.h"


#line 61 "LM3S21xx_Lib.h"


#line 65 "LM3S21xx_Lib.h"


#line 69 "LM3S21xx_Lib.h"


#line 73 "LM3S21xx_Lib.h"


#line 77 "LM3S21xx_Lib.h"
#line 11 "define.h"
#line 12 "define.h"





#line 25 "define.h"





#line 38 "define.h"





































 













 












 
















 
















 
















 















 
















 
















 
















 















 















 















 






































































 

















#line 367 "define.h"






							  




















#line 402 "define.h"

#line 411 "define.h"


#line 421 "define.h"


#line 431 "define.h"












#line 451 "define.h"








                 






















#line 490 "define.h"









#line 505 "define.h"



















                       












#line 543 "define.h"





#line 582 "define.h"

#line 598 "define.h"


#line 613 "define.h"








#line 630 "define.h"
                                               
                                               
#line 641 "define.h"








void Init_Ram(void);          
void Solid_Mtr_Tab(void);	    



  
#line 9 "\274\323\303\334.c"


 
void JtagToGPIO(void)
{
    SysCtlPeripheralEnable(((3<<8)|1 )); 
    GPIOPinTypeGPIOOutput(((GPIO_Typedef*)0x40005000), 0x00000080);    
    SysCtlPeripheralEnable(((3<<8)|2 )); 
    ((GPIO_Typedef*)0x40006000)->LOCK = 0x1ACCE551;                    
    ((GPIO_Typedef*)0x40006000)->CR   = 0x000000FF;                    
    ((GPIO_Typedef*)0x40006000)->LOCK = 0;                             
    ((GPIO_Typedef*)0x40006000)->AFSEL = 0;                            
    GPIOPinTypeGPIOOutput(((GPIO_Typedef*)0x40006000), 0x00000001|     
                                 0x00000002|
                                 0x00000004|
                                 0x00000008);
}



 
void JtagWait(void)
{
    SysCtlPeripheralEnable(((3<<8)|2 )); 
    GPIOPinTypeGPIOInput(((GPIO_Typedef*)0x40006000) , 0x00000010);       
    if (GPIOPinRead(((GPIO_Typedef*)0x40006000) , 0x00000010) )           
     {                                           
      for (;;);                                  
     }                                           
    SysCtlPeripheralDisable(((3<<8)|2 ));
}


 
void JTAG_EN_Pr(void)
{
    SysCtlPeripheralEnable(((3<<8)|1 )); 
    SysCtlPeripheralEnable(((3<<8)|2 )); 
    GPIODirModeSet(((GPIO_Typedef*)0x40005000),                        
                   0x00000080,                         
                   0x00000002);            
    GPIOPadConfigSet(((GPIO_Typedef*)0x40005000),                      
                     0x00000080,                       
                     0x00000001,          
                     0x0000000B);      
    GPIODirModeSet(((GPIO_Typedef*)0x40006000),                        
                   0x00000001|0x00000002|0x00000004|0x00000008,              
                   0x00000002);            
    GPIOPadConfigSet(((GPIO_Typedef*)0x40006000),                      
                     0x00000001|0x00000002|0x00000004|0x00000008,            
                     0x00000001,          
                     0x0000000B);      
} 
