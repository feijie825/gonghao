#line 1 "CLK_ERR.C"




 
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









#line 7 "CLK_ERR.C"
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
#line 8 "CLK_ERR.C"
#line 9 "CLK_ERR.C"
#line 1 "math.h"




 





 












 







 










#line 52 "math.h"












 
#line 73 "math.h"











 





extern __softfp unsigned __ARM_dcmp4(double  , double  );
extern __softfp unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __softfp int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __softfp int __ARM_fpclassify(double  );
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
__inline __declspec(__nothrow) __softfp int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
__inline __declspec(__nothrow) __softfp int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
__inline __declspec(__nothrow) __softfp int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

__inline __declspec(__nothrow) __softfp int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
__inline __declspec(__nothrow) __softfp int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 191 "math.h"



   
  typedef float float_t;
  typedef double double_t;
#line 206 "math.h"







extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __pure double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __pure double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  );
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  );
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    
#line 326 "math.h"
    __inline double _sqrt(double __x) { return sqrt(__x); }
    __inline float _sqrtf(float __x) { return (float)sqrt(__x); }


extern __declspec(__nothrow) __pure double ceil(double  );
    
    
extern __declspec(__nothrow) __pure double fabs(double  );
    
    

extern __declspec(__nothrow) __pure double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
__inline __declspec(__nothrow) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
__inline __declspec(__nothrow) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






extern __declspec(__nothrow) double gamma(double  );
    


 
extern __declspec(__nothrow) double gamma_r(double  , int *  );
    


 
extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double j0(double  );
    


 
extern __declspec(__nothrow) double j1(double  );
    


 
extern __declspec(__nothrow) double jn(int  , double  );
    


 
extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double lgamma_r(double  , int *  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) double rint(double  );
    

 

extern __declspec(__nothrow) double scalb(double  , double  );
    


 

extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 
extern __declspec(__nothrow) double significand(double  );
    


 
extern __declspec(__nothrow) double y0(double  );
    


 
extern __declspec(__nothrow) double y1(double  );
    


 
extern __declspec(__nothrow) double yn(int  , double  );
    


 



 
extern __declspec(__nothrow) __pure float _fabsf(float);  
__inline __declspec(__nothrow) __pure float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  );
extern __declspec(__nothrow) __pure float ceilf(float  );
extern __declspec(__nothrow) __pure float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  );

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* );
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  );
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );

#line 839 "math.h"





#line 928 "math.h"







#line 981 "math.h"





#line 1188 "math.h"



 
#line 10 "CLK_ERR.C"
#line 1 "stdio.h"
 
 
 





 






 









#line 34 "stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 115 "stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  );
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  );
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  );
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  );
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  );
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  );
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  );
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...);
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...);
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...);
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...);
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(u8 * __restrict  , const u8 * __restrict  , ...);
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(u8 * __restrict  , const u8 * __restrict  , ...);
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...);
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...);
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...);
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...);
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...);
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...);
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...);
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...);
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list);
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list);
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list);

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list);
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list);
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list);

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  );
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  );
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  );
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  );
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  );
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  );
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  );
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  );
   



 
extern __declspec(__nothrow) int fgetc(FILE *  );
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  );
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  );
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  );
   




 
extern __declspec(__nothrow) int getc(FILE *  );
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  );
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  );
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  );
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  );
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  );
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  );
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  );
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  );
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  );
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  );
   










 
extern __declspec(__nothrow) long int ftell(FILE *  );
   











 
extern __declspec(__nothrow) void rewind(FILE *  );
   





 

extern __declspec(__nothrow) void clearerr(FILE *  );
   




 

extern __declspec(__nothrow) int feof(FILE *  );
   


 
extern __declspec(__nothrow) int ferror(FILE *  );
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   );
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 934 "stdio.h"



 
#line 11 "CLK_ERR.C"
#line 1 "string.h"
 
 
 
 




 








 











#line 37 "string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  );
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  );
   







 
extern __declspec(__nothrow) char *strcpy(void * __restrict  , void * __restrict  );
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , u8 * __restrict  , size_t  );
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  );
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  );
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  );
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  );
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  );
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  );
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  );
   













 







extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  );

   





 






extern __declspec(__nothrow) char *strchr(const char *  , int  );

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  );
   




 






extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  );

   




 






extern __declspec(__nothrow) char *strrchr(const char *  , int  );

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  );
   



 






extern __declspec(__nothrow) char *strstr(const char *  , const char *  );

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  );

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  );

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  );
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  );
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  );
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  );
   






















 







#line 392 "string.h"



 
#line 12 "CLK_ERR.C"
#line 1 "CLK_ERR.h"




 




 
void Proc_Xul_OvTm(void);       


 
void MEA_XUL_TIME(void);        


 
void Proc_Clk_OvTm(void);       


 
void MEA_CLK_FREQ(void);      


 
void Check_Std_Clk(void);      


 
void Cal_Clk_Reload(u8 Sts);   


 
void Cal_Gp_Relaod(void);



 
void Set_MTR_LCT_Com(u8 Com); 


 
void Set_Clk_Freq_Pr(void);           


 
void Set_Clk_Time_Pr(void); 


 
void Set_Clk_Ctl_Pr(void);            


 
void Set_XuL_Ctl_Pr(void);            


 
void Set_XuL_Pls_Num(void);  


 
void Set_Disp_Mode(void);


 
void CLK_FREQ_Pr(void);    



 
void Set_Reload_MtrD(void);



 
void Set_Reload_LctD(void);



 
void Set_ReTx_MtrD(void);



 
void Set_ReTx_LctD(void);



 
void Set_Time_Base(void);


 
void Proc_MFuction_PLS(void);


 
void Proc_Xul_Pls(u16 m);
#line 13 "CLK_ERR.C"
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



  
#line 14 "CLK_ERR.C"
#line 1 "vari.h"




 
#line 7 "vari.h"
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
#line 8 "vari.h"
#line 9 "vari.h"
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



  
#line 10 "vari.h"
typedef struct
{
    vu32 Flag;	
    vu32 Numb;  
}Mtr_Numb;

typedef struct
{
    u32 Data          :10;	 
    u32 New_Data_Flag :1;    
}Adc_Dat;
typedef struct 
{
    u32 Break     :1;        
    u32 Parity_En :1;        
    u32 Even      :1;        
    u32 Stop2     :1;        
    u32 Fifo_En   :1;        
    u32 Data_Len  :2;        
    u32 Parity_01 :1;        
    u32 Baud      :18;       
}UART_PARA;
typedef struct
{
    u32 BAUD     :18;        
    u32 LEN      :4;         
    u32 STOP     :2;         
    u32 PARITY   :3;         
}UART_SET;

typedef enum
{
    CAL_ENG_ERR_M   = 0,       
    MTR_PLUG_M      = 1,       
    CATCH_HB_M      = 2,       
    START_STOP_M    = 3,       
    VERIFY_READY_M  = 4,       
    VERIFY_START_M  = 5,       
    VOLTAGE_DOWN_M  = 6,       
    MEASURE_ENG_M   = 7,       
    PANZHUAN_ERR_M  = 8,       
    MEA_CST_M       = 9,       
    MEA_POWER_D_M   =10,       
    MEA_ENG_DUTY_M  =11,       
    PULSE_ZZ_M      =12,       
    NYSY_M          =13,       
}MODE;

extern u32         WORD0_BIT;  
extern u32         WORD1_BIT;  
extern u32         WORD2_BIT;  
extern u32         WORD3_BIT;  

extern u8          NY_RESULT;                      
extern u8          TIME_ASC[8];                    
extern u8          PLSGD;                          
extern u8          MFClk_Mode;                     
extern u8          MFClk_Type;                     
extern u8          PLSHC_MODE;                     
extern float       Phase_Value;                    
extern u8          Wh_Var;                         
extern u8          Disp_Choose;                    
extern u8          Disp_Code_Mode;                 
extern u16         NEW_ELEC_PLS_CNT;               
extern u16         OLD_ELEC_PLS_CNT;               
extern u8          ENG_ERR_ASC[9];                 
extern u8          CLK_FREQ_ASC[13];               
extern u8          DAY_ERR_ASC[9];                 
extern u8          XUL_TIME_ASC[9];                
extern u8          VERIFY_ENG_ASC[9];              
extern u8          CURRENT_N_ASC[2];               
extern u8          CURRENT_ENG_ASC[9];             
extern u8          CURRENT_PLS_ASC[9];             
extern u8          HIGH_LVL_TIME[9];               
extern u8          LOW_LVL_TIME[9];                
extern u32         PLS_Lvl_Time[2];                
extern u32         High_Lvl_Time_Tp;               
extern u32         Low_Lvl_Time_Tp;                
extern u8          High_Lvl_CNT;                   
extern u8          Low_Lvl_CNT;                    
extern u8          CMD_DATA;                       
extern u8          WIRE_TYPE;                      
extern u16         DIVIDE_Coef;                    
extern u8          CYCLE_MEA_SEL;                  
extern u8          SY_CNT;                         
extern u8          SY_PROCESS;                     
extern u8          SY_MODE;                        
extern u8          SY_PHASE;                       
extern u8          PZ_CNT;                         
extern u8          PZ_STD_CNT;                     
extern u8          PZZS;                           
extern u8          PZBZ;                           
extern u8          PZ_ERR_ASC[3];                  
extern float       VERIFY_ENG_Kwh;                 
extern u32         VERIFY_PLS_Set;                 
extern u32         CURRENT_VERIFY_PLS;             
extern u32         ZZ_PLS_Set;                     
extern u32         CURRENT_ZZ_PLS;                 



extern u32         CURRENT_PLS_CNT;                
extern float       CURRENT_ENG_KWh;                

extern float       GP_ENG_CST;                     
extern u32         GP_CLK_SET;                     
extern u32         GP_CLK_ACT;                     
extern u16         GP_RELOAD_TIME;                 
extern u16         GP_RELOAD_VAL;                  
extern u16         GP_RELOAD_Cnt;                  

extern u32         STD_CLK_Cnt;                    

extern u8          CLK_MEA_CTL;                    
extern u16         CLK_STB_RNG;                    
extern u16         CLK_RELOAD_TIME;                
extern u16         CLK_RELOAD_VAL;                 
extern u16         CLK_RELOAD_Cnt;                 
extern u8          CLK_MEA_TIME;                   
extern float       CLK_FREQ_SET;                   
extern double      CLK_DAY_ERR;                    
extern double      CLK_NEW_ACT_FREQ;               
extern double      CLK_OLD_ACT_FREQ;               
extern double      STD_CLK_VAL_ONE;                
extern double      STD_CLK_VAL_SUM;                
extern u32         NEW_CLK_Cnt;                    
extern u32         OLD_CLK_Cnt;                    
extern u32         CLK_Cnt_OSub;                   
extern u32         CLK_Cnt_NSub;                   

extern u8          XUL_MEA_CTL;                    
extern float       XUL_TIME;                       
extern u8          XUL_RELOAD_TIME;                
extern u8          XUL_RELOAD_Cnt;                 
extern double      XUL_Cnt_Sum;                    
extern u32         NEW_XUL_Cnt;                    
extern u32         XUL_Cnt_OSub;                   
extern u32         XUL_Cnt_NSub;                   

extern u8          ENG_CLK_CH;                     
extern MODE        WORK_MODE;                      
extern u16         ENG_STB_RNG;                    
extern float       STD_ENG_CNT_VAL;                
extern float       STD_ENG_CST;                    
extern float       MTR_ENG_CST;                    
extern float       MTR_MIN_CST;                    
extern u8          CURRENT_N;                      
extern u8          ACT_N;                          
extern u8          SET_N;                          
extern u32         NEW_ENG_Cnt;                    
extern u32         OLD_ENG_Cnt;                    
extern u32         STD_ENG_Cnt;                    
extern u32         ENG_Cnt_OSub;                   
extern u32         ENG_Cnt_NSub;                   
extern float       ENG_ERR;                        
                                                   
extern u8          Mtr_Numb_ID;                    
extern u8          Current_Mtr_Tab;                
extern u8          Mtr_Tab_Sts[128];      
extern u32         Sysclk;                         
extern s32         Timer_1ms;                      
extern s32         CAN_REQ_Timer;                  
extern s32         Disp_Timer;                     
extern u32         CYCLE_OLD_Timer;                
extern u32         CYCLE_NEW_Timer;                
extern u16         SY_Timer;                       
extern u16         CMD_SEND_Timer;                 
extern u32         CLK_Timer;                      
extern u32         CLK_Timer_Max;                  
extern u8          Com_Rx_Time[2];         
extern u8          Com_Rx_OvTm[2];         
extern u8          GDT_RST_Timer;                  
extern u8          UJDQ_Timer;                     
extern u8          IJDQ_Timer;                     
extern u8          ESwitch_Timer;                  
extern u8          NY_CHK_Timer;                   
extern u8          STD_CLK_Timer;                  
extern u8          XUL_Timer;                      
extern u8          SZ_Timer;                       
extern u8          HZ_Timer;                       

extern u8          Disp_Buf[8];                    
extern UART_SET    Uart_Para[2] ;          

extern u16         CANERR_CNT;                     
extern u8          CAN_LMSG_TX_TYPE;               
extern u8          CAN_LMSG_RX_TYPE;               
extern CAN_LMSG_PR CAN_LMSG_RX_Ptr;                
extern CAN_LMSG_PR CAN_LMSG_TX_Ptr;                
extern CAN_STS_U   CAN_STS;                        
extern u8          CAN_LDATA_TX_STS;               
extern u8          CAN_NEXT_MSG_IDx;               
extern u8          CAN_LMSG_TX_STS;                
extern u8          CAN_SMSG_TX_STS;                
extern CAN_MSG     CAN_MSG_Rx;                     
extern CAN_MSG     CAN_SMSG_Tx;                    
extern CAN_MSG     CAN_LMSG_Tx;                    
extern u8          Echo_Sts;                       

extern CAN_MSG     CAN_SDATA_MSG_IBUF[50]; 
extern u8          CAN_SDATA_MSG_IHead;            
extern u8          CAN_SDATA_MSG_ITail;            
extern CAN_MSG     *CAN_MSG_IPtr;                  
extern CAN_MSG     CAN_SDATA_MSG_OBUF[50]; 
extern u8          CAN_SDATA_MSG_OHead;            
extern u8          CAN_SDATA_MSG_OTail;            
extern CAN_MSG     *CAN_MSG_OPtr;                  

extern CAN_MSG     CAN_LDATA_MSG_IBUF[20]; 
extern u8          CAN_LDATA_MSG_IHead;            
extern u8          CAN_LDATA_MSG_ITail;            


extern u8          Com0_IBuf[2048];         
extern u8          Com0_OBuf[1024];         
extern u8          Com1_IBuf[2048];         
extern u8          Com1_OBuf[1024];         

extern u16         Com_IHead[2];           
extern u16         Com_ITail[2];           
extern u8          Com_Rx_Sts[2];          
extern u16         Com_OHead[2];           
extern u16         Com_OTail[2];           
extern u8          Com_Tx_Sts[2];          

extern u8          TEMP_STR[8];                    


extern u32         SINGLE_OR_THREE ; 
extern u32         NEW_ENG_PLS     ; 
extern u32         NEW_CLK_PLS     ; 
extern u32         NEW_XUL_PLS     ; 
extern u32         FIRST_ENG_PLS   ; 
extern u32         FIRST_CLK_PLS   ; 
extern u32         FIRST_XUL_PLS   ; 
extern u32         OVER_ERR_FLAG   ; 
extern u32         NEW_ENG_DATA    ; 
extern u32         NEW_CLK_DATA    ; 
extern u32         NEW_XUL_DATA    ; 
extern u32         BEEP_EN         ; 
extern u32         MTR_PLUG        ; 
extern u32         GDT_RST_FLAG    ; 
extern u32         NEW_CMD         ; 
extern u32         HB_BACK_EDGE    ; 
extern u32         SY_START        ; 
extern u32         ZZ_PLS_LOADED   ; 
extern u32         HB_CATCHED      ; 
extern u32         VERIFY_END      ; 
extern u32         SY_ACT          ; 

extern u32         RISE_FALL_LVL   ; 
extern u32         ENG_STB_CHK     ; 
extern u32         CLK_STB_CHK     ; 
extern u32         XUL_STB_CHK     ; 
extern u32         NO_STD_CLK      ; 
extern u32         FREQ_OR_DAYERR  ; 
extern u32         CLK_SET_ERR     ; 
extern u32         NO_CLK_PLS      ; 
extern u32         NO_XUL_PLS      ; 
extern u32         NO_STD_ENG      ; 
extern u32         XUL_INT_EN      ; 
extern u32         DISP_HL_LVL     ; 
extern u32         PULSE_ZZ_END    ; 
extern u32         NEW_HZ_PLS      ; 
extern u32         NEW_TQ_PLS      ; 
extern u32         TX_ZS_BIT       ; 

extern u32         I_JDQ           ; 
extern u32         I_JDQ_CHG       ; 
extern u32         I_JDQ_EN        ; 
extern u32         UJDQ_FLAG       ; 
extern u32         ESwitch_FLAG    ; 
extern u32         U_JDQ[3]        ; 
extern u32         U_ESwitch[3]    ; 
extern u32         Test_Bit        ; 



 
extern const CAN_MSG CAN_TX_SMSG;

extern u16          TEST_TIMER;
#line 15 "CLK_ERR.C"
#line 1 "disp.h"




 
#line 7 "disp.h"
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
#line 8 "disp.h"
#line 9 "disp.h"
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



  
#line 10 "disp.h"






























#line 70 "disp.h"




#line 81 "disp.h"
                                                
                                                
                                                
                                                
                                                
                                                
                                                
                                                

                                                
                                                
                                                
                                                
                                                
                                                
                                                
                                                

                                                
                                                
                                                
                                                
                                                
                                                
                                                
                                                

                                                

                                                

                                                

                                                



void Reset_HD7279(void);                 
void Disp_Data(u8 Disp_Mode);            
void Disp_Mtr_Num(void);                 



 
void Update_N_Buf(void);


 
void Update_Mtr_Num(void);






 
void Disp_Long_Data(u8 Len,u8 Addr,u32 Data,u8 Sts);


 
void Disp_Time_Pr(void);             





 
void Copy_Str_To_DSBUF(u8 Len,u8 Offset,u8 *Ptr);


 
void Disp_ZZ_Pls(void);                    
                   
#line 16 "CLK_ERR.C"
#line 1 "ENG_ERR.h"




 






                         


                                 







 
void Send_Data(u16 CMD,u8 Len,u8 *Ptr);


 
void I_JDQ_Time_Pr(void);



 
void UJDQ_ESwitch_Time_Pr(void);


 
void Uin_Iin_Pr(void);                            



 
void I_In_Out_Pr(u8 Sts);





 
void UJDQ_Close_ESwitch_Close(void);




 
void UJDQ_Close_ESwitch_Open(void);




 
void UJDQ_Open_ESwitch_Close(void);




 
void UJDQ_Open_ESwitch_Open(void);





 
void UJDQ_Control_Pr(u8 Phase,u8 Sts);



 
void ESwitch_Control_Pr(u8 Phase,u8 Pre,u8 Sts);


 
void ReStart_Mea(void);


 
void Cal_Gp_Relaod(void);


 
void ENG_ERR_MODE_Pr(void);


 
void Cal_STD_ENG_CNT_VAL(void);


 
void Set_STD_CST_Pr(void);         


 
void Set_N_Pr(void);


 
void ERR_CLR_Pr(void);


 
void GDT_RST_Pr(void);          


 
void ELEC_HEAD_RESET_Pr(void);


 
void MTR_PLUG_Pr(void);


 
void CATCH_HB_Pr(void);



 
void Start_Stop_Pr(void);


 
void Set_PLSHC(void);



 
void BEEP_EN_Pr(void);



 
void Set_ZZ_Ds(void);      


 
void Login_Verify_Sts(void);


 
void Verify_Start_Pr(void);


 
void Set_Mtr_Cst_Pr(void);



 
void GDT_PLS_IN_DEN(u8 DEN);       



 
void ELSC_PLS_IN_DEN(u8 DEN);       


 
void ENG_PLS_IN_Pr(void);


 
void CLK_XUL_PLS_IN_Pr(void);



 
void PLS_SEL_Pr(void);



 
void Cnt_ENG_Pr(void);


 
void ENG_CLR_Pr(void);


 
void PZ_ERR_Pr(void);


 
void Set_Pzbz_Pr(void);


 
void Set_Pzzs_Pr(void);  


 
void Set_Sy_Ph(void);


 
void Login_Sy_Sts(void);


 
void Start_Sy_Pr(void);



 
void CYCLE_PLS_SEL_Pr(void); 


 
void DIVIDE_COEF_Pr(void);



 
void MEA_MTR_CST_Pr(void);


 
void Power_Test_Pr(void);


 
void MEA_PLS_CYCLE_Pr(void);


 
void WIRE_TYPE_Pr(void);



 
void ELEC_PLS_TYPE_Pr(void);


 
void Set_ZZ_PLS_Pr(void);


 
void PLS_ZZ_Pr(void);


 
void ReSet_ZZ_PLS_Pr(void);      


 
void Start_Ny_Pr(void);


 
void Ny_Time_End_Pr(void);



 
void Chg_U_In_Pr(void);


 
void STS_Light_Pr(void);


 
void Set_Sy_Ph(void);                 


 
void Set_Min_CST_Pr(void);            


 
void Set_ZZ_Ds(void);                      


 
void Set_Phase(void);                 


 
void Set_WhVar(void);                 


 
void Set_Sy(void);                    




 
void Set_MFCLK_Mode(void);






 
void Set_MFCLK_Type(void);


 
void Relay_Close_Pr(void);


 
void Relay_Open_Pr(void);           


 
void I_Open_Tst_Pr(void); 


 
void Set_Min_CST_Pr(void);


 
void MTR_PLUG_MODE_Pr(void);     


 
void CATCH_HB_MODE_Pr(void);     


 
void START_STOP_MODE_Pr(void);    


 
void ZZ_READY_MODE_Pr(void);     


 
void ZZ_START_MODE_Pr(void);    


 
void VOLTAGE_DOWN_MODE_Pr(void); 


 
void MEASURE_ENG_MODE_Pr(void);        


 
void PANZHUAN_ERR_MODE_Pr(void);       


 
void MEA_CST_MODE_Pr(void);            


 
void MEA_POWER_D_MODE_Pr(void);        


 
void MEA_ENG_DUTY_MODE_Pr(void);       


 
void PULSE_ZZ_MODE_Pr(void);           


 
void NYSY_MODE_Pr(void);               


 
void Work_Mode_Pr(void); 
#line 17 "CLK_ERR.C"




 
void Cal_Clk_Reload(u8 Sts)   
{
    u32 m;
    float f;
    f=CLK_FREQ_SET;                      
    f*=CLK_MEA_TIME;                     
    m=(u32)(f+0.5);                      
    if(m==0)                             
     m=1;                                
    if(CLK_FREQ_SET<1)                   
     {
      CLK_RELOAD_VAL=1;                  
      CLK_RELOAD_TIME=m;                 
     }
    else if(CLK_FREQ_SET<60000)          
     {                                   
      CLK_RELOAD_VAL=(u16)(CLK_FREQ_SET+0.5);
      CLK_RELOAD_TIME=CLK_MEA_TIME;      
     } 
    else                                 
     {                                   
      CLK_RELOAD_VAL=60000;              
      CLK_RELOAD_TIME=(m/60000);         
     }                                   
    CLK_RELOAD_Cnt=CLK_RELOAD_TIME;      
    STD_CLK_VAL_ONE=1000000;        
    STD_CLK_VAL_ONE*=CLK_RELOAD_VAL;     
    STD_CLK_VAL_SUM=STD_CLK_VAL_ONE*CLK_RELOAD_TIME;
    f=STD_CLK_VAL_ONE/CLK_FREQ_SET;      
    CLK_STB_RNG=(f/1000+0.5);            
    if(CLK_STB_RNG<50)
     CLK_STB_RNG=50;                     

    f=CLK_RELOAD_VAL;                    
    f*=1000;                             
    f/=CLK_FREQ_SET;                     
    if(f<20)                             
     f=20;
    CLK_Timer_Max=f+500;                 
    FIRST_CLK_PLS=1;                     
    CLK_Timer=0;                         
    if(Sts) 
     {
      TimerDisable(((TIMER_Typedef*)0x40031000),0x00000001);        
      TimerLoadSet(((TIMER_Typedef*)0x40031000),0x000000ff,CLK_RELOAD_VAL);
      TimerEnable(((TIMER_Typedef*)0x40031000),0x00000001);         
     }
}



 
void Set_MTR_LCT_Com(u8 Com) 
{
    UART_SET Uart_Set;
    if(Com>1)                        
     return;
    memcpy(&Uart_Set,
           &CAN_MSG_IPtr->Data.BYTE,
           4);
    if(Uart_Set.STOP>1)                   
     Uart_Set.STOP=1;
    else
     Uart_Set.STOP=0;
    if(Uart_Set.LEN>3)          
     Uart_Set.LEN=3;            
    if(Uart_Set.PARITY>4)
     Uart_Set.PARITY=2;      
    if((Uart_Set.BAUD<110)||
       (Uart_Set.BAUD>115200))
     Uart_Set.BAUD=1200;                 
    memcpy(&Uart_Para[Com],
           &Uart_Set,
           4);
    Init_Com(Com);
}


 
void Set_Clk_Freq_Pr(void)
{
    float f;
    memcpy(&f,
           &CAN_MSG_IPtr->Data.BYTE,
           4);
    if((f==0)||                                
       (f==CLK_FREQ_SET))                      
     return;
    CLK_FREQ_SET=f;                            
    Cal_Clk_Reload(1);                         
}           


 
void Set_Clk_Time_Pr(void) 
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              
    if(m<6)                                    
     m=6;                                                                         
    else if(m>250)                             
     m=250;
    if(m==CLK_MEA_TIME)                        
     return;	     
    Cal_Clk_Reload(1);                         
}           





 
void Set_Clk_Ctl_Pr(void)            
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              
    if(m==CLK_MEA_CTL)                         
     return;                                   
    if((m== '0')||                         
       (m== '1')||                         
       (m== '2'))                         
     {	 
      CLK_MEA_CTL=m;                           
      if(CLK_MEA_CTL==0)                       
       return;
      if(MFClk_Mode== '1')                
       if(MFClk_Type!= '1')               
        return;                                
      FIRST_CLK_PLS=1;                         
     } 
}           





 
void Set_XuL_Ctl_Pr(void)            
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              
    if(m==XUL_MEA_CTL)                         
     return;                                   
    if((m== '0')||                         
       (m== '1')||                         
       (m== '2'))                         
     {	 
      XUL_MEA_CTL=m;                             
      if(XUL_MEA_CTL==0)                         
       return;
      if(MFClk_Mode== '1')                  
       if(MFClk_Type!= '2')             	 
        return;                                  
      FIRST_XUL_PLS=1;                           
     } 
}           


 
void Set_XuL_Pls_Num(void)   
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              
    if(m==0)                                   
     m=1;	                                     
    else if(m>100)                             
     m=100;	 
    if(m==XUL_RELOAD_TIME)                     
     return;	
    XUL_RELOAD_TIME=m;                         
    if(MFClk_Mode== '1')                  
     if(MFClk_Type!= '2')             	 
      return;                                  
    FIRST_XUL_PLS=1;                           
}  







 
void Set_Disp_Mode(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              
    if((m>3)||                                 
       (m==Disp_Choose))                       
     return;
    Disp_Choose=m;                             
}    



 
void Set_Reload_MtrD(void)
{
    if(Com_Tx_Sts[0]!=0)     
     return;
    if(Com_OHead[0]!=0)              
     {
      Com_OTail[0]=0;
      Com_Tx_Sts[0]= 'E';       
     }	 
}



 
void Set_Reload_LctD(void)
{
    if(Com_Tx_Sts[1]!=0)     
     return;
    if(Com_OHead[1]!=0)              
     {
      Com_OTail[1]=0;
      Com_Tx_Sts[1]= 'E';       
     }	 
}



 
void Set_ReTx_MtrD(void)
{
    if(Com_Rx_Sts[0]!=0)    
     return;                             
    if(Com_IHead[0]!=0)             
     {	
      Com_ITail[0]=0;               
      Com_Rx_Sts[0]= 'E';     
     } 
}



 
void Set_ReTx_LctD(void)
{
    if(Com_Rx_Sts[1]!=0)    
     return;                             
    if(Com_IHead[1]!=0)             
     {	
      Com_ITail[1]=0;               
      Com_Rx_Sts[1]= 'E';     
     } 
}



 
void Set_Time_Base(void)
{
    u8 m;
    for(m=0;m<8;m++)
     TIME_ASC[m]=CAN_MSG_IPtr->Data.BYTE[m]; 
}


 
void Check_Std_Clk(void)      
{
    if(STD_CLK_Timer<50)
     return;
    NO_STD_CLK=1;                           
}  


 
void Act_Freq_to_Set(void)
{
    float f=1.0;
    for(;;)
     {
      if(CLK_NEW_ACT_FREQ>=10.0)
       {
        CLK_NEW_ACT_FREQ/=10.0;            
        f*=10.0;
       }
      else if(CLK_NEW_ACT_FREQ<1.0)
       {
        CLK_NEW_ACT_FREQ*=10.0;
        f/=10.0;
       } 
      else
       break; 
     }
    CLK_FREQ_SET=(u32)(CLK_NEW_ACT_FREQ*10000+0.5);
    CLK_FREQ_SET/=10000;                 
    CLK_FREQ_SET*=f;                     
    CLK_NEW_ACT_FREQ*=f;                 
}            


 
void Proc_Clk_OvTm(void)       
{
    if(NO_CLK_PLS)                          
     return;                                
    if(CLK_Timer>=CLK_Timer_Max)            
     {
      u16 m;
      CLK_Timer=0;                          
      m=(u16)((TIMER_Typedef*)0x40031000)->TAR;                   
      m=CLK_RELOAD_VAL-m;                   
      CLK_NEW_ACT_FREQ=1000;                
      CLK_NEW_ACT_FREQ/=CLK_Timer_Max;      
      if(m==0)                              
       CLK_NEW_ACT_FREQ/=10;                
      else
       CLK_NEW_ACT_FREQ*=m;                 
      if(CLK_NEW_ACT_FREQ<0.0001)           
       {
        NO_CLK_PLS=1;                       
        CLK_FREQ_SET=1.0;                   
       }
      else
       {
        Act_Freq_to_Set();                  
        Cal_Clk_Reload(1);                  
       } 
     } 
}   






 
void MEA_CLK_FREQ(void)      
{
    if(WORK_MODE==MEA_ENG_DUTY_M)           
     return;                                
    if(MFClk_Mode== '1')               
     if(MFClk_Type!= '1')              
      return;                               
    if(NEW_CLK_PLS)                         
     {
      u32   Cnt;     
      float f;
      NEW_CLK_PLS=0;                        
      CLK_RELOAD_Cnt--;
      if(CLK_RELOAD_Cnt==0)                 
       {  
        Cnt= NEW_CLK_Cnt-OLD_CLK_Cnt;       
        OLD_CLK_Cnt=NEW_CLK_Cnt;            
        CLK_RELOAD_Cnt=CLK_RELOAD_TIME;     
        if(Cnt<10)                          
         {
          NO_STD_CLK=1;                     
          strcpy(TEMP_STR,
                 "No JZMC");
          Send_Data((1*0x100+4),       
                    7,                      
                    TEMP_STR);              
          Send_Data((1*0x100+4)+1,       
                    7,                      
                    TEMP_STR);              
          return;
         } 
        else
         {	 
          NO_STD_CLK=0;                       
          CLK_NEW_ACT_FREQ=STD_CLK_VAL_SUM;   
          CLK_NEW_ACT_FREQ/=Cnt;              
          memset(CLK_FREQ_ASC,0,12);          
          sprintf(CLK_FREQ_ASC,               
                      "%12.10f",              
                      CLK_NEW_ACT_FREQ);      
          CLK_DAY_ERR= CLK_NEW_ACT_FREQ;      
          CLK_DAY_ERR/=CLK_FREQ_SET;          
          CLK_DAY_ERR-=1;                     
          CLK_DAY_ERR*=86400;                 
          sprintf(DAY_ERR_ASC,                
                      "%8.6f",                
                      CLK_DAY_ERR);           
          NEW_CLK_DATA=1;                     
          if(Disp_Choose==1)      
           {
            Disp_Buf[0]=0x0F;                
            Copy_Str_To_DSBUF(8,   
                              1,
                              CLK_FREQ_ASC);   
           
           }
          else if(Disp_Choose==2)  
           {  
            Disp_Buf[0]=0x17;               
            Disp_Buf[1]=0x15;           
            Copy_Str_To_DSBUF(7,   
                              2,
                              DAY_ERR_ASC);   
           }
         }
        if(CLK_FREQ_ASC[7]=='.')              
         Send_Data((1*0x100+4),          
                   7,                         
                   CLK_FREQ_ASC);             
        else        	                        
         Send_Data((1*0x100+4),          
                   8,                         
                   CLK_FREQ_ASC);             
        if(DAY_ERR_ASC[7]=='.')               
         Send_Data((1*0x100+4)+1,          
                   7,                         
                   DAY_ERR_ASC);              
        else        	                        
         Send_Data((1*0x100+4)+1,          
                   8,
                   DAY_ERR_ASC);
       }
      else
       {
       	if(NO_STD_CLK)                       
       	 return;	
        if(CLK_Cnt_NSub>CLK_Cnt_OSub)
         Cnt=CLK_Cnt_NSub-CLK_Cnt_OSub;      
        else
         Cnt=CLK_Cnt_OSub-CLK_Cnt_NSub;      
        CLK_Cnt_OSub=CLK_Cnt_NSub;
        if(CLK_STB_CHK)                      
         {
          if(CLK_RELOAD_TIME!=CLK_RELOAD_Cnt)
           CLK_STB_CHK=0;                    
          return;
         }             
        if(Cnt>CLK_STB_RNG)                  
         {
          FIRST_CLK_PLS=1;                   
          return;
         }
        CLK_NEW_ACT_FREQ=STD_CLK_VAL_ONE;    
        if(CLK_Cnt_NSub<100)                 
         {
          if(CLK_FREQ_SET<1.1)
           {
            CLK_FREQ_SET=100;                
            Cal_Clk_Reload(1);
           }
          else if(CLK_FREQ_SET>1000)         
           {
            NO_STD_CLK=1;                    
           }  
          else
           {
            CLK_FREQ_SET*=100;               
            Cal_Clk_Reload(1);               
           } 
          return; 
         }                             
        CLK_NEW_ACT_FREQ/=CLK_Cnt_NSub;      
       }
      if(CLK_OLD_ACT_FREQ==0)
       { 
        CLK_OLD_ACT_FREQ=CLK_NEW_ACT_FREQ;   
        return;
       } 
      if(CLK_NEW_ACT_FREQ>CLK_OLD_ACT_FREQ)
       f=CLK_NEW_ACT_FREQ/CLK_OLD_ACT_FREQ;  
      else
       f=CLK_OLD_ACT_FREQ/CLK_NEW_ACT_FREQ;
      CLK_OLD_ACT_FREQ=CLK_NEW_ACT_FREQ; 
      if(f>1.001)                            
       return;                               
      if(CLK_NEW_ACT_FREQ>CLK_FREQ_SET)      
       f=CLK_NEW_ACT_FREQ/CLK_FREQ_SET;      
      else                                  
       f=CLK_FREQ_SET/CLK_NEW_ACT_FREQ;      
      if(f>1.0005)                           
       {                                     
        if(!CLK_SET_ERR)                     
         {                                   
          CLK_SET_ERR=1;                     
          return;                            
         }  
        Act_Freq_to_Set();                   
        Cal_Clk_Reload(1);                   
       }                                     
      else                                   
       CLK_SET_ERR=0;                        
     }
}


 










       


 
void MEA_XUL_TIME(void)        
{
    if(WORK_MODE==MEA_ENG_DUTY_M)           
     return;                                
    if(MFClk_Mode== '1')               
     if(MFClk_Type!= '2')             
      return;                               
    if(NEW_XUL_PLS)                         
     {
      NEW_XUL_PLS=0;                        
      if(NO_STD_CLK)                        
       return;                              
      XUL_RELOAD_Cnt--;
      if(XUL_RELOAD_Cnt==0)                 
       {  
        XUL_RELOAD_Cnt=XUL_RELOAD_TIME;     
        if(XUL_Cnt_Sum<10)                  
         {
          NO_STD_CLK=1;                     
          return;                           
         } 
        XUL_Cnt_Sum/=1000000;               
        XUL_TIME = XUL_Cnt_Sum;             
        XUL_Cnt_Sum=0;                      
        memset(XUL_TIME_ASC,0,8);           
        sprintf(XUL_TIME_ASC,               
                "%8.6f",                    
                XUL_TIME);                  
        NEW_XUL_DATA=1;                     
        if(Disp_Choose==3)      
         {
          Disp_Buf[0]=0x12;               
          Disp_Buf[1]=0x15;           
          Copy_Str_To_DSBUF(7,   
                            2,
                            XUL_TIME_ASC);  
         }
        if(XUL_TIME_ASC[7]=='.')                
         Send_Data((1*0x100+4)+1+1,          
                   7,                         
                   XUL_TIME_ASC);               
        else        	                        
         Send_Data((1*0x100+4)+1+1,          
                   8,                         
                   XUL_TIME_ASC);             
       }
      else
       {
        u32 Cnt;
        if(XUL_Cnt_NSub>XUL_Cnt_OSub)
         Cnt=XUL_Cnt_NSub-XUL_Cnt_OSub;      
        else
         Cnt=XUL_Cnt_OSub-XUL_Cnt_NSub;      
        XUL_Cnt_OSub=XUL_Cnt_NSub;
        if(XUL_STB_CHK)                      
         {
          if(XUL_RELOAD_TIME!=XUL_RELOAD_Cnt)
           XUL_STB_CHK=0;                    
          return;
         }             
        if(Cnt>1000)                         
         {
          FIRST_XUL_PLS=1;                   
          return;
         }
       }  
     }
}


 
void Proc_MFuction_PLS(void)
{
    if(NEW_HZ_PLS)                           
     {
      NEW_HZ_PLS=0;                          
      strcpy(TEMP_STR,
             "HZMC");
      Send_Data((1*0x100+4)+1+1+1+1,               
                4,                           
                TEMP_STR);                   
     }
    if(NEW_TQ_PLS)                           
     {
      NEW_TQ_PLS=0;                          
      strcpy(TEMP_STR,
             "TQMC");
      Send_Data((1*0x100+4)+1+1+1,               
                4,                           
                TEMP_STR);                   
     }	 		
}


 
void Proc_Xul_Pls(u16 m)
{
    u32 Cnt;
    Cnt=STD_CLK_Cnt;                    
    Cnt=(Cnt<<15)+(0x8000-m);           
    XUL_Timer=0;                        
    NO_XUL_PLS=0;                       
    if(FIRST_XUL_PLS)                   
     {                                  
      FIRST_XUL_PLS=0;                  
      NEW_XUL_PLS=0;                    
      NEW_XUL_Cnt=Cnt;                  
      XUL_Cnt_Sum=0;                    
      XUL_STB_CHK=1;                    
      XUL_RELOAD_Cnt=XUL_RELOAD_TIME;   
      return;                           
     }                                  
    XUL_Cnt_NSub=Cnt-NEW_XUL_Cnt;       
    NEW_XUL_Cnt=Cnt;                    
    XUL_Cnt_Sum+=XUL_Cnt_NSub;          
    NEW_XUL_PLS=1;                      
}
