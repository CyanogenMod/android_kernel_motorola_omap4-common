#ifndef __PPA_CALL_H__
#define __PPA_CALL_H__


#define  API_HAL_RET_VALUE_OK                                       0x00000000
#define  API_HAL_RET_VALUE_FAIL                                     0x00000001


/* ---------------------------------  Types --------------------------------- */
typedef unsigned char       U8;
typedef unsigned short      U16;
typedef unsigned int        U32;
typedef unsigned long long  U64;

/*
 *SE entry flags
 */
#define FLAG_START_HAL_CRITICAL     0x4
#define FLAG_IRQFIQ_MASK            0x3
#define FLAG_IRQ_ENABLE             0x2
#define FLAG_FIQ_ENABLE             0x1

#ifndef NULL
#define NULL (void *)0
#endif

U32 SEC_ENTRY_pub2sec_dispatcher(U32 appl_id, U32 nArgs, U32 arg1, U32 arg2, U32 arg3, U32 arg4);

#endif  /* __PPA_CALL_H__ */
