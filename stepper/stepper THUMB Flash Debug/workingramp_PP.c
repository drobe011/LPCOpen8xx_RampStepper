# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/../stepper/workingramp.c"
# 1 "<command-line>"
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/../stepper/workingramp.c"
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/../stepper/workingramp_defs.h" 1



# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 1
# 35 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h"
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h" 1
# 35 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h"
# 1 "/home/drob/Programs/crossworks_for_arm_3.0/include/stdint.h" 1 3 4
# 14 "/home/drob/Programs/crossworks_for_arm_3.0/include/stdint.h" 3 4
typedef signed char int8_t;
typedef unsigned char uint8_t;




typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
# 35 "/home/drob/Programs/crossworks_for_arm_3.0/include/stdint.h" 3 4
typedef signed long long int64_t;
typedef unsigned long long uint64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;
typedef int32_t int_least32_t;
typedef int64_t int_least64_t;

typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;
typedef uint32_t uint_least32_t;
typedef uint64_t uint_least64_t;



typedef int32_t int_fast8_t;
typedef int32_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef int64_t int_fast64_t;

typedef uint32_t uint_fast8_t;
typedef uint32_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
typedef uint64_t uint_fast64_t;

typedef int32_t intptr_t;
typedef uint32_t uintptr_t;
# 85 "/home/drob/Programs/crossworks_for_arm_3.0/include/stdint.h" 3 4
typedef int64_t intmax_t;
typedef uint64_t uintmax_t;
# 36 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h" 2
# 1 "/home/drob/Programs/crossworks_for_arm_3.0/include/stdbool.h" 1 3 4
# 37 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h" 2
# 50 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h"
typedef enum {FALSE = 0, TRUE = !FALSE} Bool;
# 62 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h"
typedef enum {RESET = 0, SET = !RESET} FlagStatus, IntStatus, SetState;





typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;





typedef enum {ERROR = 0, SUCCESS = !ERROR} Status;




typedef enum {
 NONE_BLOCKING = 0,
 BLOCKING,
} TRANSFER_BLOCK_T;


typedef void (*PFV)();


typedef int32_t (*PFI)();
# 161 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/lpc_types.h"
typedef char CHAR;


typedef uint8_t UNS_8;


typedef int8_t INT_8;


typedef uint16_t UNS_16;


typedef int16_t INT_16;


typedef uint32_t UNS_32;


typedef int32_t INT_32;


typedef int64_t INT_64;


typedef uint64_t UNS_64;







typedef _Bool BOOL_32;


typedef _Bool BOOL_16;


typedef _Bool BOOL_8;
# 36 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sys_config.h" 1
# 37 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/cmsis.h" 1
# 88 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/cmsis.h"
typedef enum {

 Reset_IRQn = -15,
 NonMaskableInt_IRQn = -14,
 HardFault_IRQn = -13,
 SVCall_IRQn = -5,
 PendSV_IRQn = -2,
 SysTick_IRQn = -1,


 SPI0_IRQn = 0,
 SPI1_IRQn = 1,
 Reserved0_IRQn = 2,
 UART0_IRQn = 3,
 UART1_IRQn = 4,
 UART2_IRQn = 5,
 Reserved1_IRQn = 6,
 Reserved2_IRQn = 7,
 I2C_IRQn = 8,
 SCT_IRQn = 9,
 MRT_IRQn = 10,
 CMP_IRQn = 11,
 WDT_IRQn = 12,
 BOD_IRQn = 13,
 FLASH_IRQn = 14,
 WKT_IRQn = 15,
 Reserved4_IRQn = 16,
 Reserved5_IRQn = 17,
 Reserved6_IRQn = 18,
 Reserved7_IRQn = 19,
 Reserved8_IRQn = 20,
 Reserved9_IRQn = 21,
 Reserved10_IRQn = 22,
 Reserved11_IRQn = 23,
 PININT0_IRQn = 24,
 PININT1_IRQn = 25,
 PININT2_IRQn = 26,
 PININT3_IRQn = 27,
 PININT4_IRQn = 28,
 PININT5_IRQn = 29,
 PININT6_IRQn = 30,
 PININT7_IRQn = 31,
} IRQn_Type;





# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h" 1
# 112 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
# 1 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h" 1
# 325 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __asm volatile ("nop");
}







__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}







__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
  __asm volatile ("wfe");
}






__attribute__( ( always_inline ) ) static inline void __SEV(void)
{
  __asm volatile ("sev");
}
# 369 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __ISB(void)
{
  __asm volatile ("isb");
}







__attribute__( ( always_inline ) ) static inline void __DSB(void)
{
  __asm volatile ("dsb");
}







__attribute__( ( always_inline ) ) static inline void __DMB(void)
{
  __asm volatile ("dmb");
}
# 404 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV(uint32_t value)
{

  return __builtin_bswap32(value);






}
# 424 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=l" (result) : "l" (value) );
  return(result);
}
# 440 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline int32_t __REVSH(int32_t value)
{

  return (short)__builtin_bswap16(value);






}
# 461 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  return (op1 >> op2) | (op1 << (32 - op2));
}
# 113 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h" 2
# 1 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h" 1
# 329 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__( ( always_inline ) ) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}
# 352 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 367 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 379 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 394 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 409 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 424 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp\n" : "=r" (result) );
  return(result);
}
# 439 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0\n" : : "r" (topOfProcStack) : "sp");
}
# 451 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp\n" : "=r" (result) );
  return(result);
}
# 466 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack) : "sp");
}
# 478 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 493 "/home/drob/.rowley_associates_limited/CrossWorks for ARM/v3/packages/targets/CMSIS_3/CMSIS/Include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 114 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h" 2
# 191 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
typedef union
{
  struct
  {

    uint32_t _reserved0:27;





    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;

    uint32_t _reserved0:15;





    uint32_t T:1;
    uint32_t IT:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;




typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 276 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
typedef struct
{
  volatile uint32_t ISER[1];
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];
} NVIC_Type;
# 301 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;



       uint32_t RESERVED0;

  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];
  volatile uint32_t SHCSR;
} SCB_Type;
# 416 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 611 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 623 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 639 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 651 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 663 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 678 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}
# 700 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2))); }
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2))); }
}






static inline void NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FA << 16) |
                 (1UL << 2));
  __DSB();
  while(1);
}
# 752 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/core_cm0plus.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0)) return (1);

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (ticks & (0xFFFFFFUL << 0)) - 1;
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) |
                   (1UL << 1) |
                   (1UL << 0);
  return (0);
}
# 137 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/cmsis.h" 2
# 38 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 123 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h"
extern const uint32_t OscRateIn;







extern const uint32_t ExtRateIn;






# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h" 1
# 35 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h"
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/error_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/error_8xx.h"
typedef enum
{
                     LPC_OK = 0,
                     LPC_ERROR,


  ERR_ISP_BASE = 0x00000000,
                     ERR_ISP_INVALID_COMMAND = ERR_ISP_BASE + 1,
                     ERR_ISP_SRC_ADDR_ERROR,
                     ERR_ISP_DST_ADDR_ERROR,
                     ERR_ISP_SRC_ADDR_NOT_MAPPED,
                     ERR_ISP_DST_ADDR_NOT_MAPPED,
                     ERR_ISP_COUNT_ERROR,
                     ERR_ISP_INVALID_SECTOR,
                     ERR_ISP_SECTOR_NOT_BLANK,
                     ERR_ISP_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
                     ERR_ISP_COMPARE_ERROR,
                     ERR_ISP_BUSY,
                     ERR_ISP_PARAM_ERROR,
                     ERR_ISP_ADDR_ERROR,
                     ERR_ISP_ADDR_NOT_MAPPED,
                     ERR_ISP_CMD_LOCKED,
                     ERR_ISP_INVALID_CODE,
                     ERR_ISP_INVALID_BAUD_RATE,
                     ERR_ISP_INVALID_STOP_BIT,
                     ERR_ISP_CODE_READ_PROTECTION_ENABLED,


  ERR_I2C_BASE = 0x00060000,
                     ERR_I2C_NAK = ERR_I2C_BASE + 1,
                     ERR_I2C_BUFFER_OVERFLOW,
                     ERR_I2C_BYTE_COUNT_ERR,
                     ERR_I2C_LOSS_OF_ARBRITRATION,
                     ERR_I2C_SLAVE_NOT_ADDRESSED,
                     ERR_I2C_LOSS_OF_ARBRITRATION_NAK_BIT,
                     ERR_I2C_GENERAL_FAILURE,
                     ERR_I2C_REGS_SET_TO_DEFAULT,
                     ERR_I2C_TIMEOUT,


                     ERR_NO_ERROR = LPC_OK,
  ERR_UART_BASE = 0x00080000,
                     ERR_UART_RXD_BUSY = ERR_UART_BASE + 1,
                     ERR_UART_TXD_BUSY,
                     ERR_UART_OVERRUN_FRAME_PARITY_NOISE,
                     ERR_UART_UNDERRUN,
                     ERR_UART_PARAM,
} ErrorCode_t;
# 36 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_i2c_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_i2c_8xx.h"
typedef void *I2C_HANDLE_T;




typedef void (*I2C_CALLBK_T)(uint32_t err_code, uint32_t n);




typedef struct I2C_PARAM {
 uint32_t num_bytes_send;
 uint32_t num_bytes_rec;
 uint8_t *buffer_ptr_send;
 uint8_t *buffer_ptr_rec;
 I2C_CALLBK_T func_pt;
 uint8_t stop_flag;
 uint8_t dummy[3];
} I2C_PARAM_T;




typedef struct I2C_RESULT {
 uint32_t n_bytes_sent;
 uint32_t n_bytes_recd;
} I2C_RESULT_T;




typedef enum CHIP_I2C_MODE {
 IDLE,
 MASTER_SEND,
 MASTER_RECEIVE,
 SLAVE_SEND,
 SLAVE_RECEIVE
} CHIP_I2C_MODE_T;




typedef struct I2CD_API {

 void (*i2c_isr_handler)(I2C_HANDLE_T *handle);


 ErrorCode_t (*i2c_master_transmit_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_master_receive_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_master_tx_rx_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_master_transmit_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_master_receive_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_master_tx_rx_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);


 ErrorCode_t (*i2c_slave_receive_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_slave_transmit_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_slave_receive_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_slave_transmit_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
 ErrorCode_t (*i2c_set_slave_addr)(I2C_HANDLE_T *handle, uint32_t slave_addr_0_3, uint32_t slave_mask_0_3);


 uint32_t (*i2c_get_mem_size)(void);
 I2C_HANDLE_T * (*i2c_setup)( uint32_t i2c_base_addr, uint32_t * start_of_ram);
 ErrorCode_t (*i2c_set_bitrate)(I2C_HANDLE_T *handle, uint32_t p_clk_in_hz, uint32_t bitrate_in_bps);
 uint32_t (*i2c_get_firmware_version)(void);
 CHIP_I2C_MODE_T (*i2c_get_status)(I2C_HANDLE_T *handle);
 ErrorCode_t (*i2c_set_timeout)(I2C_HANDLE_T *handle, uint32_t timeout);
} I2CD_API_T;
# 37 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_pwr_8xx.h" 1
# 79 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_pwr_8xx.h"
typedef struct PWRD_API {
 void (*set_pll)(uint32_t cmd[], uint32_t resp[]);
 void (*set_power)(uint32_t cmd[], uint32_t resp[]);
} PWRD_API_T;
# 38 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_uart_8xx.h" 1
# 92 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_uart_8xx.h"
typedef void UART_HANDLE_T;




typedef void (*UART_CALLBK_T)(uint32_t err_code, uint32_t n);




typedef void (*UART_DMA_REQ_T)(uint32_t src_adr, uint32_t dst_adr, uint32_t size);




typedef struct {
 uint32_t sys_clk_in_hz;
 uint32_t baudrate_in_hz;
 uint8_t config;



 uint8_t sync_mod;







 uint16_t error_en;





} UART_CONFIG_T;




typedef struct {
 uint8_t *buffer;
 uint32_t size;
 uint16_t transfer_mode;
# 145 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/rom_uart_8xx.h"
 uint16_t driver_mode;



 UART_CALLBK_T callback_func_pt;
 UART_DMA_REQ_T dma_req_func_pt;
} UART_PARAM_T;




typedef struct UARTD_API {

 uint32_t (*uart_get_mem_size)(void);
 UART_HANDLE_T * (*uart_setup)(uint32_t base_addr, uint8_t * ram);
 uint32_t (*uart_init)(UART_HANDLE_T *handle, UART_CONFIG_T *set);


 uint8_t (*uart_get_char)(UART_HANDLE_T *handle);
 void (*uart_put_char)(UART_HANDLE_T *handle, uint8_t data);
 uint32_t (*uart_get_line)(UART_HANDLE_T *handle, UART_PARAM_T *param);
 uint32_t (*uart_put_line)(UART_HANDLE_T *handle, UART_PARAM_T *param);


 void (*uart_isr)(UART_HANDLE_T *handle);
} UARTD_API_T;
# 39 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h" 2
# 52 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/romapi_8xx.h"
typedef struct ROM_API {
 const uint32_t unused[3];
 const PWRD_API_T *pPWRD;
 const uint32_t p_dev1;
 const I2CD_API_T *pI2CD;
 const uint32_t p_dev3;
 const uint32_t p_dev4;
 const uint32_t p_dev5;
 const UARTD_API_T *pUARTD;
} ROM_API_T;
# 139 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h" 1
# 92 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
typedef struct {
 volatile uint32_t SYSMEMREMAP;
 volatile uint32_t PRESETCTRL;
 volatile uint32_t SYSPLLCTRL;
 volatile uint32_t SYSPLLSTAT;
 uint32_t RESERVED0[4];
 volatile uint32_t SYSOSCCTRL;
 volatile uint32_t WDTOSCCTRL;
 uint32_t RESERVED1[2];
 volatile uint32_t SYSRSTSTAT;
 uint32_t RESERVED2[3];
 volatile uint32_t SYSPLLCLKSEL;
 volatile uint32_t SYSPLLCLKUEN;
 uint32_t RESERVED3[10];
 volatile uint32_t MAINCLKSEL;
 volatile uint32_t MAINCLKUEN;
 volatile uint32_t SYSAHBCLKDIV;
 uint32_t RESERVED4[1];
 volatile uint32_t SYSAHBCLKCTRL;
 uint32_t RESERVED5[4];
 volatile uint32_t UARTCLKDIV;
 uint32_t RESERVED6[18];
 volatile uint32_t CLKOUTSEL;
 volatile uint32_t CLKOUTUEN;
 volatile uint32_t CLKOUTDIV;
 uint32_t RESERVED7;
 volatile uint32_t UARTFRGDIV;
 volatile uint32_t UARTFRGMULT;
 uint32_t RESERVED8[1];
 volatile uint32_t EXTTRACECMD;
 volatile uint32_t PIOPORCAP0;
 uint32_t RESERVED9[12];
 volatile uint32_t IOCONCLKDIV[7];
 volatile uint32_t BODCTRL;
 volatile uint32_t SYSTCKCAL;
 uint32_t RESERVED10[6];
 volatile uint32_t IRQLATENCY;
 volatile uint32_t NMISRC;
 volatile uint32_t PINTSEL[8];
 uint32_t RESERVED11[27];
 volatile uint32_t STARTERP0;
 uint32_t RESERVED12[3];
 volatile uint32_t STARTERP1;
 uint32_t RESERVED13[6];
 volatile uint32_t PDSLEEPCFG;
 volatile uint32_t PDAWAKECFG;
 volatile uint32_t PDRUNCFG;
 uint32_t RESERVED14[111];
 volatile const uint32_t DEVICEID;
} LPC_SYSCTL_T;




typedef enum CHIP_SYSCTL_BOOT_MODE_REMAP {
 REMAP_BOOT_LOADER_MODE,
 REMAP_USER_RAM_MODE,
 REMAP_USER_FLASH_MODE
} CHIP_SYSCTL_BOOT_MODE_REMAP_T;




typedef enum {
 RESET_SPI0,
 RESET_SPI1,
 RESET_UARTFBRG,
 RESET_USART0,
 RESET_USART1,
 RESET_USART2,
 RESET_I2C,
 RESET_MRT,
 RESET_SCT,
 RESET_WKT,
 RESET_GPIO,
 RESET_FLASH,
 RESET_ACMP
} CHIP_SYSCTL_PERIPH_RESET_T;




typedef enum CHIP_SYSCTL_BODRSTLVL {
 SYSCTL_BODRSTLVL_0,
 SYSCTL_BODRSTLVL_1,
 SYSCTL_BODRSTLVL_2,
 SYSCTL_BODRSTLVL_3,
} CHIP_SYSCTL_BODRSTLVL_T;




typedef enum CHIP_SYSCTL_BODRINTVAL {
 SYSCTL_BODINTVAL_LVL0,
 SYSCTL_BODINTVAL_LVL1,
 SYSCTL_BODINTVAL_LVL2,
 SYSCTL_BODINTVAL_LVL3,
} CHIP_SYSCTL_BODRINTVAL_T;






static inline void Chip_SYSCTL_Map(CHIP_SYSCTL_BOOT_MODE_REMAP_T remap)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSMEMREMAP = (uint32_t) remap;
}
# 208 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
static inline void Chip_SYSCTL_AssertPeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->PRESETCTRL &= ~(1 << (uint32_t) periph);
}






static inline void Chip_SYSCTL_DeassertPeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->PRESETCTRL |= (1 << (uint32_t) periph);
}






static inline void Chip_SYSCTL_PeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
 Chip_SYSCTL_AssertPeriphReset(periph);
 Chip_SYSCTL_DeassertPeriphReset(periph);
}






static inline uint32_t Chip_SYSCTL_GetSystemRSTStatus(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->SYSRSTSTAT;
}







static inline void Chip_SYSCTL_ClearSystemRSTStatus(uint32_t reset)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSRSTSTAT = reset;
}






static inline uint32_t Chip_SYSCTL_GetPORPIOStatus(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->PIOPORCAP0;
}
# 273 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
static inline void Chip_SYSCTL_SetBODLevels(CHIP_SYSCTL_BODRSTLVL_T rstlvl,
           CHIP_SYSCTL_BODRINTVAL_T intlvl)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL = ((uint32_t) rstlvl) | (((uint32_t) intlvl) << 2);
}





static inline void Chip_SYSCTL_EnableBODReset(void)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL |= (1 << 4);
}





static inline void Chip_SYSCTL_DisableBODReset(void)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL &= ~(1 << 4);
}






static inline void Chip_SYSCTL_SetSYSTCKCAL(uint32_t sysCalVal)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSTCKCAL = sysCalVal;
}
# 314 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
static inline void Chip_SYSCTL_SetIRQLatency(uint32_t latency)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->IRQLATENCY = latency;
}





static inline uint32_t Chip_SYSCTL_GetIRQLatency(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->IRQLATENCY;
}
# 335 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
static inline void Chip_SYSCTL_SetNMISource(uint32_t intsrc)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC = intsrc;
}





static inline void Chip_SYSCTL_EnableNMISource(void)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC |= ((uint32_t) 1 << 31);
}





static inline void Chip_SYSCTL_DisableNMISource(void)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC &= ~(((uint32_t) 1 << 31));
}
# 367 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
static inline void Chip_SYSCTL_SetPinInterrupt(uint32_t intno, uint32_t pin)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->PINTSEL[intno] = (uint32_t) pin;
}
# 379 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
static inline void Chip_SYSCTL_EnablePINTWakeup(uint32_t pin)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP0 |= (1 << pin);
}







static inline void Chip_SYSCTL_DisablePINTWakeup(uint32_t pin)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP0 &= ~(1 << pin);
}






static inline void Chip_SYSCTL_EnablePeriphWakeup(uint32_t periphmask)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP1 |= periphmask;
}






static inline void Chip_SYSCTL_DisablePeriphWakeup(uint32_t periphmask)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP1 &= ~periphmask;
}






static inline uint32_t Chip_SYSCTL_GetDeepSleepPD(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->PDSLEEPCFG;
}






static inline uint32_t Chip_SYSCTL_GetWakeup(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->PDAWAKECFG;
}






static inline uint32_t Chip_SYSCTL_GetPowerStates(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->PDRUNCFG;
}





static inline uint32_t Chip_SYSCTL_GetDeviceID(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->DEVICEID;
}
# 464 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
void Chip_SYSCTL_SetDeepSleepPD(uint32_t sleepmask);
# 476 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/syscon_8xx.h"
void Chip_SYSCTL_SetWakeup(uint32_t wakeupmask);






void Chip_SYSCTL_PowerDown(uint32_t powerdownmask);






void Chip_SYSCTL_PowerUp(uint32_t powerupmask);
# 140 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h" 1
# 50 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
typedef enum CHIP_SYSCTL_PLLCLKSRC {
 SYSCTL_PLLCLKSRC_IRC = 0,
 SYSCTL_PLLCLKSRC_SYSOSC,
 SYSCTL_PLLCLKSRC_RESERVED,
 SYSCTL_PLLCLKSRC_EXT_CLKIN,
} CHIP_SYSCTL_PLLCLKSRC_T;





typedef enum CHIP_WDTLFO_OSC {
 WDTLFO_OSC_ILLEGAL,
 WDTLFO_OSC_0_60,
 WDTLFO_OSC_1_05,
 WDTLFO_OSC_1_40,
 WDTLFO_OSC_1_75,
 WDTLFO_OSC_2_10,
 WDTLFO_OSC_2_40,
 WDTLFO_OSC_2_70,
 WDTLFO_OSC_3_00,
 WDTLFO_OSC_3_25,
 WDTLFO_OSC_3_50,
 WDTLFO_OSC_3_75,
 WDTLFO_OSC_4_00,
 WDTLFO_OSC_4_20,
 WDTLFO_OSC_4_40,
 WDTLFO_OSC_4_60
} CHIP_WDTLFO_OSC_T;




typedef enum CHIP_SYSCTL_MAINCLKSRC {
 SYSCTL_MAINCLKSRC_IRC = 0,
 SYSCTL_MAINCLKSRC_PLLIN,
 SYSCTL_MAINCLKSRC_WDTOSC,
 SYSCTL_MAINCLKSRC_PLLOUT,
} CHIP_SYSCTL_MAINCLKSRC_T;




typedef enum CHIP_SYSCTL_CLOCK {
 SYSCTL_CLOCK_SYS = 0,
 SYSCTL_CLOCK_ROM,
 SYSCTL_CLOCK_RAM,
 SYSCTL_CLOCK_FLASHREG,
 SYSCTL_CLOCK_FLASH,
 SYSCTL_CLOCK_I2C,
 SYSCTL_CLOCK_GPIO,
 SYSCTL_CLOCK_SWM,
 SYSCTL_CLOCK_SCT,
 SYSCTL_CLOCK_WKT,
 SYSCTL_CLOCK_MRT,
 SYSCTL_CLOCK_SPI0,
 SYSCTL_CLOCK_SPI1,
 SYSCTL_CLOCK_CRC,
 SYSCTL_CLOCK_UART0,
 SYSCTL_CLOCK_UART1,
 SYSCTL_CLOCK_UART2,
 SYSCTL_CLOCK_WWDT,
 SYSCTL_CLOCK_IOCON,
 SYSCTL_CLOCK_ACOMP
} CHIP_SYSCTL_CLOCK_T;




typedef enum CHIP_SYSCTL_CLKOUTSRC {
 SYSCTL_CLKOUTSRC_IRC = 0,
 SYSCTL_CLKOUTSRC_SYSOSC,
 SYSCTL_CLKOUTSRC_WDTOSC,
 SYSCTL_CLKOUTSRC_MAINSYSCLK,
} CHIP_SYSCTL_CLKOUTSRC_T;
# 133 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
static inline void Chip_Clock_SetupSystemPLL(uint8_t msel, uint8_t psel)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSPLLCTRL = (msel & 0x1F) | ((psel & 0x3) << 5);
}





static inline _Bool Chip_Clock_IsSystemPLLLocked(void)
{
 return (_Bool) ((((LPC_SYSCTL_T *) (0x40048000UL))->SYSPLLSTAT & 1) != 0);
}
# 154 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
static inline void Chip_Clock_SetWDTOSC(CHIP_WDTLFO_OSC_T wdtclk, uint8_t div)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->WDTOSCCTRL = (((uint32_t) wdtclk) << 5) | ((div >> 1) - 1);
}





static inline CHIP_SYSCTL_MAINCLKSRC_T Chip_Clock_GetMainClockSource(void)
{
 return (CHIP_SYSCTL_MAINCLKSRC_T) (((LPC_SYSCTL_T *) (0x40048000UL))->MAINCLKSEL);
}
# 175 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
static inline void Chip_Clock_SetSysClockDiv(uint32_t div)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKDIV = div;
}






static inline void Chip_Clock_EnablePeriphClock(CHIP_SYSCTL_CLOCK_T clk)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKCTRL |= (1 << clk);
}






static inline void Chip_Clock_DisablePeriphClock(CHIP_SYSCTL_CLOCK_T clk)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKCTRL &= ~(1 << clk);
}
# 207 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
static inline void Chip_Clock_SetUARTClockDiv(uint32_t div)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->UARTCLKDIV = div;
}






static inline uint32_t Chip_Clock_GetUARTClockDiv(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->UARTCLKDIV;
}






static inline void Chip_SYSCTL_SetUSARTFRGDivider(uint8_t div)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGDIV = (uint32_t) div;
}





static inline uint32_t Chip_SYSCTL_GetUSARTFRGDivider(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGDIV;
}






static inline void Chip_SYSCTL_SetUSARTFRGMultiplier(uint8_t mult)
{
 ((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGMULT = (uint32_t) mult;
}





static inline uint32_t Chip_SYSCTL_GetUSARTFRGMultiplier(void)
{
 return ((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGMULT;
}





static inline uint32_t Chip_Clock_GetMainOscRate(void)
{
 return OscRateIn;
}





static inline uint32_t Chip_Clock_GetIntOscRate(void)
{
 return (12000000);
}





static inline uint32_t Chip_Clock_GetExtClockInRate(void)
{
 return ExtRateIn;
}
# 294 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
void Chip_Clock_SetSystemPLLSource(CHIP_SYSCTL_PLLCLKSRC_T src);
# 305 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
void Chip_Clock_SetPLLBypass(_Bool bypass, _Bool highfr);
# 314 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
void Chip_Clock_SetMainClockSource(CHIP_SYSCTL_MAINCLKSRC_T src);
# 326 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/clock_8xx.h"
void Chip_Clock_SetCLKOUTSource(CHIP_SYSCTL_CLKOUTSRC_T src, uint32_t div);






uint32_t Chip_Clock_GetWDTOSCRate(void);





uint32_t Chip_Clock_GetSystemPLLInClockRate(void);





uint32_t Chip_Clock_GetSystemPLLOutClockRate(void);





uint32_t Chip_Clock_GetMainClockRate(void);





uint32_t Chip_Clock_GetSystemClockRate(void);
# 141 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h" 1
# 54 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
typedef struct {
 volatile uint32_t PIO0[(18) + 1];
} LPC_IOCON_T;
# 98 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
typedef enum CHIP_PINx {
 IOCON_PIO0 = 0x11,
 IOCON_PIO1 = 0x0B,
 IOCON_PIO2 = 0x06,
 IOCON_PIO3 = 0x05,
 IOCON_PIO4 = 0x04,
 IOCON_PIO5 = 0x03,

 IOCON_PIO6 = 0x10,
 IOCON_PIO7 = 0x0F,
 IOCON_PIO8 = 0x0E,
 IOCON_PIO9 = 0x0D,
 IOCON_PIO10 = 0x08,
 IOCON_PIO11 = 0x07,
 IOCON_PIO12 = 0x02,
 IOCON_PIO13 = 0x01,

 IOCON_PIO14 = 0x12,
 IOCON_PIO15 = 0x0A,
 IOCON_PIO16 = 0x09,
 IOCON_PIO17 = 0x00,
 IOCON_PIO_NUL = 0x0C
} CHIP_PINx_T;




typedef enum CHIP_PIN_MODE {
 PIN_MODE_INACTIVE = 0,
 PIN_MODE_PULLDN = 1,
 PIN_MODE_PULLUP = 2,
 PIN_MODE_REPEATER = 3
} CHIP_PIN_MODE_T;




typedef enum CHIP_PIN_SMODE {
 PIN_SMODE_BYPASS = 0,
 PIN_SMODE_CYC1 = 1,
 PIN_SMODE_CYC2 = 2,
 PIN_SMODE_CYC3 = 3
} CHIP_PIN_SMODE_T;





typedef enum CHIP_PIN_CLKDIV {
 IOCONCLKDIV0 = 0,
 IOCONCLKDIV1 = 1,
 IOCONCLKDIV2 = 2,
 IOCONCLKDIV3 = 3,
 IOCONCLKDIV4 = 4,
 IOCONCLKDIV5 = 5,
 IOCONCLKDIV6 = 6
} CHIP_PIN_CLKDIV_T;




typedef enum CHIP_PIN_I2CMODE {
 PIN_I2CMODE_STDFAST = 0,
 PIN_I2CMODE_GPIO = 1,
 PIN_I2CMODE_FASTPLUS = 2
} CHIP_PIN_I2CMODE_T;
# 173 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_MODE_T mode);
# 183 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, _Bool enable);
# 192 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
static inline void Chip_IOCON_PinEnableHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
 pIOCON->PIO0[pin] |= (0x1 << 5);
}
# 204 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
static inline void Chip_IOCON_PinDisableHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
 pIOCON->PIO0[pin] &= ~(0x1 << 5);
}
# 216 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, _Bool invert);







static inline void Chip_IOCON_PinEnableInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
 pIOCON->PIO0[pin] |= (0x1 << 6);
}







static inline void Chip_IOCON_PinDisableInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
 pIOCON->PIO0[pin] &= ~(0x1 << 6);
}
# 248 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, _Bool open_drain);







static inline void Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
 pIOCON->PIO0[pin] |= (0x1 << 10);
}







static inline void Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
 pIOCON->PIO0[pin] &= ~(0x1 << 10);
}
# 279 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetSampleMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_SMODE_T smode);
# 288 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetClockDivisor(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_CLKDIV_T clkdiv);
# 298 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/iocon_8xx.h"
void Chip_IOCON_PinSetI2CMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_I2CMODE_T mode);
# 142 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/swm_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/swm_8xx.h"
typedef struct {
 volatile uint32_t PINASSIGN[9];
 volatile const uint32_t RESERVED0[103];
 volatile uint32_t PINENABLE0;
} LPC_SWM_T;




typedef enum CHIP_SWM_PIN_MOVABLE {
 SWM_U0_TXD_O = 0x00,
 SWM_U0_RXD_I = 0x01,
 SWM_U0_RTS_O = 0x02,
 SWM_U0_CTS_I = 0x03,
 SWM_U0_SCLK_IO = 0x10,
 SWM_U1_TXD_O = 0x11,
 SWM_U1_RXD_I = 0x12,
 SWM_U1_RTS_O = 0x13,
 SWM_U1_CTS_I = 0x20,
 SWM_U1_SCLK_IO = 0x21,
 SWM_U2_TXD_O = 0x22,
 SWM_U2_RXD_I = 0x23,
 SWM_U2_RTS_O = 0x30,
 SWM_U2_CTS_I = 0x31,
 SWM_U2_SCLK_IO = 0x32,
 SWM_SPI0_SCK_IO = 0x33,
 SWM_SPI0_MOSI_IO = 0x40,
 SWM_SPI0_MISO_IO = 0x41,
 SWM_SPI0_SSEL_IO = 0x42,
 SWM_SPI1_SCK_IO = 0x43,
 SWM_SPI1_MOSI_IO = 0x50,
 SWM_SPI1_MISO_IO = 0x51,
 SWM_SPI1_SSEL_IO = 0x52,
 SWM_CTIN_0_I = 0x53,
 SWM_CTIN_1_I = 0x60,
 SWM_CTIN_2_I = 0x61,
 SWM_CTIN_3_I = 0x62,
 SWM_CTOUT_0_O = 0x63,
 SWM_CTOUT_1_O = 0x70,
 SWM_CTOUT_2_O = 0x71,
 SWM_CTOUT_3_O = 0x72,
 SWM_I2C_SDA_IO = 0x73,
 SWM_I2C_SCL_IO = 0x80,
 SWM_ACMP_O_O = 0x81,
 SWM_CLKOUT_O = 0x82,
 SWM_GPIO_INT_BMAT_O = 0x83,
} CHIP_SWM_PIN_MOVABLE_T;




typedef enum CHIP_SWM_PIN_FIXED {
 SWM_FIXED_ACMP_I1 = 0,
 SWM_FIXED_ACMP_I2 = 1,
 SWM_FIXED_SWCLK = 2,
 SWM_FIXED_SWDIO = 3,
 SWM_FIXED_XTALIN = 4,
 SWM_FIXED_XTALOUT = 5,
 SWM_FIXED_RST = 6,
 SWM_FIXED_CLKIN = 7,
 SWM_FIXED_VDDCMP = 8
} CHIP_SWM_PIN_FIXED_T;






static inline void Chip_SWM_Init(void)
{
 Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
}






static inline void Chip_SWM_Deinit(void)
{
 Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}







void Chip_SWM_MovablePinAssign(CHIP_SWM_PIN_MOVABLE_T movable, uint8_t assign);







void Chip_SWM_FixedPinEnable(CHIP_SWM_PIN_FIXED_T pin, _Bool enable);






static inline void Chip_SWM_EnableFixedPin(CHIP_SWM_PIN_FIXED_T pin)
{
 ((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 &= ~(1 << (uint32_t) pin);
}






static inline void Chip_SWM_DisableFixedPin(CHIP_SWM_PIN_FIXED_T pin)
{
 ((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 |= (1 << (uint32_t) pin);
}






static inline _Bool Chip_SWM_IsEnabled(CHIP_SWM_PIN_FIXED_T pin)
{
 return (_Bool) ((((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 & (1 << (uint32_t) pin)) == 0);
}
# 143 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/fmc_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/fmc_8xx.h"
typedef struct {
 volatile const uint32_t RESERVED1[4];
 volatile uint32_t FLASHCFG;
 volatile const uint32_t RESERVED2[3];
 volatile uint32_t FMSSTART;
 volatile uint32_t FMSSTOP;
 volatile const uint32_t RESERVED3;
 volatile const uint32_t FMSW0;
} LPC_FMC_T;




typedef enum {
 FLASHTIM_20MHZ_CPU = 0,
 FLASHTIM_30MHZ_CPU = 1,
} FMC_FLASHTIM_T;
# 72 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/fmc_8xx.h"
static inline void Chip_FMC_SetFLASHAccess(FMC_FLASHTIM_T clks)
{
 uint32_t tmp = ((LPC_FMC_T *) (0x40040000UL))->FLASHCFG & (~(0x3));


 ((LPC_FMC_T *) (0x40040000UL))->FLASHCFG = tmp | clks;
}
# 90 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/fmc_8xx.h"
static inline void Chip_FMC_ComputeSignature(uint32_t start, uint32_t stop)
{
 ((LPC_FMC_T *) (0x40040000UL))->FMSSTART = start;
 ((LPC_FMC_T *) (0x40040000UL))->FMSSTOP = stop | (1UL << 31);
}





static inline _Bool Chip_FMC_IsSignatureBusy(void)
{
 return (_Bool) ((((LPC_FMC_T *) (0x40040000UL))->FMSSTOP & (1UL << 31)) != 0);
}





static inline uint32_t Chip_FMC_GetSignature(void)
{
 return ((LPC_FMC_T *) (0x40040000UL))->FMSW0;
}
# 144 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h"
typedef struct {
 volatile uint32_t ISEL;
 volatile uint32_t IENR;
 volatile uint32_t SIENR;
 volatile uint32_t CIENR;
 volatile uint32_t IENF;
 volatile uint32_t SIENF;
 volatile uint32_t CIENF;
 volatile uint32_t RISE;
 volatile uint32_t FALL;
 volatile uint32_t IST;
 volatile uint32_t PMCTRL;
 volatile uint32_t PMSRC;
 volatile uint32_t PMCFG;
} LPC_PIN_INT_T;
# 95 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h"
typedef enum Chip_PININT_BITSLICE {
 PININTBITSLICE0 = 0,
 PININTBITSLICE1 = 1,
 PININTBITSLICE2 = 2,
 PININTBITSLICE3 = 3,
 PININTBITSLICE4 = 4,
 PININTBITSLICE5 = 5,
 PININTBITSLICE6 = 6,
 PININTBITSLICE7 = 7
} Chip_PININT_BITSLICE_T;




typedef enum Chip_PININT_BITSLICE_CFG {
    PININT_PATTERNCONST1 = 0x0,
    PININT_PATTERNRISING = 0x1,
    PININT_PATTERNFALLING = 0x2,
    PININT_PATTERNRISINGRFALLING = 0x3,
    PININT_PATTERNHIGH = 0x4,
    PININT_PATTERNLOW = 0x5,
    PININT_PATTERCONST0 = 0x6,
    PININT_PATTEREVENT = 0x7
} Chip_PININT_BITSLICE_CFG_T;







static inline void Chip_PININT_Init(LPC_PIN_INT_T *pPININT) {}






static inline void Chip_PININT_DeInit(LPC_PIN_INT_T *pPININT) {}







static inline void Chip_PININT_SetPinModeEdge(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->ISEL &= ~pins;
}







static inline void Chip_PININT_SetPinModeLevel(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->ISEL |= pins;
}
# 165 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h"
static inline uint32_t Chip_PININT_GetHighEnabled(LPC_PIN_INT_T *pPININT)
{
    return pPININT->IENR;
}







static inline void Chip_PININT_EnableIntHigh(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->SIENR = pins;
}







static inline void Chip_PININT_DisableIntHigh(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->CIENR = pins;
}
# 200 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h"
static inline uint32_t Chip_PININT_GetLowEnabled(LPC_PIN_INT_T *pPININT)
{
    return pPININT->IENF;
}







static inline void Chip_PININT_EnableIntLow(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->SIENF = pins;
}







static inline void Chip_PININT_DisableIntLow(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->CIENF = pins;
}






static inline uint32_t Chip_PININT_GetRiseStates(LPC_PIN_INT_T *pPININT)
{
    return pPININT->RISE;
}







static inline void Chip_PININT_ClearRiseStates(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
  pPININT->RISE = pins;
}






static inline uint32_t Chip_PININT_GetFallStates(LPC_PIN_INT_T *pPININT)
{
    return pPININT->FALL;
}







static inline void Chip_PININT_ClearFallStates(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
  pPININT->FALL = pins;
}






static inline uint32_t Chip_PININT_GetIntStatus(LPC_PIN_INT_T *pPININT)
{
    return pPININT->IST;
}







static inline void Chip_PININT_ClearIntStatus(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->IST = pins;
}
# 297 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h"
void Chip_PININT_SetPatternMatchSrc(LPC_PIN_INT_T *pPININT, uint8_t chan, Chip_PININT_BITSLICE_T slice);
# 307 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pinint_8xx.h"
void Chip_PININT_SetPatternMatchConfig(LPC_PIN_INT_T *pPININT, Chip_PININT_BITSLICE_T slice,
        Chip_PININT_BITSLICE_CFG_T slice_cfg, _Bool end_point);






static inline void Chip_PININT_EnablePatternMatch(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL |= (1 << 0);
}






static inline void Chip_PININT_DisablePatternMatch(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL &= ~(1 << 0);
}






static inline void Chip_PININT_EnablePatternMatchRxEv(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL |= (1 << 1);
}






static inline void Chip_PININT_DisablePatternMatchRxEv(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL &= ~(1 << 1);
}
# 145 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
typedef struct {
 volatile uint32_t PCON;
 volatile uint32_t GPREG[4];
 volatile uint32_t DPDCTRL;
} LPC_PMU_T;




typedef enum CHIP_PMU_MCUPOWER {
 PMU_MCU_SLEEP = 0,
 PMU_MCU_DEEP_SLEEP,
 PMU_MCU_POWER_DOWN,
 PMU_MCU_DEEP_PWRDOWN
} CHIP_PMU_MCUPOWER_T;
# 89 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
static inline void Chip_PMU_WriteGPREG(LPC_PMU_T *pPMU, uint8_t regIndex, uint32_t value)
{
 pPMU->GPREG[regIndex] = value;
}







static inline uint32_t Chip_PMU_ReadGPREG(LPC_PMU_T *pPMU, uint8_t regIndex)
{
 return pPMU->GPREG[regIndex];
}
# 112 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
void Chip_PMU_SleepState(LPC_PMU_T *pPMU);
# 123 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
void Chip_PMU_DeepSleepState(LPC_PMU_T *pPMU);
# 135 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
void Chip_PMU_PowerDownState(LPC_PMU_T *pPMU);
# 148 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
void Chip_PMU_DeepPowerDownState(LPC_PMU_T *pPMU);







void Chip_PMU_Sleep(LPC_PMU_T *pPMU, CHIP_PMU_MCUPOWER_T SleepMode);
# 165 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
static inline void Chip_PMU_DisableDeepPowerDown(LPC_PMU_T *pPMU)
{
 pPMU->PCON |= (1 << 3);
}
# 177 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
static inline uint32_t Chip_PMU_GetSleepFlags(LPC_PMU_T *pPMU)
{
 return (pPMU->PCON & ((1 << 8) | (1 << 11)));
}
# 190 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
static inline void Chip_PMU_ClearSleepFlags(LPC_PMU_T *pPMU, uint32_t flags)
{
 pPMU->PCON &= ~flags;
}
# 205 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
static inline void Chip_PMU_SetPowerDownControl(LPC_PMU_T *pPMU, uint32_t flags)
{
 pPMU->DPDCTRL |= flags;
}
# 220 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/pmu_8xx.h"
static inline void Chip_PMU_ClearPowerDownControl(LPC_PMU_T *pPMU, uint32_t flags)
{
 pPMU->DPDCTRL &= ~flags;
}
# 146 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/acmp_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/acmp_8xx.h"
typedef struct {
 volatile uint32_t CTRL;
 volatile uint32_t LAD;
} LPC_CMP_T;
# 67 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/acmp_8xx.h"
typedef enum {
 ACMP_EDGESEL_FALLING = (0 << 3),
 ACMP_EDGESEL_RISING = (1 << 3),
 ACMP_EDGESEL_BOTH = (2 << 3)
} ACMP_EDGESEL_T;


typedef enum {
 ACMP_HYS_NONE = (0 << 25),
 ACMP_HYS_5MV = (1 << 25),
 ACMP_HYS_10MV = (2 << 25),
 ACMP_HYS_20MV = (3 << 25)
} ACMP_HYS_T;




typedef enum CHIP_ACMP_POS_INPUT {
 ACMP_POSIN_VLO = (0 << 8),
 ACMP_POSIN_ACMP_I1 = (1 << 8),
 ACMP_POSIN_ACMP_I2 = (2 << 8),
 ACMP_POSIN_INT_REF = (6 << 8),
} ACMP_POS_INPUT_T;




typedef enum CHIP_ACMP_NEG_INPUT {
 ACMP_NEGIN_VLO = (0 << 11),
 ACMP_NEGIN_ACMP_I1 = (1 << 11),
 ACMP_NEGIN_ACMP_I2 = (2 << 11),
 ACMP_NEGIN_INT_REF = (6 << 11)
} ACMP_NEG_INPUT_T;






void Chip_ACMP_Init(LPC_CMP_T *pACMP);






void Chip_ACMP_Deinit(LPC_CMP_T *pACMP);






static inline uint32_t Chip_ACMP_GetCompStatus(LPC_CMP_T *pACMP)
{
 return pACMP->CTRL & ((1 << 21) | (1 << 23));
}






void Chip_ACMP_EdgeClear(LPC_CMP_T *pACMP);







void Chip_ACMP_SetEdgeSelection(LPC_CMP_T *pACMP, ACMP_EDGESEL_T edgeSel);






static inline void Chip_ACMP_EnableSyncCompOut(LPC_CMP_T *pACMP)
{
 pACMP->CTRL |= (1 << 6);
}






static inline void Chip_ACMP_DisableSyncCompOut(LPC_CMP_T *pACMP)
{
 pACMP->CTRL &= ~(1 << 6);
}







void Chip_ACMP_SetPosVoltRef(LPC_CMP_T *pACMP, ACMP_POS_INPUT_T Posinput);







void Chip_ACMP_SetNegVoltRef(LPC_CMP_T *pACMP, ACMP_NEG_INPUT_T Neginput);







void Chip_ACMP_SetHysteresis(LPC_CMP_T *pACMP, ACMP_HYS_T hys);
# 193 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/acmp_8xx.h"
void Chip_ACMP_SetupAMCPRefs(LPC_CMP_T *pACMP, ACMP_EDGESEL_T edgeSel,
        ACMP_POS_INPUT_T Posinput, ACMP_NEG_INPUT_T Neginput,
        ACMP_HYS_T hys);
# 205 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/acmp_8xx.h"
void Chip_ACMP_SetupVoltLadder(LPC_CMP_T *pACMP, uint32_t ladsel, _Bool ladrefVDDCMP);






static inline void Chip_ACMP_EnableVoltLadder(LPC_CMP_T *pACMP)
{
 pACMP->LAD |= (1 << 0);
}






static inline void Chip_ACMP_DisableVoltLadder(LPC_CMP_T *pACMP)
{
 pACMP->LAD &= ~(1 << 0);
}
# 147 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/crc_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/crc_8xx.h"
typedef struct {
 volatile uint32_t MODE;
 volatile uint32_t SEED;
 union {
  volatile const uint32_t SUM;
  volatile uint32_t WRDATA32;
  volatile uint16_t WRDATA16;
  volatile uint8_t WRDATA8;
 };

} LPC_CRC_T;
# 84 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/crc_8xx.h"
typedef enum IP_CRC_001_POLY {
 CRC_POLY_CCITT = (0x00),
 CRC_POLY_CRC16 = (0x01),
 CRC_POLY_CRC32 = (0x02),
 CRC_POLY_LAST,
} CRC_POLY_T;





void Chip_CRC_Init(void);





void Chip_CRC_Deinit(void);
# 111 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/crc_8xx.h"
static inline void Chip_CRC_SetPoly(CRC_POLY_T poly, uint32_t flags)
{
 ((LPC_CRC_T *) (0x50000000UL))->MODE = (uint32_t) poly | flags;
}





static inline void Chip_CRC_UseCRC16(void)
{
 ((LPC_CRC_T *) (0x50000000UL))->MODE = (0x15);
 ((LPC_CRC_T *) (0x50000000UL))->SEED = (0x00000000);
}





static inline void Chip_CRC_UseCRC32(void)
{
 ((LPC_CRC_T *) (0x50000000UL))->MODE = (0x36);
 ((LPC_CRC_T *) (0x50000000UL))->SEED = (0xFFFFFFFF);
}





static inline void Chip_CRC_UseCCITT(void)
{
 ((LPC_CRC_T *) (0x50000000UL))->MODE = (0x00);
 ((LPC_CRC_T *) (0x50000000UL))->SEED = (0x0000FFFF);
}






void Chip_CRC_UseDefaultConfig(CRC_POLY_T poly);






static inline void Chip_CRC_SetMode(uint32_t mode)
{
 ((LPC_CRC_T *) (0x50000000UL))->MODE = mode;
}





static inline uint32_t Chip_CRC_GetMode(void)
{
 return ((LPC_CRC_T *) (0x50000000UL))->MODE;
}






static inline void Chip_CRC_SetSeed(uint32_t seed)
{
 ((LPC_CRC_T *) (0x50000000UL))->SEED = seed;
}





static inline uint32_t Chip_CRC_GetSeed(void)
{
 return ((LPC_CRC_T *) (0x50000000UL))->SEED;
}






static inline void Chip_CRC_Write8(uint8_t data)
{
 ((LPC_CRC_T *) (0x50000000UL))->WRDATA8 = data;
}






static inline void Chip_CRC_Write16(uint16_t data)
{
 ((LPC_CRC_T *) (0x50000000UL))->WRDATA16 = data;
}






static inline void Chip_CRC_Write32(uint32_t data)
{
 ((LPC_CRC_T *) (0x50000000UL))->WRDATA32 = data;
}





static inline uint32_t Chip_CRC_Sum(void)
{
 return ((LPC_CRC_T *) (0x50000000UL))->SUM;
}







uint32_t Chip_CRC_CRC8(const uint8_t *data, uint32_t bytes);







uint32_t Chip_CRC_CRC16(const uint16_t *data, uint32_t hwords);







uint32_t Chip_CRC_CRC32(const uint32_t *data, uint32_t words);
# 148 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
typedef struct {
 volatile uint8_t B[128][32];
 volatile uint32_t W[32][32];
 volatile uint32_t DIR[32];
 volatile uint32_t MASK[32];
 volatile uint32_t PIN[32];
 volatile uint32_t MPIN[32];
 volatile uint32_t SET[32];
 volatile uint32_t CLR[32];
 volatile uint32_t NOT[32];
} LPC_GPIO_T;






void Chip_GPIO_Init(LPC_GPIO_T *pGPIO);






void Chip_GPIO_DeInit(LPC_GPIO_T *pGPIO);
# 82 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_WritePortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, _Bool setting)
{
 pGPIO->B[port][pin] = setting;
}
# 96 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, _Bool setting)
{
 pGPIO->B[port][pin] = setting;
}
# 109 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline _Bool Chip_GPIO_ReadPortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin)
{
 return (_Bool) pGPIO->B[port][pin];
}
# 122 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline _Bool Chip_GPIO_GetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 return (_Bool) pGPIO->B[port][pin];
}
# 138 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
void Chip_GPIO_WriteDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, _Bool setting);
# 147 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPinDIROutput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 pGPIO->DIR[port] |= 1UL << pin;
}
# 159 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPinDIRInput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 pGPIO->DIR[port] &= ~(1UL << pin);
}
# 172 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
void Chip_GPIO_SetPinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, _Bool output);
# 182 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline _Bool Chip_GPIO_ReadDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit)
{
 return (_Bool) (((pGPIO->DIR[port]) >> bit) & 1);
}
# 194 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline _Bool Chip_GPIO_GetPinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 return Chip_GPIO_ReadDirBit(pGPIO, port, pin);
}
# 209 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
void Chip_GPIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue, uint8_t out);
# 220 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortDIROutput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask)
{
 pGPIO->DIR[port] |= pinMask;
}
# 234 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortDIRInput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask)
{
 pGPIO->DIR[port] &= ~pinMask;
}
# 249 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
void Chip_GPIO_SetPortDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask, _Bool outSet);
# 259 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline uint32_t Chip_GPIO_GetPortDIR(LPC_GPIO_T *pGPIO, uint8_t port)
{
 return pGPIO->DIR[port];
}
# 273 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortMask(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t mask)
{
 pGPIO->MASK[port] = mask;
}







static inline uint32_t Chip_GPIO_GetPortMask(LPC_GPIO_T *pGPIO, uint8_t port)
{
 return pGPIO->MASK[port];
}
# 296 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t value)
{
 pGPIO->PIN[port] = value;
}







static inline uint32_t Chip_GPIO_GetPortValue(LPC_GPIO_T *pGPIO, uint8_t port)
{
 return pGPIO->PIN[port];
}
# 319 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetMaskedPortValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t value)
{
 pGPIO->MPIN[port] = value;
}







static inline uint32_t Chip_GPIO_GetMaskedPortValue(LPC_GPIO_T *pGPIO, uint8_t port)
{
 return pGPIO->MPIN[port];
}
# 345 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
 pGPIO->SET[portNum] = bitValue;
}
# 359 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortOutHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
 pGPIO->SET[port] = pins;
}
# 373 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPinOutHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 pGPIO->SET[port] = (1 << pin);
}
# 388 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_ClearValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
 pGPIO->CLR[portNum] = bitValue;
}
# 402 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortOutLow(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
 pGPIO->CLR[port] = pins;
}
# 416 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPinOutLow(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 pGPIO->CLR[port] = (1 << pin);
}
# 430 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPortToggle(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
 pGPIO->NOT[port] = pins;
}
# 444 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline void Chip_GPIO_SetPinToggle(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
 pGPIO->NOT[port] = (1 << pin);
}
# 458 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/gpio_8xx.h"
static inline uint32_t Chip_GPIO_ReadValue(LPC_GPIO_T *pGPIO, uint8_t portNum)
{
 return pGPIO->PIN[portNum];
}
# 149 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/mrt_8xx.h" 1
# 53 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/mrt_8xx.h"
typedef struct {
 volatile uint32_t INTVAL;
 volatile uint32_t TIMER;
 volatile uint32_t CTRL;
 volatile uint32_t STAT;
} LPC_MRT_CH_T;




typedef struct {
 LPC_MRT_CH_T CHANNEL[(4)];
 uint32_t unused[45];
 volatile uint32_t IDLE_CH;
 volatile uint32_t IRQ_FLAG;
} LPC_MRT_T;




typedef enum MRT_MODE {
 MRT_MODE_REPEAT = (0 << 1),
 MRT_MODE_ONESHOT = (1 << 1)
} MRT_MODE_T;
# 111 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/mrt_8xx.h"
static inline void Chip_MRT_Init(void)
{

 Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MRT);


 Chip_SYSCTL_PeriphReset(RESET_MRT);
}





static inline void Chip_MRT_DeInit(void)
{

 Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_MRT);
}






static inline LPC_MRT_CH_T *Chip_MRT_GetRegPtr(uint8_t ch)
{
 return ((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[(ch)]);
}






static inline uint32_t Chip_MRT_GetInterval(LPC_MRT_CH_T *pMRT)
{
 return pMRT->INTVAL;
}
# 161 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/mrt_8xx.h"
static inline void Chip_MRT_SetInterval(LPC_MRT_CH_T *pMRT, uint32_t interval)
{
 pMRT->INTVAL = interval;
}






static inline uint32_t Chip_MRT_GetTimer(LPC_MRT_CH_T *pMRT)
{
 return pMRT->TIMER;
}






static inline _Bool Chip_MRT_GetEnabled(LPC_MRT_CH_T *pMRT)
{
 return (_Bool) ((pMRT->CTRL & (0x01)) != 0);
}






static inline void Chip_MRT_SetEnabled(LPC_MRT_CH_T *pMRT)
{
 pMRT->CTRL |= (0x01);
}






static inline void Chip_MRT_SetDisabled(LPC_MRT_CH_T *pMRT)
{
 pMRT->CTRL &= ~(0x01);
}






static inline MRT_MODE_T Chip_MRT_GetMode(LPC_MRT_CH_T *pMRT)
{
 return (MRT_MODE_T) (pMRT->CTRL & (0x06));
}







static inline void Chip_MRT_SetMode(LPC_MRT_CH_T *pMRT, MRT_MODE_T mode)
{
 uint32_t reg;

 reg = pMRT->CTRL & ~(0x06);
 pMRT->CTRL = reg | (uint32_t) mode;
}






static inline _Bool Chip_MRT_IsRepeatMode(LPC_MRT_CH_T *pMRT)
{
 return ((pMRT->CTRL & (0x06)) != 0) ? 0 : 1;
}






static inline _Bool Chip_MRT_IsOneShotMode(LPC_MRT_CH_T *pMRT)
{
 return ((pMRT->CTRL & (0x06)) != 0) ? 1 : 0;
}






static inline _Bool Chip_MRT_IntPending(LPC_MRT_CH_T *pMRT)
{
 return (_Bool) ((pMRT->STAT & (0x01)) != 0);
}






static inline void Chip_MRT_IntClear(LPC_MRT_CH_T *pMRT)
{
 pMRT->STAT |= (0x01);
}






static inline _Bool Chip_MRT_Running(LPC_MRT_CH_T *pMRT)
{
 return (_Bool) ((pMRT->STAT & (0x02)) != 0);
}





static inline uint8_t Chip_MRT_GetIdleChannel(void)
{
 return (uint8_t) (((LPC_MRT_T *) (0x40004000UL))->IDLE_CH);
}





static inline uint8_t Chip_MRT_GetIdleChannelShifted(void)
{
 return (uint8_t) (Chip_MRT_GetIdleChannel() >> 4);
}





static inline uint32_t Chip_MRT_GetIntPending(void)
{
 return ((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG;
}






static inline _Bool Chip_MRT_GetIntPendingByChannel(uint8_t ch)
{
 return (_Bool) (((((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG >> ch) & 1) != 0);
}
# 326 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/mrt_8xx.h"
static inline void Chip_MRT_ClearIntPending(uint32_t mask)
{
 ((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG = mask;
}
# 150 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h" 1
# 39 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h" 1
# 45 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h"
typedef struct {
 void *data;
 int count;
 int itemSz;
 uint32_t head;
 uint32_t tail;
} RINGBUFF_T;
# 76 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h"
int RingBuffer_Init(RINGBUFF_T *RingBuff, void *buffer, int itemSize, int count);






static inline void RingBuffer_Flush(RINGBUFF_T *RingBuff)
{
 RingBuff->head = RingBuff->tail = 0;
}






static inline int RingBuffer_GetSize(RINGBUFF_T *RingBuff)
{
 return RingBuff->count;
}






static inline int RingBuffer_GetCount(RINGBUFF_T *RingBuff)
{
 return (*(volatile uint32_t *) &(RingBuff)->head) - (*(volatile uint32_t *) &(RingBuff)->tail);
}






static inline int RingBuffer_GetFree(RINGBUFF_T *RingBuff)
{
 return RingBuff->count - RingBuffer_GetCount(RingBuff);
}






static inline int RingBuffer_IsFull(RINGBUFF_T *RingBuff)
{
 return (RingBuffer_GetCount(RingBuff) >= RingBuff->count);
}






static inline int RingBuffer_IsEmpty(RINGBUFF_T *RingBuff)
{
 return (*(volatile uint32_t *) &(RingBuff)->head) == (*(volatile uint32_t *) &(RingBuff)->tail);
}
# 147 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h"
int RingBuffer_Insert(RINGBUFF_T *RingBuff, const void *data);
# 159 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h"
int RingBuffer_InsertMult(RINGBUFF_T *RingBuff, const void *data, int num);
# 170 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h"
int RingBuffer_Pop(RINGBUFF_T *RingBuff, void *data);
# 181 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/ring_buffer.h"
int RingBuffer_PopMult(RINGBUFF_T *RingBuff, void *data, int num);
# 40 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h" 2
# 49 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
typedef struct {
 volatile uint32_t CFG;
 volatile uint32_t CTRL;
 volatile uint32_t STAT;
 volatile uint32_t INTENSET;
 volatile uint32_t INTENCLR;
 volatile const uint32_t RXDATA;
 volatile const uint32_t RXDATA_STAT;
 volatile uint32_t TXDATA;
 volatile uint32_t BRG;
 volatile uint32_t INTSTAT;
} LPC_USART_T;
# 126 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline void Chip_UART_Enable(LPC_USART_T *pUART)
{
 pUART->CFG |= (0x01 << 0);
}






static inline void Chip_UART_Disable(LPC_USART_T *pUART)
{
 pUART->CFG &= ~(0x01 << 0);
}






static inline void Chip_UART_TXEnable(LPC_USART_T *pUART)
{
 pUART->CTRL &= ~(0x01 << 6);
}






static inline void Chip_UART_TXDisable(LPC_USART_T *pUART)
{
 pUART->CTRL |= (0x01 << 6);
}
# 169 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline void Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data)
{
 pUART->TXDATA = (uint32_t) data;
}
# 182 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline uint32_t Chip_UART_ReadByte(LPC_USART_T *pUART)
{

 return (uint32_t) (pUART->RXDATA & 0x000001FF);
}
# 196 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline void Chip_UART_IntEnable(LPC_USART_T *pUART, uint32_t intMask)
{
 pUART->INTENSET = intMask;
}
# 209 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline void Chip_UART_IntDisable(LPC_USART_T *pUART, uint32_t intMask)
{
 pUART->INTENCLR = intMask;
}
# 222 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline uint32_t Chip_UART_GetIntsEnabled(LPC_USART_T *pUART)
{
 return pUART->INTENSET;
}
# 235 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline uint32_t Chip_UART_GetIntStatus(LPC_USART_T *pUART)
{
 return pUART->INTSTAT;
}
# 251 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline void Chip_UART_ConfigData(LPC_USART_T *pUART, uint32_t config)
{
 uint32_t reg;

 reg = pUART->CFG & ~((0x3 << 2) | (0x3 << 4) | (0x1 << 6));
 pUART->CFG = reg | config;
}
# 267 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline uint32_t Chip_UART_GetStatus(LPC_USART_T *pUART)
{
 return pUART->STAT;
}
# 281 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
static inline void Chip_UART_ClearStatus(LPC_USART_T *pUART, uint32_t stsMask)
{
 pUART->STAT = stsMask;
}






void Chip_UART_Init(LPC_USART_T *pUART);






void Chip_UART_DeInit(LPC_USART_T *pUART);
# 311 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
int Chip_UART_Send(LPC_USART_T *pUART, const void *data, int numBytes);
# 323 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
int Chip_UART_Read(LPC_USART_T *pUART, void *data, int numBytes);







void Chip_UART_SetBaud(LPC_USART_T *pUART, uint32_t baudrate);
# 342 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
int Chip_UART_SendBlocking(LPC_USART_T *pUART, const void *data, int numBytes);
# 354 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
int Chip_UART_ReadBlocking(LPC_USART_T *pUART, void *data, int numBytes);
# 365 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
void Chip_UART_RXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);
# 376 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
void Chip_UART_TXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);
# 389 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
uint32_t Chip_UART_SendRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int count);
# 402 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
int Chip_UART_ReadRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, void *data, int bytes);
# 414 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/uart_8xx.h"
void Chip_UART_IRQRBHandler(LPC_USART_T *pUART, RINGBUFF_T *pRXRB, RINGBUFF_T *pTXRB);
# 151 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wkt_8xx.h" 1
# 47 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wkt_8xx.h"
typedef struct {
 volatile uint32_t CTRL;
 uint32_t Reserved[2];
 volatile uint32_t COUNT;
} LPC_WKT_T;
# 63 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wkt_8xx.h"
typedef enum {
 WKT_CLKSRC_DIVIRC = 0,
 WKT_CLKSRC_10KHZ = 1
} WKT_CLKSRC_T;






static inline WKT_CLKSRC_T Chip_WKT_GetClockSource(LPC_WKT_T *pWKT)
{
 return (WKT_CLKSRC_T) (pWKT->CTRL & ((uint32_t) (1 << 0)));
}







void Chip_WKT_SetClockSource(LPC_WKT_T *pWKT, WKT_CLKSRC_T clkSrc);






uint32_t Chip_WKT_GetClockRate(LPC_WKT_T *pWKT);






static inline _Bool Chip_WKT_GetIntStatus(LPC_WKT_T *pWKT)
{
 return (_Bool) ((pWKT->CTRL & ((uint32_t) (1 << 1))) != 0);
}






static inline void Chip_WKT_ClearIntStatus(LPC_WKT_T *pWKT)
{
 pWKT->CTRL |= ((uint32_t) (1 << 1));
}






static inline void Chip_WKT_Stop(LPC_WKT_T *pWKT)
{
 pWKT->CTRL |= ((uint32_t) (1 << 2));
}
# 130 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wkt_8xx.h"
static inline void Chip_WKT_LoadCount(LPC_WKT_T *pWKT, uint32_t count)
{
 pWKT->COUNT = count;
}
# 143 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wkt_8xx.h"
void Chip_WKT_Start(LPC_WKT_T *pWKT, WKT_CLKSRC_T clkSrc, uint32_t cntVal);
# 152 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h" 1
# 50 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
typedef struct {
 volatile uint32_t MOD;
 volatile uint32_t TC;
 volatile uint32_t FEED;
 volatile const uint32_t TV;
 volatile const uint32_t RESERVED0;
 volatile uint32_t WARNINT;
 volatile uint32_t WINDOW;
} LPC_WWDT_T;
# 79 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
void Chip_WWDT_Init(LPC_WWDT_T *pWWDT);






void Chip_WWDT_DeInit(LPC_WWDT_T *pWWDT);







static inline void Chip_WWDT_SetTimeOut(LPC_WWDT_T *pWWDT, uint32_t timeout)
{
 pWWDT->TC = timeout;
}
# 106 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
static inline void Chip_WWDT_Feed(LPC_WWDT_T *pWWDT)
{
 pWWDT->FEED = 0xAA;
 pWWDT->FEED = 0x55;
}
# 120 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
static inline void Chip_WWDT_SetWarning(LPC_WWDT_T *pWWDT, uint32_t timeout)
{
 pWWDT->WARNINT = timeout;
}
# 134 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
static inline void Chip_WWDT_SetWindow(LPC_WWDT_T *pWWDT, uint32_t timeout)
{
 pWWDT->WINDOW = timeout;
}
# 150 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
static inline void Chip_WWDT_SetOption(LPC_WWDT_T *pWWDT, uint32_t options)
{
 pWWDT->MOD |= options;
}
# 164 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
static inline void Chip_WWDT_UnsetOption(LPC_WWDT_T *pWWDT, uint32_t options)
{
 pWWDT->MOD &= (~options) & ((uint32_t) 0x1F);
}






static inline void Chip_WWDT_Start(LPC_WWDT_T *pWWDT)
{
 Chip_WWDT_SetOption(pWWDT, ((uint32_t) (1 << 0)));
 Chip_WWDT_Feed(pWWDT);
}






static inline uint32_t Chip_WWDT_GetStatus(LPC_WWDT_T *pWWDT)
{
 return pWWDT->MOD;
}
# 198 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/wwdt_8xx.h"
void Chip_WWDT_ClearStatusFlag(LPC_WWDT_T *pWWDT, uint32_t status);






static inline uint32_t Chip_WWDT_GetCurrentCount(LPC_WWDT_T *pWWDT)
{
 return pWWDT->TV;
}
# 153 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h" 1
# 54 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h"
typedef struct {
 volatile uint32_t CONFIG;
 union {
  volatile uint32_t CTRL_U;
  struct {
   volatile uint16_t CTRL_L;
   volatile uint16_t CTRL_H;
  };
 };
 union {
  volatile uint32_t LIMIT;
  struct {
   volatile uint16_t LIMIT_L;
   volatile uint16_t LIMIT_H;
  };

 };

 union {
  volatile uint32_t HALT;
  struct {
   volatile uint16_t HALT_L;
   volatile uint16_t HALT_H;
  };

 };

 union {
  volatile uint32_t STOP;
  struct {
   volatile uint16_t STOP_L;
   volatile uint16_t STOP_H;
  };

 };

 union {
  volatile uint32_t START;
  struct {
   volatile uint16_t START_L;
   volatile uint16_t START_H;
  };

 };

 uint32_t RESERVED1[10];
 union {
  volatile uint32_t COUNT_U;
  struct {
   volatile uint16_t COUNT_L;
   volatile uint16_t COUNT_H;
  };

 };

 union {
  volatile uint32_t STATE;
  struct {
   volatile uint16_t STATE_L;
   volatile uint16_t STATE_H;
  };

 };

 volatile const uint32_t INPUT;
 union {
  volatile uint32_t REGMODE;
  struct {
   volatile uint16_t REGMODE_L;
   volatile uint16_t REGMODE_H;
  };

 };

 volatile uint32_t OUTPUT;
 volatile uint32_t OUTPUTDIRCTRL;
 volatile uint32_t RES;
 volatile uint32_t DMA0REQUEST;
 volatile uint32_t DMA1REQUEST;
 uint32_t RESERVED2[35];
 volatile uint32_t EVEN;
 volatile uint32_t EVFLAG;
 volatile uint32_t CONEN;
 volatile uint32_t CONFLAG;
 union {
  volatile union {
   uint32_t U;
   struct {
    uint16_t L;
    uint16_t H;
   };

  } MATCH[(16)];

  volatile const union {
   uint32_t U;
   struct {
    uint16_t L;
    uint16_t H;
   };

  } CAP[(16)];

 };

 uint32_t RESERVED3[32 - (16)];
 union {
  volatile uint16_t MATCH_L[(16)];
  volatile const uint16_t CAP_L[(16)];
 };

 uint16_t RESERVED4[32 - (16)];
 union {
  volatile uint16_t MATCH_H[(16)];
  volatile const uint16_t CAP_H[(16)];
 };

 uint16_t RESERVED5[32 - (16)];
 union {
  volatile union {
   uint32_t U;
   struct {
    uint16_t L;
    uint16_t H;
   };

  } MATCHREL[(16)];

  volatile union {
   uint32_t U;
   struct {
    uint16_t L;
    uint16_t H;
   };

  } CAPCTRL[(16)];

 };

 uint32_t RESERVED6[32 - (16)];
 union {
  volatile uint16_t MATCHREL_L[(16)];
  volatile uint16_t CAPCTRL_L[(16)];
 };

 uint16_t RESERVED7[32 - (16)];
 union {
  volatile uint16_t MATCHREL_H[(16)];
  volatile uint16_t CAPCTRL_H[(16)];
 };

 uint16_t RESERVED8[32 - (16)];
 volatile struct {
  uint32_t STATE;
  uint32_t CTRL;
 } EVENT[(16)];

 uint32_t RESERVED9[128 - 2 * (16)];
 volatile struct {
  uint32_t SET;
  uint32_t CLR;
 } OUT[(16)];

 uint32_t RESERVED10[191 - 2 * (16)];
 volatile const uint32_t MODULECONTENT;
} LPC_SCT_T;
# 266 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h"
typedef enum CHIP_SCT_MATCH_REG {
 SCT_MATCH_0 = 0,
 SCT_MATCH_1 = 1,
 SCT_MATCH_2 = 2,
 SCT_MATCH_3 = 3,
 SCT_MATCH_4 = 4
} CHIP_SCT_MATCH_REG_T;




typedef enum CHIP_SCT_EVENT {
 SCT_EVT_0 = (1 << 0),
 SCT_EVT_1 = (1 << 1),
 SCT_EVT_2 = (1 << 2),
 SCT_EVT_3 = (1 << 3),
 SCT_EVT_4 = (1 << 4)
} CHIP_SCT_EVENT_T;







static inline void Chip_SCT_Config(LPC_SCT_T *pSCT, uint32_t value)
{
 pSCT->CONFIG = value;
}
# 308 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h"
void Chip_SCT_SetClrControl(LPC_SCT_T *pSCT, uint32_t value, FunctionalState ena);
# 324 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h"
void Chip_SCT_SetConflictResolution(LPC_SCT_T *pSCT, uint8_t outnum, uint8_t value);







static inline void Chip_SCT_SetCount(LPC_SCT_T *pSCT, uint32_t count)
{
 pSCT->COUNT_U = count;
}







static inline void Chip_SCT_SetCountL(LPC_SCT_T *pSCT, uint16_t count)
{
 pSCT->COUNT_L = count;
}







static inline void Chip_SCT_SetCountH(LPC_SCT_T *pSCT, uint16_t count)
{
 pSCT->COUNT_H = count;
}
# 366 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h"
static inline void Chip_SCT_SetMatchCount(LPC_SCT_T *pSCT, CHIP_SCT_MATCH_REG_T n, uint32_t value)
{
 pSCT->MATCH[n].U = value;
}
# 378 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/sct_8xx.h"
static inline void Chip_SCT_SetMatchReload(LPC_SCT_T *pSCT, CHIP_SCT_MATCH_REG_T n, uint32_t value)
{
 pSCT->MATCHREL[n].U = value;
}







static inline void Chip_SCT_EnableEventInt(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
 pSCT->EVEN |= evt;
}







static inline void Chip_SCT_DisableEventInt(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
 pSCT->EVEN &= ~(evt);
}







static inline void Chip_SCT_ClearEventFlag(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
 pSCT->EVFLAG |= evt;
}







static inline void Chip_SCT_SetControl(LPC_SCT_T *pSCT, uint32_t value)
{
 pSCT->CTRL_U |= value;
}







static inline void Chip_SCT_ClearControl(LPC_SCT_T *pSCT, uint32_t value)
{
 pSCT->CTRL_U &= ~(value);
}






void Chip_SCT_Init(LPC_SCT_T *pSCT);






void Chip_SCT_DeInit(LPC_SCT_T *pSCT);
# 154 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h" 1
# 46 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
typedef struct {
 volatile uint32_t CFG;
 volatile uint32_t DLY;
 volatile uint32_t STAT;
 volatile uint32_t INTENSET;
 volatile uint32_t INTENCLR;
 volatile const uint32_t RXDAT;
 volatile uint32_t TXDATCTL;
 volatile uint32_t TXDAT;
 volatile uint32_t TXCTRL;
 volatile uint32_t DIV;
 volatile const uint32_t INTSTAT;
} LPC_SPI_T;
# 252 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
typedef enum {
 SPI_MODE_MASTER = ((uint32_t) (1 << 2)),
 SPI_MODE_SLAVE = ((uint32_t) 0),
} SPI_MODE_T;


typedef enum IP_SPI_CLOCK_MODE {
 SPI_CLOCK_CPHA0_CPOL0 = ((uint32_t) (0)) | ((uint32_t) (0)),
 SPI_CLOCK_CPHA0_CPOL1 = ((uint32_t) (1 << 5)) | ((uint32_t) (0)),
 SPI_CLOCK_CPHA1_CPOL0 = ((uint32_t) (0)) | ((uint32_t) (1 << 4)),
 SPI_CLOCK_CPHA1_CPOL1 = ((uint32_t) (1 << 5)) | ((uint32_t) (1 << 4)),
 SPI_CLOCK_MODE0 = SPI_CLOCK_CPHA0_CPOL0,
 SPI_CLOCK_MODE1 = SPI_CLOCK_CPHA1_CPOL0,
 SPI_CLOCK_MODE2 = SPI_CLOCK_CPHA0_CPOL1,
 SPI_CLOCK_MODE3 = SPI_CLOCK_CPHA1_CPOL1,
} SPI_CLOCK_MODE_T;


typedef enum IP_SPI_DATA_ORDER {
 SPI_DATA_MSB_FIRST = ((uint32_t) 0),
 SPI_DATA_LSB_FIRST = ((uint32_t) (1 << 3)),
} SPI_DATA_ORDER_T;


typedef enum IP_SPI_SSEL_POL {
 SPI_SSEL_ACTIVE_LO = ((uint32_t) (0)),
 SPI_SSEL_ACTIVE_HI = ((uint32_t) (1 << 8)),
} SPI_SSEL_POL_T;




typedef struct {
 SPI_MODE_T Mode;
 SPI_CLOCK_MODE_T ClockMode;
 SPI_DATA_ORDER_T DataOrder;
 SPI_SSEL_POL_T SSELPol;
 uint16_t ClkDiv;
} SPI_CONFIG_T;




typedef struct {
 uint8_t PreDelay;
 uint8_t PostDelay;
 uint8_t FrameDelay;
 uint8_t TransferDelay;
} SPI_DELAY_CONFIG_T;




typedef struct {
 uint16_t *pTx;
 uint32_t TxCnt;
 uint16_t *pRx;
 uint32_t RxCnt;
 uint32_t Length;
 uint16_t DataSize;
} SPI_DATA_SETUP_T;







void Chip_SPI_Init(LPC_SPI_T *pSPI, SPI_CONFIG_T *pConfig);







uint32_t Chip_SPI_CalClkRateDivider(LPC_SPI_T *pSPI, uint32_t bitRate);
# 337 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
void Chip_SPI_DelayConfig(LPC_SPI_T *pSPI, SPI_DELAY_CONFIG_T *pConfig);







void Chip_SPI_DeInit(LPC_SPI_T *pSPI);
# 354 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
void Chip_SPI_Int_Cmd(LPC_SPI_T *pSPI, uint32_t IntMask, FunctionalState NewState);






static inline void Chip_SPI_Enable(LPC_SPI_T *pSPI)
{
 pSPI->CFG |= ((uint32_t) (1 << 0));
}






static inline void Chip_SPI_Disable(LPC_SPI_T *pSPI)
{
 pSPI->CFG &= (~((uint32_t) (1 << 0))) & ((uint32_t) 0x1BD);
}
# 383 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_EnableLoopBack(LPC_SPI_T *pSPI)
{
 pSPI->CFG |= ((uint32_t) (1 << 7));
}
# 395 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_DisableLoopBack(LPC_SPI_T *pSPI)
{
 pSPI->CFG &= (~((uint32_t) (1 << 7))) & ((uint32_t) 0x1BD);
}






static inline uint32_t Chip_SPI_GetStatus(LPC_SPI_T *pSPI)
{
 return pSPI->STAT;
}







static inline void Chip_SPI_ClearStatus(LPC_SPI_T *pSPI, uint32_t Flag)
{
 pSPI->STAT |= Flag;
}
# 429 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_SetControlInfo(LPC_SPI_T *pSPI, uint8_t Flen, uint32_t Flag)
{
 pSPI->TXCTRL = Flag | ((uint32_t) (((Flen - 1) & 0x0F) << 24));
}
# 441 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_SendFirstFrame_RxIgnore(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
 pSPI->TXDATCTL = ((uint32_t) 0) | ((uint32_t) (1 << 21)) | ((uint32_t) (1 << 22)) | ((uint32_t) (((DataSize - 1) & 0x0F) << 24))
                | ((uint32_t) ((Data) & 0xFFFF));
}
# 454 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_SendFirstFrame(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
 pSPI->TXDATCTL = ((uint32_t) 0) | ((uint32_t) (1 << 21)) | ((uint32_t) (((DataSize - 1) & 0x0F) << 24)) | ((uint32_t) ((Data) & 0xFFFF))
       ;
}







static inline void Chip_SPI_SendMidFrame(LPC_SPI_T *pSPI, uint16_t Data)
{
 pSPI->TXDAT = ((uint32_t) ((Data) & 0xFFFF));
}
# 478 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_SendLastFrame_RxIgnore(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
 pSPI->TXDATCTL = ((uint32_t) 0) | ((uint32_t) (1 << 21)) | ((uint32_t) (1 << 20)) | ((uint32_t) (1 << 22)) |
      ((uint32_t) (((DataSize - 1) & 0x0F) << 24)) | ((uint32_t) ((Data) & 0xFFFF));
}
# 491 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
static inline void Chip_SPI_SendLastFrame(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
 pSPI->TXDATCTL = ((uint32_t) 0) | ((uint32_t) (1 << 21)) | ((uint32_t) (1 << 20)) |
      ((uint32_t) (((DataSize - 1) & 0x0F) << 24)) | ((uint32_t) ((Data) & 0xFFFF));
}






static inline uint16_t Chip_SPI_ReceiveFrame(LPC_SPI_T *pSPI)
{
 return ((uint32_t) ((pSPI->RXDAT) & 0xFFFF));
}
# 514 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
Status Chip_SPI_Int_RWFrames(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *xf_setup);
# 527 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
uint32_t Chip_SPI_RWFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);
# 539 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
uint32_t Chip_SPI_WriteFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);
# 551 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/spi_8xx.h"
uint32_t Chip_SPI_ReadFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);
# 155 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 1 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/i2c_8xx.h" 1
# 50 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/i2c_8xx.h"
void Chip_I2C_Init(void);






void Chip_I2C_DeInit(void);
# 156 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h" 2
# 165 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/inc/chip.h"
extern uint32_t SystemCoreClock;






void SystemCoreClockUpdate(void);







void Chip_SystemInit(void);
# 5 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/../stepper/workingramp_defs.h" 2


const uint32_t DIR = 7;
const uint32_t STEP_PIN = 16;
const uint32_t EN = 17;


const uint32_t AUTOLIMIT_L = (1 << 17);
const uint32_t COMBMODE_MATCH = (1 << 12);
const uint32_t DIRECTION_UP = (1 << 21);
const uint32_t DIRECTION_DOWN = (2 << 21);
const uint32_t OUTSEL_OUTPUT = (1 << 5);
const uint32_t MATCHSEL_1 = (SCT_MATCH_1 << 0);
# 31 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/../stepper/workingramp_defs.h"
const uint32_t STEPS_PER_REV = 400;
const uint32_t START_SPEED_PPS = 200;
const uint32_t FULL_SPEED_PPS = 2000;
const uint32_t ACCEL_MUX = 3;

enum {mENABLE, mDISABLE, mFORWARD, mREVERSE};
enum {ACCEL, RUN, DECEL, STOP};

struct
{
 uint32_t m_EN;
 uint32_t m_DIR;
 volatile uint32_t m_STATE;
 volatile uint32_t m_MOVESTEPS;
 volatile uint32_t m_MAXRUNSTEPS;
 volatile uint32_t m_MAXRAMPSTEPS;
 volatile uint32_t m_DECELSTEPS;
 volatile uint32_t m_SPEED;
}m_Vals;

volatile uint32_t STEP_steps, STEP_rampsteps;
volatile uint32_t ticker;
void STEP_setupSystem();
void STEP_setupMRT(uint32_t channel, uint32_t rate);
void STEP_setupSCT(uint32_t rate);
void STEP_start(uint32_t moves);
static __inline__ uint32_t SCT_uS(uint32_t us) {return (SystemCoreClock / 1000000) * us;}
static __inline__ uint32_t SCT_PPS(uint32_t pps) {return ((SystemCoreClock / pps) / 2);}
static __inline__ void STEP_ENABLE_DRIVER() {Chip_GPIO_SetPinOutLow(((LPC_GPIO_T *) (0xA0000000UL)), 0, EN); m_Vals.m_EN = mENABLE;}
static __inline__ void STEP_DISABLE_DRIVER(){Chip_GPIO_SetPinOutHigh(((LPC_GPIO_T *) (0xA0000000UL)), 0, EN); ; m_Vals.m_EN = mDISABLE;}
static __inline__ void STEP_FORWARD() {Chip_GPIO_SetPinOutLow(((LPC_GPIO_T *) (0xA0000000UL)), 0, DIR); m_Vals.m_DIR = mFORWARD;}
static __inline__ void STEP_REVERSE() {Chip_GPIO_SetPinOutHigh(((LPC_GPIO_T *) (0xA0000000UL)), 0, DIR); m_Vals.m_DIR = mREVERSE;}
static __inline__ void STEP_CHANGE_DIR() {Chip_GPIO_SetPinToggle(((LPC_GPIO_T *) (0xA0000000UL)), 0, DIR); m_Vals.m_DIR = (m_Vals.m_DIR == mREVERSE) ? mFORWARD : mREVERSE;}
# 2 "/home/drob/Documents/CrossWorks Projects/LPCOpen8xx/lpc_chip_8xx_lib/../stepper/workingramp.c" 2


void MRT_IRQHandler(void)
{
 switch (m_Vals.m_STATE)
 {
  case ACCEL:
  if (((STEP_rampsteps+ACCEL_MUX) >= m_Vals.m_MAXRAMPSTEPS) || (m_Vals.m_SPEED >= FULL_SPEED_PPS))
  {
   m_Vals.m_STATE = RUN;
   m_Vals.m_DECELSTEPS = STEP_rampsteps;
   m_Vals.m_MAXRUNSTEPS = m_Vals.m_MOVESTEPS - (STEP_rampsteps*2);
  }
  else m_Vals.m_SPEED += ACCEL_MUX;

  break;
  case DECEL:
  m_Vals.m_SPEED = (m_Vals.m_SPEED > START_SPEED_PPS) ? m_Vals.m_SPEED - ACCEL_MUX : START_SPEED_PPS;
  if (STEP_rampsteps >= m_Vals.m_DECELSTEPS-1)
  {
   m_Vals.m_STATE = STOP;
   Chip_SCT_SetControl(((LPC_SCT_T *) (0x50004000UL)), (1 << 2));
  }
  break;
  default:
  Chip_MRT_SetDisabled(((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[0]));
 }
 Chip_SCT_SetMatchReload(((LPC_SCT_T *) (0x50004000UL)), SCT_MATCH_0, SCT_PPS(m_Vals.m_SPEED));
 ((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG |= 1;
}

void SCT_IRQHandler(void)
{
 if (m_Vals.m_STATE == RUN)
 {
  if (STEP_steps >= m_Vals.m_MAXRUNSTEPS)
  {
   m_Vals.m_STATE = DECEL;
   Chip_MRT_SetEnabled(((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[0]));
   STEP_rampsteps = 0;
  }
  else STEP_steps++;
 }
 else if (m_Vals.m_STATE == ACCEL || m_Vals.m_STATE == DECEL) STEP_rampsteps++;

 Chip_SCT_ClearEventFlag(((LPC_SCT_T *) (0x50004000UL)), SCT_EVT_0);
}

void start(uint32_t moves)
{
 m_Vals.m_DIR = mFORWARD;
 m_Vals.m_EN = mENABLE;
 m_Vals.m_STATE = ACCEL;
 m_Vals.m_SPEED = START_SPEED_PPS;
 m_Vals.m_MOVESTEPS = moves;
 m_Vals.m_MAXRAMPSTEPS = (moves / 10)* 2;
 STEP_steps = 0;
 STEP_rampsteps = 0;
 Chip_SCT_SetMatchReload(((LPC_SCT_T *) (0x50004000UL)), SCT_MATCH_0, SCT_PPS(START_SPEED_PPS));
 Chip_MRT_SetEnabled(((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[0]));
 Chip_SCT_ClearControl(((LPC_SCT_T *) (0x50004000UL)), (1 << 2));
}

void setupSystem()
{
 SystemCoreClockUpdate();
 SysTick_Config(SystemCoreClock/1000);

 Chip_SWM_Init();
 Chip_SWM_MovablePinAssign(SWM_CTOUT_0_O, STEP_PIN);
 Chip_SWM_Deinit();

 Chip_GPIO_Init(((LPC_GPIO_T *) (0xA0000000UL)));
 Chip_GPIO_SetPinDIR(((LPC_GPIO_T *) (0xA0000000UL)), 0, DIR, 1);
 Chip_GPIO_SetPinDIR(((LPC_GPIO_T *) (0xA0000000UL)), 0, EN, 1);
 Chip_GPIO_SetPinOutHigh(((LPC_GPIO_T *) (0xA0000000UL)), 0, DIR);
 Chip_GPIO_SetPinOutHigh(((LPC_GPIO_T *) (0xA0000000UL)), 0, EN);

 NVIC_EnableIRQ(SysTick_IRQn);
 NVIC_EnableIRQ(MRT_IRQn);
 NVIC_EnableIRQ(SCT_IRQn);
}

void setupMRT(uint32_t channel, uint32_t rate)
{
 Chip_MRT_Init();
 Chip_MRT_SetInterval(((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[(channel)]), (((24000000UL / 1000000) * rate)));
 Chip_MRT_IntClear(((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[(channel)]));
}

void setupSCT(uint32_t rate)
{
 Chip_SCT_Init(((LPC_SCT_T *) (0x50004000UL)));
 ((LPC_SCT_T *) (0x50004000UL))->OUTPUT = 1;

 Chip_SCT_Config(((LPC_SCT_T *) (0x50004000UL)), (0x00000001 | AUTOLIMIT_L));
 Chip_SCT_SetControl(((LPC_SCT_T *) (0x50004000UL)), (((1) & 0x01) << 4));
 Chip_SCT_SetMatchCount(((LPC_SCT_T *) (0x50004000UL)), SCT_MATCH_0, rate);
 Chip_SCT_SetMatchCount(((LPC_SCT_T *) (0x50004000UL)), SCT_MATCH_1, 1);
 Chip_SCT_SetMatchReload(((LPC_SCT_T *) (0x50004000UL)), SCT_MATCH_0, rate);
 Chip_SCT_SetMatchReload(((LPC_SCT_T *) (0x50004000UL)), SCT_MATCH_1, 1);
 ((LPC_SCT_T *) (0x50004000UL))->EVENT[SCT_MATCH_0].STATE = 0xFFFFFFFF;
 ((LPC_SCT_T *) (0x50004000UL))->EVENT[SCT_MATCH_1].STATE = 0xFFFFFFFF;
 ((LPC_SCT_T *) (0x50004000UL))->EVENT[SCT_MATCH_0].CTRL = (COMBMODE_MATCH | DIRECTION_UP | OUTSEL_OUTPUT);
 ((LPC_SCT_T *) (0x50004000UL))->EVENT[SCT_MATCH_1].CTRL = (MATCHSEL_1 | COMBMODE_MATCH | DIRECTION_DOWN | OUTSEL_OUTPUT);
 ((LPC_SCT_T *) (0x50004000UL))->OUT[0].SET = SCT_EVT_1;
    ((LPC_SCT_T *) (0x50004000UL))->OUT[0].CLR = SCT_EVT_0;
 Chip_SCT_EnableEventInt(((LPC_SCT_T *) (0x50004000UL)), SCT_EVT_0);
 Chip_SCT_SetControl(((LPC_SCT_T *) (0x50004000UL)), (1 << 3));
}
