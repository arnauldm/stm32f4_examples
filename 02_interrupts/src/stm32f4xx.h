#ifndef __STM32F4xx_H
#define __STM32F4xx_H

#include <stdint.h>

#define FLASH_BASE            ((uint32_t)0x08000000)    /* FLASH(up to 1 MB) base address in the alias region */
#define SRAM1_BASE            ((uint32_t)0x20000000)    /* SRAM1(112 KB) base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000)    /* Peripheral base address in the alias region                                */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)

#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)

#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)



#define __NVIC_PRIO_BITS          4 /* STM32F4XX uses 4 Bits for the Priority Levels */

/*
 * STM32F4XX Interrupt Number Definition, according to the selected device
 */
typedef enum IRQn {
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
    NonMaskableInt_IRQn = -14,  /* 2 Non Maskable Interrupt */
    MemoryManagement_IRQn = -12,    /* 4 Cortex-M4 Memory Management Interrupt */
    BusFault_IRQn = -11,    /* 5 Cortex-M4 Bus Fault Interrupt */
    UsageFault_IRQn = -10,  /* 6 Cortex-M4 Usage Fault Interrupt */
    SVCall_IRQn = -5,       /* 11 Cortex-M4 SV Call Interrupt */
    DebugMonitor_IRQn = -4, /* 12 Cortex-M4 Debug Monitor Interrupt */
    PendSV_IRQn = -2,       /* 14 Cortex-M4 Pend SV Interrupt */
    SysTick_IRQn = -1,      /* 15 Cortex-M4 System Tick Interrupt */
/******  STM32 specific Interrupt Numbers **********************************************************************/
    WWDG_IRQn = 0,          /* Window WatchDog Interrupt */
    PVD_IRQn = 1,           /* PVD through EXTI Line detection Interrupt */
    TAMP_STAMP_IRQn = 2,    /* Tamper and TimeStamp interrupts through the EXTI line */
    RTC_WKUP_IRQn = 3,      /* RTC Wakeup interrupt through the EXTI line */
    FLASH_IRQn = 4,         /* FLASH global Interrupt */
    RCC_IRQn = 5,           /* RCC global Interrupt */
    EXTI0_IRQn = 6,         /* EXTI Line0 Interrupt */
    EXTI1_IRQn = 7,         /* EXTI Line1 Interrupt */
    EXTI2_IRQn = 8,         /* EXTI Line2 Interrupt */
    EXTI3_IRQn = 9,         /* EXTI Line3 Interrupt */
    EXTI4_IRQn = 10,        /* EXTI Line4 Interrupt */
    DMA1_Stream0_IRQn = 11, /* DMA1 Stream 0 global Interrupt */
    DMA1_Stream1_IRQn = 12, /* DMA1 Stream 1 global Interrupt */
    DMA1_Stream2_IRQn = 13, /* DMA1 Stream 2 global Interrupt */
    DMA1_Stream3_IRQn = 14, /* DMA1 Stream 3 global Interrupt */
    DMA1_Stream4_IRQn = 15, /* DMA1 Stream 4 global Interrupt */
    DMA1_Stream5_IRQn = 16, /* DMA1 Stream 5 global Interrupt */
    DMA1_Stream6_IRQn = 17, /* DMA1 Stream 6 global Interrupt */
    ADC_IRQn = 18,          /* ADC1, ADC2 and ADC3 global Interrupts */
    CAN1_TX_IRQn = 19,      /* CAN1 TX Interrupt */
    CAN1_RX0_IRQn = 20,     /* CAN1 RX0 Interrupt */
    CAN1_RX1_IRQn = 21,     /* CAN1 RX1 Interrupt */
    CAN1_SCE_IRQn = 22,     /* CAN1 SCE Interrupt */
    EXTI9_5_IRQn = 23,      /* External Line[9:5] Interrupts */
    TIM1_BRK_TIM9_IRQn = 24,    /* TIM1 Break interrupt and TIM9 global interrupt */
    TIM1_UP_TIM10_IRQn = 25,    /* TIM1 Update Interrupt and TIM10 global interrupt */
    TIM1_TRG_COM_TIM11_IRQn = 26,   /* TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
    TIM1_CC_IRQn = 27,      /* TIM1 Capture Compare Interrupt */
    TIM2_IRQn = 28,         /* TIM2 global Interrupt */
    TIM3_IRQn = 29,         /* TIM3 global Interrupt */
    TIM4_IRQn = 30,         /* TIM4 global Interrupt */
    I2C1_EV_IRQn = 31,      /* I2C1 Event Interrupt */
    I2C1_ER_IRQn = 32,      /* I2C1 Error Interrupt */
    I2C2_EV_IRQn = 33,      /* I2C2 Event Interrupt */
    I2C2_ER_IRQn = 34,      /* I2C2 Error Interrupt */
    SPI1_IRQn = 35,         /* SPI1 global Interrupt */
    SPI2_IRQn = 36,         /* SPI2 global Interrupt */
    USART1_IRQn = 37,       /* USART1 global Interrupt */
    USART2_IRQn = 38,       /* USART2 global Interrupt */
    USART3_IRQn = 39,       /* USART3 global Interrupt */
    EXTI15_10_IRQn = 40,    /* External Line[15:10] Interrupts */
    RTC_Alarm_IRQn = 41,    /* RTC Alarm (A and B) through EXTI Line Interrupt */
    OTG_FS_WKUP_IRQn = 42,  /* USB OTG FS Wakeup through EXTI line interrupt */
    TIM8_BRK_TIM12_IRQn = 43,   /* TIM8 Break Interrupt and TIM12 global interrupt */
    TIM8_UP_TIM13_IRQn = 44,    /* TIM8 Update Interrupt and TIM13 global interrupt */
    TIM8_TRG_COM_TIM14_IRQn = 45,   /* TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
    TIM8_CC_IRQn = 46,      /* TIM8 Capture Compare Interrupt */
    DMA1_Stream7_IRQn = 47, /* DMA1 Stream7 Interrupt */
    FSMC_IRQn = 48,         /* FSMC global Interrupt */
    SDIO_IRQn = 49,         /* SDIO global Interrupt */
    TIM5_IRQn = 50,         /* TIM5 global Interrupt */
    SPI3_IRQn = 51,         /* SPI3 global Interrupt */
    UART4_IRQn = 52,        /* UART4 global Interrupt */
    UART5_IRQn = 53,        /* UART5 global Interrupt */
    TIM6_DAC_IRQn = 54,     /* TIM6 global and DAC1&2 underrun error  interrupts */
    TIM7_IRQn = 55,         /* TIM7 global interrupt */
    DMA2_Stream0_IRQn = 56, /* DMA2 Stream 0 global Interrupt */
    DMA2_Stream1_IRQn = 57, /* DMA2 Stream 1 global Interrupt */
    DMA2_Stream2_IRQn = 58, /* DMA2 Stream 2 global Interrupt */
    DMA2_Stream3_IRQn = 59, /* DMA2 Stream 3 global Interrupt */
    DMA2_Stream4_IRQn = 60, /* DMA2 Stream 4 global Interrupt */
    ETH_IRQn = 61,          /* Ethernet global Interrupt */
    ETH_WKUP_IRQn = 62,     /* Ethernet Wakeup through EXTI line Interrupt */
    CAN2_TX_IRQn = 63,      /* CAN2 TX Interrupt */
    CAN2_RX0_IRQn = 64,     /* CAN2 RX0 Interrupt */
    CAN2_RX1_IRQn = 65,     /* CAN2 RX1 Interrupt */
    CAN2_SCE_IRQn = 66,     /* CAN2 SCE Interrupt */
    OTG_FS_IRQn = 67,       /* USB OTG FS global Interrupt */
    DMA2_Stream5_IRQn = 68, /* DMA2 Stream 5 global interrupt */
    DMA2_Stream6_IRQn = 69, /* DMA2 Stream 6 global interrupt */
    DMA2_Stream7_IRQn = 70, /* DMA2 Stream 7 global interrupt */
    USART6_IRQn = 71,       /* USART6 global interrupt */
    I2C3_EV_IRQn = 72,      /* I2C3 event interrupt */
    I2C3_ER_IRQn = 73,      /* I2C3 error interrupt */
    OTG_HS_EP1_OUT_IRQn = 74,   /* USB OTG HS End Point 1 Out global interrupt */
    OTG_HS_EP1_IN_IRQn = 75,    /* USB OTG HS End Point 1 In global interrupt */
    OTG_HS_WKUP_IRQn = 76,  /* USB OTG HS Wakeup through EXTI interrupt */
    OTG_HS_IRQn = 77,       /* USB OTG HS global interrupt */
    DCMI_IRQn = 78,         /* DCMI global interrupt */
    CRYP_IRQn = 79,         /* CRYP crypto global interrupt */
    HASH_RNG_IRQn = 80,     /* Hash and Rng global interrupt */
    FPU_IRQn = 81           /* FPU global interrupt */
} IRQn_Type;

#include "core_cm4.h" /* Cortex-M4 processor and core peripherals */


/******************************************************************************/
/*                         Reset and Clock Control                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)

#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)
#define  RCC_CR_HSITRIM_0                    ((uint32_t)0x00000008) /*Bit 0 */
#define  RCC_CR_HSITRIM_1                    ((uint32_t)0x00000010) /*Bit 1 */
#define  RCC_CR_HSITRIM_2                    ((uint32_t)0x00000020) /*Bit 2 */
#define  RCC_CR_HSITRIM_3                    ((uint32_t)0x00000040) /*Bit 3 */
#define  RCC_CR_HSITRIM_4                    ((uint32_t)0x00000080) /*Bit 4 */

#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)
#define  RCC_CR_HSICAL_0                     ((uint32_t)0x00000100) /*Bit 0 */
#define  RCC_CR_HSICAL_1                     ((uint32_t)0x00000200) /*Bit 1 */
#define  RCC_CR_HSICAL_2                     ((uint32_t)0x00000400) /*Bit 2 */
#define  RCC_CR_HSICAL_3                     ((uint32_t)0x00000800) /*Bit 3 */
#define  RCC_CR_HSICAL_4                     ((uint32_t)0x00001000) /*Bit 4 */
#define  RCC_CR_HSICAL_5                     ((uint32_t)0x00002000) /*Bit 5 */
#define  RCC_CR_HSICAL_6                     ((uint32_t)0x00004000) /*Bit 6 */
#define  RCC_CR_HSICAL_7                     ((uint32_t)0x00008000) /*Bit 7 */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)
#define  RCC_CR_PLLI2SON                     ((uint32_t)0x04000000)
#define  RCC_CR_PLLI2SRDY                    ((uint32_t)0x08000000)

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLM                    ((uint32_t)0x0000003F)
#define  RCC_PLLCFGR_PLLM_0                  ((uint32_t)0x00000001)
#define  RCC_PLLCFGR_PLLM_1                  ((uint32_t)0x00000002)
#define  RCC_PLLCFGR_PLLM_2                  ((uint32_t)0x00000004)
#define  RCC_PLLCFGR_PLLM_3                  ((uint32_t)0x00000008)
#define  RCC_PLLCFGR_PLLM_4                  ((uint32_t)0x00000010)
#define  RCC_PLLCFGR_PLLM_5                  ((uint32_t)0x00000020)

#define  RCC_PLLCFGR_PLLN                     ((uint32_t)0x00007FC0)
#define  RCC_PLLCFGR_PLLN_0                   ((uint32_t)0x00000040)
#define  RCC_PLLCFGR_PLLN_1                   ((uint32_t)0x00000080)
#define  RCC_PLLCFGR_PLLN_2                   ((uint32_t)0x00000100)
#define  RCC_PLLCFGR_PLLN_3                   ((uint32_t)0x00000200)
#define  RCC_PLLCFGR_PLLN_4                   ((uint32_t)0x00000400)
#define  RCC_PLLCFGR_PLLN_5                   ((uint32_t)0x00000800)
#define  RCC_PLLCFGR_PLLN_6                   ((uint32_t)0x00001000)
#define  RCC_PLLCFGR_PLLN_7                   ((uint32_t)0x00002000)
#define  RCC_PLLCFGR_PLLN_8                   ((uint32_t)0x00004000)

#define  RCC_PLLCFGR_PLLP                    ((uint32_t)0x00030000)
#define  RCC_PLLCFGR_PLLP_0                  ((uint32_t)0x00010000)
#define  RCC_PLLCFGR_PLLP_1                  ((uint32_t)0x00020000)

#define  RCC_PLLCFGR_PLLSRC                  ((uint32_t)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSE              ((uint32_t)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSI              ((uint32_t)0x00000000)

#define  RCC_PLLCFGR_PLLQ                    ((uint32_t)0x0F000000)
#define  RCC_PLLCFGR_PLLQ_0                  ((uint32_t)0x01000000)
#define  RCC_PLLCFGR_PLLQ_1                  ((uint32_t)0x02000000)
#define  RCC_PLLCFGR_PLLQ_2                  ((uint32_t)0x04000000)
#define  RCC_PLLCFGR_PLLQ_3                  ((uint32_t)0x08000000)

/********************  Bit definition for RCC_CFGR register  ******************/
/* SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003) /* SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001) /* Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002) /* Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000) /* HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001) /* HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002) /* PLL selected as system clock */

/* SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C) /* SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004) /* Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008) /* Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000) /* HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004) /* HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008) /* PLL used as system clock */

/* HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0) /* HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010) /* Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020) /* Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040) /* Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080) /* Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000) /* SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080) /* SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090) /* SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0) /* SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0) /* SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0) /* SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0) /* SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0) /* SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0) /* SYSCLK divided by 512 */

/* PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00001C00) /* PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000400) /* Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000800) /* Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00001000) /* Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000) /* HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00001000) /* HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00001400) /* HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00001800) /* HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00001C00) /* HCLK divided by 16 */

/* PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x0000E000) /* PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00002000) /* Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00004000) /* Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00008000) /* Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000) /* HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00008000) /* HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x0000A000) /* HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x0000C000) /* HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x0000E000) /* HCLK divided by 16 */

/* RTCPRE configuration */
#define  RCC_CFGR_RTCPRE                     ((uint32_t)0x001F0000)
#define  RCC_CFGR_RTCPRE_0                   ((uint32_t)0x00010000)
#define  RCC_CFGR_RTCPRE_1                   ((uint32_t)0x00020000)
#define  RCC_CFGR_RTCPRE_2                   ((uint32_t)0x00040000)
#define  RCC_CFGR_RTCPRE_3                   ((uint32_t)0x00080000)
#define  RCC_CFGR_RTCPRE_4                   ((uint32_t)0x00100000)

/* MCO1 configuration */
#define  RCC_CFGR_MCO1                       ((uint32_t)0x00600000)
#define  RCC_CFGR_MCO1_0                     ((uint32_t)0x00200000)
#define  RCC_CFGR_MCO1_1                     ((uint32_t)0x00400000)

#define  RCC_CFGR_I2SSRC                     ((uint32_t)0x00800000)

#define  RCC_CFGR_MCO1PRE                    ((uint32_t)0x07000000)
#define  RCC_CFGR_MCO1PRE_0                  ((uint32_t)0x01000000)
#define  RCC_CFGR_MCO1PRE_1                  ((uint32_t)0x02000000)
#define  RCC_CFGR_MCO1PRE_2                  ((uint32_t)0x04000000)

#define  RCC_CFGR_MCO2PRE                    ((uint32_t)0x38000000)
#define  RCC_CFGR_MCO2PRE_0                  ((uint32_t)0x08000000)
#define  RCC_CFGR_MCO2PRE_1                  ((uint32_t)0x10000000)
#define  RCC_CFGR_MCO2PRE_2                  ((uint32_t)0x20000000)

#define  RCC_CFGR_MCO2                       ((uint32_t)0xC0000000)
#define  RCC_CFGR_MCO2_0                     ((uint32_t)0x40000000)
#define  RCC_CFGR_MCO2_1                     ((uint32_t)0x80000000)

/********************  Bit definition for RCC_CIR register  *******************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)
#define  RCC_CIR_PLLI2SRDYF                  ((uint32_t)0x00000020)
#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)
#define  RCC_CIR_PLLI2SRDYIE                 ((uint32_t)0x00002000)
#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)
#define  RCC_CIR_PLLI2SRDYC                  ((uint32_t)0x00200000)
#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               ((uint32_t)0x00000001)
#define  RCC_AHB1RSTR_GPIOBRST               ((uint32_t)0x00000002)
#define  RCC_AHB1RSTR_GPIOCRST               ((uint32_t)0x00000004)
#define  RCC_AHB1RSTR_GPIODRST               ((uint32_t)0x00000008)
#define  RCC_AHB1RSTR_GPIOERST               ((uint32_t)0x00000010)
#define  RCC_AHB1RSTR_GPIOFRST               ((uint32_t)0x00000020)
#define  RCC_AHB1RSTR_GPIOGRST               ((uint32_t)0x00000040)
#define  RCC_AHB1RSTR_GPIOHRST               ((uint32_t)0x00000080)
#define  RCC_AHB1RSTR_GPIOIRST               ((uint32_t)0x00000100)
#define  RCC_AHB1RSTR_CRCRST                 ((uint32_t)0x00001000)
#define  RCC_AHB1RSTR_DMA1RST                ((uint32_t)0x00200000)
#define  RCC_AHB1RSTR_DMA2RST                ((uint32_t)0x00400000)
#define  RCC_AHB1RSTR_ETHMACRST              ((uint32_t)0x02000000)
#define  RCC_AHB1RSTR_OTGHRST                ((uint32_t)0x10000000)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_DCMIRST                ((uint32_t)0x00000001)
#define  RCC_AHB2RSTR_CRYPRST                ((uint32_t)0x00000010)
#define  RCC_AHB2RSTR_HSAHRST                ((uint32_t)0x00000020)
#define  RCC_AHB2RSTR_RNGRST                 ((uint32_t)0x00000040)
#define  RCC_AHB2RSTR_OTGFSRST               ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define  RCC_AHB3RSTR_FSMCRST                ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM2RST                ((uint32_t)0x00000001)
#define  RCC_APB1RSTR_TIM3RST                ((uint32_t)0x00000002)
#define  RCC_APB1RSTR_TIM4RST                ((uint32_t)0x00000004)
#define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)
#define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)
#define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)
#define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)
#define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)
#define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)
#define  RCC_APB1RSTR_WWDGEN                 ((uint32_t)0x00000800)
#define  RCC_APB1RSTR_SPI2RST                ((uint32_t)0x00008000)
#define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00010000)
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000)
#define  RCC_APB1RSTR_USART3RST              ((uint32_t)0x00040000)
#define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)
#define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000)
#define  RCC_APB1RSTR_I2C2RST                ((uint32_t)0x00400000)
#define  RCC_APB1RSTR_I2C3RST                ((uint32_t)0x00800000)
#define  RCC_APB1RSTR_CAN1RST                ((uint32_t)0x02000000)
#define  RCC_APB1RSTR_CAN2RST                ((uint32_t)0x04000000)
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000)
#define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000001)
#define  RCC_APB2RSTR_TIM8RST                ((uint32_t)0x00000002)
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00000010)
#define  RCC_APB2RSTR_USART6RST              ((uint32_t)0x00000020)
#define  RCC_APB2RSTR_ADCRST                 ((uint32_t)0x00000100)
#define  RCC_APB2RSTR_SDIORST                ((uint32_t)0x00000800)
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00004000)
#define  RCC_APB2RSTR_TIM9RST                ((uint32_t)0x00010000)
#define  RCC_APB2RSTR_TIM10RST               ((uint32_t)0x00020000)
#define  RCC_APB2RSTR_TIM11RST               ((uint32_t)0x00040000)
/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 ((uint32_t)0x00000001)
#define  RCC_AHB1ENR_GPIOBEN                 ((uint32_t)0x00000002)
#define  RCC_AHB1ENR_GPIOCEN                 ((uint32_t)0x00000004)
#define  RCC_AHB1ENR_GPIODEN                 ((uint32_t)0x00000008)
#define  RCC_AHB1ENR_GPIOEEN                 ((uint32_t)0x00000010)
#define  RCC_AHB1ENR_GPIOFEN                 ((uint32_t)0x00000020)
#define  RCC_AHB1ENR_GPIOGEN                 ((uint32_t)0x00000040)
#define  RCC_AHB1ENR_GPIOHEN                 ((uint32_t)0x00000080)
#define  RCC_AHB1ENR_GPIOIEN                 ((uint32_t)0x00000100)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000)
#define  RCC_AHB1ENR_BKPSRAMEN               ((uint32_t)0x00040000)
#define  RCC_AHB1ENR_CCMDATARAMEN            ((uint32_t)0x00100000)
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00200000)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00400000)
#define  RCC_AHB1ENR_ETHMACEN                ((uint32_t)0x02000000)
#define  RCC_AHB1ENR_ETHMACTXEN              ((uint32_t)0x04000000)
#define  RCC_AHB1ENR_ETHMACRXEN              ((uint32_t)0x08000000)
#define  RCC_AHB1ENR_ETHMACPTPEN             ((uint32_t)0x10000000)
#define  RCC_AHB1ENR_OTGHSEN                 ((uint32_t)0x20000000)
#define  RCC_AHB1ENR_OTGHSULPIEN             ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_DCMIEN                  ((uint32_t)0x00000001)
#define  RCC_AHB2ENR_CRYPEN                  ((uint32_t)0x00000010)
#define  RCC_AHB2ENR_HASHEN                  ((uint32_t)0x00000020)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00000040)
#define  RCC_AHB2ENR_OTGFSEN                 ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3ENR register  ***************/
#define  RCC_AHB3ENR_FSMCEN                  ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM2EN                  ((uint32_t)0x00000001)
#define  RCC_APB1ENR_TIM3EN                  ((uint32_t)0x00000002)
#define  RCC_APB1ENR_TIM4EN                  ((uint32_t)0x00000004)
#define  RCC_APB1ENR_TIM5EN                  ((uint32_t)0x00000008)
#define  RCC_APB1ENR_TIM6EN                  ((uint32_t)0x00000010)
#define  RCC_APB1ENR_TIM7EN                  ((uint32_t)0x00000020)
#define  RCC_APB1ENR_TIM12EN                 ((uint32_t)0x00000040)
#define  RCC_APB1ENR_TIM13EN                 ((uint32_t)0x00000080)
#define  RCC_APB1ENR_TIM14EN                 ((uint32_t)0x00000100)
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800)
#define  RCC_APB1ENR_SPI2EN                  ((uint32_t)0x00004000)
#define  RCC_APB1ENR_SPI3EN                  ((uint32_t)0x00008000)
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000)
#define  RCC_APB1ENR_USART3EN                ((uint32_t)0x00040000)
#define  RCC_APB1ENR_UART4EN                 ((uint32_t)0x00080000)
#define  RCC_APB1ENR_UART5EN                 ((uint32_t)0x00100000)
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000)
#define  RCC_APB1ENR_I2C2EN                  ((uint32_t)0x00400000)
#define  RCC_APB1ENR_I2C3EN                  ((uint32_t)0x00800000)
#define  RCC_APB1ENR_CAN1EN                  ((uint32_t)0x02000000)
#define  RCC_APB1ENR_CAN2EN                  ((uint32_t)0x04000000)
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000)
#define  RCC_APB1ENR_DACEN                   ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000001)
#define  RCC_APB2ENR_TIM8EN                  ((uint32_t)0x00000002)
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00000010)
#define  RCC_APB2ENR_USART6EN                ((uint32_t)0x00000020)
#define  RCC_APB2ENR_ADC1EN                  ((uint32_t)0x00000100)
#define  RCC_APB2ENR_ADC2EN                  ((uint32_t)0x00000200)
#define  RCC_APB2ENR_ADC3EN                  ((uint32_t)0x00000400)
#define  RCC_APB2ENR_SDIOEN                  ((uint32_t)0x00000800)
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00004000)
#define  RCC_APB2ENR_TIM11EN                 ((uint32_t)0x00040000)
#define  RCC_APB2ENR_TIM10EN                 ((uint32_t)0x00020000)
#define  RCC_APB2ENR_TIM9EN                  ((uint32_t)0x00010000)

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             ((uint32_t)0x00000001)
#define  RCC_AHB1LPENR_GPIOBLPEN             ((uint32_t)0x00000002)
#define  RCC_AHB1LPENR_GPIOCLPEN             ((uint32_t)0x00000004)
#define  RCC_AHB1LPENR_GPIODLPEN             ((uint32_t)0x00000008)
#define  RCC_AHB1LPENR_GPIOELPEN             ((uint32_t)0x00000010)
#define  RCC_AHB1LPENR_GPIOFLPEN             ((uint32_t)0x00000020)
#define  RCC_AHB1LPENR_GPIOGLPEN             ((uint32_t)0x00000040)
#define  RCC_AHB1LPENR_GPIOHLPEN             ((uint32_t)0x00000080)
#define  RCC_AHB1LPENR_GPIOILPEN             ((uint32_t)0x00000100)
#define  RCC_AHB1LPENR_CRCLPEN               ((uint32_t)0x00001000)
#define  RCC_AHB1LPENR_FLITFLPEN             ((uint32_t)0x00008000)
#define  RCC_AHB1LPENR_SRAM1LPEN             ((uint32_t)0x00010000)
#define  RCC_AHB1LPENR_SRAM2LPEN             ((uint32_t)0x00020000)
#define  RCC_AHB1LPENR_BKPSRAMLPEN           ((uint32_t)0x00040000)
#define  RCC_AHB1LPENR_DMA1LPEN              ((uint32_t)0x00200000)
#define  RCC_AHB1LPENR_DMA2LPEN              ((uint32_t)0x00400000)
#define  RCC_AHB1LPENR_ETHMACLPEN            ((uint32_t)0x02000000)
#define  RCC_AHB1LPENR_ETHMACTXLPEN          ((uint32_t)0x04000000)
#define  RCC_AHB1LPENR_ETHMACRXLPEN          ((uint32_t)0x08000000)
#define  RCC_AHB1LPENR_ETHMACPTPLPEN         ((uint32_t)0x10000000)
#define  RCC_AHB1LPENR_OTGHSLPEN             ((uint32_t)0x20000000)
#define  RCC_AHB1LPENR_OTGHSULPILPEN         ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define  RCC_AHB2LPENR_DCMILPEN              ((uint32_t)0x00000001)
#define  RCC_AHB2LPENR_CRYPLPEN              ((uint32_t)0x00000010)
#define  RCC_AHB2LPENR_HASHLPEN              ((uint32_t)0x00000020)
#define  RCC_AHB2LPENR_RNGLPEN               ((uint32_t)0x00000040)
#define  RCC_AHB2LPENR_OTGFSLPEN             ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define  RCC_AHB3LPENR_FSMCLPEN              ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM2LPEN              ((uint32_t)0x00000001)
#define  RCC_APB1LPENR_TIM3LPEN              ((uint32_t)0x00000002)
#define  RCC_APB1LPENR_TIM4LPEN              ((uint32_t)0x00000004)
#define  RCC_APB1LPENR_TIM5LPEN              ((uint32_t)0x00000008)
#define  RCC_APB1LPENR_TIM6LPEN              ((uint32_t)0x00000010)
#define  RCC_APB1LPENR_TIM7LPEN              ((uint32_t)0x00000020)
#define  RCC_APB1LPENR_TIM12LPEN             ((uint32_t)0x00000040)
#define  RCC_APB1LPENR_TIM13LPEN             ((uint32_t)0x00000080)
#define  RCC_APB1LPENR_TIM14LPEN             ((uint32_t)0x00000100)
#define  RCC_APB1LPENR_WWDGLPEN              ((uint32_t)0x00000800)
#define  RCC_APB1LPENR_SPI2LPEN              ((uint32_t)0x00004000)
#define  RCC_APB1LPENR_SPI3LPEN              ((uint32_t)0x00008000)
#define  RCC_APB1LPENR_USART2LPEN            ((uint32_t)0x00020000)
#define  RCC_APB1LPENR_USART3LPEN            ((uint32_t)0x00040000)
#define  RCC_APB1LPENR_UART4LPEN             ((uint32_t)0x00080000)
#define  RCC_APB1LPENR_UART5LPEN             ((uint32_t)0x00100000)
#define  RCC_APB1LPENR_I2C1LPEN              ((uint32_t)0x00200000)
#define  RCC_APB1LPENR_I2C2LPEN              ((uint32_t)0x00400000)
#define  RCC_APB1LPENR_I2C3LPEN              ((uint32_t)0x00800000)
#define  RCC_APB1LPENR_CAN1LPEN              ((uint32_t)0x02000000)
#define  RCC_APB1LPENR_CAN2LPEN              ((uint32_t)0x04000000)
#define  RCC_APB1LPENR_PWRLPEN               ((uint32_t)0x10000000)
#define  RCC_APB1LPENR_DACLPEN               ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              ((uint32_t)0x00000001)
#define  RCC_APB2LPENR_TIM8LPEN              ((uint32_t)0x00000002)
#define  RCC_APB2LPENR_USART1LPEN            ((uint32_t)0x00000010)
#define  RCC_APB2LPENR_USART6LPEN            ((uint32_t)0x00000020)
#define  RCC_APB2LPENR_ADC1LPEN              ((uint32_t)0x00000100)
#define  RCC_APB2LPENR_ADC2PEN               ((uint32_t)0x00000200)
#define  RCC_APB2LPENR_ADC3LPEN              ((uint32_t)0x00000400)
#define  RCC_APB2LPENR_SDIOLPEN              ((uint32_t)0x00000800)
#define  RCC_APB2LPENR_SPI1LPEN              ((uint32_t)0x00001000)
#define  RCC_APB2LPENR_SYSCFGLPEN            ((uint32_t)0x00004000)
#define  RCC_APB2LPENR_TIM9LPEN              ((uint32_t)0x00010000)
#define  RCC_APB2LPENR_TIM10LPEN             ((uint32_t)0x00020000)
#define  RCC_APB2LPENR_TIM11LPEN             ((uint32_t)0x00040000)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)

#define  RCC_BDCR_RTCSEL                    ((uint32_t)0x00000300)
#define  RCC_BDCR_RTCSEL_0                  ((uint32_t)0x00000100)
#define  RCC_BDCR_RTCSEL_1                  ((uint32_t)0x00000200)

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)
#define  RCC_CSR_BORRSTF                     ((uint32_t)0x02000000)
#define  RCC_CSR_PADRSTF                     ((uint32_t)0x04000000)
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)
#define  RCC_CSR_WDGRSTF                     ((uint32_t)0x20000000)
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_SSCGR register  *****************/
#define  RCC_SSCGR_MODPER                    ((uint32_t)0x00001FFF)
#define  RCC_SSCGR_INCSTEP                   ((uint32_t)0x0FFFE000)
#define  RCC_SSCGR_SPREADSEL                 ((uint32_t)0x40000000)
#define  RCC_SSCGR_SSCGEN                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define  RCC_PLLI2SCFGR_PLLI2SN              ((uint32_t)0x00007FC0)
#define  RCC_PLLI2SCFGR_PLLI2SR              ((uint32_t)0x70000000)

/******************************************************************************/
/*                                    FLASH                                   */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((uint32_t)0x00000007)
#define FLASH_ACR_LATENCY_0WS                ((uint32_t)0x00000000)
#define FLASH_ACR_LATENCY_1WS                ((uint32_t)0x00000001)
#define FLASH_ACR_LATENCY_2WS                ((uint32_t)0x00000002)
#define FLASH_ACR_LATENCY_3WS                ((uint32_t)0x00000003)
#define FLASH_ACR_LATENCY_4WS                ((uint32_t)0x00000004)
#define FLASH_ACR_LATENCY_5WS                ((uint32_t)0x00000005)
#define FLASH_ACR_LATENCY_6WS                ((uint32_t)0x00000006)
#define FLASH_ACR_LATENCY_7WS                ((uint32_t)0x00000007)

#define FLASH_ACR_PRFTEN                     ((uint32_t)0x00000100)
#define FLASH_ACR_ICEN                       ((uint32_t)0x00000200)
#define FLASH_ACR_DCEN                       ((uint32_t)0x00000400)
#define FLASH_ACR_ICRST                      ((uint32_t)0x00000800)
#define FLASH_ACR_DCRST                      ((uint32_t)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS              ((uint32_t)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS              ((uint32_t)0x40023C03)

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         ((uint32_t)0x00000001)
#define FLASH_SR_SOP                         ((uint32_t)0x00000002)
#define FLASH_SR_WRPERR                      ((uint32_t)0x00000010)
#define FLASH_SR_PGAERR                      ((uint32_t)0x00000020)
#define FLASH_SR_PGPERR                      ((uint32_t)0x00000040)
#define FLASH_SR_PGSERR                      ((uint32_t)0x00000080)
#define FLASH_SR_BSY                         ((uint32_t)0x00010000)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((uint32_t)0x00000001)
#define FLASH_CR_SER                         ((uint32_t)0x00000002)
#define FLASH_CR_MER                         ((uint32_t)0x00000004)
#define FLASH_CR_SNB_0                       ((uint32_t)0x00000008)
#define FLASH_CR_SNB_1                       ((uint32_t)0x00000010)
#define FLASH_CR_SNB_2                       ((uint32_t)0x00000020)
#define FLASH_CR_SNB_3                       ((uint32_t)0x00000040)
#define FLASH_CR_PSIZE_0                     ((uint32_t)0x00000100)
#define FLASH_CR_PSIZE_1                     ((uint32_t)0x00000200)
#define FLASH_CR_STRT                        ((uint32_t)0x00010000)
#define FLASH_CR_EOPIE                       ((uint32_t)0x01000000)
#define FLASH_CR_LOCK                        ((uint32_t)0x80000000)

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK                  ((uint32_t)0x00000001)
#define FLASH_OPTCR_OPTSTRT                  ((uint32_t)0x00000002)
#define FLASH_OPTCR_BOR_LEV_0                ((uint32_t)0x00000004)
#define FLASH_OPTCR_BOR_LEV_1                ((uint32_t)0x00000008)
#define FLASH_OPTCR_BOR_LEV                  ((uint32_t)0x0000000C)
#define FLASH_OPTCR_WDG_SW                   ((uint32_t)0x00000020)
#define FLASH_OPTCR_nRST_STOP                ((uint32_t)0x00000040)
#define FLASH_OPTCR_nRST_STDBY               ((uint32_t)0x00000080)
#define FLASH_OPTCR_RDP_0                    ((uint32_t)0x00000100)
#define FLASH_OPTCR_RDP_1                    ((uint32_t)0x00000200)
#define FLASH_OPTCR_RDP_2                    ((uint32_t)0x00000400)
#define FLASH_OPTCR_RDP_3                    ((uint32_t)0x00000800)
#define FLASH_OPTCR_RDP_4                    ((uint32_t)0x00001000)
#define FLASH_OPTCR_RDP_5                    ((uint32_t)0x00002000)
#define FLASH_OPTCR_RDP_6                    ((uint32_t)0x00004000)
#define FLASH_OPTCR_RDP_7                    ((uint32_t)0x00008000)
#define FLASH_OPTCR_nWRP_0                   ((uint32_t)0x00010000)
#define FLASH_OPTCR_nWRP_1                   ((uint32_t)0x00020000)
#define FLASH_OPTCR_nWRP_2                   ((uint32_t)0x00040000)
#define FLASH_OPTCR_nWRP_3                   ((uint32_t)0x00080000)
#define FLASH_OPTCR_nWRP_4                   ((uint32_t)0x00100000)
#define FLASH_OPTCR_nWRP_5                   ((uint32_t)0x00200000)
#define FLASH_OPTCR_nWRP_6                   ((uint32_t)0x00400000)
#define FLASH_OPTCR_nWRP_7                   ((uint32_t)0x00800000)
#define FLASH_OPTCR_nWRP_8                   ((uint32_t)0x01000000)
#define FLASH_OPTCR_nWRP_9                   ((uint32_t)0x02000000)
#define FLASH_OPTCR_nWRP_10                  ((uint32_t)0x04000000)
#define FLASH_OPTCR_nWRP_11                  ((uint32_t)0x08000000)

/******************************************************************************/
/*                             Power Control                                  */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((uint16_t)0x0001) /* Low-Power Deepsleep */
#define  PWR_CR_PDDS                         ((uint16_t)0x0002) /* Power Down Deepsleep */
#define  PWR_CR_CWUF                         ((uint16_t)0x0004) /* Clear Wakeup Flag */
#define  PWR_CR_CSBF                         ((uint16_t)0x0008) /* Clear Standby Flag */
#define  PWR_CR_PVDE                         ((uint16_t)0x0010) /* Power Voltage Detector Enable */

#define  PWR_CR_PLS                          ((uint16_t)0x00E0) /* PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((uint16_t)0x0020) /* Bit 0 */
#define  PWR_CR_PLS_1                        ((uint16_t)0x0040) /* Bit 1 */
#define  PWR_CR_PLS_2                        ((uint16_t)0x0080) /* Bit 2 */

/* PVD level configuration */
#define  PWR_CR_PLS_LEV0                     ((uint16_t)0x0000) /* PVD level 0 */
#define  PWR_CR_PLS_LEV1                     ((uint16_t)0x0020) /* PVD level 1 */
#define  PWR_CR_PLS_LEV2                     ((uint16_t)0x0040) /* PVD level 2 */
#define  PWR_CR_PLS_LEV3                     ((uint16_t)0x0060) /* PVD level 3 */
#define  PWR_CR_PLS_LEV4                     ((uint16_t)0x0080) /* PVD level 4 */
#define  PWR_CR_PLS_LEV5                     ((uint16_t)0x00A0) /* PVD level 5 */
#define  PWR_CR_PLS_LEV6                     ((uint16_t)0x00C0) /* PVD level 6 */
#define  PWR_CR_PLS_LEV7                     ((uint16_t)0x00E0) /* PVD level 7 */

#define  PWR_CR_DBP                          ((uint16_t)0x0100) /* Disable Backup Domain write protection */
#define  PWR_CR_FPDS                         ((uint16_t)0x0200) /* Flash power down in Stop mode */
#define  PWR_CR_VOS                          ((uint16_t)0x4000) /* Regulator voltage scaling output selection */
/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((uint16_t)0x0001) /* Wakeup Flag */
#define  PWR_CSR_SBF                         ((uint16_t)0x0002) /* Standby Flag */
#define  PWR_CSR_PVDO                        ((uint16_t)0x0004) /* PVD Output */
#define  PWR_CSR_BRR                         ((uint16_t)0x0008) /* Backup regulator ready */
#define  PWR_CSR_EWUP                        ((uint16_t)0x0100) /* Enable WKUP pin */
#define  PWR_CSR_BRE                         ((uint16_t)0x0200) /* Backup regulator enable */
#define  PWR_CSR_VOSRDY                      ((uint16_t)0x4000) /* Regulator voltage scaling output selection ready */
/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    ((uint16_t)0x0500)   /* Time out for HSE start up */
#endif                          /* HSE_STARTUP_TIMEOUT */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    ((uint32_t)16000000)   /* Value of the Internal oscillator in Hz */
#endif                          /* HSI_VALUE */

#if !defined  (HSE_VALUE)
#define HSE_VALUE    ((uint32_t)25000000)   /* Value of the External oscillator in Hz */
#endif                          /* HSE_VALUE */

typedef enum { RESET = 0, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;

/*
 * General Purpose I/O
 */
typedef struct {
    volatile uint32_t MODER;    /* GPIO port mode register,               Address offset: 0x00      */
    volatile uint32_t OTYPER;   /* GPIO port output type register,        Address offset: 0x04      */
    volatile uint32_t OSPEEDR;  /* GPIO port output speed register,       Address offset: 0x08      */
    volatile uint32_t PUPDR;    /* GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    volatile uint32_t IDR;      /* GPIO port input data register,         Address offset: 0x10      */
    volatile uint32_t ODR;      /* GPIO port output data register,        Address offset: 0x14      */
    volatile uint16_t BSRRL;    /* GPIO port bit set/reset low register,  Address offset: 0x18      */
    volatile uint16_t BSRRH;    /* GPIO port bit set/reset high register, Address offset: 0x1A      */
    volatile uint32_t LCKR;     /* GPIO port configuration lock register, Address offset: 0x1C      */
    volatile uint32_t AFR[2];   /* GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/*
 * System configuration controller
 */
typedef struct
{
  volatile uint32_t MEMRMP;       /* SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile uint32_t PMC;          /* SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile uint32_t EXTICR[4];    /* SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t          RESERVED[2];  /* Reserved, 0x18-0x1C                                                          */
  volatile uint32_t CMPCR;        /* SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;


/*
 * Reset and Clock Control
 */
typedef struct {
    volatile uint32_t CR;       /* RCC clock control register,                                  Address offset: 0x00 */
    volatile uint32_t PLLCFGR;  /* RCC PLL configuration register,                              Address offset: 0x04 */
    volatile uint32_t CFGR;     /* RCC clock configuration register,                            Address offset: 0x08 */
    volatile uint32_t CIR;      /* RCC clock interrupt register,                                Address offset: 0x0C */
    volatile uint32_t AHB1RSTR; /* RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    volatile uint32_t AHB2RSTR; /* RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    volatile uint32_t AHB3RSTR; /* RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
    uint32_t RESERVED0;     /* Reserved, 0x1C                                                                    */
    volatile uint32_t APB1RSTR; /* RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    volatile uint32_t APB2RSTR; /* RCC APB2 peripheral reset register,                          Address offset: 0x24 */
    uint32_t RESERVED1[2];  /* Reserved, 0x28-0x2C                                                               */
    volatile uint32_t AHB1ENR;  /* RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    volatile uint32_t AHB2ENR;  /* RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    volatile uint32_t AHB3ENR;  /* RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
    uint32_t RESERVED2;     /* Reserved, 0x3C                                                                    */
    volatile uint32_t APB1ENR;  /* RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    volatile uint32_t APB2ENR;  /* RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
    uint32_t RESERVED3[2];  /* Reserved, 0x48-0x4C                                                               */
    volatile uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    volatile uint32_t AHB2LPENR;    /* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    volatile uint32_t AHB3LPENR;    /* RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
    uint32_t RESERVED4;     /* Reserved, 0x5C                                                                    */
    volatile uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    volatile uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
    uint32_t RESERVED5[2];  /* Reserved, 0x68-0x6C                                                               */
    volatile uint32_t BDCR;     /* RCC Backup domain control register,                          Address offset: 0x70 */
    volatile uint32_t CSR;      /* RCC clock control & status register,                         Address offset: 0x74 */
    uint32_t RESERVED6[2];  /* Reserved, 0x78-0x7C                                                               */
    volatile uint32_t SSCGR;    /* RCC spread spectrum clock generation register,               Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;

/*
 * Power Control
 */
typedef struct {
    volatile uint32_t CR;       /* PWR power control register,        Address offset: 0x00 */
    volatile uint32_t CSR;      /* PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

/*
 * FLASH Registers
 */
typedef struct {
    volatile uint32_t ACR;      /* FLASH access control register, Address offset: 0x00 */
    volatile uint32_t KEYR;     /* FLASH key register,            Address offset: 0x04 */
    volatile uint32_t OPTKEYR;  /* FLASH option key register,     Address offset: 0x08 */
    volatile uint32_t SR;       /* FLASH status register,         Address offset: 0x0C */
    volatile uint32_t CR;       /* FLASH control register,        Address offset: 0x10 */
    volatile uint32_t OPTCR;    /* FLASH option control register, Address offset: 0x14 */
} FLASH_TypeDef;

/*
 * External Interrupt/Event Controller
 */

typedef struct
{
  volatile uint32_t IMR;    /* EXTI Interrupt mask register,            Address offset: 0x00 */
  volatile uint32_t EMR;    /* EXTI Event mask register,                Address offset: 0x04 */
  volatile uint32_t RTSR;   /* EXTI Rising trigger selection register,  Address offset: 0x08 */
  volatile uint32_t FTSR;   /* EXTI Falling trigger selection register, Address offset: 0x0C */
  volatile uint32_t SWIER;  /* EXTI Software interrupt event register,  Address offset: 0x10 */
  volatile uint32_t PR;     /* EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)


#endif                          /* __STM32F4xx_H */
