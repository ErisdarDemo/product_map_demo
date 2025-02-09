/**************************************************************************************************/
/** @file     main.c
 *  @brief    CubeMX RTOS 'r1'
 *  @details  Sample demonstration of control module
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  1/30/25
 *  @last rev 2/8/25
 *
 *
 *  @section    Opens
 *      feats vers
 *      ...!
 *
 *  @section    Hardware
 *      @open STM32F091RC
 *
 *  @section    Source Repository
 *      https://gitlab.com/justinmreina/cubemx_rtos_demo
 *
 *  @section    Legal Disclaimer
 *      © 2025 Justin Reina, All rights reserved. All contents of this source file and/or any other
 *      related source files are for public redistribution at this time.
 */
/**************************************************************************************************/


//************************************************************************************************//
//                                             INCLUDES                                           //
//************************************************************************************************//

//Standard Library Includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>


//************************************************************************************************//
//                                         STM32F446 FILL CONTENT                                 //
//************************************************************************************************//

#define GPIO_TypeDef        void
#define TIM_TypeDef         void
#define USART_TypeDef       void

#define GPIOA_BASE        (1)
#define GPIOB_BASE        (2)
#define GPIOC_BASE        (3)
#define GPIOD_BASE        (4)
#define GPIOE_BASE        (5)
#define GPIOF_BASE        (6)
#define GPIOG_BASE        (7)
#define GPIOH_BASE        (8)
#define TIM1_BASE         (9)
#define USART2_BASE       (10)
#define APB1PERIPH_BASE   (11)

//Pin Definitions
#define B1_Pin              (GPIO_PIN_13)
#define B1_GPIO_Port        (GPIOC)
#define USART_TX_Pin        (GPIO_PIN_2)
#define USART_TX_GPIO_Port  (GPIOA)
#define USART_RX_Pin        (GPIO_PIN_3)
#define USART_RX_GPIO_Port  (GPIOA)
#define LD2_Pin             (GPIO_PIN_5)
#define LD2_GPIO_Port       (GPIOA)
#define TMS_Pin             (GPIO_PIN_13)
#define TMS_GPIO_Port       (GPIOA)
#define TCK_Pin             (GPIO_PIN_14)
#define TCK_GPIO_Port       (GPIOA)
#define SWO_Pin             (GPIO_PIN_3)
#define SWO_GPIO_Port       (GPIOB)

#define TIM_SMCR_ETPS_Pos (1)
#define USART_CR1_RE_Msk  (2)

#define NULL              (0)

//Code Definitions
#define _nop()              asm("NOP")

//Design Specifications
#define DFLT_STACK_SIZE     (128 * 4)

//Task Definitions
#define SYS_TASK_NAME       "sysTask"
#define DATA_TASK_NAME      "dataTask"
#define DISP_TASK_NAME      "dispTask"
#define CTRL_TASK_NAME      "ctrlTask"

//Timer Definitions
#define OS_TIMER_NAME       "osTimer"

//Mutex Definitions
#define  DATA_MUTEX_NAME    "dataMutex"

//Semaphore Definitions
#define CTRL_SEM_NAME       "ctrlSem"
#define CTRL_MAX_CT         (1)
#define CTRL_INIT_CT        (1)

#define CNTR_SEM_NAME       "cntrSem"
#define CNTR_MAX_CT         (10)
#define CNTR_INIT_CT        (0)

//Event Definitions
#define DATA_EVENT_NAME     "dataStore"

/**
  * @brief  WWDG Init structure definition
  */
typedef struct
{
  uint32_t Prescaler;     /*!< Specifies the prescaler value of the WWDG.
                               This parameter can be a value of @ref WWDG_Prescaler */

  uint32_t Window;        /*!< Specifies the WWDG window value to be compared to the downcounter.
                               This parameter must be a number Min_Data = 0x40 and Max_Data = 0x7F */

  uint32_t Counter;       /*!< Specifies the WWDG free-running downcounter  value.
                               This parameter must be a number between Min_Data = 0x40 and Max_Data = 0x7F */

  uint32_t EWIMode ;      /*!< Specifies if WWDG Early Wakeup Interrupt is enable or not.
                               This parameter can be a value of @ref WWDG_EWI_Mode */

} WWDG_InitTypeDef;

/**
  * @brief Window WATCHDOG
  */
typedef struct
{
  uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

typedef struct
{
  WWDG_TypeDef      *Instance;  /*!< Register base address */

  WWDG_InitTypeDef  Init;       /*!< WWDG required parameters */

} WWDG_HandleTypeDef;


//************************************************************************************************//
//                                              PORTMACRO.H                                       //
//************************************************************************************************//

typedef unsigned long UBaseType_t;
typedef long BaseType_t;

/* Type definitions. */
#define portCHAR    char
#define portFLOAT   float
#define portDOUBLE    double
#define portLONG    long
#define portSHORT   short
#define portSTACK_TYPE  uint32_t
#define portBASE_TYPE long

typedef portSTACK_TYPE StackType_t;


//************************************************************************************************//
//                                               PROJDEFS.H                                       //
//************************************************************************************************//

#define pdFALSE     ( ( BaseType_t ) 0 )
#define pdTRUE      ( ( BaseType_t ) 1 )

#define pdPASS      ( pdTRUE )
#define pdFAIL      ( pdFALSE )


//************************************************************************************************//
//                                               TIMERS.H                                         //
//************************************************************************************************//

/**
 * Type by which software timers are referenced.  For example, a call to
 * xTimerCreate() returns an TimerHandle_t variable that can then be used to
 * reference the subject timer in calls to other software timer API functions
 * (for example, xTimerStart(), xTimerReset(), etc.).
 */
struct tmrTimerControl; /* The old naming convention is used to prevent breaking kernel aware debuggers. */
typedef struct tmrTimerControl * TimerHandle_t;


//************************************************************************************************//
//                                              QUEUE.H                                           //
//************************************************************************************************//

/**
 * Type by which queues are referenced.  For example, a call to xQueueCreate()
 * returns an QueueHandle_t variable that can then be used as a parameter to
 * xQueueSend(), xQueueReceive(), etc.
 */
struct QueueDefinition; /* Using old naming convention so as not to break kernel aware debuggers. */
typedef struct QueueDefinition * QueueHandle_t;


//************************************************************************************************//
//                                              SEMPHR.H                                          //
//************************************************************************************************//

typedef QueueHandle_t SemaphoreHandle_t;


//************************************************************************************************//
//                                               TASK.H                                           //
//************************************************************************************************//

/**
 * task. h
 *
 * Type by which tasks are referenced.  For example, a call to xTaskCreate
 * returns (via a pointer parameter) an TaskHandle_t variable that can then
 * be used as a parameter to vTaskDelete to delete the task.
 *
 * \defgroup TaskHandle_t TaskHandle_t
 * \ingroup Tasks
 */
struct tskTaskControlBlock; /* The old naming convention is used to prevent breaking kernel aware debuggers. */
typedef struct tskTaskControlBlock* TaskHandle_t;


//************************************************************************************************//
//                                           STM32F446XX.H                                        //
//************************************************************************************************//

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)

#define PWR_CR_VOS             PWR_CR_VOS_Msk                                  /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define PWR_CR_VOS_0           0x00004000U                                     /*!< Bit 0 */
#define PWR_CR_VOS_1           0x00008000U                                     /*!< Bit 1 */

#define RCC_PLLCFGR_PLLSRC_Pos             (22U)
#define RCC_PLLCFGR_PLLSRC_Msk             (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC                 RCC_PLLCFGR_PLLSRC_Msk
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk
#define RCC_PLLCFGR_PLLSRC_HSI             0x00000000U

#define RCC_CFGR_SW_HSI                    0x00000000U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000001U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */
#define RCC_CFGR_SW_PLLR                   0x00000003U                         /*!< PLL/PLLR selected as system clock */

#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

#define FLASH_ACR_LATENCY_Pos          (0U)
#define FLASH_ACR_LATENCY_Msk          (0xFUL << FLASH_ACR_LATENCY_Pos)         /*!< 0x0000000F */
#define FLASH_ACR_LATENCY              FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_0WS          0x00000000U
#define FLASH_ACR_LATENCY_1WS          0x00000001U
#define FLASH_ACR_LATENCY_2WS          0x00000002U
#define FLASH_ACR_LATENCY_3WS          0x00000003U
#define FLASH_ACR_LATENCY_4WS          0x00000004U
#define FLASH_ACR_LATENCY_5WS          0x00000005U
#define FLASH_ACR_LATENCY_6WS          0x00000006U
#define FLASH_ACR_LATENCY_7WS          0x00000007U

#define USART_CR1_RE                  USART_CR1_RE_Msk                         /*!<Receiver Enable                        */
#define USART_CR1_TE_Pos              (3U)
#define USART_CR1_TE_Msk              (0x1UL << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_TE                  USART_CR1_TE_Msk                         /*!<Transmitter Enable                     */

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)

#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)

#define TIM_SMCR_ETPS_0           (0x1UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x1000 */

typedef struct xSTATIC_MINI_LIST_ITEM StaticMiniListItem_t;

/**
  * @brief DMA Controller
  */
typedef struct
{
  uint32_t CR;     /*!< DMA stream x configuration register      */
  uint32_t NDTR;   /*!< DMA stream x number of data register     */
  uint32_t PAR;    /*!< DMA stream x peripheral address register */
  uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

/* See the comments above the struct xSTATIC_LIST_ITEM definition. */
typedef struct xSTATIC_LIST
{
  #if( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
    TickType_t xDummy1;
  #endif
  UBaseType_t uxDummy2;
  void *pvDummy3;
  StaticMiniListItem_t xDummy4;
  #if( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
    TickType_t xDummy5;
  #endif
} StaticList_t;


//************************************************************************************************//
//                                        STM32F4XX_HAL_PWR_EXH                                   //
//************************************************************************************************//

#define PWR_REGULATOR_VOLTAGE_SCALE1         PWR_CR_VOS             /* Scale 1 mode(default value at reset): the maximum value of fHCLK is 168 MHz. It can be extended to
                                                                       180 MHz by activating the over-drive mode. */
#define PWR_REGULATOR_VOLTAGE_SCALE2         PWR_CR_VOS_1           /* Scale 2 mode: the maximum value of fHCLK is 144 MHz. It can be extended to
                                                                       168 MHz by activating the over-drive mode. */
#define PWR_REGULATOR_VOLTAGE_SCALE3         PWR_CR_VOS_0           /* Scale 3 mode: the maximum value of fHCLK is 120 MHz. */


//************************************************************************************************//
//                                              FREERTOS.H                                        //
//************************************************************************************************//

typedef struct xSTATIC_LIST_ITEM StaticListItem_t;

typedef uint32_t TickType_t;

#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
#define configMAX_TASK_NAME_LEN                  ( 16 )

typedef struct xSTATIC_EVENT_GROUP
{
  TickType_t xDummy1;
  StaticList_t xDummy2;

  #if( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t uxDummy3;
  #endif

  #if( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
      uint8_t ucDummy4;
  #endif

} StaticEventGroup_t;

typedef struct xSTATIC_TCB
{
  void        *pxDummy1;
  #if ( portUSING_MPU_WRAPPERS == 1 )
    xMPU_SETTINGS xDummy2;
  #endif
  StaticListItem_t  xDummy3[ 2 ];
  UBaseType_t     uxDummy5;
  void        *pxDummy6;
  uint8_t       ucDummy7[ configMAX_TASK_NAME_LEN ];
  #if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
    void      *pxDummy8;
  #endif
  #if ( portCRITICAL_NESTING_IN_TCB == 1 )
    UBaseType_t   uxDummy9;
  #endif
  #if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t   uxDummy10[ 2 ];
  #endif
  #if ( configUSE_MUTEXES == 1 )
    UBaseType_t   uxDummy12[ 2 ];
  #endif
  #if ( configUSE_APPLICATION_TASK_TAG == 1 )
    void      *pxDummy14;
  #endif
  #if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
    void      *pvDummy15[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
  #endif
  #if ( configGENERATE_RUN_TIME_STATS == 1 )
    uint32_t    ulDummy16;
  #endif
  #if ( configUSE_NEWLIB_REENTRANT == 1 )
    struct  _reent  xDummy17;
  #endif
  #if ( configUSE_TASK_NOTIFICATIONS == 1 )
    uint32_t    ulDummy18;
    uint8_t     ucDummy19;
  #endif
  #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
    uint8_t     uxDummy20;
  #endif

  #if( INCLUDE_xTaskAbortDelay == 1 )
    uint8_t ucDummy21;
  #endif
  #if ( configUSE_POSIX_ERRNO == 1 )
    int       iDummy22;
  #endif
} StaticTask_t;

/*
 * Defines the prototype to which task functions must conform.  Defined in this
 * file to ensure the type is known before portable.h is included.
 */
typedef void (*TaskFunction_t)( void * );

typedef struct xSTATIC_LIST_ITEM StaticListItem_t;

typedef struct xSTATIC_TIMER
{
  void        *pvDummy1;
  StaticListItem_t  xDummy2;
  TickType_t      xDummy3;
  void        *pvDummy5;
  TaskFunction_t    pvDummy6;
  #if( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t   uxDummy7;
  #endif
  uint8_t       ucDummy8;

} StaticTimer_t;

typedef struct xSTATIC_QUEUE
{
  void *pvDummy1[ 3 ];

  union
  {
    void *pvDummy2;
    UBaseType_t uxDummy2;
  } u;

  StaticList_t xDummy3[ 2 ];
  UBaseType_t uxDummy4[ 3 ];
  uint8_t ucDummy5[ 2 ];

  #if( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
    uint8_t ucDummy6;
  #endif

  #if ( configUSE_QUEUE_SETS == 1 )
    void *pvDummy7;
  #endif

  #if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t uxDummy8;
    uint8_t ucDummy9;
  #endif

} StaticQueue_t;
typedef StaticQueue_t StaticSemaphore_t;


//************************************************************************************************//
//                                             CMSIS_OS2.H                                        //
//************************************************************************************************//

// Mutex attributes (attr_bits in \ref osMutexAttr_t).
#define osMutexRecursive      0x00000001U ///< Recursive mutex.
#define osMutexPrioInherit    0x00000002U ///< Priority inherit protocol.
#define osMutexRobust         0x00000008U ///< Robust mutex.


/// \details Thread ID identifies the thread.
typedef void *osThreadId_t;

/// \details Timer ID identifies the timer.
typedef void *osTimerId_t;

/// \details Event Flags ID identifies the event flags.
typedef void *osEventFlagsId_t;

/// \details Mutex ID identifies the mutex.
typedef void *osMutexId_t;

/// \details Semaphore ID identifies the semaphore.
typedef void *osSemaphoreId_t;

/// \details Memory Pool ID identifies the memory pool.
typedef void *osMemoryPoolId_t;

/// \details Message Queue ID identifies the message queue.
typedef void *osMessageQueueId_t;

typedef uint32_t TZ_ModuleId_t;

/// Priority values.
typedef enum {
   osPriorityNone          =  0,         ///< No priority (not initialized).
   osPriorityIdle          =  1,         ///< Reserved for Idle thread.
   osPriorityLow           =  8,         ///< Priority: low
   osPriorityLow1          =  8+1,       ///< Priority: low + 1
   osPriorityLow2          =  8+2,       ///< Priority: low + 2
   osPriorityLow3          =  8+3,       ///< Priority: low + 3
   osPriorityLow4          =  8+4,       ///< Priority: low + 4
   osPriorityLow5          =  8+5,       ///< Priority: low + 5
   osPriorityLow6          =  8+6,       ///< Priority: low + 6
   osPriorityLow7          =  8+7,       ///< Priority: low + 7
   osPriorityBelowNormal   = 16,         ///< Priority: below normal
   osPriorityBelowNormal1  = 16+1,       ///< Priority: below normal + 1
   osPriorityBelowNormal2  = 16+2,       ///< Priority: below normal + 2
   osPriorityBelowNormal3  = 16+3,       ///< Priority: below normal + 3
   osPriorityBelowNormal4  = 16+4,       ///< Priority: below normal + 4
   osPriorityBelowNormal5  = 16+5,       ///< Priority: below normal + 5
   osPriorityBelowNormal6  = 16+6,       ///< Priority: below normal + 6
   osPriorityBelowNormal7  = 16+7,       ///< Priority: below normal + 7
   osPriorityNormal        = 24,         ///< Priority: normal
   osPriorityNormal1       = 24+1,       ///< Priority: normal + 1
   osPriorityNormal2       = 24+2,       ///< Priority: normal + 2
   osPriorityNormal3       = 24+3,       ///< Priority: normal + 3
   osPriorityNormal4       = 24+4,       ///< Priority: normal + 4
   osPriorityNormal5       = 24+5,       ///< Priority: normal + 5
   osPriorityNormal6       = 24+6,       ///< Priority: normal + 6
   osPriorityNormal7       = 24+7,       ///< Priority: normal + 7
   osPriorityAboveNormal   = 32,         ///< Priority: above normal
   osPriorityAboveNormal1  = 32+1,       ///< Priority: above normal + 1
   osPriorityAboveNormal2  = 32+2,       ///< Priority: above normal + 2
   osPriorityAboveNormal3  = 32+3,       ///< Priority: above normal + 3
   osPriorityAboveNormal4  = 32+4,       ///< Priority: above normal + 4
   osPriorityAboveNormal5  = 32+5,       ///< Priority: above normal + 5
   osPriorityAboveNormal6  = 32+6,       ///< Priority: above normal + 6
   osPriorityAboveNormal7  = 32+7,       ///< Priority: above normal + 7
   osPriorityHigh          = 40,         ///< Priority: high
   osPriorityHigh1         = 40+1,       ///< Priority: high + 1
   osPriorityHigh2         = 40+2,       ///< Priority: high + 2
   osPriorityHigh3         = 40+3,       ///< Priority: high + 3
   osPriorityHigh4         = 40+4,       ///< Priority: high + 4
   osPriorityHigh5         = 40+5,       ///< Priority: high + 5
   osPriorityHigh6         = 40+6,       ///< Priority: high + 6
   osPriorityHigh7         = 40+7,       ///< Priority: high + 7
   osPriorityRealtime      = 48,         ///< Priority: realtime
   osPriorityRealtime1     = 48+1,       ///< Priority: realtime + 1
   osPriorityRealtime2     = 48+2,       ///< Priority: realtime + 2
   osPriorityRealtime3     = 48+3,       ///< Priority: realtime + 3
   osPriorityRealtime4     = 48+4,       ///< Priority: realtime + 4
   osPriorityRealtime5     = 48+5,       ///< Priority: realtime + 5
   osPriorityRealtime6     = 48+6,       ///< Priority: realtime + 6
   osPriorityRealtime7     = 48+7,       ///< Priority: realtime + 7
   osPriorityISR           = 56,         ///< Reserved for ISR deferred thread.
   osPriorityError         = -1,         ///< System cannot determine priority or illegal priority.
   osPriorityReserved      = 0x7FFFFFFF  ///< Prevents enum down-size compiler optimization.
} osPriority_t;

/// Attributes structure for thread.
typedef struct {
   const char                   *name;   ///< name of the thread
   uint32_t                 attr_bits;   ///< attribute bits
   void                      *cb_mem;    ///< memory for control block
   uint32_t                   cb_size;   ///< size of provided memory for control block
   void                   *stack_mem;    ///< memory for stack
   uint32_t                stack_size;   ///< size of stack
   osPriority_t              priority;   ///< initial thread priority (default: osPriorityNormal)
   TZ_ModuleId_t            tz_module;   ///< TrustZone module identifier
   uint32_t                  reserved;   ///< reserved (must be 0)
} osThreadAttr_t;

/// Attributes structure for timer.
typedef struct {
  const char                   *name;   ///< name of the timer
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osTimerAttr_t;

/// Attributes structure for event flags.
typedef struct {
  const char                   *name;   ///< name of the event flags
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osEventFlagsAttr_t;

/// Attributes structure for mutex.
typedef struct {
  const char                   *name;   ///< name of the mutex
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osMutexAttr_t;

/// Attributes structure for semaphore.
typedef struct {
  const char                   *name;   ///< name of the semaphore
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osSemaphoreAttr_t;

/// Timer type.
typedef enum {
  osTimerOnce               = 0,          ///< One-shot timer.
  osTimerPeriodic           = 1           ///< Repeating timer.
} osTimerType_t;

/// Status code values returned by CMSIS-RTOS functions.
typedef enum {
  osOK                      =  0,         ///< Operation completed successfully.
  osError                   = -1,         ///< Unspecified RTOS error: run-time error but no other error message fits.
  osErrorTimeout            = -2,         ///< Operation not completed within the timeout period.
  osErrorResource           = -3,         ///< Resource not available.
  osErrorParameter          = -4,         ///< Parameter error.
  osErrorNoMemory           = -5,         ///< System is out of memory: it was impossible to allocate or reserve memory for the operation.
  osErrorISR                = -6,         ///< Not allowed in ISR context: the function cannot be called from interrupt service routines.
  osStatusReserved          = 0x7FFFFFFF  ///< Prevents enum down-size compiler optimization.
} osStatus_t;

/// Kernel state.
typedef enum {
  osKernelInactive        =  0,         ///< Inactive.
  osKernelReady           =  1,         ///< Ready.
  osKernelRunning         =  2,         ///< Running.
  osKernelLocked          =  3,         ///< Locked.
  osKernelSuspended       =  4,         ///< Suspended.
  osKernelError           = -1,         ///< Error.
  osKernelReserved        = 0x7FFFFFFFU ///< Prevents enum down-size compiler optimization.
} osKernelState_t;

/// Timer callback function.
typedef void (*osTimerFunc_t) (void *argument);

/* Timer callback information structure definition */
typedef struct {
  osTimerFunc_t func;
  void         *arg;
} TimerCallback_t;

/* Kernel initialization state */
static osKernelState_t KernelState = osKernelInactive;

osStatus_t osKernelInitialize (void) {
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    if (KernelState == osKernelInactive) {
      #if defined(USE_TRACE_EVENT_RECORDER)
        EvrFreeRTOSSetup(0U);
      #endif
      #if defined(USE_FreeRTOS_HEAP_5) && (HEAP_5_REGION_SETUP == 1)
        vPortDefineHeapRegions (configHEAP_5_REGIONS);
      #endif
      KernelState = osKernelReady;
      stat = osOK;
    } else {
      stat = osError;
    }
  }

  return (stat);
}


osMutexId_t osMutexNew (const osMutexAttr_t *attr) {
  SemaphoreHandle_t hMutex;
  uint32_t type;
  uint32_t rmtx;
  int32_t  mem;
  #if (configQUEUE_REGISTRY_SIZE > 0)
  const char *name;
  #endif

  hMutex = NULL;

  if (!IS_IRQ()) {
    if (attr != NULL) {
      type = attr->attr_bits;
    } else {
      type = 0U;
    }

    if ((type & osMutexRecursive) == osMutexRecursive) {
      rmtx = 1U;
    } else {
      rmtx = 0U;
    }

    if ((type & osMutexRobust) != osMutexRobust) {
      mem = -1;

      if (attr != NULL) {
        if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticSemaphore_t))) {
          mem = 1;
        }
        else {
          if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
            mem = 0;
          }
        }
      }
      else {
        mem = 0;
      }

      if (mem == 1) {
        #if (configSUPPORT_STATIC_ALLOCATION == 1)
          if (rmtx != 0U) {
            #if (configUSE_RECURSIVE_MUTEXES == 1)
            hMutex = xSemaphoreCreateRecursiveMutexStatic (attr->cb_mem);
            #endif
          }
          else {
            hMutex = xSemaphoreCreateMutexStatic (attr->cb_mem);
          }
        #endif
      }
      else {
        if (mem == 0) {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            if (rmtx != 0U) {
              #if (configUSE_RECURSIVE_MUTEXES == 1)
              hMutex = xSemaphoreCreateRecursiveMutex ();
              #endif
            } else {
              hMutex = xSemaphoreCreateMutex ();
            }
          #endif
        }
      }

      #if (configQUEUE_REGISTRY_SIZE > 0)
      if (hMutex != NULL) {
        if (attr != NULL) {
          name = attr->name;
        } else {
          name = NULL;
        }
        vQueueAddToRegistry (hMutex, name);
      }
      #endif

      if ((hMutex != NULL) && (rmtx != 0U)) {
        hMutex = (SemaphoreHandle_t)((uint32_t)hMutex | 1U);
      }
    }
  }

  return ((osMutexId_t)hMutex);
}

osMutexId_t osMutexNew (const osMutexAttr_t *attr) {
  SemaphoreHandle_t hMutex;
  uint32_t type;
  uint32_t rmtx;
  int32_t  mem;
  #if (configQUEUE_REGISTRY_SIZE > 0)
  const char *name;
  #endif

  hMutex = NULL;

  if (!IS_IRQ()) {
    if (attr != NULL) {
      type = attr->attr_bits;
    } else {
      type = 0U;
    }

    if ((type & osMutexRecursive) == osMutexRecursive) {
      rmtx = 1U;
    } else {
      rmtx = 0U;
    }

    if ((type & osMutexRobust) != osMutexRobust) {
      mem = -1;

      if (attr != NULL) {
        if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticSemaphore_t))) {
          mem = 1;
        }
        else {
          if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
            mem = 0;
          }
        }
      }
      else {
        mem = 0;
      }

      if (mem == 1) {
        #if (configSUPPORT_STATIC_ALLOCATION == 1)
          if (rmtx != 0U) {
            #if (configUSE_RECURSIVE_MUTEXES == 1)
            hMutex = xSemaphoreCreateRecursiveMutexStatic (attr->cb_mem);
            #endif
          }
          else {
            hMutex = xSemaphoreCreateMutexStatic (attr->cb_mem);
          }
        #endif
      }
      else {
        if (mem == 0) {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            if (rmtx != 0U) {
              #if (configUSE_RECURSIVE_MUTEXES == 1)
              hMutex = xSemaphoreCreateRecursiveMutex ();
              #endif
            } else {
              hMutex = xSemaphoreCreateMutex ();
            }
          #endif
        }
      }

      #if (configQUEUE_REGISTRY_SIZE > 0)
      if (hMutex != NULL) {
        if (attr != NULL) {
          name = attr->name;
        } else {
          name = NULL;
        }
        vQueueAddToRegistry (hMutex, name);
      }
      #endif

      if ((hMutex != NULL) && (rmtx != 0U)) {
        hMutex = (SemaphoreHandle_t)((uint32_t)hMutex | 1U);
      }
    }
  }

  return ((osMutexId_t)hMutex);
}

osSemaphoreId_t osSemaphoreNew (uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr) {
  SemaphoreHandle_t hSemaphore;
  int32_t mem;
  #if (configQUEUE_REGISTRY_SIZE > 0)
  const char *name;
  #endif

  hSemaphore = NULL;

  if (!IS_IRQ() && (max_count > 0U) && (initial_count <= max_count)) {
    mem = -1;

    if (attr != NULL) {
      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticSemaphore_t))) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem != -1) {
      if (max_count == 1U) {
        if (mem == 1) {
          #if (configSUPPORT_STATIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateBinaryStatic ((StaticSemaphore_t *)attr->cb_mem);
          #endif
        }
        else {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateBinary();
          #endif
        }

        if ((hSemaphore != NULL) && (initial_count != 0U)) {
          if (xSemaphoreGive (hSemaphore) != pdPASS) {
            vSemaphoreDelete (hSemaphore);
            hSemaphore = NULL;
          }
        }
      }
      else {
        if (mem == 1) {
          #if (configSUPPORT_STATIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateCountingStatic (max_count, initial_count, (StaticSemaphore_t *)attr->cb_mem);
          #endif
        }
        else {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateCounting (max_count, initial_count);
          #endif
        }
      }

      #if (configQUEUE_REGISTRY_SIZE > 0)
      if (hSemaphore != NULL) {
        if (attr != NULL) {
          name = attr->name;
        } else {
          name = NULL;
        }
        vQueueAddToRegistry (hSemaphore, name);
      }
      #endif
    }
  }

  return ((osSemaphoreId_t)hSemaphore);
}

/// Timer callback function.
typedef void (*osTimerFunc_t) (void *argument);

osTimerId_t osTimerNew (osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr) {
  const char *name;
  TimerHandle_t hTimer;
  TimerCallback_t *callb;
  UBaseType_t reload;
  int32_t mem;

  hTimer = NULL;

  if (!IS_IRQ() && (func != NULL)) {
    /* Allocate memory to store callback function and argument */
    callb = pvPortMalloc (sizeof(TimerCallback_t));

    if (callb != NULL) {
      callb->func = func;
      callb->arg  = argument;

      if (type == osTimerOnce) {
        reload = pdFALSE;
      } else {
        reload = pdTRUE;
      }

      mem  = -1;
      name = NULL;

      if (attr != NULL) {
        if (attr->name != NULL) {
          name = attr->name;
        }

        if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticTimer_t))) {
          mem = 1;
        }
        else {
          if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
            mem = 0;
          }
        }
      }
      else {
        mem = 0;
      }

      if (mem == 1) {
        #if (configSUPPORT_STATIC_ALLOCATION == 1)
          hTimer = xTimerCreateStatic (name, 1, reload, callb, TimerCallback, (StaticTimer_t *)attr->cb_mem);
        #endif
      }
      else {
        if (mem == 0) {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            hTimer = xTimerCreate (name, 1, reload, callb, TimerCallback);
          #endif
        }
      }

      if ((hTimer == NULL) && (callb != NULL)) {
        vPortFree (callb);
      }
    }
  }

  return ((osTimerId_t)hTimer);
}

/// Entry point of a thread.
typedef void (*osThreadFunc_t) (void *argument);

#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)

// Thread attributes (attr_bits in \ref osThreadAttr_t).
#define osThreadDetached      0x00000000U ///< Thread created in detached mode (default)
#define osThreadJoinable      0x00000001U ///< Thread created in joinable mode


osThreadId_t osThreadNew (osThreadFunc_t func, void *argument, const osThreadAttr_t *attr) {
  const char *name;
  uint32_t stack;
  TaskHandle_t hTask;
  UBaseType_t prio;
  int32_t mem;

  hTask = NULL;

  if (!IS_IRQ() && (func != NULL)) {
    stack = configMINIMAL_STACK_SIZE;
    prio  = (UBaseType_t)osPriorityNormal;

    name = NULL;
    mem  = -1;

    if (attr != NULL) {
      if (attr->name != NULL) {
        name = attr->name;
      }
      if (attr->priority != osPriorityNone) {
        prio = (UBaseType_t)attr->priority;
      }

      if ((prio < osPriorityIdle) || (prio > osPriorityISR) || ((attr->attr_bits & osThreadJoinable) == osThreadJoinable)) {
        return (NULL);
      }

      if (attr->stack_size > 0U) {
        /* In FreeRTOS stack is not in bytes, but in sizeof(StackType_t) which is 4 on ARM ports.       */
        /* Stack size should be therefore 4 byte aligned in order to avoid division caused side effects */
        stack = attr->stack_size / sizeof(StackType_t);
      }

      if ((attr->cb_mem    != NULL) && (attr->cb_size    >= sizeof(StaticTask_t)) &&
          (attr->stack_mem != NULL) && (attr->stack_size >  0U)) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U) && (attr->stack_mem == NULL)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem == 1) {
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
        hTask = xTaskCreateStatic ((TaskFunction_t)func, name, stack, argument, prio, (StackType_t  *)attr->stack_mem,
                                                                                      (StaticTask_t *)attr->cb_mem);
      #endif
    }
    else {
      if (mem == 0) {
        #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
          if (xTaskCreate ((TaskFunction_t)func, name, (uint16_t)stack, argument, prio, &hTask) != pdPASS) {
            hTask = NULL;
          }
        #endif
      }
    }
  }

  return ((osThreadId_t)hTask);
}

//************************************************************************************************//
//                                          STM32F4XX_HAL_DEF.H                                   //
//************************************************************************************************//

typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

//************************************************************************************************//
//                                          STM32F4XX_HAL_DMA.H                                   //
//************************************************************************************************//

/**
  * @brief  HAL DMA State structures definition
  */
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled */
  HAL_DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use   */
  HAL_DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing              */
  HAL_DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                   */
  HAL_DMA_STATE_ERROR             = 0x04U,  /*!< DMA error state                     */
  HAL_DMA_STATE_ABORT             = 0x05U,  /*!< DMA Abort state                     */
}HAL_DMA_StateTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
  uint32_t Channel;              /*!< Specifies the channel used for the specified stream.
                                      This parameter can be a value of @ref DMA_Channel_selection                    */

  uint32_t Direction;            /*!< Specifies if the data will be transferred from memory to peripheral,
                                      from memory to memory or from peripheral to memory.
                                      This parameter can be a value of @ref DMA_Data_transfer_direction              */

  uint32_t PeriphInc;            /*!< Specifies whether the Peripheral address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Peripheral_incremented_mode          */

  uint32_t MemInc;               /*!< Specifies whether the memory address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Memory_incremented_mode              */

  uint32_t PeriphDataAlignment;  /*!< Specifies the Peripheral data width.
                                      This parameter can be a value of @ref DMA_Peripheral_data_size                 */

  uint32_t MemDataAlignment;     /*!< Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_Memory_data_size                     */

  uint32_t Mode;                 /*!< Specifies the operation mode of the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_mode
                                      @note The circular buffer mode cannot be used if the memory-to-memory
                                            data transfer is configured on the selected Stream                        */

  uint32_t Priority;             /*!< Specifies the software priority for the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_Priority_level                       */

  uint32_t FIFOMode;             /*!< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                      This parameter can be a value of @ref DMA_FIFO_direct_mode
                                      @note The Direct mode (FIFO mode disabled) cannot be used if the
                                            memory-to-memory data transfer is configured on the selected stream       */

  uint32_t FIFOThreshold;        /*!< Specifies the FIFO threshold level.
                                      This parameter can be a value of @ref DMA_FIFO_threshold_level                  */

  uint32_t MemBurst;             /*!< Specifies the Burst transfer configuration for the memory transfers.
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Memory_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled. */

  uint32_t PeriphBurst;          /*!< Specifies the Burst transfer configuration for the peripheral transfers.
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Peripheral_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled. */
} DMA_InitTypeDef;

/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                        /*!< Register base address                  */

  DMA_InitTypeDef            Init;                                                             /*!< DMA communication parameters           */

  HAL_LockTypeDef            Lock;                                                             /*!< DMA locking object                     */

  HAL_DMA_StateTypeDef  State;                                                                 /*!< DMA transfer state                     */

  void                       *Parent;                                                          /*!< Parent object state                    */

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);         /*!< DMA transfer complete callback         */

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA Half transfer complete callback    */

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);       /*!< DMA transfer complete Memory1 callback */

  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);   /*!< DMA transfer Half complete Memory1 callback */

  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);        /*!< DMA transfer error callback            */

  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);        /*!< DMA transfer Abort callback            */

  uint32_t                   ErrorCode;                                                        /*!< DMA Error code                          */

  uint32_t                   StreamBaseAddress;                                                /*!< DMA Stream Base Address                */

  uint32_t                   StreamIndex;                                                      /*!< DMA Stream Index                       */

}DMA_HandleTypeDef;

//************************************************************************************************//
//                                       STM32F4XX_HAL_RCC_EX.H                                   //
//************************************************************************************************//

/**
  * @brief  RCC PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432
                            except for STM32F411xE devices where the Min_Data = 192 */

  uint32_t PLLP;       /*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */

  uint32_t PLLQ;       /*!< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */
#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F446xx) || defined(STM32F469xx) ||\
    defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) ||\
    defined(STM32F413xx) || defined(STM32F423xx)
  uint32_t PLLR;       /*!< PLLR: PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks.
                            This parameter is only available in STM32F410xx/STM32F446xx/STM32F469xx/STM32F479xx
                            and STM32F412Zx/STM32F412Vx/STM32F412Rx/STM32F412Cx/STM32F413xx/STM32F423xx devices.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 7     */
#endif /* STM32F410xx || STM32F446xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx || STM32F413xx || STM32F423xx */
} RCC_PLLInitTypeDef;


//************************************************************************************************//
//                                         STM32F4XX_HAL_RCC.H                                    //
//************************************************************************************************//

#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U

#define RCC_HSI_OFF                      ((uint8_t)0x00)
#define RCC_HSI_ON                       ((uint8_t)0x01)

#define RCC_HSICALIBRATION_DEFAULT       0x10U         /* Default HSI calibration trimming value */

#define RCC_PLL_NONE                      ((uint8_t)0x00)
#define RCC_PLL_OFF                       ((uint8_t)0x01)
#define RCC_PLL_ON                        ((uint8_t)0x02)

#define RCC_PLLSOURCE_HSI                RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE                RCC_PLLCFGR_PLLSRC_HSE

#define RCC_PLLP_DIV2                  0x00000002U
#define RCC_PLLP_DIV4                  0x00000004U
#define RCC_PLLP_DIV6                  0x00000006U
#define RCC_PLLP_DIV8                  0x00000008U

#define RCC_CLOCKTYPE_SYSCLK             0x00000001U
#define RCC_CLOCKTYPE_HCLK               0x00000002U
#define RCC_CLOCKTYPE_PCLK1              0x00000004U
#define RCC_CLOCKTYPE_PCLK2              0x00000008U

#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL
#define RCC_SYSCLKSOURCE_PLLRCLK         ((uint32_t)(RCC_CFGR_SW_0 | RCC_CFGR_SW_1))

#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16

/** @defgroup RCC_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512

/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

} RCC_ClkInitTypeDef;


/**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */

  RCC_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
} RCC_OscInitTypeDef;



//************************************************************************************************//
//                                         STM32F4XX_HAL_TIM.H                                    //
//************************************************************************************************//

/** @defgroup TIM_Counter_Mode TIM Counter Mode
  * @{
  */
#define TIM_COUNTERMODE_UP                 0x00000000U                          /*!< Counter used as up-counter   */
#define TIM_COUNTERMODE_DOWN               TIM_CR1_DIR                          /*!< Counter used as down-counter */
#define TIM_COUNTERMODE_CENTERALIGNED1     TIM_CR1_CMS_0                        /*!< Center-aligned mode 1        */
#define TIM_COUNTERMODE_CENTERALIGNED2     TIM_CR1_CMS_1                        /*!< Center-aligned mode 2        */
#define TIM_COUNTERMODE_CENTERALIGNED3     TIM_CR1_CMS                          /*!< Center-aligned mode 3        */


/** @defgroup TIM_ClockDivision TIM Clock Division
  * @{
  */
#define TIM_CLOCKDIVISION_DIV1             0x00000000U                          /*!< Clock division: tDTS=tCK_INT   */
#define TIM_CLOCKDIVISION_DIV2             TIM_CR1_CKD_0                        /*!< Clock division: tDTS=2*tCK_INT */
#define TIM_CLOCKDIVISION_DIV4             TIM_CR1_CKD_1                        /*!< Clock division: tDTS=4*tCK_INT */

/** @defgroup TIM_AutoReloadPreload TIM Auto-Reload Preload
  * @{
  */
#define TIM_AUTORELOAD_PRELOAD_DISABLE                0x00000000U               /*!< TIMx_ARR register is not buffered */
#define TIM_AUTORELOAD_PRELOAD_ENABLE                 TIM_CR1_ARPE              /*!< TIMx_ARR register is buffered */

/** @defgroup TIM_Clock_Source TIM Clock Source
  * @{
  */
#define TIM_CLOCKSOURCE_INTERNAL    TIM_SMCR_ETPS_0      /*!< Internal clock source                                 */
#define TIM_CLOCKSOURCE_ETRMODE1    TIM_TS_ETRF          /*!< External clock source mode 1 (ETRF)                   */
#define TIM_CLOCKSOURCE_ETRMODE2    TIM_SMCR_ETPS_1      /*!< External clock source mode 2                          */
#define TIM_CLOCKSOURCE_TI1ED       TIM_TS_TI1F_ED       /*!< External clock source mode 1 (TTI1FP1 + edge detect.) */
#define TIM_CLOCKSOURCE_TI1         TIM_TS_TI1FP1        /*!< External clock source mode 1 (TTI1FP1)                */
#define TIM_CLOCKSOURCE_TI2         TIM_TS_TI2FP2        /*!< External clock source mode 1 (TTI2FP2)                */
#define TIM_CLOCKSOURCE_ITR0        TIM_TS_ITR0          /*!< External clock source mode 1 (ITR0)                   */
#define TIM_CLOCKSOURCE_ITR1        TIM_TS_ITR1          /*!< External clock source mode 1 (ITR1)                   */
#define TIM_CLOCKSOURCE_ITR2        TIM_TS_ITR2          /*!< External clock source mode 1 (ITR2)                   */
#define TIM_CLOCKSOURCE_ITR3        TIM_TS_ITR3          /*!< External clock source mode 1 (ITR3)                   */

/** @defgroup TIM_Master_Mode_Selection TIM Master Mode Selection
  * @{
  */
#define TIM_TRGO_RESET            0x00000000U                                      /*!< TIMx_EGR.UG bit is used as trigger output (TRGO)              */
#define TIM_TRGO_ENABLE           TIM_CR2_MMS_0                                    /*!< TIMx_CR1.CEN bit is used as trigger output (TRGO)             */
#define TIM_TRGO_UPDATE           TIM_CR2_MMS_1                                    /*!< Update event is used as trigger output (TRGO)                 */
#define TIM_TRGO_OC1              (TIM_CR2_MMS_1 | TIM_CR2_MMS_0)                  /*!< Capture or a compare match 1 is used as trigger output (TRGO) */
#define TIM_TRGO_OC1REF           TIM_CR2_MMS_2                                    /*!< OC1REF signal is used as trigger output (TRGO)                */
#define TIM_TRGO_OC2REF           (TIM_CR2_MMS_2 | TIM_CR2_MMS_0)                  /*!< OC2REF signal is used as trigger output(TRGO)                 */
#define TIM_TRGO_OC3REF           (TIM_CR2_MMS_2 | TIM_CR2_MMS_1)                  /*!< OC3REF signal is used as trigger output(TRGO)                 */
#define TIM_TRGO_OC4REF           (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0)  /*!< OC4REF signal is used as trigger output(TRGO)                 */

/** @defgroup TIM_Master_Slave_Mode TIM Master/Slave Mode
  * @{
  */
#define TIM_MASTERSLAVEMODE_ENABLE         TIM_SMCR_MSM                         /*!< No action */
#define TIM_MASTERSLAVEMODE_DISABLE        0x00000000U                          /*!< Master/slave mode is selected */

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  HAL_TIM_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  HAL_TIM_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
  HAL_TIM_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
  HAL_TIM_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                */
} HAL_TIM_StateTypeDef;

/**
  * @brief  TIM Channel States definition
  */
typedef enum
{
  HAL_TIM_CHANNEL_STATE_RESET             = 0x00U,    /*!< TIM Channel initial state                         */
  HAL_TIM_CHANNEL_STATE_READY             = 0x01U,    /*!< TIM Channel ready for use                         */
  HAL_TIM_CHANNEL_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing on the TIM channel */
} HAL_TIM_ChannelStateTypeDef;

/**
  * @brief  DMA Burst States definition
  */
typedef enum
{
  HAL_DMA_BURST_STATE_RESET             = 0x00U,    /*!< DMA Burst initial state */
  HAL_DMA_BURST_STATE_READY             = 0x01U,    /*!< DMA Burst ready for use */
  HAL_DMA_BURST_STATE_BUSY              = 0x02U,    /*!< Ongoing DMA Burst       */
} HAL_TIM_DMABurstStateTypeDef;

/**
  * @brief  Clock Configuration Handle Structure definition
  */
typedef struct
{
  uint32_t ClockSource;     /*!< TIM clock sources
                                 This parameter can be a value of @ref TIM_Clock_Source */
  uint32_t ClockPolarity;   /*!< TIM clock polarity
                                 This parameter can be a value of @ref TIM_Clock_Polarity */
  uint32_t ClockPrescaler;  /*!< TIM clock prescaler
                                 This parameter can be a value of @ref TIM_Clock_Prescaler */
  uint32_t ClockFilter;     /*!< TIM clock filter
                                 This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TIM_ClockConfigTypeDef;

/**
  * @brief  TIM Master configuration Structure definition
  */
typedef struct
{
  uint32_t  MasterOutputTrigger;   /*!< Trigger output (TRGO) selection
                                        This parameter can be a value of @ref TIM_Master_Mode_Selection */
  uint32_t  MasterSlaveMode;       /*!< Master/slave mode selection
                                        This parameter can be a value of @ref TIM_Master_Slave_Mode
                                        @note When the Master/slave mode is enabled, the effect of
                                        an event on the trigger input (TRGI) is delayed to allow a
                                        perfect synchronization between the current timer and its
                                        slaves (through TRGO). It is not mandatory in case of timer
                                        synchronization mode. */
} TIM_MasterConfigTypeDef;

/**
  * @brief  HAL Active channel structures definition
  */
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,    /*!< The active channel is 1     */
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,    /*!< The active channel is 2     */
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,    /*!< The active channel is 3     */
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,    /*!< The active channel is 4     */
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U     /*!< All active channels cleared */
} HAL_TIM_ActiveChannel;

/**
  * @brief  TIM Time base Configuration Structure definition
  */
typedef struct
{
  uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref TIM_Counter_Mode */

  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */

  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref TIM_ClockDivision */

  uint32_t RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     GP timers: this parameter must be a number between Min_Data = 0x00 and
                                     Max_Data = 0xFF.
                                     Advanced timers: this parameter must be a number between Min_Data = 0x0000 and
                                     Max_Data = 0xFFFF. */

  uint32_t AutoReloadPreload;  /*!< Specifies the auto-reload preload.
                                   This parameter can be a value of @ref TIM_AutoReloadPreload */
} TIM_Base_InitTypeDef;

/**
  * @brief  TIM Time Base Handle Structure definition
  */
#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
typedef struct __TIM_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
{
  TIM_TypeDef                        *Instance;         /*!< Register base address                             */
  TIM_Base_InitTypeDef               Init;              /*!< TIM Time Base required parameters                 */
  HAL_TIM_ActiveChannel              Channel;           /*!< Active channel                                    */
  DMA_HandleTypeDef                  *hdma[7];          /*!< DMA Handlers array
                                                             This array is accessed by a @ref DMA_Handle_index */
  HAL_LockTypeDef                    Lock;              /*!< Locking object                                    */
  HAL_TIM_StateTypeDef          State;                  /*!< TIM operation state                               */
  HAL_TIM_ChannelStateTypeDef   ChannelState[4];        /*!< TIM channel operation state                       */
  HAL_TIM_ChannelStateTypeDef   ChannelNState[4];       /*!< TIM complementary channel operation state         */
  HAL_TIM_DMABurstStateTypeDef  DMABurstState;          /*!< DMA burst operation state                         */

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  void (* Base_MspInitCallback)(struct __TIM_HandleTypeDef *htim);              /*!< TIM Base Msp Init Callback                              */
  void (* Base_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);            /*!< TIM Base Msp DeInit Callback                            */
  void (* IC_MspInitCallback)(struct __TIM_HandleTypeDef *htim);                /*!< TIM IC Msp Init Callback                                */
  void (* IC_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);              /*!< TIM IC Msp DeInit Callback                              */
  void (* OC_MspInitCallback)(struct __TIM_HandleTypeDef *htim);                /*!< TIM OC Msp Init Callback                                */
  void (* OC_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);              /*!< TIM OC Msp DeInit Callback                              */
  void (* PWM_MspInitCallback)(struct __TIM_HandleTypeDef *htim);               /*!< TIM PWM Msp Init Callback                               */
  void (* PWM_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);             /*!< TIM PWM Msp DeInit Callback                             */
  void (* OnePulse_MspInitCallback)(struct __TIM_HandleTypeDef *htim);          /*!< TIM One Pulse Msp Init Callback                         */
  void (* OnePulse_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);        /*!< TIM One Pulse Msp DeInit Callback                       */
  void (* Encoder_MspInitCallback)(struct __TIM_HandleTypeDef *htim);           /*!< TIM Encoder Msp Init Callback                           */
  void (* Encoder_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);         /*!< TIM Encoder Msp DeInit Callback                         */
  void (* HallSensor_MspInitCallback)(struct __TIM_HandleTypeDef *htim);        /*!< TIM Hall Sensor Msp Init Callback                       */
  void (* HallSensor_MspDeInitCallback)(struct __TIM_HandleTypeDef *htim);      /*!< TIM Hall Sensor Msp DeInit Callback                     */
  void (* PeriodElapsedCallback)(struct __TIM_HandleTypeDef *htim);             /*!< TIM Period Elapsed Callback                             */
  void (* PeriodElapsedHalfCpltCallback)(struct __TIM_HandleTypeDef *htim);     /*!< TIM Period Elapsed half complete Callback               */
  void (* TriggerCallback)(struct __TIM_HandleTypeDef *htim);                   /*!< TIM Trigger Callback                                    */
  void (* TriggerHalfCpltCallback)(struct __TIM_HandleTypeDef *htim);           /*!< TIM Trigger half complete Callback                      */
  void (* IC_CaptureCallback)(struct __TIM_HandleTypeDef *htim);                /*!< TIM Input Capture Callback                              */
  void (* IC_CaptureHalfCpltCallback)(struct __TIM_HandleTypeDef *htim);        /*!< TIM Input Capture half complete Callback                */
  void (* OC_DelayElapsedCallback)(struct __TIM_HandleTypeDef *htim);           /*!< TIM Output Compare Delay Elapsed Callback               */
  void (* PWM_PulseFinishedCallback)(struct __TIM_HandleTypeDef *htim);         /*!< TIM PWM Pulse Finished Callback                         */
  void (* PWM_PulseFinishedHalfCpltCallback)(struct __TIM_HandleTypeDef *htim); /*!< TIM PWM Pulse Finished half complete Callback           */
  void (* ErrorCallback)(struct __TIM_HandleTypeDef *htim);                     /*!< TIM Error Callback                                      */
  void (* CommutationCallback)(struct __TIM_HandleTypeDef *htim);               /*!< TIM Commutation Callback                                */
  void (* CommutationHalfCpltCallback)(struct __TIM_HandleTypeDef *htim);       /*!< TIM Commutation half complete Callback                  */
  void (* BreakCallback)(struct __TIM_HandleTypeDef *htim);                     /*!< TIM Break Callback                                      */
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
} TIM_HandleTypeDef;


//************************************************************************************************//
//                                         STM32F4XX_HAL_UART.H                                   //
//************************************************************************************************//

/** @defgroup UART_Word_Length UART Word Length
  * @{
  */
#define UART_WORDLENGTH_8B                  0x00000000U
#define UART_WORDLENGTH_9B                  ((uint32_t)USART_CR1_M)

/** @defgroup UART_Stop_Bits UART Number of Stop Bits
  * @{
  */
#define UART_STOPBITS_1                     0x00000000U
#define UART_STOPBITS_2                     ((uint32_t)USART_CR2_STOP_1)
/**
  * @}
  */

/** @defgroup UART_Parity UART Parity
  * @{
  */
#define UART_PARITY_NONE                    0x00000000U
#define UART_PARITY_EVEN                    ((uint32_t)USART_CR1_PCE)
#define UART_PARITY_ODD                     ((uint32_t)(USART_CR1_PCE | USART_CR1_PS))

/** @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
  * @{
  */
#define UART_HWCONTROL_NONE                  0x00000000U
#define UART_HWCONTROL_RTS                   ((uint32_t)USART_CR3_RTSE)
#define UART_HWCONTROL_CTS                   ((uint32_t)USART_CR3_CTSE)
#define UART_HWCONTROL_RTS_CTS               ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))
/**
  * @}
  */

/** @defgroup UART_Mode UART Transfer Mode
  * @{
  */
#define UART_MODE_RX                        ((uint32_t)USART_CR1_RE)
#define UART_MODE_TX                        ((uint32_t)USART_CR1_TE)
#define UART_MODE_TX_RX                     ((uint32_t)(USART_CR1_TE | USART_CR1_RE))

/** @defgroup UART_Over_Sampling UART Over Sampling
  * @{
  */
#define UART_OVERSAMPLING_16                    0x00000000U
#define UART_OVERSAMPLING_8                     ((uint32_t)USART_CR1_OVER8)

typedef uint32_t HAL_UART_RxTypeTypeDef;
typedef uint32_t HAL_UART_RxEventTypeTypeDef;

/**
  * @brief UART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (huart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8 * (OVR8+1)) + 0.5
                                           Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode */

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control */

  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                           This parameter can be a value of @ref UART_Over_Sampling */
} UART_InitTypeDef;

/**
  * @brief HAL UART State structures definition
  * @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
  */
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  HAL_UART_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  HAL_UART_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  HAL_UART_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  HAL_UART_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  HAL_UART_STATE_ERROR             = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} HAL_UART_StateTypeDef;


/**
  * @brief  UART handle Structure definition
  */
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;        /*!< UART registers base address        */

  UART_InitTypeDef              Init;             /*!< UART communication parameters      */

  const uint8_t                 *pTxBuffPtr;      /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                      TxXferSize;       /*!< UART Tx Transfer size              */

  uint16_t                      TxXferCount;      /*!< UART Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< UART Rx Transfer size              */

  uint16_t                      RxXferCount;      /*!< UART Rx Transfer Counter           */

  HAL_UART_RxTypeTypeDef      ReceptionType;      /*!< Type of ongoing reception          */

  HAL_UART_RxEventTypeTypeDef      RxEventType;   /*!< Type of Rx Event                   */

  DMA_HandleTypeDef             *hdmatx;          /*!< UART Tx DMA Handle parameters      */

  DMA_HandleTypeDef             *hdmarx;          /*!< UART Rx DMA Handle parameters      */

  HAL_LockTypeDef               Lock;             /*!< Locking object                     */

  HAL_UART_StateTypeDef    gState;                /*!< UART state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref HAL_UART_StateTypeDef */

  HAL_UART_StateTypeDef    RxState;               /*!< UART state information related to Rx operations.
                                                       This parameter can be a value of @ref HAL_UART_StateTypeDef */

  uint32_t                 ErrorCode;            /*!< UART Error code                    */

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  void (* TxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Tx Half Complete Callback        */
  void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Tx Complete Callback             */
  void (* RxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Half Complete Callback        */
  void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Rx Complete Callback             */
  void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);             /*!< UART Error Callback                   */
  void (* AbortCpltCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Abort Complete Callback          */
  void (* AbortTransmitCpltCallback)(struct __UART_HandleTypeDef *huart); /*!< UART Abort Transmit Complete Callback */
  void (* AbortReceiveCpltCallback)(struct __UART_HandleTypeDef *huart);  /*!< UART Abort Receive Complete Callback  */
  void (* WakeupCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Wakeup Callback                  */
  void (* RxEventCallback)(struct __UART_HandleTypeDef *huart, uint16_t Pos); /*!< UART Reception Event Callback     */

  void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);           /*!< UART Msp Init callback                */
  void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Msp DeInit callback              */
#endif  /* USE_HAL_UART_REGISTER_CALLBACKS */

} UART_HandleTypeDef;

//************************************************************************************************//
//                                         STM32F4XX_HAL_WWDG.H                                   //
//************************************************************************************************//

/** @defgroup WWDG_Prescaler WWDG Prescaler
  * @{
  */
#define WWDG_PRESCALER_1                    0x00000000u                              /*!< WWDG counter clock = (PCLK1/4096)/1 */
#define WWDG_PRESCALER_2                    WWDG_CFR_WDGTB_0                         /*!< WWDG counter clock = (PCLK1/4096)/2 */
#define WWDG_PRESCALER_4                    WWDG_CFR_WDGTB_1                         /*!< WWDG counter clock = (PCLK1/4096)/4 */
#define WWDG_PRESCALER_8                    (WWDG_CFR_WDGTB_1 | WWDG_CFR_WDGTB_0)    /*!< WWDG counter clock = (PCLK1/4096)/8 */
/**
  * @}
  */

/** @defgroup WWDG_EWI_Mode WWDG Early Wakeup Interrupt Mode
  * @{
  */
#define WWDG_EWI_DISABLE                    0x00000000u       /*!< EWI Disable */
#define WWDG_EWI_ENABLE                     WWDG_CFR_EWI      /*!< EWI Enable */


//************************************************************************************************//
//                                       STM32F4XX_HAL_FLASH_EX.H                                 //
//************************************************************************************************//

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||\
    defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
#define FLASH_LATENCY_0                FLASH_ACR_LATENCY_0WS   /*!< FLASH Zero Latency cycle      */
#define FLASH_LATENCY_1                FLASH_ACR_LATENCY_1WS   /*!< FLASH One Latency cycle       */
#define FLASH_LATENCY_2                FLASH_ACR_LATENCY_2WS   /*!< FLASH Two Latency cycles      */
#define FLASH_LATENCY_3                FLASH_ACR_LATENCY_3WS   /*!< FLASH Three Latency cycles    */
#define FLASH_LATENCY_4                FLASH_ACR_LATENCY_4WS   /*!< FLASH Four Latency cycles     */
#define FLASH_LATENCY_5                FLASH_ACR_LATENCY_5WS   /*!< FLASH Five Latency cycles     */
#define FLASH_LATENCY_6                FLASH_ACR_LATENCY_6WS   /*!< FLASH Six Latency cycles      */
#define FLASH_LATENCY_7                FLASH_ACR_LATENCY_7WS   /*!< FLASH Seven Latency cycles    */
#define FLASH_LATENCY_8                FLASH_ACR_LATENCY_8WS   /*!< FLASH Eight Latency cycles    */
#define FLASH_LATENCY_9                FLASH_ACR_LATENCY_9WS   /*!< FLASH Nine Latency cycles     */
#define FLASH_LATENCY_10               FLASH_ACR_LATENCY_10WS  /*!< FLASH Ten Latency cycles      */
#define FLASH_LATENCY_11               FLASH_ACR_LATENCY_11WS  /*!< FLASH Eleven Latency cycles   */
#define FLASH_LATENCY_12               FLASH_ACR_LATENCY_12WS  /*!< FLASH Twelve Latency cycles   */
#define FLASH_LATENCY_13               FLASH_ACR_LATENCY_13WS  /*!< FLASH Thirteen Latency cycles */
#define FLASH_LATENCY_14               FLASH_ACR_LATENCY_14WS  /*!< FLASH Fourteen Latency cycles */
#define FLASH_LATENCY_15               FLASH_ACR_LATENCY_15WS  /*!< FLASH Fifteen Latency cycles  */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx|| STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */

//************************************************************************************************//
//                                        STM32F4XX_HAL_GPIO.H                                    //
//************************************************************************************************//

#define GPIO_MODE_Pos                           0U
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)
#define OUTPUT_TYPE_Pos                         4U
#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)
#define OUTPUT_OD                               (0x1UL << OUTPUT_TYPE_Pos)
#define EXTI_MODE_Pos                           16U
#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
#define EXTI_IT                                 (0x1UL << EXTI_MODE_Pos)
#define EXTI_EVT                                (0x2UL << EXTI_MODE_Pos)
#define TRIGGER_MODE_Pos                         20U
#define TRIGGER_MODE                            (0x7UL << TRIGGER_MODE_Pos)
#define TRIGGER_RISING                          (0x1UL << TRIGGER_MODE_Pos)
#define TRIGGER_FALLING                         (0x2UL << TRIGGER_MODE_Pos)

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define GPIO_PIN_MASK              0x0000FFFFU /* PIN mask for assert test */

#define  GPIO_MODE_INPUT                        MODE_INPUT                                                  /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    (MODE_OUTPUT | OUTPUT_PP)                                   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    (MODE_OUTPUT | OUTPUT_OD)                                   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        (MODE_AF | OUTPUT_PP)                                       /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        (MODE_AF | OUTPUT_OD)                                       /*!< Alternate Function Open Drain Mode    */

#define  GPIO_MODE_ANALOG                       MODE_ANALOG                                                 /*!< Analog Mode  */

#define  GPIO_MODE_IT_RISING                    (MODE_INPUT | EXTI_IT | TRIGGER_RISING)                     /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   (MODE_INPUT | EXTI_IT | TRIGGER_FALLING)                    /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            (MODE_INPUT | EXTI_IT | TRIGGER_RISING | TRIGGER_FALLING)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

#define  GPIO_MODE_EVT_RISING                   (MODE_INPUT | EXTI_EVT | TRIGGER_RISING)                     /*!< External Event Mode with Rising edge trigger detection             */
#define  GPIO_MODE_EVT_FALLING                  (MODE_INPUT | EXTI_EVT | TRIGGER_FALLING)                    /*!< External Event Mode with Falling edge trigger detection            */
#define  GPIO_MODE_EVT_RISING_FALLING           (MODE_INPUT | EXTI_EVT | TRIGGER_RISING | TRIGGER_FALLING)   /*!< External Event Mode with Rising/Falling edge trigger detection     */

/** @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
#define  GPIO_SPEED_FREQ_LOW         0x00000000U  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM      0x00000001U  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH        0x00000002U  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define  GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */

/** @defgroup GPIO_pull_define GPIO pull define
  * @brief GPIO Pull-Up or Pull-Down Activation
  * @{
  */
#define  GPIO_NOPULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        0x00000001U   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      0x00000002U   /*!< Pull-down activation                */

/**
  * @brief  GPIO Bit SET and Bit RESET enumeration
  */
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

/**
  * @brief GPIO Init structure definition
  */
typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins.
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDef;


//************************************************************************************************//
//                                             FREE.C                                             //
//************************************************************************************************//

struct EventGroupDef_t;
typedef struct EventGroupDef_t * EventGroupHandle_t;

osEventFlagsId_t osEventFlagsNew (const osEventFlagsAttr_t *attr) {
  EventGroupHandle_t hEventGroup;
  int32_t mem;

  hEventGroup = NULL;

  if (!IS_IRQ()) {
    mem = -1;

    if (attr != NULL) {
      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticEventGroup_t))) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem == 1) {
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
      hEventGroup = xEventGroupCreateStatic (attr->cb_mem);
      #endif
    }
    else {
      if (mem == 0) {
        #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
          hEventGroup = xEventGroupCreate();
        #endif
      }
    }
  }

  return ((osEventFlagsId_t)hEventGroup);
}


osStatus_t osKernelStart (void) {
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    if (KernelState == osKernelReady) {
      /* Ensure SVC priority is at the reset value */
      SVC_Setup();
      /* Change state to enable IRQ masking check */
      KernelState = osKernelRunning;
      /* Start the kernel scheduler */
      vTaskStartScheduler();
      stat = osOK;
    } else {
      stat = osError;
    }
  }

  return (stat);
}

//Task Definitions
#define SYSTEM_TASK_LOOP_DELAY_CTS  (800)
#define DATA_TASK_LOOP_DELAY_CTS    (800)
#define DISPLAY_TASK_LOOP_DELAY_CTS (800)
#define CONTROL_TASK_LOOP_DELAY_CTS (800)

//Tasks
osThreadId_t sysTaskHandle;
osThreadId_t dataTaskHandle;
osThreadId_t dispTaskHandle;
osThreadId_t ctrlTaskHandle;


//Config
const osThreadAttr_t sysTask_attributes = {
  .name       = SYS_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t dataTask_attributes = {
  .name       = DATA_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t dispTask_attributes = {
  .name       = DISP_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t ctrlTask_attributes = {
  .name       = CTRL_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityLow,
};


//-------------------------------------------- Timers --------------------------------------------//

//Timers
osTimerId_t osTimerHandle;

//Config
const osTimerAttr_t osTimer_attributes = {
  .name = OS_TIMER_NAME
};


//------------------------------------------- Mutexes --------------------------------------------//

//Mutexes
osMutexId_t dataMutexHandle;

//Config
const osMutexAttr_t dataMutex_attributes = {
  .name = DATA_MUTEX_NAME
};


//------------------------------------------ Semaphores -------------------------------------------//

//Semaphores
osSemaphoreId_t ctrlSemHandle;
osSemaphoreId_t cntrSemHandle;

//Config
const osSemaphoreAttr_t ctrlSem_attributes = {
  .name = CTRL_SEM_NAME
};

const osSemaphoreAttr_t cntrSem_attributes = {
  .name = CNTR_SEM_NAME
};


//-------------------------------------------- Events --------------------------------------------//

//Events
osEventFlagsId_t dataStoreHandle;

//Config
const osEventFlagsAttr_t dataStore_attributes = {
  .name = DATA_EVENT_NAME
};

//PUBLIC FUNCTIONS


/**************************************************************************************************/
/** @fcn        void sysTask_Init(void *argument)
 *  @brief      Function implementing the sysTask thread.
 *  @details    GPIO & UART demos
 *
 *  @param    [in]  (void *) argument - x
 *
 *  @section  WDT Refresh
 *    Update counter value to !127, the refresh window is between
 *    !35 ms (!~728 * (!127-!80)) and !46 ms (!~728 * !64)
 */
/**************************************************************************************************/
void sysTask_Init(void *argument) {

  //Locals
#ifdef WDT_IS_WORKING
  HAL_StatusTypeDef stat = HAL_ERROR;       /* status of HAL operations for review      */
#endif

  //Loop
  for(;;) {

    //Notify
    _printf("Calling System Task\n\r");

    //Wiggle
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

#ifdef WDT_IS_WORKING
    //Refresh
    stat = HAL_WWDG_Refresh(&hwwdg);
    Error_Catch(stat);
#endif

    //Delay
    osDelay(SYSTEM_TASK_LOOP_DELAY_CTS);
  }
}


/**************************************************************************************************/
/** @fcn        void dataTask_Init(void *argument)
 *  @brief      Function implementing the dataTask thread.
 *  @details    Timer demo
 *
 *  @param    [in]  (void *) argument - x
 *
 *  @section  Opens
 *    Move print to each thread! with staggered timing
 */
/**************************************************************************************************/
void dataTask_Init(void *argument) {

  //Locals
  uint32_t timer_val  = 0;
  char buff[100]      = {0};

  //Init
  memset(&buff, 0x00, sizeof(buff));

  //Loop
  for(;;) {

    //Notify
    _printf("Calling Data Task\n\r");

    //Check
    timer_val = __HAL_TIM_GetCounter(&htim1);

    sprintf(&buff[0], "Timer: 0x%08lu\n\r\n\r", timer_val);

    //Print
    _printf(buff);

    //Delay
    osDelay(DATA_TASK_LOOP_DELAY_CTS);
  }
}


/**************************************************************************************************/
/** @fcn        void dispTask_Init(void *argument)
 *  @brief      Function implementing the dispTask thread.
 *  @details    Semaphore demo
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void dispTask_Init(void *argument) {

  //Loop
  for(;;) {

    //Notify
    _printf("Calling Disp Task\n\r");

    //Delay
    osDelay(DISPLAY_TASK_LOOP_DELAY_CTS);
  }
}


/**************************************************************************************************/
/** @fcn        void ctrlTask_Init(void *argument)
 *  @brief      Function implementing the ctrlTask thread.
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void ctrlTask_Init(void *argument) {

  //Loop
  for(;;) {

    //Notify
    _printf("Calling Control Task\n\r");

    //Delay
    osDelay(DISPLAY_TASK_LOOP_DELAY_CTS);
  }
}


/**************************************************************************************************/
/** @fcn        void osTimer_Callback(void *argument)
 *  @brief      osTimer_Callback function
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void osTimer_Callback(void *argument) {
  return;
}


//************************************************************************************************//
//                                              MAIN.H                                            //
//************************************************************************************************//

//Error Handling Prototypes
void Error_Handler(void);
void Error_Catch(HAL_StatusTypeDef stat);

//Print Prototypes
void _printf(const char *str);

//Peripheral Variables
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart2;
WWDG_HandleTypeDef hwwdg;


//Task Variables
osThreadId_t sysTaskHandle;
osThreadId_t dataTaskHandle;
osThreadId_t dispTaskHandle;
osThreadId_t ctrlTaskHandle;

//Attributes
const osThreadAttr_t sysTask_attributes;
const osThreadAttr_t dataTask_attributes;
const osThreadAttr_t dispTask_attributes;
const osThreadAttr_t ctrlTask_attributes;

//Timer Variables
osTimerId_t osTimerHandle;

//Mutex Variables
osMutexId_t dataMutexHandle;

//Semaphore Variables
osSemaphoreId_t ctrlSemHandle;
osSemaphoreId_t cntrSemHandle;

//Event Variables
osEventFlagsId_t dataStoreHandle;

//Config
const osTimerAttr_t osTimer_attributes;
const osMutexAttr_t dataMutex_attributes;
const osSemaphoreAttr_t ctrlSem_attributes;
const osSemaphoreAttr_t cntrSem_attributes;
const osEventFlagsAttr_t dataStore_attributes;


//************************************************************************************************//
//                                             MAIN.C                                             //
//************************************************************************************************//

//PLL Definitions
#define PLL_DIV_FACTOR    (16)
#define PLL_MULT_FACTOR   (336)
#define PLL_DIV_Q_FACTOR  (2)
#define PLL_DIV_R_FACTOR  (2)

//Timer Definitions
#define TIM_PRESCALER_VAL (0)
#define TIM_PERIOD_VAL    (65535)
#define TIM_REP_VAL       (0)

//USART Definitions
#define UART_BAUD_RATE_BPS  (115200)

//Watchdog Definitions
#define WWDG_WINDOW_VAL   (64)
#define WWDG_CTR_VAL      (64)


//Periph Variables
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart2;
WWDG_HandleTypeDef hwwdg;


//System Prototypes
void SystemClock_Config(void);

//Cube Prototypes
static void GPIO_Init(void);
static void USART2_UART_Init(void);
static void TIM1_Init(void);
static void WWDG_Init(void);


/**************************************************************************************************/
/** @fcn        int main(void)
 *  @brief      The application entry point
 *  @details    x
 *
 *  @return   (int) exit return status
 *
 *  @section  Opens
 *    sys_init()
 */
/**************************************************************************************************/
int main(void) {

  //********************************************************************************************//
  //                                          MCU INIT                                          //
  //********************************************************************************************//

  //Init HAL
  HAL_Init();

  //Init Clocks
  SystemClock_Config();

  //Init Peripheral
  GPIO_Init();
  USART2_UART_Init();
  TIM1_Init();
  WWDG_Init();

  //Init Scheduler
  osKernelInitialize();


  //------------------------------------ Initialize Mutexes ------------------------------------//

  //Init Mutexes
  dataMutexHandle = osMutexNew(&dataMutex_attributes);


  //----------------------------------- Initialize Semaphores ----------------------------------//

  //Init Semaphores
  ctrlSemHandle = osSemaphoreNew(CTRL_MAX_CT, CTRL_INIT_CT, &ctrlSem_attributes);
  cntrSemHandle = osSemaphoreNew(CNTR_MAX_CT, CNTR_INIT_CT, &cntrSem_attributes);


  //------------------------------------- Initialize Timers ------------------------------------//

  //Init Timers
  osTimerHandle = osTimerNew(osTimer_Callback, osTimerPeriodic, NULL, &osTimer_attributes);


  //------------------------------------- Initialize Tasks -------------------------------------//

  //Init Tasks
  sysTaskHandle  = osThreadNew(sysTask_Init,  NULL, &sysTask_attributes);
  dataTaskHandle = osThreadNew(dataTask_Init, NULL, &dataTask_attributes);
  dispTaskHandle = osThreadNew(dispTask_Init, NULL, &dispTask_attributes);
  ctrlTaskHandle = osThreadNew(ctrlTask_Init, NULL, &ctrlTask_attributes);


  //------------------------------------ Initialize Events -------------------------------------//

  //Init Events
  dataStoreHandle = osEventFlagsNew(&dataStore_attributes);

  //------------------------------------- Initialize RTOS --------------------------------------//

  //Notify
  _printf("Operating system in initialized\n\r");

  //Start Scheduler
  osKernelStart();

  //Loop
  for(;;) {
    _nop();                   /* @open ErrorHandler? should never get here  */
  }
}


/**************************************************************************************************/
/** @fcn        void SystemClock_Config(void)
 *  @brief      System Clock Configuration
 *  @details    x
 */
/**************************************************************************************************/
void SystemClock_Config(void) {

  //Locals
  HAL_StatusTypeDef stat = HAL_ERROR;       /* status of HAL operations for review      */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  //Init Variables
  memset(&RCC_OscInitStruct, 0x00, sizeof(RCC_OscInitStruct));
  memset(&RCC_ClkInitStruct, 0x00, sizeof(RCC_ClkInitStruct));


  //------------------------------------------- Setup ------------------------------------------//

  //Setup VREG
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  //RCC Oscillator Config
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = PLL_DIV_FACTOR;
  RCC_OscInitStruct.PLL.PLLN            = PLL_MULT_FACTOR;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ            = PLL_DIV_Q_FACTOR;
  RCC_OscInitStruct.PLL.PLLR            = PLL_DIV_R_FACTOR;

  //RCC Clock Config (CPU/AHB/APB bus clks)
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK
                                   | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1
                                   | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  //---------------------------------------- Initialize ----------------------------------------//

  //Init Oscillator
  stat = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  Error_Catch(stat);

  //Init Clock
  stat = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  Error_Catch(stat);

  return;
}


/**************************************************************************************************/
/** @fcn        static void TIM1_Init(void)
 *  @brief      TIM1 Initialization Function
 *  @details    x
 *
 *  @section  Opens
 *    Variable init check!
 */
/**************************************************************************************************/
static void TIM1_Init(void) {

  //Locals
  HAL_StatusTypeDef stat = HAL_ERROR;       /* status of HAL operations for review      */
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  //Init Variables
  memset(&sClockSourceConfig, 0x00, sizeof(sClockSourceConfig));
  memset(&sMasterConfig,      0x00, sizeof(sMasterConfig));


  //------------------------------------------- Setup ------------------------------------------//

  //Timer Config
  htim1.Instance                    = TIM1;
  htim1.Init.Prescaler              = TIM_PRESCALER_VAL;
  htim1.Init.CounterMode            = TIM_COUNTERMODE_UP;
  htim1.Init.Period                 = TIM_PERIOD_VAL;
  htim1.Init.ClockDivision          = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter      = TIM_REP_VAL;
  htim1.Init.AutoReloadPreload      = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim1.Channel                     = HAL_TIM_ACTIVE_CHANNEL_1;                   //@open   CATCH!

  //Source Config
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL;

  //Sync Config
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;


  //---------------------------------------- Initialize ----------------------------------------//

  //Init Timer
  stat = HAL_TIM_Base_Init(&htim1);
  Error_Catch(stat);

  //Init Source Config
  stat = HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  Error_Catch(stat);

  //Init Sync Config
  stat = HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
  Error_Catch(stat);

  //Enable Timer
  HAL_TIM_Base_Start(&htim1);

  return;
}


/**************************************************************************************************/
/** @fcn        static void USART2_UART_Init(void)
 *  @brief      USART2 Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void USART2_UART_Init(void) {

  //Locals
  HAL_StatusTypeDef stat = HAL_ERROR;       /* status of HAL operations for review      */

  //------------------------------------------- Setup ------------------------------------------//

  //UART Config
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = UART_BAUD_RATE_BPS;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  //---------------------------------------- Initialize ----------------------------------------//

  //Init UART
  stat = HAL_UART_Init(&huart2);
  Error_Catch(stat);

  return;
}


/**************************************************************************************************/
/** @fcn        static void WWDG_Init(void)
 *  @brief      WWDG Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void WWDG_Init(void) {

  //Locals
  HAL_StatusTypeDef stat = HAL_ERROR;       /* status of HAL operations for review      */

  //------------------------------------------- Setup ------------------------------------------//

  //WDT Config

  hwwdg.Instance       = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window    = WWDG_WINDOW_VAL;
  hwwdg.Init.Counter   = WWDG_CTR_VAL;
  hwwdg.Init.EWIMode   = WWDG_EWI_DISABLE;

  //---------------------------------------- Initialize ----------------------------------------//

  //Init WWDG
  stat = HAL_WWDG_Init(&hwwdg);
  Error_Catch(stat);

  return;
}


/**************************************************************************************************/
/** @fcn        static void GPIO_Init(void)
 *  @brief      GPIO Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void GPIO_Init(void) {

  //Locals
  GPIO_InitTypeDef GPIO_B1_InitStruct  = {0};
  GPIO_InitTypeDef GPIO_PC3_InitStruct = {0};
  GPIO_InitTypeDef GPIO_LD2_InitStruct = {0};

  //Init Variables
  memset(&GPIO_B1_InitStruct,  0x00, sizeof(GPIO_B1_InitStruct));
  memset(&GPIO_PC3_InitStruct, 0x00, sizeof(GPIO_PC3_InitStruct));
  memset(&GPIO_LD2_InitStruct, 0x00, sizeof(GPIO_LD2_InitStruct));


  //------------------------------------------- Setup ------------------------------------------//

  //Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  //B1_Pin Config
  GPIO_B1_InitStruct.Pin  = B1_Pin;
  GPIO_B1_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_B1_InitStruct.Pull = GPIO_NOPULL;

  //PC3 Config
  GPIO_PC3_InitStruct.Pin   = GPIO_PIN_3;
  GPIO_PC3_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_PC3_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_PC3_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  //LD2_Pin Config
  GPIO_LD2_InitStruct.Pin   = LD2_Pin;
  GPIO_LD2_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_LD2_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_LD2_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;


  //---------------------------------------- Initialize ----------------------------------------//

  //Conf GPIO Output Levels
  HAL_GPIO_WritePin(GPIOC,         GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,    GPIO_PIN_RESET);

  //Init GPIO Pins
  HAL_GPIO_Init(B1_GPIO_Port,  &GPIO_B1_InitStruct);
  HAL_GPIO_Init(GPIOC,         &GPIO_PC3_InitStruct);
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_LD2_InitStruct);

  return;
}


//************************************************************************************************//
//                                         PUBLIC ROUTINES                                        //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        void _printf(const char *str)
 *  @brief      x
 *  @details    x
 *
 *  @param  [in] (const char *) str - EOS terminated string to print
 *
 *  @pre  USART2_UART_Init()
 *
 *  @section    Opens
 *      consider using printf() replacement direct
 */
/**************************************************************************************************/
void _printf(const char *str) {

  //Transmit
  HAL_UART_Transmit(&huart2, (const uint8_t *) str, strlen(str), 1000000);

  return;
}


/**************************************************************************************************/
/** @fcn        void Error_Handler(void)
 *  @brief      This function is executed in case of error occurrence.
 *  @details    no return
 *
 *  @section    Opens
 *      move to new system.c/h
 *
 *  @note   User can add his own implementation to report the HAL error return state
 */
/**************************************************************************************************/
void Error_Handler(void) {

  //Disable
  __disable_irq();

  //Catch
  for(;;);
}


/**************************************************************************************************/
/** @fcn        void Error_Catch(HAL_StatusTypeDef stat)
 *  @brief      Respond to HAL status
 *  @warn   No return on error
 *
 *  @section    Opens
 *      move to new system.c/h
 */
/**************************************************************************************************/
 void Error_Catch(HAL_StatusTypeDef stat) {

  //Catch & Handle
  if(stat != HAL_OK) {
    Error_Handler();
  }

  return;
}

