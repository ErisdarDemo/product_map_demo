/**************************************************************************************************/
/** @file     main.c
 *   @brief    Troll Product Map Demo
 *   @details  Sample demonstration of control module
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  1/30/25
 *  @last rev 2/2/25
 *
 *
 *  @section    Opens
 *      feats vers
 *      ...!
 *
 *  @section    Hardware
 *      @open STM32F091RC
 *
 *  @section    Legal Disclaimer
 *      © 2025 Justin Reina, All rights reserved. All contents of this source file and/or any other
 *      related source files are for public redistribution at this time.
 */
/**************************************************************************************************/

//Standard Library Includes
#include <stdint.h>


//************************************************************************************************//
//                                           DEFINITIONS                                          //
//************************************************************************************************//

#define NVIC_PRIORITYGROUP_4         (0)
#define TICK_INT_PRIORITY            (0)
#define B1_Pin                       (GPIO_PIN_13)
#define B1_GPIO_Port                 (GPIOC)
#define USART_TX_Pin                 (GPIO_PIN_2)
#define USART_TX_GPIO_Port           (GPIOA)
#define USART_RX_Pin                 (GPIO_PIN_3)
#define USART_RX_GPIO_Port           (GPIOA)
#define LD2_Pin                      (GPIO_PIN_5)
#define LD2_GPIO_Port                (GPIOA)
#define TMS_Pin                      (GPIO_PIN_13)
#define TMS_GPIO_Port                (GPIOA)
#define TCK_Pin                      (GPIO_PIN_14)
#define TCK_GPIO_Port                (GPIOA)
#define SWO_Pin                      (GPIO_PIN_3)
#define SWO_GPIO_Port                (GPIOB)
#define RCC_OSCILLATORTYPE_HSI       (0x00000002U)
#define RCC_HSICALIBRATION_DEFAULT   (0x10U)
#define RCC_HSI_ON                   ((uint8_t)0x01)
#define RCC_CLOCKTYPE_SYSCLK         (0x00000001U)
#define RCC_CLOCKTYPE_HCLK           (0x00000002U)
#define RCC_CLOCKTYPE_PCLK1          (0x00000004U)
#define RCC_CLOCKTYPE_PCLK2          (0x00000008U)
#define RCC_CFGR_SW_PLL              (0x00000002U)
#define RCC_SYSCLKSOURCE_PLLCLK      (RCC_CFGR_SW_PLL)
#define RCC_CFGR_HPRE_DIV1           (0x00000000U)
#define RCC_CFGR_PPRE1_DIV1          (0x00000000U)
#define RCC_CFGR_PPRE1_DIV2          (0x00001000U)
#define RCC_SYSCLK_DIV1              (RCC_CFGR_HPRE_DIV1)
#define RCC_HCLK_DIV1                (RCC_CFGR_PPRE1_DIV1)
#define RCC_HCLK_DIV2                (RCC_CFGR_PPRE1_DIV2)
#define NULL                         (0)


//************************************************************************************************//
//                                          CUBEMX TYPEDEFS                                       //
//************************************************************************************************//

typedef enum {
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


typedef struct {
  uint32_t OscillatorType;       // The oscillators to be configured.
  uint32_t HSEState;             // The new state of the HSE.
  uint32_t LSEState;             // The new state of the LSE.
  uint32_t HSIState;             // The new state of the HSI.
  uint32_t HSICalibrationValue;  // The HSI calibration trimming value
  uint32_t LSIState;             // The new state of the LSI.
} RCC_OscInitTypeDef;


typedef struct {
  uint32_t ClockType;             // The clock to be configured.
  uint32_t SYSCLKSource;          // The clock source (SYSCLKS) used as system clock.
  uint32_t AHBCLKDivider;         // The AHB clock (HCLK) divider
  uint32_t APB1CLKDivider;        // The APB1 clock (PCLK1) divider
  uint32_t APB2CLKDivider;        // The APB2 clock (PCLK2) divider
} RCC_ClkInitTypeDef;


//************************************************************************************************//
//                                        FREERTOS TYPEDEFS                                       //
//************************************************************************************************//

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


typedef struct {
  const char                   *name;   ///< name of the mutex
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osMutexAttr_t;


typedef struct {
  const char                   *name;   ///< name of the semaphore
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osSemaphoreAttr_t;


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

typedef uint32_t TZ_ModuleId_t;

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

typedef struct {
  const char                   *name;   ///< name of the timer
  uint32_t                 attr_bits;   ///< attribute bits
  void                      *cb_mem;    ///< memory for control block
  uint32_t                   cb_size;   ///< size of provided memory for control block
} osTimerAttr_t;


typedef enum {
  osTimerOnce               = 0,          ///< One-shot timer.
  osTimerPeriodic           = 1           ///< Repeating timer.
} osTimerType_t;


//************************************************************************************************//
//                                            VARIABLE                                            //
//************************************************************************************************//

typedef void *osMutexId_t;

typedef void *osSemaphoreId_t;

typedef void *osThreadId_t;

typedef void *osTimerId_t;

typedef void (*osThreadFunc_t) (void *argument);

typedef void (*osTimerFunc_t) (void *argument);

typedef void *UART_HandleTypeDef;

UART_HandleTypeDef huart2;

osThreadId_t systemHandle;


//************************************************************************************************//
//                                            CONSTANTS                                           //
//************************************************************************************************//

const osThreadAttr_t system_attributes = {
  .name = "system",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Definitions for wifi */
osThreadId_t wifiHandle;

const osThreadAttr_t wifi_attributes = {
  .name = "wifi",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* Definitions for pmu */
osThreadId_t pmuHandle;

const osThreadAttr_t pmu_attributes = {
  .name = "pmu",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* Definitions for data */
osThreadId_t dataHandle;

const osThreadAttr_t data_attributes = {
  .name = "data",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* Definitions for myTimer */
osTimerId_t myTimerHandle;

const osTimerAttr_t myTimer_attributes = {
  .name = "myTimer"
};



/* Definitions for myMutex */
osMutexId_t myMutexHandle;

const osMutexAttr_t myMutex_attributes = {
  .name = "myMutex"
};


/* Definitions for mySem */
osSemaphoreId_t mySemHandle;

const osSemaphoreAttr_t mySem_attributes = {
  .name = "mySem"
};


//************************************************************************************************//
//                                          CUBE MX SOURCE                                        //
//************************************************************************************************//

//Private function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void Callback01(void *argument);

//HAL Prototypes
HAL_StatusTypeDef HAL_Init(void);

void Error_Handler(void);

void __HAL_RCC_PWR_CLK_ENABLE(void);
void __HAL_PWR_VOLTAGESCALING_CONFIG(void);

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct);

//Private function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);


//************************************************************************************************//
//                                         FREERTOS ROUTINES                                      //
//************************************************************************************************//

osMutexId_t osMutexNew(const osMutexAttr_t *attr);
osStatus_t osKernelInitialize(void);
osStatus_t osDelay(uint32_t ticks);

void Callback01(void *argument);

osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr);

osTimerId_t osTimerNew(osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr);

osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr);

osStatus_t osKernelStart(void);


//************************************************************************************************//
//                                            DEMO SOURCE                                         //
//************************************************************************************************//

//Standard Library Includes
#include <stdio.h>
#include <stdlib.h>


//Definitions
#define DEMO_LOOP_CTS       (10)


//Application Routines
static void os_init(void);
static void os_delay(int delay_cts);

//Utility Routines
static int println(char *str);

//Task Routines
static void system_task(void);
static void pmu_task(void);
static void data_task(void);
static void wifi_task(void);

//Initialization Routines
static void system_init(void);
static void pmu_init(void);
static void data_init(void);
static void wifi_init(void);


/**************************************************************************************************/
/** @fcn        int main(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
int main(void) {

  //Notify
    println("Begin Demo! -\r\n");

    //------------------------------------------ HAL Init ------------------------------------------//

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  //----------------------------------------- Demo Init ------------------------------------------//

  //FREERTOS Init
  os_init();

  //Notify
  printf("Begin Demo! -\r\n\r\n");


  //-------------------------------------------- Demo --------------------------------------------//

  //Operate
  wifi_task();
  pmu_task();
  data_task();
  system_task();


  //Notify
  println("\r\nDemo complete.");

  return EXIT_SUCCESS;
}



/**************************************************************************************************/
/** @fcn        static void os_init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void os_init(void) {

  //Notify
  println("Begin Initialization -\r\n");

  //-------------------------------------------- Tasks -------------------------------------------//

  //Init Tasks
  system_init();
  pmu_init();
  data_init();
  wifi_init();


  //--------------------------------------------- OS ---------------------------------------------//

  /* Init scheduler */
  osKernelInitialize();

  /* creation of myMutex */
  myMutexHandle = osMutexNew(&myMutex_attributes);

  /* creation of mySem */
  mySemHandle = osSemaphoreNew(1, 1, &mySem_attributes);

  /* creation of myTimer */
  myTimerHandle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer_attributes);

  /* creation of system */
  systemHandle = osThreadNew(StartDefaultTask, NULL, &system_attributes);

  /* creation of wifi */
  wifiHandle = osThreadNew(StartTask02, NULL, &wifi_attributes);

  /* creation of pmu */
  pmuHandle = osThreadNew(StartTask03, NULL, &pmu_attributes);

  /* creation of data */
  dataHandle = osThreadNew(StartTask04, NULL, &data_attributes);

  /* Start scheduler */
  osKernelStart();


  //Loop
  os_delay(DEMO_LOOP_CTS);

  //Notify
  println("\r\nOS Initialization Complete.\r\n");

  return;
}


/**************************************************************************************************/
/** @fcn        static void os_delay(int delay_cts)
 *  @brief      x
 *  @details    x
 *
 *  @param  [in] (int) delay_cts - number of counts to delay
 */
/**************************************************************************************************/
static void os_delay(int delay_cts) {

  //Notify
  println("\r\nLoop Delay");

  return;
}


/**************************************************************************************************/
/** @fcn        static int println(char *str)
 *  @brief      x
 *  @details    x
 *
 *  @parm [in] (char *) str - pointer to str w/EOS term
 *
 *  @section  Opens
 *      using a local for stat??
 */
/**************************************************************************************************/
static int println(char *str) {

  //Locals
  int stat;

  //Print
  stat = printf("%s\r\n", str);

  return stat;
}


//************************************************************************************************//
//                                              TASKS                                             //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        static void system_task(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void system_task(void) {

  //Notify
  println("PWM Task");                              /* Share a clean message                      */

  return;
}


/**************************************************************************************************/
/** @fcn        static void pmu_task(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void pmu_task(void) {

  //Notify
  println("PMU Task");                              /* Share a clean message                      */

  return;
}


/**************************************************************************************************/
/** @fcn        static void data_task(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void data_task(void) {

  //Notify
  println("Data Task");                             /* Share a clean message                      */

  return;
}


/**************************************************************************************************/
/** @fcn        static void wifi_task(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void wifi_task(void) {

  //Notify
  printf("WiFi Task");                              /* Share a clean message                      */

  return;
}


//************************************************************************************************//
//                                              INIT                                              //
//                                                                                                //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        static void system_init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void system_init(void) {

  //Notify
  println("System Init");                           /* Share a clean message                      */

  return;
}


/**************************************************************************************************/
/** @fcn        static void pmu_init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void pmu_init(void) {

  //Notify
  println("PMU Init");                              /* Share a clean message                      */

  return;
}


/**************************************************************************************************/
/** @fcn        static void data_init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void data_init(void) {

  //Notify
  println("Data Init");                             /* Share a clean message                      */

  return;
}


/**************************************************************************************************/
/** @fcn        static void wifi_init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void wifi_init(void) {

  //Notify
  println("WiFi Init");                             /* Share a clean message                      */

  return;
}

//************************************************************************************************//
//                                          CUBEMX SOURCE                                         //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        static void MX_GPIO_Init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void MX_GPIO_Init(void) {
  volatile int b = 0;
  b++;
  return;
}

/**************************************************************************************************/
/** @fcn        static void MX_USART2_UART_Init(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
static void MX_USART2_UART_Init(void) {
  volatile int c = 0;
  c++;
  return;
}


/**************************************************************************************************/
/** @fcn        void SystemClock_Config(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG();

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  return;
}


/**************************************************************************************************/
/** @fcn        void __HAL_RCC_PWR_CLK_ENABLE(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
void __HAL_RCC_PWR_CLK_ENABLE(void) {
  volatile int x = 0;
  x++;
  return;
}

/**************************************************************************************************/
/** @fcn        void __HAL_PWR_VOLTAGESCALING_CONFIG(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
void __HAL_PWR_VOLTAGESCALING_CONFIG(void)  {
  volatile int y = 0;
  y++;
  return;
}

/**************************************************************************************************/
/** @fcn        void Error_Handler(void)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
void Error_Handler(void) {
  volatile int a = 0;
  a++;
  return;
}


/**************************************************************************************************/
/** @fcn        void StartDefaultTask(void *argument)
 *  @brief      Function implementing the system thread.
 *  @details    x
 *
 *  @param [in[ (void *) argument - not used
 */
/**************************************************************************************************/
void StartDefaultTask(void *argument) {
  for(;;) {
    osDelay(1);
  }
}


/**************************************************************************************************/
/** @fcn        void StartTask02(void *argument)
 *  @brief      Function implementing the wifi thread.
 *  @details    x
 *
 *  @param [in[ (void *) argument - not used
 */
/**************************************************************************************************/
void StartTask02(void *argument) {
  for(;;) {
    osDelay(1);
  }
}


/**************************************************************************************************/
/** @fcn        void StartTask03(void *argument)
 *  @brief      Function implementing the pmu thread.
 *  @details    x
 *
 *  @param [in[ (void *) argument - not used
 */
/**************************************************************************************************/
void StartTask03(void *argument) {
  for(;;) {
    osDelay(1);
  }
}


/**************************************************************************************************/
/** @fcn        void StartTask04(void *argument)
 *  @brief      Function implementing the data thread.
 *  @details    x
 *
 *  @param [in[ (void *) argument - not used
 */
/**************************************************************************************************/
void StartTask04(void *argument) {
  for(;;) {
    osDelay(1);
  }
}


/**************************************************************************************************/
/** @fcn        void Callback01(void *argument)
 *  @brief      Callback01 function
 *  @details    x
 *
 *  @param [in[ (void *) argument - not used
 */
/**************************************************************************************************/
void Callback01(void *argument) {
  return;
}


//************************************************************************************************//
//                                      STM32F4 HAL_RCC SOURCE                                    //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct) {
  return HAL_OK;
}

/**************************************************************************************************/
/** @fcn        HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct)
 *  @brief      x
 *  @details    x
 */
/**************************************************************************************************/
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct) {
  return HAL_OK;
}


//************************************************************************************************//
//                                        STM32F4 HAL SOURCE                                      //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        void __HAL_FLASH_INSTRUCTION_CACHE_ENABLE(void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
void __HAL_FLASH_INSTRUCTION_CACHE_ENABLE(void) {
  return;
}


/**************************************************************************************************/
/** @fcn        void __HAL_FLASH_DATA_CACHE_ENABLE(void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
void __HAL_FLASH_DATA_CACHE_ENABLE(void) {
  return;
}


/**************************************************************************************************/
/** @fcn        void __HAL_FLASH_PREFETCH_BUFFER_ENABLE(void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
void __HAL_FLASH_PREFETCH_BUFFER_ENABLE(void) {
  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_NVIC_SetPriorityGrouping(int x)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
void HAL_NVIC_SetPriorityGrouping(int x) {
  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_InitTick(int x)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
void HAL_InitTick(int x) {
  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_MspInit(void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
void HAL_MspInit(void) {
  return;
}


/**************************************************************************************************/
/** @fcn        HAL_StatusTypeDef HAL_Init(void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
HAL_StatusTypeDef HAL_Init(void) {

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  HAL_MspInit();

  /* Return function status */
return HAL_OK;
}


//************************************************************************************************//
//                                         FREERTOS SOURCE                                        //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        osMutexId_t osMutexNew (const osMutexAttr_t *attr)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osMutexId_t osMutexNew(const osMutexAttr_t *attr) {
  return NULL;
}


/**************************************************************************************************/
/** @fcn        osStatus_t osKernelInitialize (void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osStatus_t osKernelInitialize(void) {
  return osOK;
}



/**************************************************************************************************/
/** @fcn        osStatus_t osDelay (uint32_t ticks)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osStatus_t osDelay(uint32_t ticks) {
  return osOK;
}


/**************************************************************************************************/
/** @fcn        osSemaphoreId_t osSemaphoreNew ()
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr) {
  return NULL;
}


/**************************************************************************************************/
/** @fcn        osThreadId_t osThreadNew ()
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr) {
  return NULL;
}


/**************************************************************************************************/
/** @fcn        osTimerId_t osTimerNew()
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osTimerId_t osTimerNew(osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr) {
  return NULL;
}


/**************************************************************************************************/
/** @fcn       osStatus_t osKernelStart (void)
*   @brief      x
*   @details    x
*/
/**************************************************************************************************/
osStatus_t osKernelStart(void) {
  return osOK;
}

