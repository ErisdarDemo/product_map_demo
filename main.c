/**************************************************************************************************/
/** @file     main.c
 *  @brief    Troll Product Map Demo
 *  @details  Sample demonstration of control module
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  1/30/25
 *  @last rev 2/2/25
 *
 *
 *  @section    Opens
 *      review use of static declarations
 *      switch to target hw
 *		rtos vers
 *		feats vers
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


//************************************************************************************************//
//                                             TYPEDEFS                                           //
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
//                                            VARIABLE                                            //
//************************************************************************************************//

//Private variables
void *huart2;



//************************************************************************************************//
//                                          CUBE MX SOURCE                                        //
//************************************************************************************************//

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
	//Init HAL
  HAL_Init();             /* Reset of all periphs, Init Flash interface and the Systick. */

  //Init Clock
  SystemClock_Config();   /* Configure the system clock */

  //Init Periphs
  MX_GPIO_Init();
  MX_USART2_UART_Init();


  //----------------------------------------- Demo Init ------------------------------------------//

  //Init
  os_init();

  //Notify
  printf("Begin Demo -\r\n\r\n");


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

  //Init Tasks
  system_init();
  pmu_init();
  data_init();
  wifi_init();

  //Loop
  os_delay(DEMO_LOOP_CTS);

  //Notify
  println("\r\nInitialization Complete.\r\n");

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
/** @fcn        static void os_init(void)
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
  int stat;                                         /* @todo   descr                              */

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

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
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

  //ST Micro HAL Support
  void __HAL_FLASH_INSTRUCTION_CACHE_ENABLE(void) { return; }
  void __HAL_FLASH_DATA_CACHE_ENABLE(void)        { return; }
  void __HAL_FLASH_PREFETCH_BUFFER_ENABLE(void)   { return; }
  void HAL_NVIC_SetPriorityGrouping(int x)        { return; }
  void HAL_InitTick(int x)                        { return; }
  void HAL_MspInit(void)                          { return; }


/**************************************************************************************************/
/** @fcn        HAL_StatusTypeDef HAL_Init(void)
 *  @brief      x
 *  @details    x
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

