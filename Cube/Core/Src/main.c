/**************************************************************************************************/
/** @file     main.c
 *  @brief    Main program body
 *  @details  Copyright (c) 2025 STMicroelectronics. All rights reserved.
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  2/24/25
 *  @last rev 2/24/25
 *
 *  @section    Legal Disclaimer
 * 		This software is licensed under terms that can be found in the LICENSE file in the root
 * 		directory of this software component. If no LICENSE file comes with this software, it is
 * 		provided AS-IS.
 */
/**************************************************************************************************/

//************************************************************************************************//
//                                            INCLUDES                                            //
//************************************************************************************************//

//Standard Library Includes
#include <stdbool.h>
#include <stdint.h>

//Project Includes
#include "main.h"
#include "cmsis_os.h"


//************************************************************************************************//
//                                        DEFINITIONS & TYPES                                     //
//************************************************************************************************//

//-----------------------------------------  Definitions -----------------------------------------//

//GPIO Definitions
#define RD_TASK_NAME		("defaultTask")
#define GPIO_RD_PORT		(GPIOA)
#define GPIO_RD_PIN			(GPIO_PIN_2)
#define WR_TASK_NAME		("myTask02")
#define GPIO_WR_PORT		(GPIOA)
#define GPIO_WR_PIN			(GPIO_PIN_1)

//Timing Definitions
#define RD_LOOP_DELAY_CTS	(1000)
#define WR_LOOP_DELAY_CTS	(1000)
#define SEM_DELAY_CTS	    (1000)

//------------------------------------------- Typedefs -------------------------------------------//

typedef StaticSemaphore_t osStaticSemaphoreDef_t;


//************************************************************************************************//
//                                            VARIABLES                                           //
//************************************************************************************************//

//---------------------------------------------- HAL ---------------------------------------------//

//Timers
TIM_HandleTypeDef htim1;


//----------------------------------------------- OS ---------------------------------------------//

/* Definitions for read task */
osThreadId_t readTaskHandle;

const osThreadAttr_t readTask_attributes = {
  .name = RD_TASK_NAME,
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for write task */
osThreadId_t writeTaskHandle;

const osThreadAttr_t writeTask_attributes = {
  .name = WR_TASK_NAME,
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

//Semaphore
osSemaphoreId_t        gpio_inHandle;
osStaticSemaphoreDef_t gpio_inControlBlock;

const osSemaphoreAttr_t gpio_in_attributes = {
  .name = "gpio_in",
  .cb_mem = &gpio_inControlBlock,
  .cb_size = sizeof(gpio_inControlBlock),
};

//----------------------------------------- APPLICATION ------------------------------------------//

int gpio_input_ct = 0;								/* Input toggles received by the read task	  */


//************************************************************************************************//
//                                       FUNCTION DECLARATIONS                                    //
//************************************************************************************************//

//System Prototypes
void SystemClock_Config(void);

//HAL Prototypes
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);

//OS Prototypes
void Task_Read(void *argument);
void Task_Write(void *argument);

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_tim.h"

//************************************************************************************************//
//                                          PRIVATE FUNCTIONS                                     //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        int main(void)
 *  @brief      The application entry point
 *  @details    x
 */
/**************************************************************************************************/
int main(void) {
	MPU_Region_InitTypeDef
	//-----------------------------------------HAL INIT ------------------------------------------//
	HAL_NVIC_SetPriority();
//	`HAL_NVIC_SetPriorityGrouping'
//	`HAL_SYSTICK_Config'
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();

	//----------------------------------------- OS INIT ------------------------------------------//

	/* Init scheduler */
	osKernelInitialize();

	/* Create the semaphores(s) */
	gpio_inHandle = osSemaphoreNew(1, 1, &gpio_in_attributes);

	/* Create the thread(s) */
	readTaskHandle  = osThreadNew(Task_Read,  NULL, &readTask_attributes);
	writeTaskHandle = osThreadNew(Task_Write, NULL, &writeTask_attributes);

	/* Start scheduler */
	osKernelStart();

	//----------------------------------------- DEMO ---------------------------------------------//

	//Loop
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void SystemClock_Config(void)
 *  @brief      System Clock Configuration
 *  @details    x
 */
/**************************************************************************************************/
void SystemClock_Config(void) {

	//Locals
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Osc. according to the spec params in RCC_OscInitTypeDef struct.
  */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

  //Apply
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  //Apply
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }

  return;
}


/**************************************************************************************************/
/** @fcn        static void MX_TIM1_Init(void)
 *  @brief      TIM1 Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void MX_TIM1_Init(void) {

	//Locals
  TIM_SlaveConfigTypeDef sSlaveConfig   = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};


  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 65535;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if(HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;

  if(HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

  if(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  return;
}


/**************************************************************************************************/
/** @fcn        static void MX_GPIO_Init(void)
 *  @brief      GPIO Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void MX_GPIO_Init(void) {

	//Locals
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin   = GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  //Init Pint
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin  = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  //Init Pin
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  return;
}


/**************************************************************************************************/
/** @fcn        void Task_Read(void *argument)
 *  @brief      Function implementing the read task thread
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void Task_Read(void *argument) {

	//Locals
	bool gpio_pin[2] = {false, false};				/* gpio input values					      */

	/* Infinite loop */
	for(;;) {

		//Store prev
		gpio_pin[1] = gpio_pin[0];

		//Check pin
		gpio_pin[0] = HAL_GPIO_ReadPin(GPIO_RD_PORT, GPIO_RD_PIN);

		//Signal
		if(gpio_pin[0] != gpio_pin[1]) {
			gpio_input_ct++;
		}

		//Loop Delay
		osDelay(RD_LOOP_DELAY_CTS);
	}
}


/**************************************************************************************************/
/** @fcn        void Task_Write(void *argument)
 *  @brief      Function implementing the write task thread
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 *
 *  @note	semaphore is DNC here just used for ProductMap inspect
 */
/**************************************************************************************************/
void Task_Write(void *argument) {

	//Loop
	for(;;) {

		//Check
		if(gpio_input_ct > 0) {

			//Handle input
			gpio_input_ct--;

			//Wait for semaphore
			osSemaphoreAcquire(gpio_inHandle, osWaitForever);

			//Signal
			HAL_GPIO_TogglePin(GPIO_WR_PORT, GPIO_WR_PIN);

			//Release
			osSemaphoreRelease(gpio_inHandle);
		}

		//Loop Delay
		osDelay(WR_LOOP_DELAY_CTS);
	}
}

/**************************************************************************************************/
/** @fcn        void Error_Handler(void)
 *  @brief      This function is executed in case of error occurrence
 *  @details    x
 */
/**************************************************************************************************/
void Error_Handler(void) {

	//Silence
  __disable_irq();

  //Spin
  for(;;);
}

