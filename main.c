/**************************************************************************************************/
/** @file     main.c
 *  @brief    Troll Product Map Demo
 *  @details  x
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  1/30/25
 *  @last rev 1/30/25
 *
 *
 *  @section    Opens
 *      Eclipse stdlib includes
 *      outline map for test of tool
 *      review use of static declarations
 *      switch to target hw
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
#include <stdio.h>
#include <stdlib.h>


//Definitions
#define DEMO_LOOP_CTS       (10)


//Application Routines
static void os_init(void);
static void os_delay(int delay_cts);

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

  //Init
  os_init();

  //Notify
  printf("Begin Demo -\r\n\r\n");


  //Demo
    wifi_task();
    pmu_task();
    data_task();
    system_task();


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
  printf("Begin Initialization -\r\n\r\n");

  //Init Tasks
  system_init();
  pmu_init();
  data_init();
  wifi_init();

  //Loop
  os_delay(DEMO_LOOP_CTS);

  //Notify
  printf("\r\nInitialization Complete.\r\n\r\n");

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
  printf("Loop Delay\r\n");

  return;
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
  printf("PWM Task\r\n");                           /* Share a clean message                      */

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
  printf("PWM Task\r\n");                           /* Share a clean message                      */

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
  printf("PWM Task\r\n");                           /* Share a clean message                      */

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
  printf("WiFi Task\r\n");                          /* Share a clean message                      */

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
  printf("System Init\r\n");                        /* Share a clean message                      */

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
  printf("PWM Init\r\n");                           /* Share a clean message                      */

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
  printf("Data Init\r\n");                          /* Share a clean message                      */

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
  printf("WiFi Init\r\n");                          /* Share a clean message                      */

  return;
}

