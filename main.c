/**************************************************************************************************/
/** @file     main.c
 *  @brief    Troll Product Map Demo
 *  @details  Sample demonstration of control module
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  1/30/25
 *  @last rev 1/30/25
 *
 *
 *  @section    Opens
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

