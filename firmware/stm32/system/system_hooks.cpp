/*
 * system_hooks.cpp
 *
 *  Created on: 12 lut 2023
 *      Author: robal
 */

#include "FreeRTOS.h"
#include "task.h"


extern "C" void vApplicationIdleHook( void )
{

}

extern "C" void vApplicationMallocFailedHook( void )
{

}

extern "C" void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. */
  (void)xTask;
  (void)pcTaskName;

  /* Force an assert. */
  while(1);
}
