/*
 * teensy41_message_generator.c
 *
 *  Created on: Oct 3, 2020
 *      Author: jrosen
 */

#include "teensy41_message_generator.h"
#include "FreeRTOS.h"
#include "task.h"

static msg_gen_params_t s_Parameters;
uint32_t g_Counter;

void teensy41_MessageGeneratorInit( void *pvParameters )
{
	s_Parameters.counterInitialValue = ((msg_gen_params_t*)pvParameters)->counterInitialValue;
	s_Parameters.delayMilliseconds = ((msg_gen_params_t*)pvParameters)->delayMilliseconds;
}

void teensy41_MessageGeneratorTask( void *pvParameters )
{

	teensy41_MessageGeneratorInit(pvParameters);

	for( ;; )
	{
	}

	vTaskDelete(NULL);
}

