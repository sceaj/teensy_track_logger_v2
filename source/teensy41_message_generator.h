/*
 * teensy41_message_generator.h
 *
 *  Created on: Oct 3, 2020
 *      Author: jrosen
 */

#ifndef TEENSY41_MESSAGE_GENERATOR_H_
#define TEENSY41_MESSAGE_GENERATOR_H_

#include <stdint.h>

typedef struct _msg_gen_params {
	uint32_t	counterInitialValue;
	uint32_t    delayMilliseconds;
} msg_gen_params_t;

void teensy41_MessageGeneratorTask( void *pvParameters );

#endif /* TEENSY41_MESSAGE_GENERATOR_H_ */
