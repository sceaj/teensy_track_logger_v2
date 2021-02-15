/*
 * blinker.c
 *
 *  Created on: Feb 5, 2021
 *      Author: sceaj
 *
 *      MIT License
 *
 * Copyright (c) 2021 Jeff Rosen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <blinker.h>
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"

uint32_t g_blinkOnTicks = 400;
uint32_t g_blinkOffTicks = 600;

void BlinkerInit( void *pvParameters )
{
	/* GPIO configuration of LED on GPIO_B0_03 (pin D8) */
	gpio_pin_config_t LED_config = {
	  .direction = kGPIO_DigitalOutput,
	  .outputLogic = 0U,
	  .interruptMode = kGPIO_NoIntmode
	};

	/* Initialize GPIO functionality on GPIO_B0_03 (pin D8) */
	GPIO_PinInit(GPIO2, 3U, &LED_config);

	IOMUXC_SetPinMux(
			IOMUXC_GPIO_B0_03_GPIO2_IO03,           /* GPIO_B0_03 is configured as GPIO2_IO03 */
			0U);                                    /* Software Input On Field: Input Path is determined by functionality */

	IOMUXC_GPR->GPR27 = (
			(IOMUXC_GPR->GPR27 & (~(IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL_MASK))) /* Mask bits to zero which are setting */
			| IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL(0x00U) /* GPIO2 and GPIO7 share same IO MUX function, GPIO_MUX2 selects one GPIO function: 0x00U */
	);

	IOMUXC_SetPinConfig(
			IOMUXC_GPIO_B0_03_GPIO2_IO03,           /* GPIO_B0_03 PAD functional properties : */
			0x10A0U);

}

void BlinkerTask( void *pvParameters )
{

	BlinkerInit(pvParameters);

	for( ;; )
	{
        GPIO_PinWrite(BLINK_GPIO, BLINK_GPIO_PIN, 1);
        vTaskDelay(g_blinkOnTicks);
        GPIO_PinWrite(BLINK_GPIO, BLINK_GPIO_PIN, 0);
        vTaskDelay(g_blinkOffTicks);
	}

	vTaskDelete(NULL);
}

