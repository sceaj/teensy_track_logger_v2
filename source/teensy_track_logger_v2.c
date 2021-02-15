/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file    teensy_track_logger_v2.c
 * @brief   Application entry point.
 * @author  sceaj
 *
 *      MIT License
 *
 * Portions Copyright (c) 2021 Jeff Rosen
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
#include <stdio.h>
#include "blinker.h"
#include "board.h"
#include "init.h"
#include "logger.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "serial_manager.h"
#include "shell_task.h"
#include "task.h"

#define STACK_SIZE (256U)
#define LG_STACK_SIZE (1024U)

/* TODO: insert other definitions and declarations here. */
TaskHandle_t g_InitTaskHandle = NULL;
TaskHandle_t g_BlinkTaskHandle = NULL;
TaskHandle_t g_LoggerTaskHandle = NULL;
TaskHandle_t g_ShellTaskHandle = NULL;
TaskHandle_t g_MsgGenTaskHandle = NULL;
TaskHandle_t g_DebugConsoleTaskHandle = NULL;

void BOARD_InitHardware(void) {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
}

/*
 * @brief   Application entry point.
 */
int main(void) {

	/* Initialize MCU hardware */
	BOARD_InitHardware();

	// Perform Initialization
	BaseType_t xStatus = xTaskCreate(InitTask, /* Function that implements the task. */
    		"Initialization",					   /* Text name for the task. */
            384U,		      					   /* Stack size in words, not bytes. */
            NULL,    							   /* Parameter passed into the task. */
            tskIDLE_PRIORITY + 2,				   /* Priority at which the task is created. */
			&g_InitTaskHandle);				   /* Used to pass out the created task's handle. */

	xStatus = xTaskCreate(BlinkerTask, "Blink", 128U, NULL, tskIDLE_PRIORITY + 2, &g_BlinkTaskHandle);
    if (xStatus == pdPASS) {
    	vTaskSuspend(g_BlinkTaskHandle);
    } else {
    	PRINTF("Error: Failed to create Blink task\n");
    }
    xStatus = xTaskCreate(ShellTask, "Shell", 256U, NULL, tskIDLE_PRIORITY + 1, &g_ShellTaskHandle);
    if (xStatus == pdPASS) {
    	vTaskSuspend(g_ShellTaskHandle);
    } else {
    	PRINTF("Error: Failed to create Shell task\n");
    }
    xStatus = xTaskCreate(LoggerTask, "Logger", 384U, NULL, tskIDLE_PRIORITY + 3, &g_LoggerTaskHandle);
    if (xStatus == pdPASS) {
    	vTaskSuspend(g_LoggerTaskHandle);
    } else {
    	PRINTF("Error: Failed to create Logger task\n");
    }

	vTaskStartScheduler();

    return 0;
}

