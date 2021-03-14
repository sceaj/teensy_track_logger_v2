/*
 * init.c
 *
 *  Created on: Feb 14, 2021
 *      Author: sceaj
 */

#include "init.h"
#include "blinker.h"
#include "gps.h"
#include "logger.h"
#include "shell_task.h"

extern TaskHandle_t g_BlinkTaskHandle;
extern TaskHandle_t g_GpsTaskHandle;
extern TaskHandle_t g_LoggerTaskHandle;
extern TaskHandle_t g_ShellTaskHandle;

static status_t s_loggerStatus;
static status_t s_gpsStatus;


void StartTasks(void *pvParameters) {
    vTaskResume(g_BlinkTaskHandle);
    vTaskDelay(500);
    vTaskResume(g_ShellTaskHandle);
	vTaskDelay(500);
	if (kStatus_Success == s_loggerStatus) {
	    vTaskResume(g_LoggerTaskHandle);
	    vTaskDelay(500);
	} else {
		PRINTF("Logger initialization failed - not resuming task\n");
	}
	if (kStatus_Success == s_gpsStatus) {
		vTaskResume(g_GpsTaskHandle);
		vTaskDelay(500);
	} else {
		PRINTF("GPS initialization failed - not resuming task\n");
	}
}

void InitTask(void *pvParameters) {

	s_loggerStatus = LoggerInit();
	s_gpsStatus = GpsInit();
	StartTasks(pvParameters);
	vTaskDelete(NULL);
}

