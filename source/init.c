/*
 * init.c
 *
 *  Created on: Feb 14, 2021
 *      Author: sceaj
 */

#include "init.h"
#include "blinker.h"
#include "fsl_snvs_hp.h"
#include "gps.h"
#include "neo_m9n.h"
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

    snvs_hp_rtc_datetime_t rtcDateTime;
	s_loggerStatus = LoggerInit();
	s_gpsStatus = GpsInit();

	// Wait for GPS to acquire a valid time
	// Wait up to 3 min - 720 250ms ticks
	gps_time_t* pGpsTime = NEOM9N_GpsTime();
	uint32_t timeTicks = 0U;
	while (((pGpsTime->valid & 0x07) != 0x07)
	        && (timeTicks++ < 720)) {
	    vTaskDelay(250U);
	    NEOM9N_Process();
	}
	if ((pGpsTime->valid & 0x07) == 0x07) {
	    rtcDateTime.year = pGpsTime->year;
	    rtcDateTime.month = pGpsTime->month;
	    rtcDateTime.day = pGpsTime->day;
	    rtcDateTime.hour = pGpsTime->hour;
	    rtcDateTime.minute = pGpsTime->min;
	    rtcDateTime.second = pGpsTime->sec;
	    SNVS_HP_RTC_SetDatetime(SNVS, &rtcDateTime);
	}

	StartTasks(pvParameters);

	vTaskDelete(NULL);
}

