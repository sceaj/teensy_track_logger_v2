/*
 * init.c
 *
 *  Created on: Feb 14, 2021
 *      Author: sceaj
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

#include "init.h"
#include "blinker.h"
#include "fsl_snvs_hp.h"
#include "gps.h"
#include "neo_m9n.h"
#include "logger.h"
#include "sensor.h"
#include "shell_task.h"

extern TaskHandle_t g_BlinkTaskHandle;
extern TaskHandle_t g_GpsTaskHandle;
extern TaskHandle_t g_LoggerTaskHandle;
extern TaskHandle_t g_SensorTaskHandle;
extern TaskHandle_t g_ShellTaskHandle;

static status_t s_loggerStatus;
static status_t s_gpsStatus;
static status_t s_sensorStatus;


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
    if (kStatus_Success == s_sensorStatus) {
        vTaskResume(g_SensorTaskHandle);
        vTaskDelay(500);
    } else {
        PRINTF("Sensor initialization failed - not resuming task\n");
    }
}

void InitTask(void *pvParameters) {

    snvs_hp_rtc_datetime_t rtcDateTime;
    gps_time_t gpsTime;
	s_loggerStatus = LoggerInit();
	s_gpsStatus = GpsInit();
	s_sensorStatus = SensorInit();

	// Wait for GPS to acquire a valid time
	// Wait up to 3 min - 720 250ms ticks
	uint32_t timeTicks = 0U;
	gpsTime.valid = 0;
	while (((gpsTime.valid & 0x07) != 0x07)
	        && (timeTicks++ < 720)) {
	    vTaskDelay(250U);
	    NEOM9N_Process(&gpsTime, NULL, NULL);
	}
	if ((gpsTime.valid & 0x07) == 0x07) {
	    rtcDateTime.year   = gpsTime.year;
	    rtcDateTime.month  = gpsTime.month;
	    rtcDateTime.day    = gpsTime.day;
	    rtcDateTime.hour   = gpsTime.hour;
	    rtcDateTime.minute = gpsTime.min;
	    rtcDateTime.second = gpsTime.sec;
	    SNVS_HP_RTC_SetDatetime(SNVS, &rtcDateTime);
	}

	StartTasks(pvParameters);

	vTaskDelete(NULL);
}

