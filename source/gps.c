/*
 * gps.c
 *
 *  Created on: Feb 15, 2021
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

#include "gps.h"
#include "FreeRTOS.h"
#include "fsl_debug_console.h"
#include "fsl_snvs_hp.h"
#include "neo_m9n.h"
#include "task.h"


status_t GpsInit(void) {
	status_t status = NEOM9N_UartInit();
	if (kStatus_Success == status) {
		status = NEOM9N_GpsConfig();
	}
	return status;
}

void GpsTask(void *pvParameters) {

    const TickType_t xPeriod = 20;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t outputCounter = 0U;
    snvs_hp_rtc_datetime_t rtcDateTime;

    for (;;) {

        NEOM9N_Process();

        if ((++outputCounter % 50) == 0) {
            gps_time_t *gpsTime = NEOM9N_GpsTime();
//            PRINTF(
//                    "GPS Time  iTOW:%u  year:%u month:%u day:%u hour:%u min:%u sec:%u nano:%d valid:0x%X flags2:0x%X\n",
//                    gpsTime->iTOW, gpsTime->year, gpsTime->month,
//                    gpsTime->day, gpsTime->hour, gpsTime->min, gpsTime->sec,
//                    gpsTime->nano, gpsTime->valid, gpsTime->flags2);
            SNVS_HP_RTC_GetDatetime(SNVS, &rtcDateTime);
//            PRINTF(
//                    "RTC Time                 year:%u month:%u day:%u hour:%u min:%u sec:%u\n",
//                    rtcDateTime.year, rtcDateTime.month,
//                    rtcDateTime.day, rtcDateTime.hour,
//                    rtcDateTime.minute, rtcDateTime.second);


            gps_position_t *gpsPosition = NEOM9N_GpsPosition();
//            PRINTF(
//                    "GPS Position  iTOW:%u lon:%d lat:%d height:%d hMSL:%u hAcc:%u fixType:%u flags:0x%X\n",
//                    gpsPosition->iTOW, gpsPosition->lon, gpsPosition->lat,
//                    gpsPosition->height, gpsPosition->hMSL,
//                    gpsPosition->hAcc, gpsPosition->fixType,
//                    gpsPosition->flags);

            gps_velocity_t *gpsVelocity = NEOM9N_GpsVelocity();
//            PRINTF(
//                    "GPS Velocity  iTOW:%u gSpeed:%d headMot:%d sAcc:%u headAcc:%u pDOP:%u flags:0x%X\n",
//                    gpsVelocity->iTOW, gpsVelocity->gSpeed,
//                    gpsVelocity->headMot, gpsVelocity->sAcc,
//                    gpsVelocity->headAcc, gpsVelocity->pDOP,
//                    gpsVelocity->flags);
        }
        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

