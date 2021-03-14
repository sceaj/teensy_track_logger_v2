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
#include "neo_m9n.h"
#include "task.h"


status_t GpsInit(void) {
	status_t status = NEOM9N_UartInit();
	if (kStatus_Success == status) {
		status = NEOM9N_GpsConfig();
	}
	// TODO: Use real status
	return status;
}

void GpsTask(void *pvParameters) {

    const TickType_t xPeriod = 20;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t lastGpsTimeITOW = 0U;
    uint32_t lastGpsPositionITOW = 0U;
    uint32_t lastGpsVelocityITOW = 0U;

    for (;;) {

        NEOM9N_Process();

        gps_time_t *gpsTime = NEOM9N_GpsTime();
        if (gpsTime->iTOW != lastGpsTimeITOW) {
            PRINTF("GPS Time  iTOW:%u  year:%u month:%u day:%u hour:%u min:%u sec:%u nano:%d valid:0x%X flags2:0x%X\n",
                    gpsTime->iTOW, gpsTime->year, gpsTime->month, gpsTime->day,
                    gpsTime->hour, gpsTime->min, gpsTime->sec, gpsTime->nano,
                    gpsTime->valid, gpsTime->flags2);
            lastGpsTimeITOW = gpsTime->iTOW;
        }

        gps_position_t *gpsPosition = NEOM9N_GpsPosition();
        if (gpsPosition->iTOW != lastGpsPositionITOW) {
            PRINTF("GPS Position  iTOW:%u lon:%d lat:%d height:%d hMSL:%u hAcc:%u fixType:%u flags:0x%X\n",
                    gpsPosition->iTOW, gpsPosition->lon, gpsPosition->lat,
                    gpsPosition->height, gpsPosition->hMSL, gpsPosition->hAcc,
                    gpsPosition->fixType, gpsPosition->flags);
            lastGpsPositionITOW = gpsPosition->iTOW;
        }

        gps_velocity_t *gpsVelocity = NEOM9N_GpsVelocity();
        if (gpsVelocity->iTOW != lastGpsVelocityITOW) {
            PRINTF("GPS Velocity  iTOW:%u gSpeed:%d headMot:%d sAcc:%u headAcc:%u pDOP:%u flags:0x%X\n",
                    gpsVelocity->iTOW, gpsVelocity->gSpeed, gpsVelocity->headMot,
                    gpsVelocity->sAcc, gpsVelocity->headAcc, gpsVelocity->pDOP,
                    gpsVelocity->flags);
            lastGpsVelocityITOW = gpsVelocity->iTOW;
        }
        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

