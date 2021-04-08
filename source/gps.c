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
#include "logger.h"
#include "logrecord.h"
#include "neo_m9n.h"
#include "queue.h"
#include "task.h"

typedef struct gps_time_record {
    record_header_t     header;
    gps_time_t          gpsTime;
    uint8_t             unused[11];
    uint8_t             chkA;
    uint8_t             chkB;
} log_gps_time_t;

typedef struct gps_position_record {
    record_header_t     header;
    gps_position_t      gpsPosition;
    uint8_t             unused[3];
    uint8_t             chkA;
    uint8_t             chkB;
} log_gps_position_t;

typedef struct gps_velocity_record {
    record_header_t     header;
    gps_velocity_t      gpsVelocity;
    uint8_t             unused[3];
    uint8_t             chkA;
    uint8_t             chkB;
} log_gps_velocity_t;

static log_gps_time_t       s_timeRecord;
static log_gps_position_t   s_positionRecord;
static log_gps_velocity_t   s_velocityRecord;

status_t GpsInit(void) {
	status_t status = NEOM9N_UartInit();
	if (kStatus_Success == status) {
		status = NEOM9N_GpsConfig();
	}
	return status;
}

void GpsTask(void *pvParameters) {

    const TickType_t xPeriod = 5;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t outputCounter = 0U;

    LogRecordHeaderInit(&s_timeRecord.header, LOGRECORD_CLASS_GPS, GPS_TIME);
    LogRecordHeaderInit(&s_positionRecord.header, LOGRECORD_CLASS_GPS, GPS_POSITION);
    LogRecordHeaderInit(&s_velocityRecord.header, LOGRECORD_CLASS_GPS, GPS_VELOCITY);

    for (;;) {

        if (NEOM9N_Process(&s_timeRecord.gpsTime, &s_positionRecord.gpsPosition, &s_velocityRecord.gpsVelocity)) {
            s_timeRecord.header.timestamp = xTaskGetTickCount();
            LogRecordFinalize((log_record_t*)&s_timeRecord);
            xQueueSend(dataLogQueue, &s_timeRecord, 2U);
            s_positionRecord.header.timestamp = s_timeRecord.header.timestamp;
            LogRecordFinalize((log_record_t*)&s_positionRecord);
            xQueueSend(dataLogQueue, &s_positionRecord, 2U);
            s_velocityRecord.header.timestamp = s_timeRecord.header.timestamp;
            LogRecordFinalize((log_record_t*)&s_velocityRecord);
            xQueueSend(dataLogQueue, &s_velocityRecord, 2U);
            outputCounter++;
        }

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

