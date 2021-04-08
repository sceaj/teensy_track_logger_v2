/*
 * sensor.c
 *
 *  Created on: Mar 15, 2021
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

#include "sensor.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "fsl_debug_console.h"
#include "fxos8700.h"
#include "fxas21002.h"
#include "logger.h"
#include "logrecord.h"
#include "peripherals.h"
#include "queue.h"
#include "task.h"

volatile bool busy;

typedef struct accel_record {
    record_header_t     header;
    sensor_accel_t      accelerations;
    sensor_mags_t       magnetometer;
    uint8_t             fxos8700Status;
    uint8_t             unused[9];
    uint8_t             chkA;
    uint8_t             chkB;
} log_accel_t;

typedef struct gyro_record {
    record_header_t     header;
    sensor_gyro_t       gyroRates;
    uint8_t             fxas21002Status;
    uint8_t             unused[15];
    uint8_t             chkA;
    uint8_t             chkB;
} log_gyro_t;

static log_accel_t s_accelRecord;
static log_gyro_t s_gyroRecord;

volatile bool fxos8700DataAvailable;
volatile bool fxas21002DataAvailable;

void Sensor_CompletionCallback(LPI2C_Type *base,
        lpi2c_master_handle_t *handle,
        status_t completionStatus,
        void *userData) {

    uint32_t completionType = (uint32_t)userData;
    switch (completionType) {

    case fxos8700_WhoAmI:
        PRINTF("FXOS8700 WhoAmI: 0x%X  [%d]\n", (*(uint8_t*)handle->transfer.data), completionStatus);
        break;

    case fxos8700_Config:
        PRINTF("FXOS8700 Config: [%d]\n", completionStatus);
        break;

    case fxos8700_Data:
        s_accelRecord.fxos8700Status = fxos8700SensorData[0];
        int16_t rawAccelX = (int16_t)(fxos8700SensorData[1]<<8 | fxos8700SensorData[2]) >> 2;
        s_accelRecord.accelerations.x = (((rawAccelX * 25) + 1280) / 2560) * 5;
        int16_t rawAccelY = (int16_t)(fxos8700SensorData[3]<<8 | fxos8700SensorData[4]) >> 2;
        s_accelRecord.accelerations.y = (((rawAccelY * 25) + 1280) / 2560) * 5;
        int16_t rawAccelZ = (int16_t)(fxos8700SensorData[5]<<8 | fxos8700SensorData[6]) >> 2;
        s_accelRecord.accelerations.z = (((rawAccelZ * 25) + 1280) / 2560) * 5;

        s_accelRecord.magnetometer.x = (int16_t)(fxos8700SensorData[7]<<8 | fxos8700SensorData[8]);
        s_accelRecord.magnetometer.y = (int16_t)(fxos8700SensorData[9]<<8 | fxos8700SensorData[10]);
        s_accelRecord.magnetometer.z = (int16_t)(fxos8700SensorData[11]<<8 | fxos8700SensorData[12]);
        fxos8700DataAvailable = true;
        break;

    case fxas21002_WhoAmI:
        PRINTF("FXAS21002 WhoAmI: 0x%X  [%d]\n", (*(uint8_t*)handle->transfer.data), completionStatus);
        break;

    case fxas21002_Config:
        PRINTF("FXAS21002 Config: [%d]\n", completionStatus);
        break;

    case fxas21002_Data:
        s_gyroRecord.fxas21002Status = fxas21002SensorData[0];
        int16_t rawGyroX = (int16_t)(fxas21002SensorData[1]<<8 | fxas21002SensorData[2]);
        s_gyroRecord.gyroRates.x = (((rawGyroX * 625) + 312) / 20480);
        int16_t rawGyroY = (int16_t)(fxas21002SensorData[3]<<8 | fxas21002SensorData[4]);
        s_gyroRecord.gyroRates.y = (((rawGyroY * 625) + 312) / 20480);
        int16_t rawGyroZ = (int16_t)(fxas21002SensorData[5]<<8 | fxas21002SensorData[6]);
        s_gyroRecord.gyroRates.z = (((rawGyroZ * 625) + 312) / 20480);
        fxas21002DataAvailable = true;
        break;
    }
    busy = false;
}

status_t SensorInit(void) {

    LPI2C1_masterHandle.completionCallback = Sensor_CompletionCallback;

    busy = true;
    status_t status = FXOS8700_Configure();
    if (kStatus_Success == status) {
        while (busy) {
        }
        busy = true;
        status = FXAS21002_Configure();
        while (busy) {
        }
    }
    return status;
}

void SensorTask( void *pvParameters ) {

    const TickType_t xPeriod = 40;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t outputCounter;

    LogRecordHeaderInit(&s_accelRecord.header, LOGRECORD_CLASS_DYNAMICS, DYNAMICS_ACCELEROMETER);
    LogRecordHeaderInit(&s_gyroRecord.header, LOGRECORD_CLASS_DYNAMICS, DYNAMICS_GYRO);


    for (;;) {

        FXOS8700_RequestData();
        outputCounter = 0U;
        while (!fxos8700DataAvailable && (outputCounter < 100U)) {
            SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
            outputCounter++;
        }
        if (fxos8700DataAvailable) {
            s_accelRecord.header.timestamp = xTaskGetTickCount();
            LogRecordFinalize((log_record_t*)&s_accelRecord);
            xQueueSend(dataLogQueue, &s_accelRecord, 0U);
            fxos8700DataAvailable = false;
        }

        FXAS21002_RequestData();
        outputCounter = 0U;
        while (!fxas21002DataAvailable && (outputCounter < 100U)) {
            SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
            outputCounter++;
        }
        if (fxas21002DataAvailable) {
            s_gyroRecord.header.timestamp = xTaskGetTickCount();
            LogRecordFinalize((log_record_t*)&s_gyroRecord);
            xQueueSend(dataLogQueue, &s_gyroRecord, 0U);
            fxas21002DataAvailable = false;
        }

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

    }
}
