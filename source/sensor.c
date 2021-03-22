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
#include "FreeRTOS.h"
#include "fsl_debug_console.h"
#include "fxos8700.h"
#include "fxas21002.h"
#include "peripherals.h"
#include "task.h"

volatile bool busy;

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

    case fxos8700_Acceleration:
        break;

    case fxos8700_Magnetometer:
        break;

    case fxas21002_WhoAmI:
        PRINTF("FXAS21002 WhoAmI: 0x%X  [%d]\n", (*(uint8_t*)handle->transfer.data), completionStatus);
        break;

    case fxas21002_Config:
        PRINTF("FXAS21002 Config: [%d]\n", completionStatus);
        break;

    case fxas21002_Gyro:
        break;
    }
    busy = false;
}

status_t SensorInit(void) {

    LPI2C1_masterHandle.completionCallback = Sensor_CompletionCallback;

    busy = true;
//    FXOS8700_WhoAmI();
//    busy = true;
//    FXAS21002_WhoAmI();
//    while (busy) {
//    }
    status_t status = FXOS8700_Configure();
    if (kStatus_Success == status) {
        while (busy) {
        }
        status = FXAS21002_Configure();
        while (busy) {
        }
    }
    return status;
}

void SensorTask( void *pvParameters ) {

    const TickType_t xPeriod = 20;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t outputCounter = 0U;

    for (;;) {

        FXOS8700_Process();
        FXAS21002_Process();

        if ((++outputCounter % 50) == 0) {
            sensor_accel_t* pAccelerometer = FXOS8700_Acceleration();
            PRINTF("Accelerometer x:%03d  y:%03d  z:%03d\n",
                    pAccelerometer->x,
                    pAccelerometer->y,
                    pAccelerometer->z);

            sensor_mags_t* pMagnetometer = FXOS8700_Compass();
            PRINTF("Magnetometer x:%03d  y:%03d  z:%03d\n",
                    pMagnetometer->x,
                    pMagnetometer->y,
                    pMagnetometer->z);

            sensor_gyro_t* pGyro = FXAS21002_Gyro();
            PRINTF("Gyro  x:%03d  y:%03d  z:%03d\n",
                    pGyro->x,
                    pGyro->y,
                    pGyro->z);
        }

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
