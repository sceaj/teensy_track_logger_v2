/*
 * fxas21002.h
 *
 *  Created on: Mar 11, 2021
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

#ifndef FXAS21002_H_
#define FXAS21002_H_

#include "fsl_common.h"

#define kStatusGroup_FXAS21002 203
enum {
    kStatus_FXAS21002_WrongDevice = MAKE_STATUS(kStatusGroup_FXAS21002, 0),
    kStatus_FXAS21002_BusBusy = MAKE_STATUS(kStatusGroup_FXAS21002, 1),
    kStatus_FXAS21002_Error2 = MAKE_STATUS(kStatusGroup_FXAS21002, 2),
    kStatus_FXAS21002_Error3 = MAKE_STATUS(kStatusGroup_FXAS21002, 3),
};

// UserData specified for specific requests
enum {
    fxas21002_WhoAmI        = 0xD701U,
    fxas21002_Config        = 0xD702U,
    fxas21002_Data          = 0xD703U,
};

typedef enum {
    FXAS21002_Standby,
    FXAS21002_Ready,
    FXAS21002_Active,
} sensor_gyro_mode_t;


typedef struct sensor_gyro {
    int16_t     x;
    int16_t     y;
    int16_t     z;
} sensor_gyro_t;

extern uint8_t fxas21002SensorData[7];

status_t FXAS21002_WhoAmI();

status_t FXAS21002_Mode(sensor_gyro_mode_t mode);

status_t FXAS21002_Configure();

status_t FXAS21002_RequestData();

#endif /* FXAS21002_H_ */
