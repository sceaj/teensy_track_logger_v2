/*
 * fxos8700cq.h
 *
 *  Created on: Mar 8, 2021
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

#ifndef FXOS8700_H_
#define FXOS8700_H_

#include "fsl_common.h"

#define kStatusGroup_FXOS8700 202
enum {
    kStatus_FXOS8700_WrongDevice = MAKE_STATUS(kStatusGroup_FXOS8700, 0),
    kStatus_FXOS8700_BusBusy = MAKE_STATUS(kStatusGroup_FXOS8700, 1),
    kStatus_FXOS8700_Error2 = MAKE_STATUS(kStatusGroup_FXOS8700, 2),
    kStatus_FXOS8700_Error3 = MAKE_STATUS(kStatusGroup_FXOS8700, 3),
};

// UserData specified for specific requests
enum {
    fxos8700_WhoAmI         = 0xC701U,
    fxos8700_Config         = 0xC702U,
    fxos8700_Data           = 0xC703U,
};

// scaled to LSB = 0.01g
typedef struct sensor_accel {
    int16_t     x;
    int16_t     y;
    int16_t     z;
} sensor_accel_t;

typedef struct sensor_mags {
    int16_t     x;
    int16_t     y;
    int16_t     z;

} sensor_mags_t;

extern uint8_t fxos8700SensorData[13];

status_t FXOS8700_WhoAmI();

status_t FXOS8700_Configure();

uint8_t FXOS8700_Flags();

status_t FXOS8700_RequestData();

#endif /* FXOS8700_H_ */
