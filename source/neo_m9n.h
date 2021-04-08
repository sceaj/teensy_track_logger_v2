/*
 * neo_m9n.h
 *
 *  Created on: Feb 21, 2021
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

#ifndef NEO_M9N_H_
#define NEO_M9N_H_

#include "fsl_common.h"

#define kStatusGroup_GPS 201
enum {
    kStatus_GPS_NoFix = MAKE_STATUS(kStatusGroup_GPS, 0),
    kStatus_GPS_NoComm = MAKE_STATUS(kStatusGroup_GPS, 1),
    kStatus_GPS_CommError = MAKE_STATUS(kStatusGroup_GPS, 2),
    kStatus_GPS_Timeout = MAKE_STATUS(kStatusGroup_GPS, 3),
};

typedef struct gps_time {
    uint16_t    year;
    int16_t     millis;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     min;
    uint8_t     sec;
    uint8_t     valid;
    uint8_t     flags2;
} gps_time_t;

typedef struct gps_position {
    int32_t     lon;
    int32_t     lat;
    uint32_t    hMSL;
    uint32_t    hAcc;
    uint8_t     fixType;
    uint8_t     flags;
    uint8_t     flags3;
} gps_position_t;

typedef struct gps_velocity {
    int32_t     gSpeed;
    int32_t     headMot;
    uint32_t    sAcc;
    uint32_t    headAcc;
    uint16_t    pDOP;
    uint8_t     flags;
} gps_velocity_t;

status_t SetUartBaud(uint32_t newBaud);

status_t ReadData();

void NEOM9N_DebugCommCounters();

status_t NEOM9N_UartInit();

status_t NEOM9N_GpsConfig();

bool NEOM9N_Process();

#endif /* NEO_M9N_H_ */
