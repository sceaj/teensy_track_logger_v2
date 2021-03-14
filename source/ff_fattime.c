/*
 * ff_fattime.c
 *
 *  Created on: Mar 13, 2021
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

#include "ff.h"
#include "fsl_snvs_hp.h"

typedef struct fatfs_ftime {
    unsigned second:5;
    unsigned minute:6;
    unsigned hour:5;
    unsigned day:5;
    unsigned month:4;
    unsigned year:7;
} fatfs_ftime_t;

typedef union {
    DWORD           value;
    fatfs_ftime_t   fileTime;
} ftime_t;

/*
 * Implementation of the realtime clock function to timestamp files in FatFS.
 * We use the RTC peripheral to get the datatime.
 */
DWORD get_fattime(void) {

    snvs_hp_rtc_datetime_t rtcDateTime;
    ftime_t timestamp;

    SNVS_HP_RTC_GetDatetime(SNVS, &rtcDateTime);
    timestamp.fileTime.year = rtcDateTime.year - 1980;
    timestamp.fileTime.month = rtcDateTime.month;
    timestamp.fileTime.day = rtcDateTime.day;
    timestamp.fileTime.hour = rtcDateTime.hour;
    timestamp.fileTime.minute = rtcDateTime.minute;
    timestamp.fileTime.second = rtcDateTime.second / 2;

    return timestamp.value;
}
