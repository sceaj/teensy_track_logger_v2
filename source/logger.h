/*
 * logger.h
 *
 *  Created on: Feb 6, 2021
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

#ifndef LOGGER_H_
#define LOGGER_H_

#include "ff.h"
#include "fsl_sd_disk.h"

#define kStatusGroup_LOGGER 203
enum {
    kStatus_LOGGER_NoDir = MAKE_STATUS(kStatusGroup_LOGGER, 0),
    kStatus_LOGGER_Error1 = MAKE_STATUS(kStatusGroup_LOGGER, 1),
    kStatus_LOGGER_Error2 = MAKE_STATUS(kStatusGroup_LOGGER, 2),
    kStatus_LOGGER_Error3 = MAKE_STATUS(kStatusGroup_LOGGER, 3),
};

extern QueueHandle_t dataLogQueue;
extern QueueHandle_t debugLogQueue;

void LOGGER_WriteDebug(const char* message);
void LOGGER_WriteDebugFromISR(const char* message);

status_t LoggerInit(void);

void LoggerTask( void *pvParameters );


#endif /* LOGGER_H_ */
