/*
 * logger.c
 *
 *  Created on: Feb 6, 2021
 *      Author: jrosen
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

#include "logger.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_sd.h"


#define LOGGER_SD_CARD_RETRY_DELAY (5000U)

FATFS g_fileSystem; /* File system object */
static FIL g_fileObject;   /* File object */
static DIR s_directory;
static FILINFO s_fileInformation;

void LoggerTask( void *pvParameters )
{

	PRINTF("Entering LoggerTask...\n");
	for(;;) {

		vTaskDelay(10000U);

        uint32_t pres_state = g_sd.host->hostController.base->PRES_STATE;
        PRINTF("PRES_STATE: 0x%X\n", pres_state);
        uint32_t int_status = g_sd.host->hostController.base->INT_STATUS;
        PRINTF("INT_STATUS: 0x%X\n", int_status);

	    PRINTF("Create directory......\n");
	    FRESULT error = f_mkdir("2:/dir_1");
	    if (error)
	    {
	        if (error == FR_EXIST)
	        {
	            PRINTF("Directory exists.\n");
	        }
	        else
	        {
	            PRINTF("Make directory failed.\n");
	        }
	    }

	    PRINTF("Create a file in that directory......\n");
	    error = f_open(&g_fileObject, "2:/dir_1/f_1.dat", (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
	    if (error)
	    {
	        if (error == FR_EXIST)
	        {
	            PRINTF("File exists.\n");
	        }
	        else
	        {
	            PRINTF("Open file failed.\n");
	        }
	    }

	    PRINTF("List the files in directory......\n");
	    if (f_opendir(&s_directory, "2:/dir_1"))
	    {
	        PRINTF("Open directory failed.\n");
	        continue;
	    }

	    for (;;)
	    {
	        error = f_readdir(&s_directory, &s_fileInformation);

	        /* To the end. */
	        if ((error != FR_OK) || (s_fileInformation.fname[0U] == 0U))
	        {
	            break;
	        }
	        if (s_fileInformation.fname[0] == '.')
	        {
	            continue;
	        }
	        if (s_fileInformation.fattrib & AM_DIR)
	        {
	            PRINTF("Directory file : %s.\n", s_fileInformation.fname);
	        }
	        else
	        {
	            PRINTF("General file : %s.\n", s_fileInformation.fname);
	        }
	    }
	}

	vTaskDelete(NULL);

}



