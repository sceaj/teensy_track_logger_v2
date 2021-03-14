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
#include "board.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_iomuxc.h"
#include "fsl_sd.h"
#include "sdmmc_config.h"


#define LOGGER_SD_CARD_RETRY_DELAY (5000U)

FATFS g_fileSystem; /* File system object */
static FIL g_fileObject;   /* File object */
static DIR s_directory;
static FILINFO s_fileInformation;

static void SdcardInsertedCallback(bool isInserted, void *userData) {
	PRINTF("Card detection: %s\n", (isInserted ? "present" : "absent"));
}

static status_t SdcardWaitCardInsert(void)
{
	PRINTF("Entering BOARD_SD_Config...\n");
    BOARD_SD_Config(&g_sd, SdcardInsertedCallback, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

    PRINTF("Initializing SD host...\n");
	/* SD host init function */
	if (SD_HostInit(&g_sd) != kStatus_Success)
	{
		PRINTF("SD host init failed\n");
		return kStatus_Fail;
	}
	/* power off card */
	SD_SetCardPower(&g_sd, false);

    PRINTF("Polling for SD card...\n");
    /* wait card insert */
    if (SD_PollingCardInsert(&g_sd, kSD_Inserted) == kStatus_Success) {
        PRINTF("Card inserted.\n");
        /* power on the card */
        SD_SetCardPower(&g_sd, true);
    } else {
        PRINTF("Card detect fail.\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t MountSdFileSystem(void) {
	FRESULT fresult = f_mount(&g_fileSystem, "2:/", 1);
	if (fresult == FR_OK) {
		PRINTF("SD filesystem mounted.\n");
		return kStatus_Success;
	} else {
		PRINTF("f_mount failed! [%d]\n", fresult);
		fresult = f_mount(NULL, "2:/", 0);
		PRINTF("f_mount (unmount): [%d]\n", fresult);
		return kStatus_Fail;
	}
}

status_t LoggerInit(void) {
	PRINTF("Entering LoggerInit...\n");
	int attempts = 0;
	status_t initStatus = -1;
	while (attempts++ < 6 && initStatus != kStatus_Success) {
		initStatus = SdcardWaitCardInsert();
		if (kStatus_Success == initStatus) {
			vTaskDelay(200);
			initStatus = MountSdFileSystem();
			if (kStatus_Success != initStatus) {
				PRINTF("Failed to mount filesystem. Waiting to retry...\n");
				IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD, 0U);
				IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK, 0U);
				IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0, 0U);
				IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1, 0U);
				IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2, 0U);
				IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, 0U);
				vTaskDelay(5000);
			}
		} else {
			PRINTF("Waiting for SD card...\n");
		}
	}
	return initStatus;
}

void LoggerTask( void *pvParameters )
{

	PRINTF("Entering LoggerTask...\n");
	for(;;) {

		vTaskDelay(10000U);

        uint32_t pres_state = g_sd.host->hostController.base->PRES_STATE;
//        PRINTF("PRES_STATE: 0x%X\n", pres_state);
        uint32_t int_status = g_sd.host->hostController.base->INT_STATUS;
//        PRINTF("INT_STATUS: 0x%X\n", int_status);

//	    PRINTF("Create directory......\n");
	    FRESULT error = f_mkdir("2:/dir_1");
	    if (error)
	    {
	        if (error == FR_EXIST)
	        {
//	            PRINTF("Directory exists.\n");
	        }
	        else
	        {
	            PRINTF("Make directory failed.\n");
	        }
	    }

//	    PRINTF("Create a file in that directory......\n");
	    error = f_open(&g_fileObject, "2:/dir_1/f_1.dat", (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
	    if (error)
	    {
	        if (error == FR_EXIST)
	        {
//	            PRINTF("File exists.\n");
	        }
	        else
	        {
	            PRINTF("Open file failed.\n");
	        }
	    }

//	    PRINTF("List the files in directory......\n");
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
//	            PRINTF("Directory file : %s.\n", s_fileInformation.fname);
	        }
	        else
	        {
//	            PRINTF("General file : %s.\n", s_fileInformation.fname);
	        }
	    }
	}

	vTaskDelete(NULL);

}



