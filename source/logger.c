/*
 * logger.c
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

#include "logger.h"
#include "board.h"
#include "FreeRTOS.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_iomuxc.h"
#include "fsl_sd.h"
#include "fsl_snvs_hp.h"
#include "logrecord.h"
#include "sdmmc_config.h"
#include "semphr.h"
#include "stdio.h"
#include "string.h"


#define LOGGER_SD_CARD_RETRY_DELAY (5000U)
#define LOGGER_SECTOR_PER_WRITE (4U)
#define LOGGER_BUFFER_SECTOR_COUNT (8U)
#define LOG_RECORD_PER_SECTOR (16U)


typedef union log_sector {
    uint8_t         sector[LOG_RECORD_PER_SECTOR * sizeof(log_record_t)];
    log_record_t    record[LOG_RECORD_PER_SECTOR];
} log_buffer_t;

QueueHandle_t dataLogQueue;
QueueHandle_t debugLogQueue;

FATFS g_fileSystem; /* File system object */

static const char* s_debugPathname = "2:/tracklogger.dbg";
static FIL s_debugFile;
static char debugMessageBuffer[80];

static char s_logPathname[40];
static FIL s_logFile;   /* File object */
static uint32_t s_logFileNum;
static char s_logFilePrefix[6];
static uint32_t s_logFileSize;
static uint32_t s_logRollSize = 67108864U;
static const char* s_logRootPath = "2:/logger";

static __attribute__((aligned(4))) log_buffer_t s_logBuffer[LOGGER_BUFFER_SECTOR_COUNT];
static int s_logBufferHead;
static int s_logBufferTail;
static int s_logBufferFrame;

static
void setPathname(int filenum) {
    static char filename[13];

    sprintf(filename, "%s%03d", s_logFilePrefix, filenum);
    strcpy(s_logPathname, s_logRootPath);
    strcat(s_logPathname, "/");
    strcat(s_logPathname, filename);
    strcat(s_logPathname, ".tlg");
}

static status_t OpenLog(const char* prefix) {
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory search object */
    static FILINFO fno;    /* File information */

    strcpy(s_logFilePrefix, prefix);

    static char filename[13];
    memset(filename, 0, sizeof filename);
    // Check that the logging directory exists, create it if not
    fr = f_stat(s_logRootPath, &fno);
    // May need to check FR_NO_PATH
    if (fr == FR_NO_FILE) {
        fr = f_mkdir(s_logRootPath);
    }
    if (fr != FR_OK) {
        return kStatus_LOGGER_NoDir;
    }
    // Search existing files of requested prefix
    char logFilePattern[13];
    strcpy(logFilePattern, prefix);
    strcat(logFilePattern, "*.tlg");
    int fileNum = 0;

    fr = f_findfirst(&dj, &fno, s_logRootPath, logFilePattern);  /* Start to search for logger files */
    while (fr == FR_OK && fno.fname[0]) { /* Repeat while an item is found */
        strcpy(filename, fno.fname);
        *strrchr(filename, '.') = 0;
        int currentFileNum = atoi(&filename[strlen(prefix)]);
        if (currentFileNum > fileNum) {
            fileNum = currentFileNum;
        }
        fr = f_findnext(&dj, &fno); /* Search for next item */
    }
    fr = f_closedir(&dj);

    s_logFileNum = ++fileNum;

    if (fr == FR_OK) {
        // Create the next filename
        setPathname(s_logFileNum);
        // Open the file
        fr = f_open(&s_logFile, s_logPathname, FA_CREATE_NEW | FA_WRITE);
        PRINTF("Opening: %s [%s]\r\n", s_logPathname, fr == FR_OK ? "OK" : "Failed");
        s_logFileSize = 0;
    }

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

static status_t CloseLog(void) {
    FRESULT fr = f_close(&s_logFile);
    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

static status_t RollLog(void) {

    FRESULT fr;     /* Return value */
    //
    if (s_logFileSize >= s_logRollSize) {
        // Roll file...
        f_close(&s_logFile);
        setPathname(++s_logFileNum);
        fr = f_open(&s_logFile, s_logPathname, FA_OPEN_ALWAYS | FA_WRITE);
        PRINTF("Opening: %s [%s]\r\n", s_logPathname, fr == FR_OK ? "OK" : "Failed");
        s_logFileSize = 0;
    } else {
        fr = f_sync(&s_logFile);
    }

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

static status_t WriteLog(int bufferCount) {
    static unsigned int writeCount = 0;
    unsigned int writeSize;
    unsigned int bytesWritten;
    FRESULT fr = FR_DISK_ERR;
    if ((s_logBufferHead + bufferCount) <= LOGGER_BUFFER_SECTOR_COUNT) {
        writeSize = bufferCount * sizeof(log_buffer_t);
        fr = f_write(&s_logFile, &s_logBuffer[s_logBufferHead], writeSize, &bytesWritten);
        s_logBufferHead = (s_logBufferHead + bufferCount) % LOGGER_BUFFER_SECTOR_COUNT;
        if ((fr != FR_OK) || (bytesWritten != writeSize)) {
            // TODO: error output
        }
    } else {
        writeSize = sizeof(log_buffer_t);
        for (int i = 0; i < bufferCount; i++) {
            fr = f_write(&s_logFile, &s_logBuffer[s_logBufferHead], writeSize, &bytesWritten);
            s_logBufferHead = (s_logBufferHead + 1) % LOGGER_BUFFER_SECTOR_COUNT;
            if ((fr != FR_OK) || (bytesWritten != writeSize)) {
                // TODO: error output
            }
        }
    }
    s_logFileSize += bytesWritten;
    if (!(++writeCount % 16) && (fr == FR_OK)) {
        RollLog();
    }

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

static status_t OpenDebug(void) {
    FRESULT fr = f_open(&s_debugFile, s_debugPathname, FA_CREATE_ALWAYS | FA_WRITE);
    PRINTF("Opening: %s [%s]\r\n", s_debugPathname, fr == FR_OK ? "OK" : "Failed");
    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

static status_t CloseDebug(void) {
    FRESULT fr = f_close(&s_debugFile);
    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

static void WriteDebug(const char* message) {

    snvs_hp_rtc_datetime_t rtcDateTime;
    TickType_t millis = xTaskGetTickCount();
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();

    SNVS_HP_RTC_GetDatetime(SNVS, &rtcDateTime);

    f_printf(&s_debugFile, "%04u-%02u-%02uT%02u:%02u:%02u [%u.%u] [%s] %s\n",
            rtcDateTime.year,
            rtcDateTime.month,
            rtcDateTime.day,
            rtcDateTime.hour,
            rtcDateTime.minute,
            rtcDateTime.second,
            millis / 1000U,
            millis % 1000U,
            pcTaskGetName(taskHandle),
            message);
}


static void SdcardInsertedCallback(bool isInserted, void *userData) {
	PRINTF("Card detection: %s\n", (isInserted ? "present" : "absent"));
}

static status_t SdcardWaitCardInsert(void)
{
    BOARD_SD_Config(&g_sd, SdcardInsertedCallback, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

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
    dataLogQueue = xQueueCreate(8, sizeof(log_record_t));
    if (dataLogQueue == NULL) {
        PRINTF("Data logging queue creation failed!\n");
    }
    debugLogQueue = xQueueCreate(4, 80);
    if (debugLogQueue == NULL) {
        PRINTF("Debug logging queue creation failed!\n");
    }

	return initStatus;
}

void LoggerTask( void *pvParameters )
{

	PRINTF("Entering LoggerTask...\n");
	OpenDebug();
	OpenLog("data");
	WriteDebug("Test Message");
	CloseDebug();
	for(;;) {
	    log_record_t* pBuffer = &s_logBuffer[s_logBufferTail].record[s_logBufferFrame];
	    if (xQueueReceive(dataLogQueue, pBuffer, 4U) == pdTRUE) {
	        s_logBufferFrame++;
	        if (s_logBufferFrame == LOG_RECORD_PER_SECTOR) {
	            s_logBufferTail++;
	            s_logBufferFrame = 0;
	        }
	        if ((s_logBufferTail - s_logBufferHead) >= LOGGER_SECTOR_PER_WRITE) {
	            WriteLog(LOGGER_SECTOR_PER_WRITE);
	        }
	        s_logBufferTail = s_logBufferTail % LOGGER_BUFFER_SECTOR_COUNT;
	    }
        if (xQueueReceive(debugLogQueue, debugMessageBuffer, 0U) == pdTRUE) {
            WriteDebug(debugMessageBuffer);
        }
	}
	CloseLog();
	CloseDebug();
	vTaskDelete(NULL);

}



