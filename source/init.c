/*
 * init.c
 *
 *  Created on: Feb 14, 2021
 *      Author: sceaj
 */

#include "init.h"
#include "blinker.h"
#include "fsl_iomuxc.h"
#include "gps.h"
#include "logger.h"
#include "sdmmc_config.h"
#include "shell_task.h"

extern TaskHandle_t g_BlinkTaskHandle;
extern TaskHandle_t g_GpsTaskHandle;
extern TaskHandle_t g_LoggerTaskHandle;
extern TaskHandle_t g_ShellTaskHandle;

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

static status_t LoggerInit(void) {
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

void StartTasks(void *pvParameters) {
    vTaskResume(g_BlinkTaskHandle);
    vTaskDelay(500);
    vTaskResume(g_ShellTaskHandle);
	vTaskDelay(500);
//	vTaskResume(g_GpsTaskHandle);
//	vTaskDelay(500);
    vTaskResume(g_LoggerTaskHandle);
}

void InitTask(void *pvParameters) {
	if ((LoggerInit() == kStatus_Success)
			&& (GpsInit() == kStatus_Success)) {
		StartTasks(pvParameters);
		vTaskDelete(NULL);
	} else {
		for(;;) {
			PRINTF("Initialization failed!\n");
			vTaskDelay(5000);
		}
	}
}

