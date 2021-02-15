/*
 * shell_task.c
 *
 *  Created on: Jan 30, 2021
 *      Author: jrosen
 */

#include <blinker.h>
#include <stdarg.h>
#include <stdlib.h>

#include "shell_task.h"
#include "eeprom.h"
#include "FreeRTOS.h"
#include "fsl_debug_console.h"
#include "fsl_shell.h"
#include "task.h"

static SERIAL_MANAGER_HANDLE_DEFINE(s_serialHandle);
static SHELL_HANDLE_DEFINE(s_shellHandle);

static uint8_t s_shellReadRingBuffer[SHELL_RECEIVE_BUFFER_LEN];

shell_status_t BlinkCommand(shell_handle_t shellHandle, int argc, const char **argv);
static SHELL_COMMAND_DEFINE(blink,
					"\"blink\": Update the blink timings. On ticks in arg1, Off ticks in arg2\n",
					BlinkCommand, 2);
shell_status_t EchoCommand(shell_handle_t shellHandle, int argc, const char **argv);
static SHELL_COMMAND_DEFINE(echo,
					"\"echo\": Tesing the shell input/output by echoing the input\n",
					EchoCommand, SHELL_IGNORE_PARAMETER_COUNT);
shell_status_t EepromCommand(shell_handle_t shellHandle, int argc, const char **argv);
static SHELL_COMMAND_DEFINE(eeprom,
					"\"eeprom\": Read or write to simulated eeprom (in device flash)\n",
					EepromCommand, 3);

static volatile bool shellRunning = true;

void ShutdownShell() {
	shellRunning = false;
}

static serial_manager_status_t SerialInit(uint8_t instance, uint32_t baudRate, serial_port_type_t device, uint32_t clkSrcFreq) {

    serial_manager_config_t serialConfig;

    serial_port_uart_config_t uartConfig = {
        .instance     = instance,
        .clockRate    = clkSrcFreq,
        .baudRate     = baudRate,
        .parityMode   = kSerialManager_UartParityDisabled,
        .stopBitCount = kSerialManager_UartOneStopBit,
        .enableRx     = 1,
        .enableTx     = 1,
    };

    serialConfig.type           = device;
    serialConfig.ringBuffer     = &s_shellReadRingBuffer[0];
    serialConfig.ringBufferSize = SHELL_RECEIVE_BUFFER_LEN;

    serialConfig.portConfig = &uartConfig;

    return SerialManager_Init(s_serialHandle, &serialConfig);
}

shell_status_t ShellTaskInit( void *pvParameters ) {
	serial_manager_status_t serialStatus = SerialInit(SHELL_SERIAL_UART_INSTANCE, SHELL_SERIAL_BAUD_RATE, kSerialPort_Uart, SHELL_SERIAL_CLK_FREQ);
	shell_status_t status = kStatus_SHELL_Error;
	if (serialStatus == kStatus_SerialManager_Success) {
		shell_status_t status = SHELL_Init(s_shellHandle, s_serialHandle, "$>");
		if (status == kStatus_SHELL_Success) {
			SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(blink));
			SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(echo));
			SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(eeprom));
		}
	}
	return status;
}

void ShellTask( void *pvParameters ) {

	PRINTF("Entering ShellTask...\n");
	if (ShellTaskInit(pvParameters) == kStatus_SHELL_Success) {
		while (shellRunning) {
			taskYIELD();
		}
	}

	vTaskDelete(NULL);
}

shell_status_t BlinkCommand(shell_handle_t shellHandle, int argc, const char **argv) {

	g_blinkOnTicks = atoi(argv[1]);
	g_blinkOffTicks = atoi(argv[2]);

	return kStatus_Success;
}

shell_status_t EchoCommand(shell_handle_t shellHandle, int argc, const char **argv) {

	PRINTF(" Echo command received: \n");

	for (int i = 0; i < argc; i++) {
		DbgConsole_Printf("%s  ", *argv++);
	}

	DbgConsole_Putchar('\n');

	return kStatus_Success;
}

shell_status_t EepromCommand(shell_handle_t shellHandle, int argc, const char **argv) {

	PRINTF(" eeprom command received: \n");

	char* hexEnd;
	unsigned int *addr = (unsigned int*)(strtol(argv[2], &hexEnd, 16));
	unsigned int value = (unsigned int)(strtol(argv[3], &hexEnd, 16));
	if (argv[1][0] == 'w') {
		PRINTF("Writing %x to %x\n", value, addr);
		eeprom_write_dword(addr, value);
		PRINTF("Done!\n");
	} else {
		uint32_t readValue = eeprom_read_dword(addr);
		PRINTF("Read %x @ %x\n", value, addr);
	}

	return kStatus_Success;
}
