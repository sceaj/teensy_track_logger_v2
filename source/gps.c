/*
 * gps.c
 *
 *  Created on: Feb 15, 2021
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

#include "gps.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c_freertos.h"

#define UBX_CONFIG_SIZE_MASK  0x70000000
#define UBX_CONFIG_SIZE_SHIFT 28U
#define UBX_CONFIG_SIZE(key) ((key & UBX_CONFIG_SIZE_MASK) >> UBX_CONFIG_SIZE_SHIFT)

#define CFG_I2CINPROT_UBX     0x10710001
#define CFG_I2CINPROT_NMEA    0x10710002
#define CFG_I2CINPROT_RTCM3X  0x10710004
#define CFG_I2COUTPROT_UBX    0x10720001
#define CFG_I2COUTPROT_NMEA   0x10720002
#define CFG_INFMSG_UBX_I2C    0x20920001
#define CFG_INFMSG_UBX_UART1  0x20920002
#define CFG_INFMSG_UBX_UART2  0x20920003
#define CFG_INFMSG_UBX_USB    0x20920004
#define CFG_INFMSG_UBX_SPI    0x20920005
#define CFG_INFMSG_NMEA_I2C   0x20920006
#define CFG_INFMSG_NMEA_UART1 0x20920007
#define CFG_INFMSG_NMEA_UART2 0x20920008
#define CFG_INFMSG_NMEA_USB   0x20920009
#define CFG_INFMSG_NMEA_SPI   0x2092000a
#define CFG_INFMSG_ERROR      0x01
#define CFG_INFMSG_WARNING    0x02
#define CFG_INFMSG_NOTICE     0x04
#define CFG_INFMSG_TEST       0x08
#define CFG_INFMSG_DEBUG      0x10
#define CFG_NAVSPG_DYNMODEL   0x20110021
#define CFG_NAVSPG_DYNMODEL_PORT    0
#define CFG_NAVSPG_DYNMODEL_STAT    2
#define CFG_NAVSPG_DYNMODEL_PED     3
#define CFG_NAVSPG_DYNMODEL_AUTOMOT 4
#define CFG_NAVSPG_DYNMODEL_SEA     5
#define CFG_NAVSPG_DYNMODEL_AIR1    6
#define CFG_NAVSPG_DYNMODEL_AIR2    7
#define CFG_NAVSPG_DYNMODEL_AIR4    8
#define CFG_NAVSPG_DYNMODEL_WRIST   9
#define CFG_UART1_BAUDRATE    0x40520001
#define CFG_UART1_STOPBITS    0x20520002
#define CFG_UART1_DATABITS    0x20520003
#define CFG_UART1_PARITY      0x20520004
#define CFG_UART1_ENABLED     0x10520005
#define CFG_UART1INPROT_UBX   0x10730001
#define CFG_UART1INPROT_NMEA  0x10730002
#define CFG_UART1INPROT_RTCM3X 0x10730001
#define CFG_UART1OUTPROT_UBX  0x10740001
#define CFG_UART1OUTPROT_NMEA 0x10740002
#define CFG_UART2_BAUDRATE    0x40530001
#define CFG_UART2_STOPBITS    0x20530002
#define CFG_UART2_DATABITS    0x20530003
#define CFG_UART2_PARITY      0x20530004
#define CFG_UART2_ENABLED     0x10530005
#define CFG_UART2INPROT_UBX   0x10750001
#define CFG_UART2INPROT_NMEA  0x10750002
#define CFG_UART2INPROT_RTCM3X 0x10750001
#define CFG_UART2OUTPROT_UBX  0x10760001
#define CFG_UART2OUTPROT_NMEA 0x10760002
#define CFG_UART_STOPBITS_HALF      0
#define CFG_UART_STOPBITS_ONE       1
#define CFG_UART_STOPBITS_ONE_HALF  2
#define CFG_UART_STOPBITS_TWO       3
#define CFG_UART_DATABITS_EIGHT     0
#define CFG_UART_DATABITS_SEVENT    1
#define CFG_UART_PARITY_NONE        0
#define CFG_UART_PARITY_ODD         1
#define CFG_UART_PARITY_EVEN        2

const uint16_t NEO_M9N_I2C_ADDR = 0x84;

typedef struct ubx_frame {
	uint8_t preamble[2];
	uint8_t messageClass;
	uint8_t messageId;
	uint16_t payloadLength;
	uint8_t payload[64];

} ubx_frame_t;

static lpi2c_rtos_handle_t s_lpi2c_handle;
static lpi2c_master_transfer_t s_lpi2c_transfer;
static ubx_frame_t s_ubxFrame;
static uint8_t s_gpsBuffer[32];

const lpi2c_master_config_t LPI2C_config = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};

static void GpsUbxAppendConfigBit(ubx_frame_t *ubxFrame, uint32_t keyId, bool value) {

	assert(UBX_CONFIG_SIZE(keyId) == 1);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* keyPtr = (uint32_t*)&ubxFrame->payload[idx];
	*keyPtr++ = keyId;
	uint8_t* valuePtr = (uint8_t*)keyPtr;
	*valuePtr = (value) ? 0x01 : 0x00;
	ubxFrame->payloadLength += 5;
}

static void GpsUbxAppendConfig8(ubx_frame_t *ubxFrame, uint32_t keyId, uint8_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 2);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* keyPtr = (uint32_t*)&ubxFrame->payload[idx];
	*keyPtr++ = keyId;
	uint8_t* valuePtr = (uint8_t*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 5;
}

static void GpsUbxAppendConfig16(ubx_frame_t *ubxFrame, uint32_t keyId, uint16_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 3);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* keyPtr = (uint32_t*)&ubxFrame->payload[idx];
	*keyPtr++ = keyId;
	uint16_t* valuePtr = (uint16_t*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 6;
}

static void GpsUbxAppendConfig32(ubx_frame_t *ubxFrame, uint32_t keyId, uint32_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 4);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* cfgPtr = (uint32_t*)&ubxFrame->payload[idx];
	*cfgPtr++ = keyId;
	*cfgPtr = value;
	ubxFrame->payloadLength += 8;
}

static void GpsUbxAppedConfigFloat(ubx_frame_t *ubxFrame, uint32_t keyId, float value) {

	assert(UBX_CONFIG_SIZE(keyId) == 4);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* keyPtr = (uint32_t*)&ubxFrame->payload[idx];
	*keyPtr++ = keyId;
	float* valuePtr = (float*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 8;
}

static void GpsUbxAppendConfig64(ubx_frame_t *ubxFrame, uint32_t keyId, uint64_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 5);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* keyPtr = (uint32_t*)&ubxFrame->payload[idx];
	*keyPtr++ = keyId;
	uint64_t* valuePtr = (uint64_t*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 12;
}

static void GpsUbxAppedConfigDouble(ubx_frame_t *ubxFrame, uint32_t keyId, double value) {

	assert(UBX_CONFIG_SIZE(keyId) == 5);

	uint32_t idx = ubxFrame->payloadLength;
	uint32_t* keyPtr = (uint32_t*)&ubxFrame->payload[idx];
	*keyPtr++ = keyId;
	double* valuePtr = (double*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 12;
}

static void GpsUbxCfgValSetFrameInitialize(ubx_frame_t *ubxFrame) {
	memset(ubxFrame, 0x00, sizeof(ubx_frame_t));
	ubxFrame->messageClass = 0x06;
	ubxFrame->messageId = 0x8A;
	ubxFrame->payloadLength = 4;
	ubxFrame->payload[0] = 0x00;
	ubxFrame->payload[1] = 0x03; // ram & battery-backed ram updated
	ubxFrame->payload[2] = 0x00;
	ubxFrame->payload[3] = 0x00;
}

static void GpsUbxFrameFinalize(ubx_frame_t *ubxFrame) {

	ubxFrame->preamble[0] = 0xb5;
	ubxFrame->preamble[1] = 0x62;

	uint8_t ckA = ubxFrame->messageClass + ubxFrame->messageId;
	uint8_t ckB = ubxFrame->messageClass;

	ckA += ubxFrame->payloadLength & 0xFF;
	ckB += ckA;
	ckA += ubxFrame->payloadLength >> 8;
	ckB += ckA;

	int i = 0;
	while (i < ubxFrame->payloadLength) {
		ckA += ubxFrame->payload[i++];
		ckB += ckA;
	}

	ubxFrame->payload[i++] = ckA;
	ubxFrame->payload[i++] = ckB;
}

status_t GpsInit(void) {
    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 0U);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 5U);
    NVIC_SetPriority(LPI2C3_IRQn, 3);
	status_t gpsStatus = LPI2C_RTOS_Init(&s_lpi2c_handle, LPI2C3, &LPI2C_config, 60000000U);
	if (gpsStatus != kStatus_Success) {
		PRINTF("I2C1 initialization failed\n");
	} else {
		GpsUbxCfgValSetFrameInitialize(&s_ubxFrame);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_I2CINPROT_NMEA, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_I2CINPROT_RTCM3X, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_I2COUTPROT_NMEA, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_UART1INPROT_NMEA, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_UART1INPROT_RTCM3X, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_UART1OUTPROT_NMEA, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_UART2INPROT_NMEA, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_UART2INPROT_RTCM3X, false);
		GpsUbxAppendConfigBit(&s_ubxFrame, CFG_UART2OUTPROT_NMEA, false);
		GpsUbxFrameFinalize(&s_ubxFrame);
		s_lpi2c_transfer.flags = kLPI2C_TransferDefaultFlag;
		s_lpi2c_transfer.subaddress = 0x00;
		s_lpi2c_transfer.subaddressSize = 0;
		s_lpi2c_transfer.slaveAddress = NEO_M9N_I2C_ADDR;
		s_lpi2c_transfer.direction = kLPI2C_Write;
		s_lpi2c_transfer.data = &s_ubxFrame;
		s_lpi2c_transfer.dataSize= 0x08 + s_ubxFrame.payloadLength;
		status_t gpsStatus = LPI2C_RTOS_Transfer(&s_lpi2c_handle, &s_lpi2c_transfer);
		PRINTF("u-Blox Configuration [0x%X]\n", gpsStatus);
	}
	return gpsStatus;
}

void GpsTask( void *pvParameters )
{
	s_lpi2c_transfer.flags = kLPI2C_TransferDefaultFlag;
	s_lpi2c_transfer.subaddress = 0xFF;
	s_lpi2c_transfer.subaddressSize = 1;
	s_lpi2c_transfer.slaveAddress = NEO_M9N_I2C_ADDR;
	s_lpi2c_transfer.direction = kLPI2C_Read;
	s_lpi2c_transfer.data = s_gpsBuffer;
	s_lpi2c_transfer.dataSize= 32;

	LPI2C_MasterEnable(LPI2C3, true);
	uint32_t masterFlags = LPI2C_MasterGetStatusFlags(LPI2C3);
	PRINTF("LPI2C3 Master Status: 0x%X\n", masterFlags);
	uint32_t intrFlags = LPI2C_MasterGetEnabledInterrupts(LPI2C3);
	PRINTF("LPI2C3 Interrupt Flags: 0x%X\n", intrFlags);
	for(;;) {
//	    status_t gpsStatus = LPI2C_RTOS_Transfer(&s_lpi2c1_handle, &s_lpi2c1_transfer);
//	    if ((gpsStatus == kStatus_Success) && (s_gpsBuffer[0] < 0xFF)) {
//	    	PRINTF("GPS Data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", s_gpsBuffer[0], s_gpsBuffer[1], s_gpsBuffer[2], s_gpsBuffer[3], s_gpsBuffer[4], s_gpsBuffer[5], s_gpsBuffer[6], s_gpsBuffer[7]);
//	    } else {
//	    	PRINTF("GPS [0x%X]\n", gpsStatus);
//	    }
	    vTaskDelay(800);
		PRINTF("LPI2C3 Master Start: [0x%X]\n", LPI2C_MasterStart(LPI2C3, NEO_M9N_I2C_ADDR, kLPI2C_Write));
		uint32_t status = LPI2C_MasterGetStatusFlags(LPI2C3);
		status_t i2cStatus = LPI2C_MasterCheckAndClearError(LPI2C3, status);
		PRINTF("LPI2C3 Error Status: [0x%X]\n", i2cStatus);
	}
}

