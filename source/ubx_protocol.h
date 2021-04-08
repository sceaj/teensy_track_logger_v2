/*
 * ubx_protocol.h
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

#ifndef UBX_PROTOCOL_H_
#define UBX_PROTOCOL_H_

#include "fsl_common.h"

/*
 * UBX Class and ID
 */

#define UBX_ACK_CLASS           0x05
#define UBX_ACK_ID              0x01
#define UBX_NAK_ID              0x00

#define UBX_CFG_CLASS           0x06
#define UBX_CFG_VALDEL_ID       0x8C
#define UBX_CFG_VALGET_ID       0x8B
#define UBX_CFG_VALSET_ID       0x8A

#define UBX_INF_CLASS           0x04
#define UBX_INF_DEBUG_ID        0x04
#define UBX_INF_ERROR_ID        0x00
#define UBX_INF_NOTICE_ID       0x02
#define UBX_INF_TEST_ID         0x03
#define UBX_INF_WARNING_ID      0x01

#define UBX_MON_CLASS           0x0A
#define UBX_MON_COMMS_ID        0x36
#define UBX_MON_GNSS_ID         0x38
#define UBX_MON_HW_ID           0x09
#define UBX_MON_HW2_ID          0x0B
#define UBX_MON_HW3_ID          0x37
#define UBX_MON_IO_ID           0x02
#define UBX_MON_MSGPP_ID        0x06
#define UBX_MON_PATCH_ID        0x27
#define UBX_MON_RF_ID           0x38
#define UBX_MON_RXBUF_ID        0x07
#define UBX_MON_RXR_ID          0x21
#define UBX_MON_SPAN_ID         0x31
#define UBX_MON_TXBUF_ID        0x08
#define UBX_MON_VER_ID          0x04

#define UBX_NAV_CLASS           0x01
#define UBX_NAV_CLOCK_ID        0x22
#define UBX_NAV_COV_ID          0x36
#define UBX_NAV_DOP_ID          0x04
#define UBX_NAV_EOE_ID          0x61
#define UBX_NAV_GEOFENCE_ID     0x39
#define UBX_NAV_ODO_ID          0x09
#define UBX_NAV_ORB_ID          0x34
#define UBX_NAV_POSECEF_ID      0x01
#define UBX_NAV_POSLLH_ID       0x02
#define UBX_NAV_PVT_ID          0x07
#define UBX_NAV_RESETODO_ID     0x10
#define UBX_NAV_SAT_ID          0x35
#define UBX_NAV_SIG_ID          0x43
#define UBX_NAV_STATUS_ID       0x03
#define UBX_NAV_TIMEBDS_ID      0x24
#define UBX_NAV_TIMEGAL_ID      0x25
#define UBX_NAV_TIMEGLO_ID      0x23
#define UBX_NAV_TIMEGPS_ID      0x20
#define UBX_NAV_TIMEELS_ID      0x26
#define UBX_NAV_TIMEQZSS_ID     0x27
#define UBX_NAV_TIMEUTC_ID      0x21
#define UBX_NAV_VELECEF_ID      0x11
#define UBX_NAV_VELNED_ID       0x12


/*
 * UBX Configuration Keys
 */
#define CFG_I2C_ADDRESS         0x20510001
#define CFG_I2C_EXTENDEDTIMEOUT 0x10510002
#define CFG_I2C_ENABLED         0x10510003
#define CFG_I2CINPROT_UBX       0x10710001
#define CFG_I2CINPROT_NMEA      0x10710002
#define CFG_I2CINPROT_RTCM3X    0x10710004
#define CFG_I2COUTPROT_UBX      0x10720001
#define CFG_I2COUTPROT_NMEA     0x10720002
#define CFG_INFMSG_UBX_I2C      0x20920001
#define CFG_INFMSG_UBX_UART1    0x20920002
#define CFG_INFMSG_UBX_UART2    0x20920003
#define CFG_INFMSG_UBX_USB      0x20920004
#define CFG_INFMSG_UBX_SPI      0x20920005
#define CFG_INFMSG_NMEA_I2C     0x20920006
#define CFG_INFMSG_NMEA_UART1   0x20920007
#define CFG_INFMSG_NMEA_UART2   0x20920008
#define CFG_INFMSG_NMEA_USB     0x20920009
#define CFG_INFMSG_NMEA_SPI     0x2092000a
#define CFG_INFMSG_ERROR        0x01
#define CFG_INFMSG_WARNING      0x02
#define CFG_INFMSG_NOTICE       0x04
#define CFG_INFMSG_TEST         0x08
#define CFG_INFMSG_DEBUG        0x10
#define CFG_NAVSPG_DYNMODEL     0x20110021
#define CFG_NAVSPG_DYNMODEL_PORT    0
#define CFG_NAVSPG_DYNMODEL_STAT    2
#define CFG_NAVSPG_DYNMODEL_PED     3
#define CFG_NAVSPG_DYNMODEL_AUTOMOT 4
#define CFG_NAVSPG_DYNMODEL_SEA     5
#define CFG_NAVSPG_DYNMODEL_AIR1    6
#define CFG_NAVSPG_DYNMODEL_AIR2    7
#define CFG_NAVSPG_DYNMODEL_AIR4    8
#define CFG_NAVSPG_DYNMODEL_WRIST   9
#define CFG_RATE_MEAS           0x30210001
#define CFG_RATE_NAV            0x30210002
#define CFG_RATE_TIMEREF        0x30210003
#define CFG_RATE_TIMEREF_UTC        0
#define CFG_RATE_TIMEREF_GPS        1
#define CFG_RATE_TIMEREF_GLO        2
#define CFG_RATE_TIMEREF_BDS        3
#define CFG_RATE_TIMEREF_GAL        4
#define CFG_UART1_BAUDRATE      0x40520001
#define CFG_UART1_STOPBITS      0x20520002
#define CFG_UART1_DATABITS      0x20520003
#define CFG_UART1_PARITY        0x20520004
#define CFG_UART1_ENABLED       0x10520005
#define CFG_UART1INPROT_UBX     0x10730001
#define CFG_UART1INPROT_NMEA    0x10730002
#define CFG_UART1INPROT_RTCM3X  0x10730001
#define CFG_UART1OUTPROT_UBX    0x10740001
#define CFG_UART1OUTPROT_NMEA   0x10740002
#define CFG_UART2_BAUDRATE      0x40530001
#define CFG_UART2_STOPBITS      0x20530002
#define CFG_UART2_DATABITS      0x20530003
#define CFG_UART2_PARITY        0x20530004
#define CFG_UART2_ENABLED       0x10530005
#define CFG_UART2INPROT_UBX     0x10750001
#define CFG_UART2INPROT_NMEA    0x10750002
#define CFG_UART2INPROT_RTCM3X  0x10750001
#define CFG_UART2OUTPROT_UBX    0x10760001
#define CFG_UART2OUTPROT_NMEA   0x10760002
#define CFG_UART_STOPBITS_HALF      0
#define CFG_UART_STOPBITS_ONE       1
#define CFG_UART_STOPBITS_ONE_HALF  2
#define CFG_UART_STOPBITS_TWO       3
#define CFG_UART_DATABITS_EIGHT     0
#define CFG_UART_DATABITS_SEVENT    1
#define CFG_UART_PARITY_NONE        0
#define CFG_UART_PARITY_ODD         1
#define CFG_UART_PARITY_EVEN        2
#define CFG_USB_ENABLED         0x10650001
#define CFG_MSGOUT_UBX_NAV_POSLLH_UART1 0x2091002a
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007
#define CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1 0x2091005c

#define kStatusGroup_UBXPROT 202
enum {
    kStatus_UBXPROT_Syncing         = MAKE_STATUS(kStatusGroup_UBXPROT, 0),
    kStatus_UBXPROT_Preamble        = MAKE_STATUS(kStatusGroup_UBXPROT, 1),
    kStatus_UBXPROT_Parsing         = MAKE_STATUS(kStatusGroup_UBXPROT, 2),
    kStatus_UBXPROT_BadChecksum     = MAKE_STATUS(kStatusGroup_UBXPROT, 3),
    kStatus_UBXPROT_ValidFrame      = MAKE_STATUS(kStatusGroup_UBXPROT, 4),
    kStatus_UBXPROT_BufferOverflow   = MAKE_STATUS(kStatusGroup_UBXPROT, 5),
};

typedef struct ubx_frame {
	uint8_t preamble[2];
	uint8_t messageClass;
	uint8_t messageId;
	uint16_t payloadLength;
} ubx_frame_t;

// Payload for either ACK or NAK
typedef struct ubx_ack_payload {
    uint8_t messageClass;
    uint8_t messageId;
} ubx_ack_payload_t;

typedef struct ubx_nav_posllh {
    uint32_t iTOW;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
} ubx_nav_posllh_t;

typedef struct ubx_nav_pvt {
    uint32_t iTOW;              //  0
    uint16_t year;              //  4
    uint8_t  month;             //  6
    uint8_t  day;               //  7
    uint8_t  hour;              //  8
    uint8_t  min;               //  9
    uint8_t  sec;               // 10
    uint8_t  valid;             // 11
    uint32_t tAcc;              // 12
    int32_t  nano;              // 16
    uint8_t  fixType;           // 20
    uint8_t  flags;             // 21
    uint8_t  flags2;            // 22
    uint8_t  numSV;             // 23
    int32_t  lon;               // 24
    int32_t  lat;               // 28
    int32_t  height;            // 32
    int32_t  hMSL;              // 36
    uint32_t hAcc;              // 40
    uint32_t vAcc;              // 44
    int32_t  velN;              // 48
    int32_t  velE;              // 52
    int32_t  velD;              // 56
    int32_t  gSpeed;            // 60
    int32_t  headMot;           // 64
    uint32_t sAcc;              // 68
    uint32_t headAcc;           // 72
    uint16_t pDOP;              // 76
    uint8_t  flags3;            // 78
    uint8_t  reserved0[5];      // 79
    int32_t  headVeh;           // 84
    int16_t  magDec;            // 88
    uint16_t magAcc;            // 90
} ubx_nav_pvt_t;

typedef enum {
    ubxWaitingForPreamble   = 1,
    ubxPreamble1            = 2,
    ubxMessageClass         = 3,
    ubxMessageId            = 4,
    ubxPayloadLength        = 5,
    ubxPayload              = 6,
    ubxChecksum             = 7,
    ubxFrameComplete        = 8,
} ubx_parser_state_t;

typedef struct ubx_parser_handle {
    ubx_parser_state_t  parserState;
    uint32_t            remainingBytes;
    uint32_t            frameRemaining;
    int                 frameIndex;
    uint8_t*            framePtr;
} ubx_parser_handle_t;

static inline size_t UbxFrameSize(ubx_frame_t *ubxFrame) {
    return sizeof(ubx_frame_t) + ubxFrame->payloadLength + 2U;
}

static inline bool UbxFrameIsClass(ubx_frame_t *ubxFrame, uint8_t class) {
    return (ubxFrame->messageClass == class);
}

static inline bool UbxFrameIsMessage(ubx_frame_t *ubxFrame, uint8_t class, uint8_t id) {
    return ((ubxFrame->messageClass == class) && (ubxFrame->messageId == id));
}

int UbxCurrentFrame();

ubx_frame_t* UbxFrameBuffer(int frameIndex);

void UbxDebugPrintFrame(ubx_frame_t *ubxFrame);

void UbxCfgValSetFrameInitialize(ubx_frame_t *ubxFrame, uint8_t cfgVersion, uint8_t layerFlags);

bool UbxFrameValidate(ubx_frame_t *ubxFrame);

uint32_t UbxFrameFinalize(ubx_frame_t *ubxFrame);

void UbxAppendConfigBit(ubx_frame_t *ubxFrame, uint32_t keyId, bool value);

void UbxAppendConfig8(ubx_frame_t *ubxFrame, uint32_t keyId, uint8_t value);

void UbxAppendConfig16(ubx_frame_t *ubxFrame, uint32_t keyId, uint16_t value);

void UbxAppendConfig32(ubx_frame_t *ubxFrame, uint32_t keyId, uint32_t value);

void UbxAppendConfigFloat(ubx_frame_t *ubxFrame, uint32_t keyId, float value);

void UbxAppendConfig64(ubx_frame_t *ubxFrame, uint32_t keyId, uint64_t value);

void UbxAppedConfigDouble(ubx_frame_t *ubxFrame, uint32_t keyId, double value);

void UbxParserHandleInit(ubx_parser_handle_t* ubxParserHandle);

status_t UbxParserStateMachine(uint8_t byte, ubx_parser_handle_t* ubxParserHandle);

#endif /* UBX_PROTOCOL_H_ */
