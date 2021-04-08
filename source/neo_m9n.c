/*
 * neo_m9n.c
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

#include "neo_m9n.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_os_abstraction.h"
#include "fsl_lpuart.h"
//#include "ringbuffer.h"
#include "ubx_protocol.h"

#define NEOM9N_UART				LPUART1
#define NEOM9N_IRQHandler 		LPUART1_IRQHandler
#define NEOM9N_IRQn				LPUART1_IRQn

#define UART_CLK_FREQ    BOARD_BOOTCLOCKRUN_UART_CLK_ROOT
#define RX_RING_BUFFER_SIZE (512U)

// OSA Prototype - not sure why this is not in the header?
uint32_t OSA_TimeDiff(uint32_t time_start, uint32_t time_end);

/* UART transfer state. */
enum
{
    uart_TxIdle,         /* TX idle. */
    uart_TxBusy,         /* TX busy. */
    uart_RxIdle,         /* RX idle. */
    uart_RxBusy,         /* RX busy. */
    uart_RxFramingError, /* Rx framing error */
    uart_RxParityError   /* Rx parity error */
};

volatile static uint32_t s_uartTxState;
volatile uint32_t txByteCounter;
static uint8_t* s_pUartIrqTxBuffer;
static size_t s_uartIrqTxBufferSize;
uint8_t g_txFrameBuffer[256];

volatile bool ubxFrameAvailable[4]  = { false, false, false, false };
volatile uint32_t rxByteCounter;
volatile uint32_t uartBufferOverrunCounter;
volatile uint32_t uartFramingErrorCounter;
volatile uint32_t ubxValidFrameCounter;
volatile uint32_t ubxBadChecksumCounter;
volatile uint32_t ubxBufferOverflowCounter;
static uint8_t s_uartIrqRxBuffer[16];

static ubx_parser_handle_t ubxParserHandle;


/*
 * UART Communication - START
 */
// Copy of the UART Driver UART_WriteNonBlocking
// Moved here because it is static in the driver
static void NEOM9N_SendNonBlocking(const uint8_t *data, size_t length)
{
    assert(data);

    size_t i;

    /* The Non Blocking write data API assume user have ensured there is enough space in
    peripheral to write. */
    for (i = 0; i < length; i++) {
        NEOM9N_UART->DATA = data[i];
    }
}

// Copy of the UART Driver UART_ReadNonBlocking
// Moved here because it is static in the driver
static void NEOM9N_ReceiveNonBlocking(uint8_t *data, size_t length)
{
    assert(data);

    size_t i;
#if defined(FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT) && FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT
    uint32_t ctrl        = NEOM9N_UART->CTRL;
    bool isSevenDataBits = (((ctrl & LPUART_CTRL_M7_MASK) != 0U) ||
                            (((ctrl & LPUART_CTRL_M_MASK) == 0U) && ((ctrl & LPUART_CTRL_PE_MASK) != 0U)));
#endif

    /* The Non Blocking read data API assume user have ensured there is enough space in
    peripheral to write. */
    for (i = 0; i < length; i++)
    {
#if defined(FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT) && FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT
        if (isSevenDataBits) {
            data[i] = (uint8_t)(NEOM9N_UART->DATA & 0x7FU);
        } else {
            data[i] = (uint8_t)NEOM9N_UART->DATA;
        }
#else
        data[i] = (uint8_t)(NEOM9N_UART->DATA);
#endif
    }
}

/* UART user callback */
void NEOM9N_UserCallback(status_t status, void *userData)
{
    if (kStatus_LPUART_RxHardwareOverrun == status) {
        uartBufferOverrunCounter++;
    }
    if (kStatus_LPUART_FramingError == status) {
        uartFramingErrorCounter++;
    }

    if (kStatus_UBXPROT_ValidFrame == status) {
        ubxValidFrameCounter++;
    }

    if (kStatus_UBXPROT_BadChecksum == status) {
    	ubxBadChecksumCounter++;
    }

    if (kStatus_UBXPROT_BufferOverflow == status) {
    	ubxBufferOverflowCounter++;
    }

    if (kStatus_LPUART_RxIdle == status) {
        int frameIndex = (int)userData;
        ubxFrameAvailable[frameIndex] = true;
    }

    if (kStatus_LPUART_TxIdle == status) {
    	// Nothing to do...
    }
}


// Override the UART driver provided interrupt handler
void NEOM9N_IRQHandler(void) {

    uint8_t count;
    uint8_t tempCount;
    uint32_t status = LPUART_GetStatusFlags(NEOM9N_UART);
    uint32_t enabledInterrupts = LPUART_GetEnabledInterrupts(NEOM9N_UART);

    /* If RX overrun. */
    if ((uint32_t)kLPUART_RxOverrunFlag == ((uint32_t)kLPUART_RxOverrunFlag & status)) {
        /* Clear overrun flag, otherwise the RX does not work. */
    	NEOM9N_UART->STAT = ((NEOM9N_UART->STAT & 0x3FE00000U) | LPUART_STAT_OR_MASK);

        /* Trigger callback. */
        NEOM9N_UserCallback(kStatus_LPUART_RxHardwareOverrun, NULL);
    }

    if ((uint32_t)kLPUART_FramingErrorFlag == ((uint32_t)kLPUART_FramingErrorFlag & status)) {
        /* Clear framing error flag, otherwise the RX does not work. */
    	LPUART_ClearStatusFlags(NEOM9N_UART, (uint32_t)kLPUART_FramingErrorFlag);

        /* Trigger callback. */
        NEOM9N_UserCallback(kStatus_LPUART_FramingError, NULL);
    }

    /* If IDLE flag is set and the IDLE interrupt is enabled. */
    if ((0U != ((uint32_t)kLPUART_IdleLineFlag & status)) &&
        (0U != ((uint32_t)kLPUART_IdleLineInterruptEnable & enabledInterrupts))) {
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
        count = ((uint8_t)((NEOM9N_UART->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT));

        while (0U != count) {
            tempCount = (uint8_t)MIN(sizeof(s_uartIrqRxBuffer), (uint32_t)count);

            /* Using non block API to read the data from the registers. */
            NEOM9N_ReceiveNonBlocking(s_uartIrqRxBuffer, tempCount);
            count -= tempCount;
            rxByteCounter += tempCount;

            for (int i = 0; i < tempCount; i++) {
                // TODO: Handle status and frame completion
                status_t parserStatus = UbxParserStateMachine(s_uartIrqRxBuffer[i], &ubxParserHandle);
                if (parserStatus == kStatus_UBXPROT_ValidFrame) {
                    NEOM9N_UserCallback(kStatus_LPUART_RxIdle, (void*)UbxCurrentFrame());
                    UbxParserHandleInit(&ubxParserHandle);
                }
                if (parserStatus == kStatus_UBXPROT_BadChecksum) {
                    NEOM9N_UserCallback(kStatus_UBXPROT_BadChecksum, NULL);
                    UbxParserHandleInit(&ubxParserHandle);
                }
                if (parserStatus == kStatus_UBXPROT_BufferOverflow) {
                    NEOM9N_UserCallback(kStatus_UBXPROT_BufferOverflow, NULL);
                    UbxParserHandleInit(&ubxParserHandle);
                }
            }
        }
#endif
        /* Clear IDLE flag.*/
        NEOM9N_UART->STAT |= LPUART_STAT_IDLE_MASK;
    }

    /* Receive data register full */
    if ((0U != ((uint32_t)kLPUART_RxDataRegFullFlag & status)) &&
        (0U != ((uint32_t)kLPUART_RxDataRegFullInterruptEnable & enabledInterrupts))) {
/* Get the size that can be stored into buffer for this interrupt. */
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
        count = ((uint8_t)((NEOM9N_UART->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT));
#else
        count = 1;
#endif

        while (0U != count) {
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
            tempCount = (uint8_t)MIN(sizeof(s_uartIrqRxBuffer), (uint32_t)count);
#else
            tempCount = 1;
#endif

            /* Using non block API to read the data from the registers. */
            NEOM9N_ReceiveNonBlocking(s_uartIrqRxBuffer, tempCount);
            count -= tempCount;
            rxByteCounter += tempCount;

            for (int i = 0; i < tempCount; i++) {
                // TODO: Handle status and frame completion
                status_t parserStatus = UbxParserStateMachine(s_uartIrqRxBuffer[i], &ubxParserHandle);
                if (parserStatus == kStatus_UBXPROT_ValidFrame) {
                    ubxValidFrameCounter++;
                    NEOM9N_UserCallback(kStatus_LPUART_RxIdle, (void*)UbxCurrentFrame());
                    UbxParserHandleInit(&ubxParserHandle);
                }
                if (parserStatus == kStatus_UBXPROT_BadChecksum) {
                	NEOM9N_UserCallback(kStatus_UBXPROT_BadChecksum, NULL);
                    UbxParserHandleInit(&ubxParserHandle);
                }
                if (parserStatus == kStatus_UBXPROT_BufferOverflow) {
                    ubxBufferOverflowCounter++;
                    UbxParserHandleInit(&ubxParserHandle);
                }
            }
        }
    }

    /* Send data register empty and the interrupt is enabled. */
    if ((0U != ((uint32_t)kLPUART_TxDataRegEmptyFlag & status)) &&
        (0U != ((uint32_t)kLPUART_TxDataRegEmptyInterruptEnable & enabledInterrupts)))
    {
/* Get the bytes that available at this moment. */
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
        count = (uint8_t)FSL_FEATURE_LPUART_FIFO_SIZEn(NEOM9N_UART) -
                (uint8_t)((NEOM9N_UART->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT);
#else
        count = 1;
#endif

        while ((count != 0U) && (s_uartIrqTxBufferSize != 0U))
        {
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
            tempCount = (uint8_t)MIN(s_uartIrqTxBufferSize, (uint32_t)count);
#else
            tempCount = 1;
#endif

            /* Using non block API to write the data to the registers. */
            NEOM9N_SendNonBlocking(s_pUartIrqTxBuffer, tempCount);
            s_pUartIrqTxBuffer += tempCount;
            s_uartIrqTxBufferSize -= tempCount;
            count -= tempCount;
            txByteCounter += tempCount;

            /* If all the data are written to data register, notify user with the callback, then TX finished. */
            if (0U == s_uartIrqTxBufferSize)
            {
                /* Disable TX register empty interrupt. */
                NEOM9N_UART->CTRL = (NEOM9N_UART->CTRL & ~LPUART_CTRL_TIE_MASK);
                /* Enable transmission complete interrupt. */
                LPUART_EnableInterrupts(NEOM9N_UART, (uint32_t)kLPUART_TransmissionCompleteInterruptEnable);
            }
        }
    }

    /* Transmission complete and the interrupt is enabled. */
    if ((0U != ((uint32_t)kLPUART_TransmissionCompleteFlag & status)) &&
        (0U != ((uint32_t)kLPUART_TransmissionCompleteInterruptEnable & enabledInterrupts))) {
        /* Set txState to idle only when all data has been sent out to bus. */
    	s_uartTxState = (uint8_t)uart_TxIdle;
        /* Disable transmission complete interrupt. */
        LPUART_DisableInterrupts(NEOM9N_UART, (uint32_t)kLPUART_TransmissionCompleteInterruptEnable);

        NEOM9N_UserCallback(kStatus_LPUART_TxIdle, NULL);
    }
}

void NEOM9N_DebugCommCounters() {
    PRINTF("Transmission Tx: %d  Rx: %d\n", txByteCounter, rxByteCounter);
    PRINTF("UART  Overruns: %d  Framing Errors: %d\n", uartBufferOverrunCounter, uartFramingErrorCounter);
    PRINTF("Frames  Valid: %d  Bad Checksum: %d  Buffer Overflow: %d\n", ubxValidFrameCounter, ubxBadChecksumCounter, ubxBufferOverflowCounter);
}

void NEOM9N_StartParser() {

    UbxParserHandleInit(&ubxParserHandle);
    /* Enable RX/Rx overrun/framing error/idle line interrupt. */
    LPUART_EnableInterrupts(NEOM9N_UART,
            (uint32_t)kLPUART_RxDataRegFullInterruptEnable | (uint32_t)kLPUART_RxOverrunInterruptEnable
            | (uint32_t)kLPUART_FramingErrorInterruptEnable | (uint32_t)kLPUART_IdleLineInterruptEnable);
}

status_t NEOM9N_SendFrameNonBlocking(ubx_frame_t* ubxFrame) {

    assert(ubxFrame);

    status_t status;

    /* Return error if current TX busy. */
    if ((uint8_t)uart_TxBusy == s_uartTxState)
    {
        status = kStatus_LPUART_TxBusy;
    } else {

        s_pUartIrqTxBuffer = (uint8_t*)ubxFrame;
        s_uartIrqTxBufferSize = UbxFrameSize(ubxFrame);
        s_uartTxState = uart_TxBusy;

        /* Enable transmitter interrupt. */
        LPUART_EnableInterrupts(NEOM9N_UART, (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);

        status = kStatus_Success;
    }

    return status;
}

bool NEOM9N_WaitForAckOrNak(ubx_ack_payload_t *payload, uint32_t msecTimeout) {

    bool lookingForAckOrNak = true;
    bool foundAck = false;

    uint32_t start = OSA_TimeGetMsec();
    while (lookingForAckOrNak && (msecTimeout > (OSA_TimeGetMsec() - start))) {
        for (int i = 0; i < 4; i++) {
            if (ubxFrameAvailable[i]) {

                ubx_frame_t *ubxFrame = UbxFrameBuffer(i);
                UbxDebugPrintFrame(ubxFrame);

                ubx_ack_payload_t *receivedPayload = (ubx_ack_payload_t*)(ubxFrame + 1);

                if (UbxFrameValidate(ubxFrame)
                        && ubxFrame->messageClass == UBX_ACK_CLASS
                        && receivedPayload->messageClass == payload->messageClass
                        && receivedPayload->messageId == payload->messageId) {
                    foundAck = ubxFrame->messageId;
                    PRINTF("Found ACK: %d\n", foundAck);
                    lookingForAckOrNak = false;
                }

                ubxFrameAvailable[i] = false;
            }
        }
    }

    return foundAck;
}

status_t NEOM9N_UartInit() {

    lpuart_config_t config;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 38400;
    config.enableTx     = true;
    config.enableRx     = true;

    status_t uartStatus = LPUART_Init(NEOM9N_UART, &config, UART_CLK_FREQ);
    if (kStatus_Success == uartStatus) {
    	(void)EnableIRQ(NEOM9N_IRQn);
    	NEOM9N_StartParser();
    }

    return uartStatus;
}

status_t NEOM9N_GpsConfig() {

    ubx_frame_t *ubxFrame = (ubx_frame_t*)g_txFrameBuffer;

    PRINTF("Turn off NMEA Output\n");
    UbxCfgValSetFrameInitialize(ubxFrame, 0x00, 0x01);
    UbxAppendConfigBit(ubxFrame, CFG_UART1OUTPROT_NMEA, false);
    UbxFrameFinalize(ubxFrame);

    ubx_ack_payload_t ackPayload;
    ackPayload.messageClass = 0x06;
    ackPayload.messageId = 0x8A;
    status_t status = NEOM9N_SendFrameNonBlocking(ubxFrame);
    NEOM9N_WaitForAckOrNak(&ackPayload, 3000U);

    uint32_t baudRate = 460800;
    PRINTF("Switching baud rate: %d\n", baudRate);
    UbxCfgValSetFrameInitialize(ubxFrame, 0x00, 0x01);
    UbxAppendConfig32(ubxFrame, CFG_UART1_BAUDRATE, baudRate);
    UbxFrameFinalize(ubxFrame);

    status = NEOM9N_SendFrameNonBlocking(ubxFrame);
    NEOM9N_WaitForAckOrNak(&ackPayload, 1200U);
    status = LPUART_SetBaudRate(NEOM9N_UART, baudRate, UART_CLK_FREQ);
    PRINTF("LPUART_SetBaudRate [%d]\n", status);

    OSA_TimeDelay(5000U);
    PRINTF("Requesting automotive dynamic model and UBX_NAV_PVT @ 25Hz\n");
    UbxCfgValSetFrameInitialize(ubxFrame, 0x00, 0x01);
    UbxAppendConfig8(ubxFrame, CFG_NAVSPG_DYNMODEL, CFG_NAVSPG_DYNMODEL_AUTOMOT);
    UbxAppendConfig8(ubxFrame, CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
    UbxAppendConfig16(ubxFrame, CFG_RATE_MEAS, 40);
    UbxFrameFinalize(ubxFrame);

    status = NEOM9N_SendFrameNonBlocking(ubxFrame);
    NEOM9N_WaitForAckOrNak(&ackPayload, 5000U);

    return status;
}

bool NEOM9N_Process(gps_time_t* time, gps_position_t* position, gps_velocity_t* velocity) {

    bool dataAvailable = false;

    for (int i = 0; i < 4; i++) {
        if (ubxFrameAvailable[i]) {
            ubx_frame_t *ubxFrame = UbxFrameBuffer(i);
            if (UbxFrameIsMessage(ubxFrame, UBX_NAV_CLASS, UBX_NAV_PVT_ID)) {
                ubx_nav_pvt_t *ubxNavPvt = (ubx_nav_pvt_t*)(ubxFrame + 1);
                time->year = ubxNavPvt->year;
                time->month = ubxNavPvt->month;
                time->day = ubxNavPvt->day;
                time->hour = ubxNavPvt->hour;
                time->min = ubxNavPvt->min;
                time->sec = ubxNavPvt->sec;
                time->millis = ubxNavPvt->nano / 1000000;
                time->valid = ubxNavPvt->valid;
                time->flags2 = ubxNavPvt->flags2;

                if (position != NULL) {
                    position->lon = ubxNavPvt->lon;
                    position->lat = ubxNavPvt->lat;
                    position->hMSL = ubxNavPvt->hMSL;
                    position->hAcc = ubxNavPvt->hAcc;
                    position->fixType = ubxNavPvt->fixType;
                    position->flags = ubxNavPvt->flags;
                    position->flags3 = ubxNavPvt->flags3;
                }

                if (velocity != NULL) {
                    velocity->gSpeed = ubxNavPvt->gSpeed;
                    velocity->headMot = ubxNavPvt->headMot;
                    velocity->sAcc = ubxNavPvt->sAcc;
                    velocity->headAcc = ubxNavPvt->headAcc;
                    velocity->pDOP = ubxNavPvt->pDOP;
                    velocity->flags = ubxNavPvt->flags;
                }

                dataAvailable = true;
            }
            memset(UbxFrameBuffer(i), 0x00, 128);
            ubxFrameAvailable[i] = false;
        }
    }

    return dataAvailable;
}
