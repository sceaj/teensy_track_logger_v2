/*
 * ubx_protocol.c
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

#include "ubx_protocol.h"
#include "fsl_debug_console.h"

#define UBX_CONFIG_SIZE_MASK  0x70000000
#define UBX_CONFIG_SIZE_SHIFT 28U
#define UBX_CONFIG_SIZE(key) ((key & UBX_CONFIG_SIZE_MASK) >> UBX_CONFIG_SIZE_SHIFT)

#define UBX_MAX_REMAINING_BYTES  0xFFFFFFFF

#define UBX_FRAME_BUFFER_COUNT 4
#define UBX_FRAME_BUFFER_SIZE 128

static int s_frameIndex = UBX_FRAME_BUFFER_COUNT;
static uint8_t s_frameBuffers[UBX_FRAME_BUFFER_COUNT][UBX_FRAME_BUFFER_SIZE];

int UbxCurrentFrame() {
    return s_frameIndex;
}

ubx_frame_t* UbxFrameBuffer(int frameIndex) {
    return (ubx_frame_t*)s_frameBuffers[frameIndex];
}

void UbxDebugPrintFrame(ubx_frame_t *ubxFrame) {
    if (ubxFrame->payloadLength < 0x100) {
        PRINTF("\nPreamble: 0x%02X%02X\n", ubxFrame->preamble[0],
                ubxFrame->preamble[1]);
        PRINTF("Class ID: 0x%02X  Message ID: 0x%02X\n", ubxFrame->messageClass,
                ubxFrame->messageId);
        PRINTF("Payload Length: 0x%04X\n", ubxFrame->payloadLength);

        uint8_t *payload = (uint8_t*) ubxFrame + sizeof(ubx_frame_t);
        for (int i = 0; i < ubxFrame->payloadLength; i++) {
            if (!(i % 8))
                PRINTF("\n");
            PRINTF("  0x%02X", *payload++);
        }
        PRINTF("\n\nChkA: 0x%02X  ChkB: 0x%02X\n", *payload, *(payload + 1));
    }
}

void UbxAppendConfigBit(ubx_frame_t *ubxFrame, uint32_t keyId, bool value) {

	assert(UBX_CONFIG_SIZE(keyId) == 1);

	uint32_t* keyPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*keyPtr++ = keyId;
	uint8_t* valuePtr = (uint8_t*)keyPtr;
	*valuePtr = (value) ? 0x01 : 0x00;
	ubxFrame->payloadLength += 5;
}

void UbxAppendConfig8(ubx_frame_t *ubxFrame, uint32_t keyId, uint8_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 2);

	uint32_t* keyPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*keyPtr++ = keyId;
	uint8_t* valuePtr = (uint8_t*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 5;
}

void UbxAppendConfig16(ubx_frame_t *ubxFrame, uint32_t keyId, uint16_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 3);

	uint32_t* keyPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*keyPtr++ = keyId;
	uint16_t* valuePtr = (uint16_t*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 6;
}

void UbxAppendConfig32(ubx_frame_t *ubxFrame, uint32_t keyId, uint32_t value) {

//	assert(UBX_CONFIG_SIZE(keyId) == 4);

	uint32_t* cfgPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*cfgPtr++ = keyId;
	*cfgPtr = value;
	ubxFrame->payloadLength += 8;
}

void UbxAppendConfigFloat(ubx_frame_t *ubxFrame, uint32_t keyId, float value) {

	assert(UBX_CONFIG_SIZE(keyId) == 4);

	uint32_t* keyPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*keyPtr++ = keyId;
	float* valuePtr = (float*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 8;
}

void UbxAppendConfig64(ubx_frame_t *ubxFrame, uint32_t keyId, uint64_t value) {

	assert(UBX_CONFIG_SIZE(keyId) == 5);

	uint32_t* keyPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*keyPtr++ = keyId;
	uint64_t* valuePtr = (uint64_t*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 12;
}

void UbxAppendConfigDouble(ubx_frame_t *ubxFrame, uint32_t keyId, double value) {

	assert(UBX_CONFIG_SIZE(keyId) == 5);

	uint32_t* keyPtr = (uint32_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*keyPtr++ = keyId;
	double* valuePtr = (double*)keyPtr;
	*valuePtr = value;
	ubxFrame->payloadLength += 12;
}

void UbxCfgValSetFrameInitialize(ubx_frame_t *ubxFrame, uint8_t cfgVersion, uint8_t layerFlags) {
	ubxFrame->messageClass = 0x06;
	ubxFrame->messageId = 0x8A;
	ubxFrame->payloadLength = 4;
	uint8_t *payload = ((uint8_t*)ubxFrame) + sizeof(ubx_frame_t);
	payload[0] = cfgVersion;
	payload[1] = layerFlags; // 0x01: ram, 0x02: battery-backed ram, 0x04: flash
	payload[2] = 0x00;
	payload[3] = 0x00;
}

uint16_t UbxCalculateChecksum(ubx_frame_t *ubxFrame) {

    uint8_t ckA = ubxFrame->messageClass + ubxFrame->messageId;
    uint8_t ckB = ubxFrame->messageClass + ckA;

    ckA += ubxFrame->payloadLength & 0xFF;
    ckB += ckA;
    ckA += (ubxFrame->payloadLength >> 8) & 0xFF;
    ckB += ckA;

    uint8_t* payload = ((uint8_t*)ubxFrame) + sizeof(ubx_frame_t);
    int i = 0;
    while (i < ubxFrame->payloadLength) {
        ckA += payload[i++];
        ckB += ckA;
    }

    uint16_t checksum = ckB;
    checksum *= 0x100;
    checksum += ckA;
    return checksum;
}

bool UbxFrameValidate(ubx_frame_t *ubxFrame) {

    uint16_t calculatedChecksum = UbxCalculateChecksum(ubxFrame);
    uint16_t actualChecksum = *((uint16_t*)(((uint8_t*)ubxFrame) + sizeof(ubx_frame_t) + ubxFrame->payloadLength));
    return actualChecksum == calculatedChecksum;
}

uint32_t UbxFrameFinalize(ubx_frame_t *ubxFrame) {

	ubxFrame->preamble[0] = 0xb5;
	ubxFrame->preamble[1] = 0x62;

	uint16_t* pChecksum = (uint16_t*)((char*)ubxFrame + sizeof(ubx_frame_t) + ubxFrame->payloadLength);
	*pChecksum = UbxCalculateChecksum(ubxFrame);

	return (uint32_t)(sizeof(ubx_frame_t) + ubxFrame->payloadLength + 2);
}

void UbxParserHandleInit(ubx_parser_handle_t* ubxParserHandle) {
    ubxParserHandle->parserState = ubxWaitingForPreamble;
    ubxParserHandle->remainingBytes = UBX_MAX_REMAINING_BYTES;
    ubxParserHandle->frameRemaining = UBX_FRAME_BUFFER_SIZE;
    ubxParserHandle->frameIndex = s_frameIndex = (s_frameIndex + 1) % 4;
    ubxParserHandle->framePtr = s_frameBuffers[s_frameIndex];
}

status_t UbxParserStateMachine(uint8_t byte, ubx_parser_handle_t* ubxParserHandle) {

    status_t parserStatus = kStatus_UBXPROT_Syncing;

    if (ubxParserHandle->frameRemaining == 0U) {
        return kStatus_UBXPROT_BufferOverflow;
    }

    switch (ubxParserHandle->parserState) {
    case ubxWaitingForPreamble:
        if (byte == 0xB5) {
            ubxParserHandle->parserState = ubxPreamble1;
            ubxParserHandle->remainingBytes = 1U;
            ubxParserHandle->frameRemaining--;
            *ubxParserHandle->framePtr++ = byte;
            parserStatus = kStatus_UBXPROT_Preamble;
        }
        break;
    case ubxPreamble1:
        if (byte == 0x62) {
            ubxParserHandle->parserState = ubxMessageClass;
            ubxParserHandle->frameRemaining--;
            *ubxParserHandle->framePtr++ = byte;
            parserStatus = kStatus_UBXPROT_Preamble;
        } else {
            ubxParserHandle->parserState = ubxWaitingForPreamble;
            ubxParserHandle->remainingBytes = UBX_MAX_REMAINING_BYTES;
        }
        break;
    case ubxMessageClass:
        ubxParserHandle->parserState = ubxMessageId;
        ubxParserHandle->frameRemaining--;
        *ubxParserHandle->framePtr++ = byte;
        parserStatus = kStatus_UBXPROT_Parsing;
        break;
    case ubxMessageId:
        ubxParserHandle->parserState = ubxPayloadLength;
        ubxParserHandle->remainingBytes = 2;
        ubxParserHandle->frameRemaining--;
        *ubxParserHandle->framePtr++ = byte;
        parserStatus = kStatus_UBXPROT_Parsing;
        break;
//    case ubxReserved1:
//        ubxParserHandle->remainingBytes--;
//        ubxParserHandle->frameRemaining--;
//        *ubxParserHandle->framePtr++ = byte;
//        if (ubxParserHandle->remainingBytes == 0) {
//            ubxParserHandle->parserState = ubxPayloadLength;
//            ubxParserHandle->remainingBytes = 2;
//        }
//        parserStatus = kStatus_UBXPROT_Parsing;
//        break;
    case ubxPayloadLength:
        ubxParserHandle->remainingBytes--;
        ubxParserHandle->frameRemaining--;
        *ubxParserHandle->framePtr++ = byte;
        if (ubxParserHandle->remainingBytes == 0) {
            ubxParserHandle->parserState = ubxPayload;
            ubx_frame_t* ubxFrame = (ubx_frame_t*)s_frameBuffers[ubxParserHandle->frameIndex];
            ubxParserHandle->remainingBytes = ubxFrame->payloadLength;
        }
        parserStatus = kStatus_UBXPROT_Parsing;
        break;
    case ubxPayload:
        ubxParserHandle->remainingBytes--;
        ubxParserHandle->frameRemaining--;
        *ubxParserHandle->framePtr++ = byte;
        if (ubxParserHandle->remainingBytes == 0) {
            ubxParserHandle->parserState = ubxChecksum;
            ubxParserHandle->remainingBytes = 2;
        }
        parserStatus = kStatus_UBXPROT_Parsing;
        break;
    case ubxChecksum:
        ubxParserHandle->remainingBytes--;
        ubxParserHandle->frameRemaining--;
        *ubxParserHandle->framePtr++ = byte;
        if (ubxParserHandle->remainingBytes == 0) {
            if (UbxFrameValidate((ubx_frame_t*)s_frameBuffers[ubxParserHandle->frameIndex])) {
                parserStatus = kStatus_UBXPROT_ValidFrame;
            } else {
                parserStatus = kStatus_UBXPROT_BadChecksum;
            }
            ubxParserHandle->parserState = ubxFrameComplete;
            ubxParserHandle->remainingBytes = UBX_MAX_REMAINING_BYTES;
        } else {
            parserStatus = kStatus_UBXPROT_Parsing;
        }
        break;
    case ubxFrameComplete:
        parserStatus = kStatus_UBXPROT_BufferOverflow;
        break;
    }

    return parserStatus;
}


