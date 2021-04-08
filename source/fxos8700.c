/*
 * fxos8700cq.c
 *
 *  Created on: Mar 8, 2021
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

#include "fxos8700.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"
#include "peripherals.h"
#include "task.h"

#define FXOS8700_I2C        LPI2C1
#define FXOS8700_I2C_ADDR   0x1F
#define FXOS8700_I2C_HANDLE LPI2C1_masterHandle

#define FXOS8700_STATUS           0x00
#define FXOS8700_WHOAMI           0x0D
#define FXOS8700_XYZ_DATA_CFG     0x0E
#define FXOS8700_CTRL_REG1        0x2A
#define FXOS8700_M_CTRL_REG1      0x5B
#define FXOS8700_M_CTRL_REG2      0x5C
#define FXOS8700_WHOAMI_VAL       0xC7

uint8_t         fxos8700SensorData[13];


status_t FXOS8700_WhoAmI() {

    memset(LPI2C1_masterBuffer, 0, sizeof(LPI2C1_masterBuffer));

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXOS8700_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Read;
    i2cXfer.subaddress = FXOS8700_WHOAMI;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data = LPI2C1_masterBuffer;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferDefaultFlag;

    FXOS8700_I2C_HANDLE.userData = (void*)fxos8700_WhoAmI;

    return LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
}

status_t FXOS8700_Configure() {

    uint8_t i2cData[2];

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXOS8700_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Read;
    i2cXfer.subaddress = FXOS8700_WHOAMI;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data = i2cData;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferDefaultFlag;

    FXOS8700_I2C_HANDLE.userData = (void*)fxos8700_Config;

    status_t configStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
    if (configStatus == kStatus_Success) {
        while (FXOS8700_I2C_HANDLE.state) {
            // Wait for transfer to be complete (and the handle returns to idle state)
        }
        configStatus = (i2cData[0] == 0xC7) ? kStatus_Success : kStatus_FXOS8700_WrongDevice;
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXOS8700_CTRL_REG1;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // Go into standby mode
        i2cData[0] = 0x00;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
        while (FXOS8700_I2C_HANDLE.state) {
            // Wait for transfer to be complete (and the handle returns to idle state)
        }
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXOS8700_M_CTRL_REG1;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // Select hybrid mode, 4x oversampling
        i2cData[0] = 0x13;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
        while (FXOS8700_I2C_HANDLE.state) {
            // Wait for transfer to be complete (and the handle returns to idle state)
        }
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXOS8700_M_CTRL_REG2;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // Select hybrid autoinc mode
        i2cData[0] = 0x20;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
        while (FXOS8700_I2C_HANDLE.state) {
            // Wait for transfer to be complete (and the handle returns to idle state)
        }
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXOS8700_XYZ_DATA_CFG;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // Select +/-4g scale
        i2cData[0] = 0x01;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
        while (FXOS8700_I2C_HANDLE.state) {
            // Wait for transfer to be complete (and the handle returns to idle state)
        }
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXOS8700_CTRL_REG1;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // Select 25Hz (hybrid), low-noise, active
        i2cData[0] = 0x25;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
        while (FXOS8700_I2C_HANDLE.state) {
            // Wait for transfer to be complete (and the handle returns to idle state)
        }
    }

    return configStatus;

}

status_t FXOS8700_RequestData() {

    uint8_t i2cReg = 0x00;

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXOS8700_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Read;
    i2cXfer.subaddress = FXOS8700_STATUS;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data = fxos8700SensorData;
    i2cXfer.dataSize = sizeof(fxos8700SensorData);
    i2cXfer.flags = kLPI2C_TransferNoStopFlag;

    FXOS8700_I2C_HANDLE.userData = (void*)fxos8700_Data;

    status_t i2cStatus = LPI2C_MasterTransferNonBlocking(FXOS8700_I2C, &FXOS8700_I2C_HANDLE, &i2cXfer);
    if (i2cStatus != kStatus_Success) {
        i2cStatus = kStatus_FXOS8700_BusBusy;
    }

    return i2cStatus;
}
