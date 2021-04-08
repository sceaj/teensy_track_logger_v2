/*
 * fxas21002.c
 *
 *  Created on: Mar 11, 2021
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

#include "fxas21002.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"
#include "peripherals.h"

#define FXAS21002_I2C           LPI2C1
#define FXAS21002_I2C_ADDR      0x21
#define FXAS21002_I2C_HANDLE    LPI2C1_masterHandle

#define FXAS21002_STATUS        0x00
#define FXAS21002_WHOAMI        0x0C
#define FXAS21002_CTRL_REG0     0x0D
#define FXAS21002_CTRL_REG1     0x13
#define FXAS21002_CTRL_REG2     0x14
#define FXAS21002_CTRL_REG3     0x15
#define FXAS21002_WHOAMI_VAL    0xD7

uint8_t fxas21002SensorData[7];

status_t FXAS21002_WhoAmI() {

    memset(LPI2C1_masterBuffer, 0, sizeof(LPI2C1_masterBuffer));

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXAS21002_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Read;
    i2cXfer.subaddress = FXAS21002_WHOAMI;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data = LPI2C1_masterBuffer;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferDefaultFlag;

    FXAS21002_I2C_HANDLE.userData = (void*)fxas21002_WhoAmI;

    return LPI2C_MasterTransferNonBlocking(FXAS21002_I2C, &FXAS21002_I2C_HANDLE, &i2cXfer);
}

//status_t FXAS21002_Mode(sensor_gyro_mode_t mode) {
//
//}

status_t FXAS21002_Configure() {

    uint8_t i2cData[2];

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXAS21002_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Read;
    i2cXfer.subaddress = FXAS21002_WHOAMI;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data = i2cData;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferDefaultFlag;

    FXAS21002_I2C_HANDLE.userData = (void*)fxas21002_Config;

    status_t configStatus = LPI2C_MasterTransferNonBlocking(FXAS21002_I2C, &FXAS21002_I2C_HANDLE, &i2cXfer);
    if (configStatus == kStatus_Success) {
        while (FXAS21002_I2C_HANDLE.state) {
            // Wait for transfer to complete
        }
        configStatus = (i2cData[0] == 0xD7) ? kStatus_Success : kStatus_FXAS21002_WrongDevice;
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXAS21002_CTRL_REG0;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // max LPF cut-off freq, no HPF, +/-1000dps full scall
        i2cData[0] = 0x01;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXAS21002_I2C, &FXAS21002_I2C_HANDLE, &i2cXfer);
        while (FXAS21002_I2C_HANDLE.state) {
            // Wait for transfer to complete
        }
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.subaddress = FXAS21002_CTRL_REG1;
        i2cXfer.subaddressSize = 1;
        i2cXfer.data = i2cData;
        i2cXfer.dataSize = 1;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // 50hz, active mode
        i2cData[0] = 0x12;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferNonBlocking(FXAS21002_I2C, &FXAS21002_I2C_HANDLE, &i2cXfer);
        while (FXAS21002_I2C_HANDLE.state) {
            // Wait for transfer to complete
        }
    }

    return configStatus;

}

status_t FXAS21002_RequestData() {

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXAS21002_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Read;
    i2cXfer.subaddress = FXAS21002_STATUS;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data = fxas21002SensorData;
    i2cXfer.dataSize = 7;
    i2cXfer.flags = kLPI2C_TransferDefaultFlag;

    FXAS21002_I2C_HANDLE.userData = (void*)fxas21002_Data;

    status_t i2cStatus = LPI2C_MasterTransferNonBlocking(FXAS21002_I2C, &FXAS21002_I2C_HANDLE, &i2cXfer);
    if (i2cStatus == kStatus_Success) {
        i2cStatus = kStatus_FXAS21002_BusBusy;
    }

    return i2cStatus;
}
