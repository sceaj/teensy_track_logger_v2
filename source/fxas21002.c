/*
 * fxas21002.c
 *
 *  Created on: Mar 11, 2021
 *      Author: jrosen
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

static uint8_t          s_flags;
static sensor_gyro_t    s_gyro;


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
    i2cData[0]= FXAS21002_WHOAMI;
    i2cData[1] = 0x00;

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXAS21002_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Write;
    i2cXfer.subaddress = 0;
    i2cXfer.subaddressSize = 0;
    i2cXfer.data = i2cData;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferNoStopFlag;

    status_t configStatus = LPI2C_MasterTransferBlocking(FXAS21002_I2C, &i2cXfer);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Read;
        i2cXfer.flags = kLPI2C_TransferRepeatedStartFlag;
        configStatus = LPI2C_MasterTransferBlocking(FXAS21002_I2C, &i2cXfer);
        if (configStatus == kStatus_Success) configStatus = (i2cData[0] == 0xD7) ? kStatus_Success : kStatus_FXAS21002_WrongDevice;
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.dataSize = 2;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        i2cData[0] = FXAS21002_CTRL_REG0;
        // max LPF cut-off freq, no HPF, +/-1000dps full scall
        i2cData[1] = 0x01;
        configStatus = LPI2C_MasterTransferBlocking(FXAS21002_I2C, &i2cXfer);
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cData[0] = FXAS21002_CTRL_REG1;
        // 50hz, active mode
        i2cData[1] = 0x12;
        configStatus = LPI2C_MasterTransferBlocking(FXAS21002_I2C, &i2cXfer);
    }

    return configStatus;

}

uint8_t FXAS21002_Flags() {
    return s_flags;
}

sensor_gyro_t* FXAS21002_Gyro() {
    return &s_gyro;
}

status_t FXAS21002_Process() {

    uint8_t i2cReg = 0x00;
    uint8_t sensorData[7];

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXAS21002_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Write;
    i2cXfer.subaddress = 0;
    i2cXfer.subaddressSize = 0;
    i2cXfer.data = &i2cReg;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferNoStopFlag;

    status_t i2cStatus = LPI2C_MasterTransferBlocking(FXAS21002_I2C, &i2cXfer);
    if (i2cStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Read;
        i2cXfer.data = sensorData;
        i2cXfer.dataSize = sizeof(sensorData);
        i2cXfer.flags = kLPI2C_TransferRepeatedStartFlag;
        i2cStatus = LPI2C_MasterTransferBlocking(FXAS21002_I2C, &i2cXfer);
        if (i2cStatus == kStatus_Success) {
            s_flags = sensorData[0];
            int16_t rawX = (int16_t)(sensorData[1]<<8 | sensorData[2]);
            s_gyro.x = (((rawX * 625) + 312) / 20480);
            int16_t rawY = (int16_t)(sensorData[3]<<8 | sensorData[4]);
            s_gyro.y = (((rawY * 625) + 312) / 20480);
            int16_t rawZ = (int16_t)(sensorData[5]<<8 | sensorData[6]);
            s_gyro.z = (((rawZ * 625) + 312) / 20480);
        }
    }

    return i2cStatus;
}
