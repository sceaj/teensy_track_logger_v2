/*
 * fxos8700cq.c
 *
 *  Created on: Mar 8, 2021
 *      Author: jrosen
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

static uint8_t          s_flags;
static sensor_accel_t   s_acceleration;
static sensor_mags_t    s_magnetometer;


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
    i2cData[0]= FXOS8700_WHOAMI;
    i2cData[1] = 0x00;

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXOS8700_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Write;
    i2cXfer.subaddress = 0;
    i2cXfer.subaddressSize = 0;
    i2cXfer.data = i2cData;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferNoStopFlag;

    status_t configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Read;
        i2cXfer.flags = kLPI2C_TransferRepeatedStartFlag;
        configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
        if (configStatus == kStatus_Success) configStatus = (i2cData[0] == 0xC7) ? kStatus_Success : kStatus_FXOS8700_WrongDevice;
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Write;
        i2cXfer.dataSize = 2;
        i2cXfer.flags = kLPI2C_TransferDefaultFlag;
        // Go into standby mode
        i2cData[0] = FXOS8700_CTRL_REG1;
        i2cData[1] = 0x00;
        configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        // Select hybrid mode, 4x oversampling
        i2cData[0] = FXOS8700_M_CTRL_REG1;
        i2cData[1] = 0x13;
        configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        // Select hybrid autoinc mode
        i2cData[0] = FXOS8700_M_CTRL_REG2;
        i2cData[1] = 0x20;
        configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        // Select +/-4g scale
        i2cData[0] = FXOS8700_XYZ_DATA_CFG;
        i2cData[1] = 0x01;
        configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    }

    SDK_DelayAtLeastUs(100U, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
    if (configStatus == kStatus_Success) {
        // Select 25Hz (hybrid), low-noise, active
        i2cData[0] = FXOS8700_CTRL_REG1;
        i2cData[1] = 0x25;
        configStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    }

    return configStatus;

}

uint8_t FXOS8700_Flags() {
    return s_flags;
}

sensor_accel_t* FXOS8700_Acceleration() {
    return &s_acceleration;
}

sensor_mags_t* FXOS8700_Compass() {
    return &s_magnetometer;
}

status_t FXOS8700_Process() {

    uint8_t i2cReg = 0x00;
    uint8_t sensorData[13];

    lpi2c_master_transfer_t i2cXfer;
    memset(&i2cXfer, 0, sizeof(i2cXfer));
    i2cXfer.slaveAddress = FXOS8700_I2C_ADDR;
    i2cXfer.direction = kLPI2C_Write;
    i2cXfer.subaddress = 0;
    i2cXfer.subaddressSize = 0;
    i2cXfer.data = &i2cReg;
    i2cXfer.dataSize = 1;
    i2cXfer.flags = kLPI2C_TransferNoStopFlag;

    status_t i2cStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
    if (i2cStatus == kStatus_Success) {
        i2cXfer.direction = kLPI2C_Read;
        i2cXfer.data = sensorData;
        i2cXfer.dataSize = sizeof(sensorData);
        i2cXfer.flags = kLPI2C_TransferRepeatedStartFlag;
        i2cStatus = LPI2C_MasterTransferBlocking(FXOS8700_I2C, &i2cXfer);
        if (i2cStatus == kStatus_Success) {
            s_flags = sensorData[0];
            int16_t rawX = (int16_t)(sensorData[1]<<8 | sensorData[2]) >> 2;
            s_acceleration.x = (((rawX * 25) + 1280) / 2560) * 5;
            int16_t rawY = (int16_t)(sensorData[3]<<8 | sensorData[4]) >> 2;
            s_acceleration.y = (((rawY * 25) + 1280) / 2560) * 5;
            int16_t rawZ = (int16_t)(sensorData[5]<<8 | sensorData[6]) >> 2;
            s_acceleration.z = (((rawZ * 25) + 1280) / 2560) * 5;

            s_magnetometer.x = (int16_t)(sensorData[7]<<8 | sensorData[8]);
            s_magnetometer.y = (int16_t)(sensorData[9]<<8 | sensorData[10]);
            s_magnetometer.z = (int16_t)(sensorData[11]<<8 | sensorData[12]);
        }
    }

    return i2cStatus;
}
