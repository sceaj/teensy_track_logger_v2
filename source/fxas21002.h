/*
 * fxas21002.h
 *
 *  Created on: Mar 11, 2021
 *      Author: jrosen
 */

#ifndef FXAS21002_H_
#define FXAS21002_H_

#include "fsl_common.h"

#define kStatusGroup_FXAS21002 203
enum {
    kStatus_FXAS21002_WrongDevice = MAKE_STATUS(kStatusGroup_FXAS21002, 0),
    kStatus_FXAS21002_Error1 = MAKE_STATUS(kStatusGroup_FXAS21002, 1),
    kStatus_FXAS21002_Error2 = MAKE_STATUS(kStatusGroup_FXAS21002, 2),
    kStatus_FXAS21002_Error3 = MAKE_STATUS(kStatusGroup_FXAS21002, 3),
};

// UserData specified for specific requests
enum {
    fxas21002_WhoAmI        = 0xD701U,
    fxas21002_Gyro          = 0xD703U,
};

typedef enum {
    FXAS21002_Standby,
    FXAS21002_Ready,
    FXAS21002_Active,
} sensor_gyro_mode_t;


typedef struct sensor_gyro {
    int16_t     x;
    int16_t     y;
    int16_t     z;
} sensor_gyro_t;

status_t FXAS21002_WhoAmI();

status_t FXAS21002_Mode(sensor_gyro_mode_t mode);

status_t FXAS21002_Configure();

uint8_t FXAS21002_Flags();

sensor_gyro_t* FXAS21002_Gyro();

status_t FXAS21002_Process();

#endif /* FXAS21002_H_ */
