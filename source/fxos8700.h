/*
 * fxos8700cq.h
 *
 *  Created on: Mar 8, 2021
 *      Author: jrosen
 */

#ifndef FXOS8700_H_
#define FXOS8700_H_

#include "fsl_common.h"

#define kStatusGroup_FXOS8700 202
enum {
    kStatus_FXOS8700_WrongDevice = MAKE_STATUS(kStatusGroup_FXOS8700, 0),
    kStatus_FXOS8700_Error1 = MAKE_STATUS(kStatusGroup_FXOS8700, 1),
    kStatus_FXOS8700_Error2 = MAKE_STATUS(kStatusGroup_FXOS8700, 2),
    kStatus_FXOS8700_Error3 = MAKE_STATUS(kStatusGroup_FXOS8700, 3),
};

// UserData specified for specific requests
enum {
    fxos8700_WhoAmI         = 0xC701U,
    fxos8700_Config         = 0xC702U,
    fxos8700_Acceleration   = 0xC703U,
    fxos8700_Magnetometer   = 0xC704U,
};

// scaled to LSB = 0.01g
typedef struct sensor_accel {
    int16_t     x;
    int16_t     y;
    int16_t     z;
} sensor_accel_t;

typedef struct sensor_mags {
    int16_t     x;
    int16_t     y;
    int16_t     z;

} sensor_mags_t;

status_t FXOS8700_WhoAmI();

status_t FXOS8700_Configure();

uint8_t FXOS8700_Flags();

sensor_accel_t* FXOS8700_Acceleration();

sensor_mags_t* FXOS8700_Compass();

status_t FXOS8700_Process();

#endif /* FXOS8700_H_ */
