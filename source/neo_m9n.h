/*
 * neo_m9n.h
 *
 *  Created on: Feb 21, 2021
 *      Author: jrosen
 */

#ifndef NEO_M9N_H_
#define NEO_M9N_H_

#include "fsl_common.h"

#define kStatusGroup_GPS 201
enum {
    kStatus_GPS_NoFix = MAKE_STATUS(kStatusGroup_GPS, 0),
    kStatus_GPS_NoComm = MAKE_STATUS(kStatusGroup_GPS, 1),
    kStatus_GPS_CommError = MAKE_STATUS(kStatusGroup_GPS, 2),
    kStatus_GPS_Timeout = MAKE_STATUS(kStatusGroup_GPS, 3),
};

typedef struct gps_time {
    uint32_t    iTOW;
    int32_t     nano;
    uint16_t    year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     min;
    uint8_t     sec;
    uint8_t     valid;
    uint8_t     flags2;
} gps_time_t;

typedef struct gps_position {
    uint32_t    iTOW;
    int32_t     lon;
    int32_t     lat;
    int32_t     height;
    uint32_t    hMSL;
    uint32_t    hAcc;
    uint8_t     fixType;
    uint8_t     flags;
} gps_position_t;

typedef struct gps_velocity {
    uint32_t    iTOW;
    int32_t     gSpeed;
    int32_t     headMot;
    uint32_t    sAcc;
    uint32_t    headAcc;
    uint16_t    pDOP;
    uint8_t     flags;
} gps_velocity_t;

status_t SetUartBaud(uint32_t newBaud);

status_t ReadData();

void NEOM9N_DebugCommCounters();

status_t NEOM9N_UartInit();

status_t NEOM9N_GpsConfig();

void NEOM9N_Process();

gps_time_t* NEOM9N_GpsTime();

gps_position_t* NEOM9N_GpsPosition();

gps_velocity_t* NEOM9N_GpsVelocity();

#endif /* NEO_M9N_H_ */
