/*
 * logrecord.h
 *
 *  Created on: Mar 28, 2021
 *      Author: jrosen
 */

#ifndef LOGRECORD_H_
#define LOGRECORD_H_

#include "fsl_common.h"

#define LOGRECORD_CLASS_GPS         0x20
#define LOGRECORD_CLASS_DYNAMICS    0x30

#define GPS_TIME                    0x01
#define GPS_POSITION                0x02
#define GPS_VELOCITY                0x03

#define DYNAMICS_WHEELSPEED         0x01
#define DYNAMICS_ACCELEROMETER      0x02
#define DYNAMICS_GYRO               0x03

typedef struct record_header {
    uint8_t     prelude[2];
    uint8_t     class;
    uint8_t     id;
    uint32_t    timestamp;
} record_header_t;

typedef struct log_record {
    record_header_t     header;
    uint8_t             payload[22];
    uint8_t             chkA;
    uint8_t             chkB;
} log_record_t;

void LogRecordHeaderInit(record_header_t* header, uint8_t class, uint8_t id);

void LogRecordFinalize(log_record_t *record);

#endif /* LOGRECORD_H_ */
