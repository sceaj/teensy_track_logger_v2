/*
 * logrecord.c
 *
 *  Created on: Mar 28, 2021
 *      Author: jrosen
 */



#include "logrecord.h"
#

void LogRecordHeaderInit(record_header_t* header, uint8_t recordClass, uint8_t recordId) {

    header->prelude[0] = 0x05;
    header->prelude[1] = 0xA7;

    header->class = recordClass;
    header->id = recordId;
}

void LogRecordFinalize(log_record_t *record) {

    record->chkA = record->header.class + record->header.id;
    record->chkB = record->header.class + record->chkA;

    for (int i = 0; i < sizeof(record->payload); i++) {
        record->chkA += record->payload[i++];
        record->chkB += record->chkA;
    }
}
