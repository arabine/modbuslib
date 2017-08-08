/**
 * Implementation of the Modbus protocol
 *
 * Copyright (c) 2017, Anthony Rabine
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the BSD license.
 * See LICENSE.txt for more details.
 *
 */


#ifndef MODBUS_H
#define MODBUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

//--------------------------------------------------------------------------
// Definitions 
//--------------------------------------------------------------------------

#define MB_PACKET_ERROR_SIZE    -1
#define MB_PACKET_ERROR_CRC     -2
#define MB_PACKET_ERROR_ADDR    -3
#define MB_PACKET_ERROR_UNKOWN  -4

/* Official modbus errors */
#define MODBUS_NO_ERROR             0
#define MODBUS_ILLEGAL_FUNCTION     1
#define MODBUS_ILLEGAL_ADDRESS      2
#define MODBUS_ILLEGAL_DATA_VALUE   3
#define MODBUS_DEVICE_FAILURE       4
#define MODBUS_ACKNOWLEDGE          5
#define MODBUS_DEVICE_BUSY          6
#define MODBUS_NO_ACKNOWLEDGE       7
#define MODBUS_WRITE_ERROR          8
#define MODBUS_OVERLAPPED_AREA      9

/* Définitions MODBUS */
#define MAX_WORD_TO_READ      125
#define MAX_WORD_TO_WRITE     123
#define MAX_MODBUS_LENGTH     256
#define MAX_DATA_LENGTH       253


/* Mode : RTU ou ASCII */
#define MODBUS_RTU   0U
#define MODBUS_ASCII 1U
#define MODBUS_TCP   2U



typedef enum {
   MDB_READ_ONLY,
   MDB_READ_WRITE
} SECTION_ACCESS;

#define SECTION_SIZE(a)     (sizeof(a)/2U)

typedef struct {
   uint16_t *data;  // raw data
   uint16_t addr;   // modbus address in the mapping
   uint16_t size;   // size of the data, in words
   SECTION_ACCESS access;   // access type selector
} VIRTUAL_SECTION;


//--------------------------------------------------------------------------
// API 
//--------------------------------------------------------------------------
void modbus_initialize(uint8_t slave_addr, uint8_t mode /* ascii or rtu */, const VIRTUAL_SECTION *mapping, uint32_t mapping_size /* number of sections */);

/**
 * @brief modbus_process
 * @param packet
 * @return < 0  : error
 *         == 0 : Ok, no reply to send
 *         > 0  : ok, size of the reply
 */
int32_t modbus_process(uint8_t *packet, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H */

// End of file

