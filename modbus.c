﻿/**
 * Implementation of the Modbus protocol
 *
 * Copyright (c) 2017, Anthony Rabine
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the BSD license.
 * See LICENSE.txt for more details.
 *
 */

#include <stdlib.h>
#include "modbus.h"

//--------------------------------------------------------------------------
// Prototypes
//--------------------------------------------------------------------------
uint8_t modbus_function3(uint8_t *data, uint16_t len, uint16_t *rep_len);
uint8_t modbus_function16(uint8_t *data, uint16_t len, uint16_t *rep_len);
uint8_t modbus_process_pdu(uint8_t function_code, uint8_t *data, uint16_t len, uint16_t *rep_len);


//--------------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------------
static const uint16_t modbus_crc_table[256] = {
   0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
   0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
   0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
   0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
   0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
   0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
   0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
   0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
   0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
   0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
   0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
   0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
   0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
   0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
   0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
   0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
   0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
   0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
   0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
   0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
   0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
   0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
   0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
   0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
   0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
   0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
   0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
   0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
   0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
   0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
   0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
   0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

//--------------------------------------------------------------------------
// Globals
//--------------------------------------------------------------------------
static uint8_t mb_slave_address = 0U;
static uint8_t mb_mode = MODBUS_RTU;
static const VIRTUAL_SECTION *mb_mapping = NULL;
static uint32_t mb_mapping_size = 0U;

//--------------------------------------------------------------------------
// Modbus functions
//--------------------------------------------------------------------------
void modbus_initialize(uint8_t slave_addr, uint8_t mode, const VIRTUAL_SECTION *mapping, uint32_t mapping_size)
{
    mb_slave_address = slave_addr;
    mb_mode = mode;
    mb_mapping = mapping;
    mb_mapping_size = mapping_size;
}

uint8_t modbus_lrc_calc(uint8_t *data, uint16_t len)
{
   uint8_t lrc = 0U;
   int i;

   for (i = 0U; i < len; i++)
   {
      lrc += data[i];
   }
   lrc = (0xFFU - lrc)+1U;
   return(lrc);
}


static uint16_t crc16(uint16_t crc, uint8_t c)
{
   crc  = modbus_crc_table[((crc & 255U )^(c))];
   crc ^= crc >> 8U;

   return crc;
}


uint16_t modbus_crc_calc(uint8_t *buffer, uint16_t size)
{
   uint16_t crc = 0xFFFFU;
   uint8_t *ptr;

   for (ptr = buffer; ptr < (buffer + size); ptr++)
   {
        crc = crc16(crc, (*ptr) & 0xFFU);
   }
   return(crc);
}

static inline uint8_t modbus_crc_size()
{
    return (mb_mode == MODBUS_ASCII) ? 1U : (mb_mode == MODBUS_TCP) ? 0U : 2U;
}

uint8_t modbus_check_crc(uint8_t *packet, uint16_t length)
{
    uint8_t retcode = 1U;

    if(mb_mode == MODBUS_ASCII )
    {
       uint8_t lrc = packet[length - 1];
       if (lrc != modbus_lrc_calc(packet, length - modbus_crc_size()))
       {
           retcode = 0U;
       }
    }
    else if (mb_mode == MODBUS_RTU)
    {
        uint16_t crc = (uint16_t)(packet[length - 1] << 8) +
               (uint16_t)(packet[length - 2]);
        if (crc != modbus_crc_calc(packet, length - modbus_crc_size()))
        {
           retcode = 0U;
        }
    }
    else if (mb_mode == MODBUS_TCP)
    {
        // no error, no checksum in TCP
    }
    else
    {
        retcode = 0U;
    }

    return retcode;
}

static inline uint16_t get_uint16(uint8_t *data)
{
    uint16_t val = (uint16_t)(data[0] << 8) + (uint16_t)(data[1]);
    return val;
}


int32_t modbus_process(uint8_t *packet, uint16_t length)
{
    int32_t retcode = MB_PACKET_ERROR_SIZE;
    uint8_t *data = &packet[0];
    uint16_t data_size = length;
    uint8_t valid = 1U;

    if (mb_mode == MODBUS_TCP)
    {
       // uint16_t trans_id = get_uint16(data); // FIXME: test transaction ID, should be greater than last one
        uint16_t proto_id = get_uint16(data+2);
        data_size = get_uint16(data+4);

        if (proto_id != 0U)
        {
            valid = 0U;
        }

        if (data_size != (length - 6U))
        {
            valid = 0U;
        }

        // Skip MBAP header
        data = &packet[6];
    }

    // basic packet size check
    if ((data_size < 5U) || (data_size > MAX_MODBUS_LENGTH))
    {
        valid = 0U;
    }

    // CRC check
    if (modbus_check_crc(data, data_size) == 0U)
    {
        valid = 0U;
        retcode = MB_PACKET_ERROR_CRC;
    }

    if (valid)
    {
        uint8_t dst_addr = data[0];

        if ((dst_addr == mb_slave_address) || (dst_addr == 0U))
        {
            // Proccess request, size is packet length minus : slave addr + function code + CRC
            data_size -= (2U + modbus_crc_size());
            uint16_t rep_size = 0U;
            uint8_t status = modbus_process_pdu(data[1], &data[2], data_size, &rep_size);

            if(dst_addr != 0U)
            {
                if (status == MODBUS_NO_ERROR)
                {
                    // No broadcast, reply
                    if (mb_mode == MODBUS_RTU )
                    {
                        uint16_t crc = modbus_crc_calc(data, rep_size);
                        // Append CRC in little endian at the end of the packet
                        data[rep_size] = crc & 0xFFU;
                        data[rep_size + 1U] = crc >> 8U;

                        // Update size of the reply: data + CRC + function code + slave address
                        retcode = rep_size + 4U;
                    }
                    else if (mb_mode == MODBUS_ASCII )
                    {
                        uint8_t lrc = modbus_lrc_calc(data, rep_size);
                        data[rep_size] = lrc & 0xFFU;
                        // Update size of the reply: data + LRC + function code + slave address
                        retcode = rep_size + 3U;
                    }
                    else if (mb_mode == MODBUS_TCP )
                    {
                        // No check bytes in TCP mode
                        // But we must indicate the Modbus frame size in the MBAP
                        rep_size += 2U; // Add function code + slave address
                        packet[4] = rep_size >> 8U;
                        packet[5] = rep_size & 0xFFU;
                        retcode = rep_size + 6U; // Add MBAP size
                    }
                }
                else
                {
                    // Exception code
                   data[1] |= 0x80U;
                   data[2] = status;
                   retcode = 1U;
                }
            }
        }
        else
        {
            retcode = MB_PACKET_ERROR_ADDR;
        }
    }
    else
    {
        retcode = MB_PACKET_ERROR_CRC;
    }


    return retcode;
}

uint8_t modbus_process_pdu(uint8_t function_code, uint8_t *data, uint16_t len, uint16_t *rep_len)
{
    uint8_t retcode = MODBUS_ILLEGAL_FUNCTION;

    switch (function_code)
    {
    case 3:
    case 4:
        retcode = modbus_function3(data, len, rep_len);
    break;
    case 16 :
        retcode = modbus_function16(data, len, rep_len);
    break;
        // TODO: allow custom functions (call user defined function)
    default:
     break;
    }
    return(retcode);
}

static inline uint8_t is_in_section(uint16_t addr, uint8_t sec)
{
    uint8_t ret = 0U;
    if ((addr >= mb_mapping[sec].addr) && (addr < (mb_mapping[sec].addr + mb_mapping[sec].size)))
    {
        ret = 1U;
    }
    return ret;
}

static inline uint8_t find_section(uint16_t addr, uint16_t *section)
{
    uint8_t retcode = MODBUS_ILLEGAL_ADDRESS;
    // Find section
    for (uint16_t i = 0U; i < mb_mapping_size; i++)
    {
        if (is_in_section(addr, i))
        {
            *section = i;
            retcode = MODBUS_NO_ERROR;
            break;
        }
    }

    return retcode;
}

static inline uint16_t clamp(uint16_t section, uint16_t start_addr, uint16_t nb_words)
{
    // Maximum number of words that can be accessed in this section
    uint16_t max_words = mb_mapping[section].size - (start_addr - mb_mapping[section].addr);
    // Set the limit to the lower number of words to read ro write
    max_words = (max_words < nb_words) ? max_words : nb_words;

    return max_words;
}

/**
 * @brief modbus_function3
 * @param data
 * @param len
 * @return == 0 : OK
 *          > 0 : Modbus exception code
 */
uint8_t modbus_function3(uint8_t *data, uint16_t len, uint16_t *rep_len)
{
    uint16_t nb_words, start_addr;
    uint16_t i;
    uint8_t retcode = MODBUS_ILLEGAL_ADDRESS;

    if (len == 4U)
    {
        start_addr = get_uint16(data);
        nb_words = get_uint16(data + 2);
        if ((nb_words <= MAX_WORD_TO_READ) && (nb_words > 0U))
        {
            uint16_t section = 0;

            if (find_section(start_addr, &section) == MODBUS_NO_ERROR)
            {
                // Maximum number of words that can be read in this section
                nb_words = clamp(section, start_addr, nb_words);

                // Point to the first data to read
                uint16_t *ptr = mb_mapping[section].data + (start_addr - mb_mapping[section].addr);

                // Read data words as most as possible (limited to section size)
                for (i = 0U; i < nb_words; i++, ptr++ )
                {
                    data[1 + i*2] = *ptr >> 8U;
                    data[2 + i*2] = *ptr & 0xFFU;
                }
                // Request is good, prepare reply
                data[0] = i * 2;
                *rep_len = 1 + data[0];
                retcode = MODBUS_NO_ERROR;
            }
        }
        else
        {
            retcode = MODBUS_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        retcode = MODBUS_ILLEGAL_DATA_VALUE;
    }

    return (retcode);
}


uint8_t modbus_function16(uint8_t *data, uint16_t len, uint16_t *rep_len)
{
   uint16_t ret = MODBUS_ILLEGAL_DATA_VALUE;
   uint16_t nb_words, start_addr;
   uint16_t i;

   /**
    * Test de la longueur de la trame :
    * adresse 1er mot + nbr de mots + nbr d'octets N + N octets
    *        2              2              1              N       = 5 + N octets
    * Au moins 1 mot est transféré, donc taille mini = 7
    */
   if (len >= 7U)
   {
       // Adresse du premier mot à écrire
       start_addr = (uint16_t)(data[0] << 8) + data[1];
       // Nombre de mots à écrire
       nb_words = (uint16_t)(data[2] << 8) + data[3];

       /**
        * Test validité des données : nombre de mots egal à 2 fois
        * nombre d'octets et nombre d'octets != 0
        */
       if (((nb_words * 2) == data[4]) &&
           (nb_words != 0) &&
           (nb_words <= MAX_WORD_TO_WRITE))
       {
            uint16_t section = 0;

            // Maximum number of words that can be written in this section
            nb_words = clamp(section, start_addr, nb_words);

            // Point to the first data to read
            uint16_t *ptr = mb_mapping[section].data + (start_addr - mb_mapping[section].addr);

            ret = find_section(start_addr, &section);
            if ((ret == MODBUS_NO_ERROR) &&
               (mb_mapping[section].access == MDB_READ_WRITE))
            {
                 for( i = 0U; i < nb_words; i++ )
                 {
                    (*ptr++) = (uint16_t)( data[5 + 2*i] << 8) | data[6 + 2*i];
                 }

                 ret = MODBUS_NO_ERROR;
                 *rep_len = 4;   /* réponse sans le CRC */
            }
            else
            {
                ret = MODBUS_ILLEGAL_ADDRESS;
            }
       }
   }
   return (ret);
}


#ifdef AUTOTEST

struct ReadOnlyData
{
    uint16_t hop;
    uint16_t hip;
};

ReadOnlyData data1 = {
  1000, 4
};

static const VIRTUAL_SECTION mapping[] = {
   { (uint16_t *)&data1, 0x0000, SECTION_SIZE(ReadOnlyData), MDB_READ_WRITE }
};

#define NB_SECTIONS  (sizeof(mapping)/sizeof(mapping[0]))



int main(void)
{
    static const uint8_t read_holding[] = { 0x00, 0x12, 0x00, 0x00, 0x00, 0x06, 0x0C, 0x03, 0x00, 0x00, 0x00, 0x0A };

    modbus_initialize(12, MODBUS_TCP, mapping, NB_SECTIONS);

    int32_t ret = modbus_process(data, size);
}
#endif



// End of file
