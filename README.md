# Modbus library

Tiny Modbus RTU and ASCII library in C.

## Goals

They are many existing Modbus libraries out there. This one will differ from them on these aspects:

  * Pure C99 software implementation, no OS specific calls
  * Just one function to call!
  * BSD license

## Features

  * Modbus TCP, RTU and ASCII (maybe broken) packets
  * Slave functions 3, 4, 6 and 16
  * Exception
  
## TODO

  * More unit tests
  * Coils support
  * Master support

## Install

Just add the two modbus.c and modbus.h files in your project.
  
## Example

Here is a minimal example:

```c
#include <string.h>
#include <stdio.h>

struct app_data_t
{
    uint16_t hop;
    uint16_t hip;
} app_data = {1000U, 4U};


static const modbus_section_t section = { (uint16_t *)&app_data, 0x0000U, SECTION_SIZE(app_data), MDB_READ_WRITE };

const modbus_ctx_t context = { 12U, MODBUS_TCP, &section, 1U };

int main(void)
{
    static const uint8_t read_holding[] = { 0x00, 0x12, 0x00, 0x00, 0x00, 0x06, 0x0C, 0x03, 0x00, 0x00, 0x00, 0x0A };

    uint8_t data[MAX_DATA_LENGTH];

    memcpy(data, read_holding, sizeof(read_holding));

    int32_t ret = modbus_process(&context, data, sizeof(read_holding));

    if (ret >= 0)
    {
        printf("Success! response size is %d bytes.\r\n", (int)ret);
    }
}

```

Copy paste this code snippet in a file called main.c, then in call gcc -o modbus modbus.c main.c.

## License

Copyright (c) 2017, Anthony Rabine
All rights reserved.

This software may be modified and distributed under the terms of the BSD license.
See LICENSE.txt for more details.
 