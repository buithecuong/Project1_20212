#ifndef __INC_BOOT_LOADER_H__
#define __INC_BOOT_LOADER_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include "sim800c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

typedef void (application_t)(void);
typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;         // Program Counter
} JumpStruct;

void enter_firmware(uint32_t firmware_address);

#endif
