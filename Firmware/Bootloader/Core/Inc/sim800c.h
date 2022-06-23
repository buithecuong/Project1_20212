#ifndef _INC_SIM800C_H__
#define _INC_SIM800C_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define SIM_NOT_CHECK           0
#define SIM_CHECK               1

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct 
{
    int method;
    int statusCode;
    int length;
} httpactionType_t;

typedef enum
{
    SIM_NOT_OK,
    SIM_OK
} simState_t;

typedef enum
{
    SIM_NOT_RX_CPLT,
    SIM_RX_CPLT
} simRxCpltStruct_t;

// simState_t sim800c_trigPwrkey(void);
// GPIO_PinState sim800c_checkStatus(bool delayornot);
void sim800c_debug(uint8_t *pData, uint16_t Size, uint32_t Timeout);
void sim800c_init(void);
simState_t sim800c_sendCommand(char *simCommand, char *trueResponse, int checkOrNot);
void sim800c_callback(void);
void sim800c_timer1Callback(void);
void sim800c_timer2Callback(void);
void sim800c_sendSMS(char *latData, char *longData, char *simNumber, char *simMessage);
void sim800c_errorHandle(void);
void sim800c_GSM_GPRS(void);
simState_t sim800c_startOTA(void);
void sim800c_connectServer(char *link);
void sim800c_initHTTP(void);
#endif



