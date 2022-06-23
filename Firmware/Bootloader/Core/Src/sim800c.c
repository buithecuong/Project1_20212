#include "sim800c.h"

char simRxBuf[100] = {0};
int simRxIndex = 0;
char simRxData = '\0';
char simRxResponse[100] = {0};
simRxCpltStruct_t simRxCplt = SIM_NOT_RX_CPLT;

char simRxOTABuf[1100] = {0};
int simRxOTAIndex = 0;
simRxCpltStruct_t simRxOTACplt = SIM_NOT_RX_CPLT;
httpactionType_t httpaction;

extern bool is_sim_on;
bool is_ota = false;

void sim800c_debug(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_UART_Transmit(&huart1, pData, Size , Timeout);
}

void sim800c_init(void)
{
    if(sim800c_sendCommand("ATE0\r\n", "OK\r\n", SIM_NOT_CHECK) != SIM_OK)
    {
        sim800c_errorHandle();
    }
    if(sim800c_sendCommand("AT\r\n", "OK\r\n", SIM_CHECK) != SIM_OK)
    {
        sim800c_errorHandle();
    }
    sim800c_GSM_GPRS();    
}

simState_t sim800c_sendCommand(char *simCommand, char *trueResponse, int checkOrNot)
{   
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&simRxData, 1);
    sim800c_debug((uint8_t*)simCommand, strlen(simCommand), 100);
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)simCommand, strlen(simCommand));

//    HAL_Delay(2000);
    HAL_TIM_Base_Start_IT(&htim2);
    while(!(simRxCplt == SIM_RX_CPLT));
    sim800c_debug((uint8_t*)simRxResponse, strlen(simRxResponse), 100);
    if((checkOrNot == SIM_CHECK) && (strstr(simRxResponse, trueResponse) == NULL))
    {
        memset(simRxResponse, '\0', strlen(simRxResponse));
        simRxCplt = SIM_NOT_RX_CPLT;
        return SIM_NOT_OK;
    }
    memset(simRxResponse, '\0', strlen(simRxResponse));
    simRxCplt = SIM_NOT_RX_CPLT;
    return SIM_OK;
}

void sim800c_callback(void)
{
    if(is_ota == false)
    {
        TIM2->CNT = 0;
        simRxBuf[simRxIndex++] = simRxData;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&simRxData, 1);
        if((strstr(simRxBuf, "\r\n") != NULL) && (simRxIndex == 2))
        {
            memset(simRxBuf, '\0', strlen(simRxBuf));
            simRxIndex = 0;
        }
    }
    // OTA
    else
    {
        TIM3->CNT = 0;
        simRxOTABuf[simRxOTAIndex++] = simRxData;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&simRxData, 1);
        if((strstr(simRxOTABuf, "\r\n") != NULL) && (simRxOTAIndex == 2))
        {
            memset(simRxOTABuf, '\0', strlen(simRxOTABuf));
            simRxOTAIndex = 0;
        }
    }
}

void sim800c_timer2Callback(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    TIM2->CNT = 0;
    if((strstr(simRxBuf, "\r\n") != NULL) | (strstr(simRxBuf, ">") != NULL))
    {
        memcpy(simRxResponse, simRxBuf, strlen(simRxBuf));
        memset(simRxBuf, '\0', strlen(simRxBuf));
        simRxIndex = 0;
    }
    simRxCplt = SIM_RX_CPLT;       
}

void sim800c_timer3Callback(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    TIM1->CNT = 0;
    is_ota = false;
    simRxOTACplt = SIM_RX_CPLT;
}


void sim800c_sendSMS(char *latData, char *longData, char *simNumber, char *simMessage)
{
    char smsATCommand[50] = {0};
    char endSMS[2] = {0};
    endSMS[0] = 0x1A;
    endSMS[1] = '\0';
    sprintf(smsATCommand, "AT+CMGS=\"%s\"\r\n", simNumber);
    if(sim800c_sendCommand("AT+CMGF=1\r\n", "OK\r\n", SIM_CHECK) != SIM_OK)
    {
        sim800c_errorHandle();
    }
    if(sim800c_sendCommand(smsATCommand, ">", SIM_CHECK) != SIM_OK)
    {
        sim800c_errorHandle();
    }
    if(sim800c_sendCommand(simMessage, NULL, SIM_NOT_CHECK) != SIM_OK)
    {
        sim800c_errorHandle();
    }
    if(sim800c_sendCommand(endSMS, NULL, SIM_NOT_CHECK) != SIM_OK)
    {
        sim800c_errorHandle();
    }
}

void sim800c_errorHandle(void)
{
    sim800c_debug((uint8_t*)"Sim error\r\n", strlen("Sim error\r\n"), 100);
    __disable_irq();
    while(1)
    {

    }
}

void sim800c_initHTTP(void)
{
    sim800c_sendCommand("AT+HTTPINIT\r\n", "OK\r\n", SIM_CHECK);
}

void sim800c_connectServer(char *link)
{
    char urlServer[100] = {0};
    //AT+HTTPPARA=URL,"http://t2techhub.000webhostapp.com/index.php?from=1&to=2"
    sprintf(urlServer, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", link);
    sim800c_debug((uint8_t*)"Connecting Server\r\n", strlen("Connecting Server\r\n"), 100);
    sim800c_sendCommand(urlServer, "OK\r\n", SIM_NOT_CHECK);
}
void sim800c_disconnectServer(char *link)
{
    
}

simState_t sim800c_startOTA(void)
{   
    is_ota = true;
    TIM3->ARR = 4999;
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&simRxData, 1);
    sim800c_debug((uint8_t*)"AT+HTTPACTION=0\r\n", strlen("AT+HTTPACTION=0\r\n"), 100);
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)"AT+HTTPACTION=0\r\n", strlen("AT+HTTPACTION=0\r\n"));
    HAL_TIM_Base_Start_IT(&htim3);
    while(!(simRxOTACplt == SIM_RX_CPLT));
    sscanf(simRxOTABuf, "+HTTPACTION: %d, %d, %d", &httpaction.method, &httpaction.statusCode, &httpaction.length);
    if(httpaction.statusCode == 200)
    {
        is_ota = true;
        TIM3->ARR = 49999;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&simRxData, 1);
        sim800c_debug((uint8_t*)"AT+HTTPREAD\r\n", strlen("AT+HTTPREAD\r\n"), 100);
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)"AT+HTTPREAD\r\n", strlen("AT+HTTPREAD\r\n"));
        while(!(simRxOTACplt == SIM_RX_CPLT));
        sim800c_debug((uint8_t*)simRxOTABuf, strlen(simRxOTABuf), 100);
    }
    else if(httpaction.statusCode == 304)
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)"No new firmware\r\n", strlen("No new firmware\r\n"), 100);
    }
    else
    {
        sim800c_debug((uint8_t*)"Server Error", strlen("Server Error"), 100);
        return SIM_NOT_OK;
    }
    return SIM_OK;
}

void sim800c_GSM_GPRS(void)
{
    while(!(sim800c_sendCommand("AT+CPIN?\r\n", "+CPIN: READY\r\n\r\nOK\r\n", SIM_CHECK) == SIM_OK))
    {
        HAL_Delay(500);
    }
    if(is_sim_on == false)
    {
        while(!(sim800c_sendCommand("AT+SAPBR=1,1\r\n", "OK\r\n", SIM_CHECK) == SIM_OK))
        {
            HAL_Delay(500);
        }
    }

    // sim800c_sendCommand("AT+SAPBR=1,1\r\n", "OK\r\n", SIM_NOT_CHECK);
    HAL_UART_Transmit(&huart1, (uint8_t*)"Setup GSM, GPRS done\r\n", strlen("Setup GSM, GPRS done\r\n"), 100);
}
