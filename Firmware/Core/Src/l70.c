#include "l70.h"

l70GPGGAType_t l70GPGGA = {0};
int k = 0;
char l70RxData[100] = {0};
char l70Data = 0;
l70Data_t l70RxState;

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
  
    while (i < d)
        str[i++] = '0';
  
    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(double n, char* res, int afterpoint)
{
    int ipart = (int)n;
    double fpart = n - (double)ipart;
    int i = intToStr(ipart, res, 0);
    if (afterpoint != 0) 
    {
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}


void l70_callback(void)
{
    if(l70Data == '$' && l70RxState == L70_START_DATA)
    {
        l70RxData[k++] = l70Data;
        l70RxState = L70_DATA;
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&l70Data, 1);
    }
    else if(l70Data == '\n' && l70RxState == L70_DATA)
    {
        l70RxData[k++] = l70Data;
        l70RxState = L70_END_DATA;
    }
    else if(l70RxState == L70_DATA)
    {
        l70RxData[k++] = l70Data;
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&l70Data, 1);
    }
    else 
    {
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&l70Data, 1);
    }
}

char *l70_receiveGPS(void)
{
    k = 0;
    memset(l70RxData, '\0', sizeof(l70RxData));
    l70RxState = L70_START_DATA;
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&l70Data, 1);
    while(!(l70RxState == L70_END_DATA));
    return l70RxData;
}

void l70_init(void)
{
    HAL_UART_Transmit_IT(&huart3 ,(uint8_t*)"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n", strlen("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"));
    // HAL_Delay(100);
    // HAL_UART_Transmit_IT(&huart3, (uint8_t*)"$PMTK285,2,100*3E\r\n", strlen("$PMTK285,2,100*3E\r\n"));
}

void l70_handleGPS(char *latData, char *longData, char *GPSResponse)
{
    int temp = 0;
    double decPart = 0;
    double intPart = 0;
    double l70latitude = 0;
    double l70longitude = 0;
    GPSResponse = strstr(GPSResponse, "$GPGGA");
    sscanf(GPSResponse, "$GPGGA,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]",\
     l70GPGGA.utcTime, l70GPGGA.latitude, &l70GPGGA.NorS, l70GPGGA.longitude,\
     &l70GPGGA.EorW, &l70GPGGA.fixStatus, l70GPGGA.numberOfSatellites);
    l70latitude = atof(l70GPGGA.latitude);
    temp = (int)l70latitude;
    intPart = (double)(temp / 100);
    decPart = (double)(temp % 100) + l70latitude - (double)temp;
    l70latitude = intPart + decPart / 60;

    l70longitude = atof(l70GPGGA.longitude);
    temp = (int)l70longitude;
    intPart = (double)(temp / 100);
    decPart = (double)(temp % 100) + l70longitude - (double)temp;
    l70longitude = intPart + decPart / 60;
    ftoa(l70latitude, latData, 6);
    ftoa(l70longitude, longData, 6);
}

