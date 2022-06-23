#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

#define FLASH_SIZE 65535

int convertHex(char *data)
{
    int sum = 0;
    int haftbyte;
    int leng = strlen(data);
    for(int k = 0; k < strlen(data); k++)
    {
        haftbyte = toupper(data[k]) - '0';
        if(haftbyte > 9)
        {
            haftbyte = haftbyte - 7;
        }
        sum += haftbyte * (int)pow(16, strlen(data) - k - 1);
    }
    return sum;
}

int main(void)
{
    FILE *file;
    char line[200] = {0};
    char data[FLASH_SIZE] = {0};
    char byte_count_arr[2] = {0};
    char address_arr[4] = {0};
    char record_arr[2] = {0};
    int sum_data = 0;
    int byte_count = 0, address = 0, record_type = 0;
    char rc_arr[2] = "00";
    int length = 0;
    file = fopen("firmware.txt", "r");
    if(file == NULL)
    {
        return 0;
    }
    else
    {
        while(fgets(line, 200, file) != NULL)
        {
            char byte_count_arr[2];
            strncpy(byte_count_arr, line + 1, 2);
            byte_count_arr[2] = '\0';
            byte_count = convertHex(byte_count_arr);

            char address_arr[4];
            strncpy(address_arr, line + 3, 4);
            address_arr[4] = '\0';
            address = convertHex(address_arr);

            char record_arr[2];
            strncpy(record_arr, line + 7, 2);
            record_arr[2] = '\0';
            record_type = convertHex(record_arr);
            if(record_type == 0)
            {
                for(int index = sum_data; index < byte_count + sum_data; index++)
                {
                    data[2 * index] = line[9 + 2 * (index - sum_data)];
                    data[(2 * index) + 1] = line[9 + 2 *(index - sum_data) + 1];
                }
                sum_data += byte_count;
            }
            memset(line, '\0', sizeof(line));
        }
    }
    fclose(file);
    length = strlen(data);
    file = fopen("ProgramLoad.txt", "w");
    for(int i = 0; i < length; i++)
    {
        if((i % 1024 == 0) && (i > 0))
        {
            fputc('\n', file);
        }
        fputc((int)data[i], file);
    }
    fclose(file);
    return 0;
}