#ifndef FLIGHT_MINITOR_H
#define FLIGHT_MINITOR_H
#include "IMU6050_Elon.h"


void status_send(float roll,float pitch,float yaw);
void data_send(MPU6050_T *sensor);
void data_print(float var1,float var2);

void UART1ByteWrite( uint16_t txData);
static uint16_t BYTE0(int32_t dwTemp);
static uint16_t BYTE1(int32_t dwTemp);
static uint16_t BYTE2(int32_t dwTemp);
static uint16_t BYTE3(int32_t dwTemp);




#endif

