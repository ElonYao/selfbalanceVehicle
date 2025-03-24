#ifndef IMU6050_ELON_H
#define IMU6050_ELON_H
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "driverlib.h"
#include "board.h"

#define IMUADDR 0x68 // MPU6050 I2C address
#define MATH_ACCSCALEFACTOR 5.9814e-4f //16384/g (2g range)
#define MATH_GYROSCALEFACTOR 5.32640e-4f // (1000 deg/s) change to rad
#define MATH_TEMPC 2.9411764e-3f
#define MATH_R2D 57.29577951f
#define ZERO_OFFSET 1.2f // the mechanical balance point IMU reading
#ifdef Calibrate_Enable

    #define CALBUFSIZE 1000U //calibration buffer size
    #define GYROERROR 1U
    #define ACCELERROR 8U

#endif

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}euler_t;

typedef struct _imu_
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
    #if defined(Calibrate_Enable)

	    int16_t averageAX;
	    int16_t averageAY;
	    int16_t averageAZ;
	    int16_t averageGX;
	    int16_t averageGY;
	    int16_t averageGZ;
    #endif
    float AX;
    float AY;
    float AZ;
    float GX;
    float GY;
    float GZ;
    int16_t offsetAX;
    int16_t offsetAY;
    int16_t offsetAZ;
    int16_t offsetGX;
    int16_t offsetGY;
    int16_t offsetGZ;
    int16_t temperature;
    //DMA buffer to store the raw value from MPU6050
    uint16_t dataBuffer[14];
    euler_t orientation;
    float CMfilter_alpha;
    //Flags
    volatile uint16_t flag_newRaw;
    volatile uint16_t flag_dataReady;
    uint16_t flag_calibrattionDone;
    float32_t rollOffset;

}MPU6050_T;

typedef struct  _imu_ *IMUHandle;

IMUHandle MPU6050init(void *memory,const size_t memorySize);

void IMURead(uint16_t addrSlave,uint16_t addrRegs,uint16_t dataLen ,uint16_t *dataBuffer);
void IMUWrite(uint16_t addrSlave,uint16_t addrRegs,uint16_t dataLen ,uint16_t *dataBuffer);
void deviceReset(void);
void checkAttendence(void);

void MPU_dataProcessing(IMUHandle handle);
void getGAdata_floating(IMUHandle handle);
void complemenaryEuler(IMUHandle handle);
void setOffset(IMUHandle handle);

//calibration related functions

#ifdef Calibrate_Enable

    void getGAdata(IMUHandle handle);
    void averageRead(IMUHandle handle);
    void averageRead_offset(IMUHandle handle);
    void IMUcalibration(IMUHandle handle);
    void calibration(IMUHandle handle);

#endif

#endif


