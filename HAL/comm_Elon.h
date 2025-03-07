#ifndef _COMM_ELON_H
#define _COMM_ELON_H

#ifdef __cplusplus
extern "C" {
#endif
#include "driverlib.h"
#include "board.h"
#include "IMU6050_Elon.h"
#include "hal.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#define TSC_ID 0x0C00FEFD
typedef struct _canCom_
{
    uint16_t flagError;
    uint16_t flagRxDone;
    uint16_t rxMsgCount;
    uint16_t txMsgCount;
    int16_t txBuffer[8];
    int16_t rxBuffer[8];
    uint16_t timeBaseCounter1;
    uint16_t timeBaseCounter2;
    float32_t targetSpeed;
    int16_t targetTurn;
}can_t;

typedef struct _cmdSerial_
{
    uint16_t rawCMD[8];
    uint16_t cmdName[3];
    uint16_t cmdPara[5];
    float32_t cmdValue;
    uint16_t flagNewcmd;

}serialCMD;

typedef struct _canCom_ *canHandle;
typedef struct _cmdSerial_ *cmdHandle;

//CAN communication APIs
canHandle canInit(void *memory,const size_t memorySize);
uint16_t messageValidation(int16_t *buff,uint16_t messageCounter,uint32_t messageID);
void updateCAN(canHandle CANhandler,IMUHandle IMUhandler,vehicleHandle vehicleHandler);

//UsartB communication for HC-05 and PIC18F45K22
cmdHandle cmdInit(void *memory,const size_t memorySize);
void cmdParse(cmdHandle handle);
void comDispatch(cmdHandle handle,vehicleHandle vehicleHandler);
uint16_t usartChecksum(uint16_t *input);

extern __interrupt void INT_mainCAN_ISR(void);




#ifdef __cplusplus
}
#endif

#endif
