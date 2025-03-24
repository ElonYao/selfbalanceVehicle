#include "comm_Elon.h"

#ifdef _FLASH
#pragma CODE_SECTION(INT_mainCAN_ISR, ".TI.ramfunc");
#endif

canHandle canInit(void *memory,const size_t memorySize)
{
    canHandle handle;
    can_t *obj;
    if(memorySize < sizeof(can_t))
    {
        return (canHandle)NULL;
    }
    handle = (canHandle)memory;
    obj = (can_t *)handle;
    obj->flagError=0;
    obj->flagRxDone=0;
    obj->txMsgCount=0;
    obj->rxMsgCount=0;
    obj->timeBaseCounter1=0;
    obj->timeBaseCounter2=0;
    return handle;
}
uint16_t messageValidation(int16_t *buff,uint16_t messageCounter,uint32_t messageID)
{
    uint16_t index,checkSum=0;
    for(index=0;index<7;index++)
    {
        checkSum+=buff[index];
    }
    checkSum+=(messageID & 0x0F)+(messageID>>8 & 0x0F)+(messageID>>16 & 0x0F)+(messageID>>24 & 0x0F)+(messageCounter & 0x0F);
    checkSum = ((checkSum >> 4) + checkSum) & 0x0F;

    return checkSum;
}
void updateCAN(canHandle CANhandler,IMUHandle IMUhandler,vehicleHandle vehicleHandler)
{
    can_t *canObj= (can_t *)CANhandler;
    MPU6050_T *IMUObj= (MPU6050_T *)IMUhandler;
    vehicle_t *vehicleObj=(vehicle_t*)vehicleHandler;

    float32_t temp;
    uint16_t index,checkSum;
    if(canObj->flagRxDone)
    {
        //validate checkSum
        checkSum=(canObj->rxBuffer[7]>>4)&0x0F;
        if(checkSum==messageValidation(canObj->rxBuffer,(canObj->rxBuffer[7]&0x0F),TSC_ID))
        {
            canObj->targetSpeed=(((uint16_t)canObj->rxBuffer[1]<<8)|((uint16_t)canObj->rxBuffer[0]))*0.09155413f-3000;
            if(0x01==(canObj->rxBuffer[2]&0x0C)>>2)
            {
                canObj->targetTurn=1;
            }
            else if(0x10==(canObj->rxBuffer[2]&0x0C)>>2)
            {
                canObj->targetTurn=-1;
            }
            else if(0x00==(canObj->rxBuffer[2]&0x0C)>>2)
            {
                canObj->targetTurn=0;
            }

        }
        canObj->flagRxDone=0;
    }
    //packaging Euler angle and vehicle status(~50ms transmission rate)
    if(canObj->timeBaseCounter1>=10)
    {
        //roll
        temp=(IMUObj->orientation.roll*MATH_R2D+90)*364.0865069f;
        canObj->txBuffer[0]=((int32_t)temp) & 0xFF;
        canObj->txBuffer[1]=((int32_t)temp>>8) & 0xFF;
        //pitch
        temp=(IMUObj->orientation.pitch*MATH_R2D+90)*364.0865069f;
        canObj->txBuffer[2]=(int32_t)temp & 0xFF;
        canObj->txBuffer[3]=((int32_t)temp>>8) & 0xFF;
        //yaw
        temp=(IMUObj->orientation.yaw*MATH_R2D+90)*364.0865069f;
        canObj->txBuffer[4]=(int32_t)temp & 0xFF;
        canObj->txBuffer[5]=((int32_t)temp>>8) & 0xFF;
        //sum check and add check
        canObj->txBuffer[6]=0;
        canObj->txBuffer[7]=0;
        for(index=0;index<6;index++)
        {
           canObj->txBuffer[6]+=(canObj->txBuffer[index]&0x00FF);
           canObj->txBuffer[6]&=0x00FF;
           canObj->txBuffer[7]+=canObj->txBuffer[6];
           canObj->txBuffer[7]&=0x00FF;
        }
        CAN_sendMessage(mainCAN_BASE, 2, 8, (uint16_t *)canObj->txBuffer);
        canObj->timeBaseCounter1=0;
    }
    //Packaging vehicle status(~100ms transmission rate)
    if(canObj->timeBaseCounter2>=20)
    {
        //left wheel speed
        temp=(vehicleObj->vehicleDirection*vehicleObj->speedMSLeft*1000+3000)*10.9225f;
        canObj->txBuffer[0]=((int32_t)temp) & 0xFF;
        canObj->txBuffer[1]=((int32_t)temp>>8) & 0xFF;
        //right wheel speed
        temp=(vehicleObj->vehicleDirection*vehicleObj->speedMSRight*1000+3000)*10.9225f;
        canObj->txBuffer[2]=((int32_t)temp) & 0xFF;
        canObj->txBuffer[3]=((int32_t)temp>>8) & 0xFF;
        //vehicle speed
        temp=(vehicleObj->vehicleDirection*vehicleObj->speedMS*1000+3000)*10.9225f;
        canObj->txBuffer[4]=((int32_t)temp) & 0xFF;
        canObj->txBuffer[5]=((int32_t)temp>>8) & 0xFF;

        //Battery voltage
        temp=vehicleObj->batteryVolt*68.2666667f;
        canObj->txBuffer[6]=((int32_t)temp) & 0xFF;
        canObj->txBuffer[7]=((int32_t)temp>>8) & 0x03;

        //turning
        if(vehicleObj->flag_turning==-1)
        {
            canObj->txBuffer[7]|=0x08;//left
        }
        else if(vehicleObj->flag_turning==1)
        {
            canObj->txBuffer[7]|=0x04;//right
        }

        //vehicle status
        canObj->txBuffer[7]|=((int16_t)(vehicleObj->status)<<4);
        //send out
        CAN_sendMessage(mainCAN_BASE, 3, 8, (uint16_t *)canObj->txBuffer);

        canObj->timeBaseCounter2=0;
    }

}
cmdHandle cmdInit(void *memory,const size_t memorySize)
{
    cmdHandle handle;
    serialCMD *obj;
    if(memorySize < sizeof(serialCMD))
    {
        return (cmdHandle)NULL;
    }
    handle = (cmdHandle)memory;
    obj = (serialCMD *)handle;
    obj->flagNewcmd=0;
    obj->cmdValue=0.0f;
    memset(obj->cmdPara,0,sizeof(obj->cmdPara));
    memset(obj->rawCMD,0,sizeof(obj->rawCMD));
    memset(obj->cmdName,0,sizeof(obj->cmdName));
    return handle;
}

uint16_t usartChecksum(uint16_t *input)
{
    uint16_t checksum=0,index=0;
    //only confirm the first 6 elements
    for(index=0;index<6;index++)
    {
        checksum^=input[index];
    }
    checksum &=0x00FF;
    return checksum;
}

void cmdParse(cmdHandle handle)
{
    serialCMD *obj= (serialCMD *)handle;
    uint16_t dataIndex,n;
    if(obj->flagNewcmd==1 && obj->rawCMD[6]==usartChecksum(obj->rawCMD))
    {
        for(dataIndex=0;obj->rawCMD[dataIndex]!=' ';dataIndex++)
        {
          if(isalpha(obj->rawCMD[dataIndex]))
          {
              obj->cmdName[dataIndex]=toupper(obj->rawCMD[dataIndex]);
          }
          else
          {
              obj->flagNewcmd=0;
              return;
          }
        }
        //byte 3~6
        for(dataIndex=dataIndex+1,n=0;dataIndex<6;dataIndex++,n++)
        {
          if(isdigit(obj->rawCMD[dataIndex]))
          {
            obj->cmdPara[n]=obj->rawCMD[dataIndex];
          }
          else
          {
            obj->flagNewcmd=0;
            return;
          }
        }
        obj->cmdValue=atof(obj->cmdPara);
        memset(obj->cmdPara,0,sizeof(obj->cmdPara));
    }
}
void comDispatch(cmdHandle handle,vehicleHandle vehicleHandler)
{
    serialCMD *obj= (serialCMD *)handle;
    vehicle_t * vehicleObj = (vehicle_t *) vehicleHandler;
        if(obj->flagNewcmd)
        {
            switch(obj->cmdName[0])
            {
                case 'F'://Forward
                    vehicleObj->targetSpeed=-(obj->cmdValue<=500? obj->cmdValue:500);//mm/s
                    break;
                case 'B': //Backward
                    vehicleObj->targetSpeed=(obj->cmdValue<=500? obj->cmdValue:500);
                    break;
                case 'L':// Left turn
                    //vehicleObj->flag_turning=-1;
                    vehicleObj->targetYawRate=-35.0f;
                    break;
                case 'R': // Right turn
                    //vehicleObj->flag_turning=1;
                    vehicleObj->targetYawRate=35.0f;
                    break;
                case 'C': //clear setting
                    vehicleObj->targetSpeed=0.0f;
                    vehicleObj->flag_turning=0;
                    vehicleObj->targetYawRate=0;
                    break;
                default:
                    break;
            }
            obj->flagNewcmd=0;
        }
}





