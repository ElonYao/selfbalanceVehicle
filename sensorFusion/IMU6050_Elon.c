#include "IMU6050_Elon.h"

void IMURead(uint16_t addrSlave,uint16_t addrRegs,uint16_t dataLen ,uint16_t *dataBuffer)
{
        uint16_t index=0;
        while (I2C_getStopConditionStatus(IMU_6050_BASE));
        I2C_setSlaveAddress(IMU_6050_BASE,addrSlave);

        I2C_setConfig(IMU_6050_BASE, (I2C_MASTER_SEND_MODE|I2C_REPEAT_MODE));
        I2C_sendStartCondition(IMU_6050_BASE);

        while(!(I2C_getStatus(IMU_6050_BASE) & I2C_STS_REG_ACCESS_RDY));
        I2C_putData(I2CA_BASE, addrRegs);

       // while(!(I2C_getStatus(IMU_6050_BASE) & I2C_STS_TX_DATA_RDY));

        I2C_sendStopCondition(IMU_6050_BASE);

        while (I2C_getStopConditionStatus(IMU_6050_BASE));


        I2C_setDataCount(IMU_6050_BASE, dataLen);
        I2C_setConfig(IMU_6050_BASE, (I2C_MASTER_RECEIVE_MODE));
        I2C_sendStartCondition(IMU_6050_BASE);

       while((I2C_getStatus(IMU_6050_BASE) & I2C_STS_BUS_BUSY));
        for (index = 0; index < dataLen; index++)
        {
            while(!(I2C_getStatus(IMU_6050_BASE) & I2C_STS_RX_DATA_RDY));
            dataBuffer[index] = I2C_getData(IMU_6050_BASE);
        }
       // counter=0;
        I2C_sendStopCondition(IMU_6050_BASE);//stop
}

void IMUWrite(uint16_t addrSlave,uint16_t addrRegs,uint16_t dataLen ,uint16_t *dataBuffer)
{
        while(I2C_getStopConditionStatus(IMU_6050_BASE));
        while(I2C_isBusBusy(IMU_6050_BASE));

        I2C_setTargetAddress(IMU_6050_BASE,addrSlave);

        I2C_setConfig(IMU_6050_BASE,I2C_CONTROLLER_SEND_MODE|I2C_REPEAT_MODE);

        I2C_sendStartCondition(IMU_6050_BASE);



        while(!(I2C_getStatus(IMU_6050_BASE) & I2C_STS_REG_ACCESS_RDY));

        while(!(I2C_getStatus(IMU_6050_BASE) & I2C_STS_TX_DATA_RDY));
        I2C_putData(IMU_6050_BASE,addrRegs);


        while(dataLen--)
            {
            while(!(I2C_getStatus(IMU_6050_BASE) & I2C_STS_TX_DATA_RDY));
                I2C_putData(IMU_6050_BASE,*dataBuffer++);

            }
        I2C_sendStopCondition(IMU_6050_BASE);//stop
}

IMUHandle MPU6050init(void *memory,const size_t memorySize)
{
    IMUHandle handle;
    MPU6050_T *obj;
    uint16_t temp=0;

    if(memorySize < sizeof(MPU6050_T))
    {
        return (IMUHandle)NULL;
    }

    handle = (IMUHandle)memory;
    obj = (MPU6050_T *)handle;

	obj->flag_newRaw=0;
	obj->flag_dataReady=0;
	obj->orientation.pitch=0.0f;
	obj->orientation.roll=0.0f;
	obj->orientation.yaw=0.0f;
	obj->CMfilter_alpha=0.02f;
	obj->rollOffset=-2.9f;
	//obj->dataBuffer={0};
    deviceReset();
    temp=0x01;//sleep off and use x gyro as clock source
    IMUWrite(IMUADDR,0x6B,1,&temp);
    temp=0x02;//Enable low pass filter on chip BW acc 94Hz gyro 98Hz
    IMUWrite(IMUADDR,0x1A,1,&temp);
    temp=0x10;//set gyro config No self test, +/-1000deg/s
    IMUWrite(IMUADDR,0x1B,1,&temp);
    temp=0x00;//set acc range  No self test, 2g
    IMUWrite(IMUADDR,0x1C,1,&temp);
    temp=0x10 ;//INT_PIN_CFG
    IMUWrite(IMUADDR,0x37,1,&temp);
    /*
    temp=0x01;//Interrupt enable
    IMUWrite(IMUADDR,0x38,1,&temp);
    */
    return handle;
}
void checkAttendence(void)
{
	uint16_t id=0;
	unsigned char *msg;
	IMURead(IMUADDR,0x75,1U,&id);
	id>>=1;
	if(id==0x34){
	    msg = "\r\nMPU6050 connected!\0";
	    SCI_writeCharArray(Data_output_BASE, (uint16_t*)msg, 21);
	}
	else{
	    msg = "\r\nMPU6050 connection failed!\0";
	    SCI_writeCharArray(Data_output_BASE, (uint16_t*)msg, 29);
	    ESTOP0;
	}
	return;
}

void deviceReset(void)
{
    uint16_t temp=0;
    temp=0x80;// device reset
    IMUWrite(IMUADDR,0x6B,1,&temp);
    DEVICE_DELAY_US(100000);
    temp=0x01;// device reset
    IMUWrite(IMUADDR,0x6B,1,&temp);
    DEVICE_DELAY_US(100000);
}
void MPU_dataProcessing(IMUHandle handle)
{
        MPU6050_T *obj=( MPU6050_T *) handle;
        obj->ax=((obj->dataBuffer[0]<<8)| obj->dataBuffer[1])+obj->offsetAX;
        obj->ay=((obj->dataBuffer[2]<<8)| obj->dataBuffer[3])+obj->offsetAY;
        obj->az=((obj->dataBuffer[4]<<8)| obj->dataBuffer[5])+obj->offsetAZ;
        obj->temperature=((obj->dataBuffer[6]<<8)| obj->dataBuffer[7]);
        obj->gx=((obj->dataBuffer[8]<<8)| obj->dataBuffer[9])+obj->offsetGX;
        obj->gy=((obj->dataBuffer[10]<<8)| obj->dataBuffer[11])+obj->offsetGY;
        obj->gz=((obj->dataBuffer[12]<<8)| obj->dataBuffer[13])+obj->offsetGZ;

        //re-maping axis with the actual board
        obj->AX=obj->ax*MATH_ACCSCALEFACTOR;
        obj->AY=-(obj->ay*MATH_ACCSCALEFACTOR);
        obj->AZ=-(obj->az*MATH_ACCSCALEFACTOR);
        obj->GX=obj->gx*MATH_GYROSCALEFACTOR;
        obj->GY=-(obj->gy*MATH_GYROSCALEFACTOR);
        obj->GZ=-(obj->gz*MATH_GYROSCALEFACTOR);
}
void getGAdata_floating(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;

    IMURead(IMUADDR,0x3B,14,obj->dataBuffer);
	obj->ax=((obj->dataBuffer[0]<<8)| obj->dataBuffer[1])+obj->offsetAX;
	obj->ay=((obj->dataBuffer[2]<<8)| obj->dataBuffer[3])+obj->offsetAY;
	obj->az=((obj->dataBuffer[4]<<8)| obj->dataBuffer[5])+obj->offsetAZ;
    obj->temperature=((obj->dataBuffer[6]<<8)| obj->dataBuffer[7]);
    obj->gx=((obj->dataBuffer[8]<<8)| obj->dataBuffer[9])+obj->offsetGX;
    obj->gy=((obj->dataBuffer[10]<<8)| obj->dataBuffer[11])+obj->offsetGY;
    obj->gz=((obj->dataBuffer[12]<<8)| obj->dataBuffer[13])+obj->offsetGZ;
    //re-maping axis with the actual board
    obj->AX=obj->ax*MATH_ACCSCALEFACTOR;
    obj->AY=-(obj->ay*MATH_ACCSCALEFACTOR);
    obj->AZ=-(obj->az*MATH_ACCSCALEFACTOR);
    obj->GX=obj->gx*MATH_GYROSCALEFACTOR;
    obj->GY=-(obj->gy*MATH_GYROSCALEFACTOR);
    obj->GZ=-(obj->gz*MATH_GYROSCALEFACTOR);
}
#ifdef Calibrate_Enable

void getGAdata(IMUHandle handle)
{
        MPU6050_T *obj=( MPU6050_T *) handle;

        IMURead(IMUADDR,0x3B,14,obj->dataBuffer);
        obj->ax=((obj->dataBuffer[0]<<8)| obj->dataBuffer[1]);
        obj->ay=((obj->dataBuffer[2]<<8)| obj->dataBuffer[3]);
        obj->az=((obj->dataBuffer[4]<<8)| obj->dataBuffer[5]);
        obj->temperature=((obj->dataBuffer[6]<<8)| obj->dataBuffer[7]);
        obj->gx=((obj->dataBuffer[8]<<8)| obj->dataBuffer[9]);
        obj->gy=((obj->dataBuffer[10]<<8)| obj->dataBuffer[11]);
        obj->gz=((obj->dataBuffer[12]<<8)| obj->dataBuffer[13]);

}


void averageRead(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;

    uint16_t index=0;
    int32_t sumAX=0,sumAY=0,sumAZ=0,sumGX=0,sumGY=0,sumGZ=0;
    while(index<(CALBUFSIZE+200))
    {
         getGAdata(handle);
         //Discard the first 200 readings
         if(index>199)
         {
            sumAX+=obj->ax;
            sumAY+=obj->ay;
            sumAZ+=obj->az;
            sumGX+=obj->gx;
            sumGY+=obj->gy;
            sumGZ+=obj->gz;
         }
         if(index==CALBUFSIZE+199)
         {
            obj->averageAX=sumAX/CALBUFSIZE;
            obj->averageAY=sumAY/CALBUFSIZE;
            obj->averageAZ=sumAZ/CALBUFSIZE;
            obj->averageGX=sumGX/CALBUFSIZE;
            obj->averageGY=sumGY/CALBUFSIZE;
            obj->averageGZ=sumGZ/CALBUFSIZE;
         }
         index++;
         DEVICE_DELAY_US(1000);// delay 1ms to wait for data update 40/4e6
    }
}
void averageRead_offset(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;

    uint16_t index=0;
    int32_t sumAX=0,sumAY=0,sumAZ=0,sumGX=0,sumGY=0,sumGZ=0;
    while(index<(CALBUFSIZE+200))
    {
        getGAdata_floating(handle);
         if(index>199)
         {
            sumAX+=obj->ax;
            sumAY+=obj->ay;
            sumAZ+=obj->az;
            sumGX+=obj->gx;
            sumGY+=obj->gy;
            sumGZ+=obj->gz;
         }
         if(index==CALBUFSIZE+199)
         {
            obj->averageAX=sumAX/CALBUFSIZE;
            obj->averageAY=sumAY/CALBUFSIZE;
            obj->averageAZ=sumAZ/CALBUFSIZE;
            obj->averageGX=sumGX/CALBUFSIZE;
            obj->averageGY=sumGY/CALBUFSIZE;
            obj->averageGZ=sumGZ/CALBUFSIZE;
         }
         index++;
         DEVICE_DELAY_US(1000);// delay 1ms to wait for data update
    }
}

void IMUcalibration(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;

    uint16_t done;
    //set initial offset to zero 
    obj->offsetAX=-obj->averageAX/8;
    obj->offsetAY=-obj->averageAY/8;
    obj->offsetAZ=(16384-obj->averageAZ)/8;//gravity effect
    obj->offsetGX=-obj->averageGX/4;
    obj->offsetGY=-obj->averageGY/4;
    obj->offsetGZ=-obj->averageGZ/4;

    for(;;)
    {
        done=0;
        averageRead_offset(handle);

        if(abs(obj->averageAX)<=ACCELERROR) done++;
        else obj->offsetAX-=obj->averageAX/ACCELERROR;

        if(abs(obj->averageAY)<=ACCELERROR) done++;
        else obj->offsetAY-=obj->averageAY/ACCELERROR;

        if(abs(16384-obj->averageAZ)<=ACCELERROR) done++;
        else obj->offsetAZ+=(16384-obj->averageAZ)/ACCELERROR;

        if(abs(obj->averageGX)<=GYROERROR) done++;
        else obj->offsetGX-=obj->averageGX/(GYROERROR+1);

         if(abs(obj->averageGY)<=GYROERROR) done++;
        else obj->offsetGY-=obj->averageGY/(GYROERROR+1);

        if(abs(obj->averageGZ)<=GYROERROR) done++;
        else obj->offsetGZ-=obj->averageGZ/(GYROERROR+1);
        if(6==done) break;
    }

}
void calibration(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;

    uint16_t step=0;
    unsigned char *msg;

    if(0==step)
    {
        msg="\r\nInitial Gyro and accelerometer reading...\0";
        SCI_writeCharArray(Data_output_BASE, (uint16_t*)msg, 44);
        averageRead(handle);
        DEVICE_DELAY_US(5000000);//delay 500ms
        step++;
    }
    if(1==step)
    {
        msg="\r\nCalculate offset values\0";
        SCI_writeCharArray(Data_output_BASE, (uint16_t*)msg, 26);

        IMUcalibration(handle);
        DEVICE_DELAY_US(5000000);//delay 500ms
        step++;
    }

    if(2==step)
    {
        averageRead_offset(handle);
        msg="\r\nComplete!\0";
        SCI_writeCharArray(Data_output_BASE, (uint16_t*)msg, 12);
        obj->flag_calibrattionDone=1;
        //ESTOP0;
    }
}

#endif

void setOffset(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;
    obj->offsetAX=-84;
    obj->offsetAY= 418;
    obj->offsetAZ= 397;
    obj->offsetGX=128;
    obj->offsetGY=-153;
    obj->offsetGZ=122;
}

void complemenaryEuler(IMUHandle handle)
{
    MPU6050_T *obj=( MPU6050_T *) handle;
    float rollEstimate_dot,pitchEstimate_dot,yawEstimate_dot,rollEstimate,pitchEstimate,yawEstimate;
    //initial value
    rollEstimate=obj->orientation.roll;
    pitchEstimate=obj->orientation.pitch;
    yawEstimate=obj->orientation.yaw;

    //rate of change
    rollEstimate_dot=obj->GX+obj->GY*sinf(rollEstimate)*tanf(pitchEstimate)+obj->GZ*cosf(rollEstimate)*tanf(pitchEstimate);
    pitchEstimate_dot=obj->GY*cosf(rollEstimate)-obj->GZ*sinf(rollEstimate);
    yawEstimate_dot=(obj->GY*sinf(rollEstimate)+obj->GZ*cosf(rollEstimate))/cosf(pitchEstimate);
    //combination
    rollEstimate=atanf(obj->AY/obj->AZ)*obj->CMfilter_alpha+(1-obj->CMfilter_alpha)*(rollEstimate+rollEstimate_dot*0.005f);
    pitchEstimate=asinf(obj->AX/9.8f)*obj->CMfilter_alpha+(1-obj->CMfilter_alpha)*(pitchEstimate+pitchEstimate_dot*0.005f);
    yawEstimate+=yawEstimate_dot*0.005f;
    obj->orientation.roll=rollEstimate;
    obj->orientation.pitch=pitchEstimate;
    obj->orientation.yaw=yawEstimate;

}

