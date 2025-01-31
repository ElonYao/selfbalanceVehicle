#include "flightmointor.h"
#include "driverlib.h"
#include "board.h"

void status_send(float roll,float pitch,float yaw)
{
    uint16_t index,counter=0;
    uint16_t sumCheck=0,addCheck=0;
    int32_t temp;
    uint16_t dataBuffer[24];// Based on the data number to send


    dataBuffer[counter++]=0xAB;//Frame header
    dataBuffer[counter++]=0xDD;//S_ADDR IMU
    dataBuffer[counter++]=0xFE;//D_ADDR 
    dataBuffer[counter++]=0x03;// Euler angle

    dataBuffer[counter++]=0;// DLC1 which will be determined after all data has been converted
    dataBuffer[counter++]=0;// DLC0

    temp=(int32_t)(roll*100);
    dataBuffer[counter++]=BYTE0(temp);
    dataBuffer[counter++]=BYTE1(temp);


    temp=(int32_t)(pitch*100);
    dataBuffer[counter++]=BYTE0(temp);
    dataBuffer[counter++]=BYTE1(temp);


    temp=(int32_t)(yaw*100);
    dataBuffer[counter++]=BYTE0(temp);
    dataBuffer[counter++]=BYTE1(temp);


    dataBuffer[counter++]=1;//fusion_sta,no use now

     dataBuffer[4]=counter-6;//DLC1

     //sum check and add check 
     for(index=0;index<counter;index++){
        sumCheck+=(dataBuffer[index]&0x00FF);
        sumCheck&=0x00FF;
        addCheck+=sumCheck;
        addCheck&=0x00FF;
     }
     
    dataBuffer[counter++]=sumCheck;
    dataBuffer[counter++]=addCheck;

    for(index=0;index<counter;index++)
    {
        UART1ByteWrite(dataBuffer[index]);
    }

}
void data_send(MPU6050_T *sensor)
{
    uint16_t index,counter=0;
    uint16_t sumCheck=0,addCheck=0;
    uint16_t dataBuffer[24];// Based on the data number to send


    dataBuffer[counter++]=0xAB;//Frame header
    dataBuffer[counter++]=0xDD;//S_ADDR IMU
    dataBuffer[counter++]=0xFE;//D_ADDR 
    dataBuffer[counter++]=0x01;// imu raw data

    dataBuffer[counter++]=0;// DLC1 which will be determined after all data has been converted
    dataBuffer[counter++]=0;// DLC0

    // accelerometer data
    dataBuffer[counter++]=BYTE0(sensor->ax);
    dataBuffer[counter++]=BYTE1(sensor->ax);
    dataBuffer[counter++]=BYTE0(sensor->ay);
    dataBuffer[counter++]=BYTE1(sensor->ay);
    dataBuffer[counter++]=BYTE0(sensor->az);
    dataBuffer[counter++]=BYTE1(sensor->az);

    // gyro data
    dataBuffer[counter++]=BYTE0(sensor->gx);
    dataBuffer[counter++]=BYTE1(sensor->gx);
    dataBuffer[counter++]=BYTE0(sensor->gy);
    dataBuffer[counter++]=BYTE1(sensor->gy);
    dataBuffer[counter++]=BYTE0(sensor->gz);
    dataBuffer[counter++]=BYTE1(sensor->gz);
    dataBuffer[counter++]=1;//SHOCK_STA 

    dataBuffer[4]=counter-6;//DLC1

     //sum check and add check 
    for(index=0;index<counter;index++){
        sumCheck+=(dataBuffer[index]&0x00FF);
        sumCheck&=0x00FF;
        addCheck+=sumCheck;
        addCheck&=0x00FF;
     }
     
    dataBuffer[counter++]=sumCheck;
    dataBuffer[counter++]=addCheck;

    for(index=0;index<counter;index++)
    {
        UART1ByteWrite(dataBuffer[index]);
    }

}
void data_print(float var1,float var2)
{

    uint16_t index,counter=0;
    int32_t temp;
    uint16_t dataBuffer[15];// Based on the data number to send


    temp=(int32_t)(var1*1000);
    dataBuffer[counter++]=0x30+temp*0.001f;
    dataBuffer[counter++]=0x30+(temp%1000)*0.01f;
    dataBuffer[counter++]=0x30+(temp%100)*0.1f;
    dataBuffer[counter++]=0x30+temp%10;
    //add a space
    dataBuffer[counter++]=0x20;


    temp=(int32_t)(var2*1000);
    dataBuffer[counter++]=0x30+temp*0.001f;
    dataBuffer[counter++]=0x30+(temp%1000)*0.01f;
    dataBuffer[counter++]=0x30+(temp%100)*0.1f;
    dataBuffer[counter++]=0x30+temp%10;

    /*
    //add a space
    dataBuffer[counter++]=0x20;

    temp=(int32_t)(var3*1000);
    dataBuffer[counter++]=0x30+temp*0.001f;
    dataBuffer[counter++]=0x30+(temp%1000)*0.01f;
    dataBuffer[counter++]=0x30+(temp%100)*0.1f;
    dataBuffer[counter++]=0x30+temp%10;
    //add a space
    dataBuffer[counter++]=0x20;

    temp=(int32_t)(var4*1000);
    dataBuffer[counter++]=0x30+temp*0.001f;
    dataBuffer[counter++]=0x30+(temp%1000)*0.01f;
    dataBuffer[counter++]=0x30+(temp%100)*0.1f;
    dataBuffer[counter++]=0x30+temp%10;
     */

    //add a newline and return
    dataBuffer[counter++]=0x0A;
    dataBuffer[counter++]=0x0D;
    //dataBuffer[counter++]='\0';


    //send buffer out
    for(index=0;index<counter;index++)
    {
        UART1ByteWrite(dataBuffer[index]);
    }
}
//interface APIs
void UART1ByteWrite( uint16_t txData)
{    
    SCI_writeCharBlockingFIFO(Data_output_BASE,txData);
}

static uint16_t BYTE0(int32_t dwTemp) {
    int32_t temp= dwTemp&0xFF;
    return (*(uint16_t*)(&(temp)));
    }
static uint16_t BYTE1(int32_t dwTemp){
    int32_t temp=(dwTemp>>8)& 0xFF;
    return (*(uint16_t*)(&(temp)));
}
static uint16_t BYTE2(int32_t dwTemp){
    int32_t temp= (dwTemp>>16)& 0xFF;
    return (*(uint16_t*)(&(temp)));
}
static uint16_t BYTE3(int32_t dwTemp){
    int32_t temp= (dwTemp>>24)& 0xFF;
    return (*(uint16_t*)(&(temp)));
}
