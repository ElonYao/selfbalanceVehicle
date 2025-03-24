//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
// TITLE:  Empty Project
//
// Empty Project Example
//
// This example is an empty project setup for Driverlib development.
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2013-2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
#include "hal.h"
#include "flightmointor.h"
#include "IMU6050_Elon.h"
#include "PID_Elon.h"
//#include "PID_ElonM.h"
#include "ramp_Elon.h"
//#include "DLOG_4CH_F.h"
#include "comm_Elon.h"

#ifdef _FLASH
#pragma CODE_SECTION(INT_mainController_ISR, ".TI.ramfunc");
#pragma INTERRUPT(INT_mainController_ISR, {HP});
#endif

//Motor instances
motor_t motor1;
motorHandle motor1Handle;

motor_t motor2;
motorHandle motor2Handle;

//IMU instance
MPU6050_T imu1;
IMUHandle imu1Handle;

//Vehicle instance
vehicle_t vehicle1;
vehicleHandle vehicle1handle;

// wheel speed quadrature encoder
quadratureEncoder_t eqepMotorA;
quadratureHandle eqepMotorAHandle;

quadratureEncoder_t eqepMotorB;
quadratureHandle eqepMotorBHandle;

//speed trajectory
rampControl_t speedRamp=RAMPCTL_DEFAULTS;

PIDController_t speedController_Right;
pidHandle speedController_RightHandle;

PIDController_t speedController_Left;
pidHandle speedController_LeftHandle;

can_t can1;
canHandle CANHandle;

serialCMD usartB;
cmdHandle usartHandle;
//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
      Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();
    //delay 500us for I2C module capture the hardware status
    DEVICE_DELAY_US(1000);
    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();

    //CAN interrupt
    // Interrupt Settings for INT_mainCAN_0
    // ISR need to be defined for the registered interrupts
    Interrupt_register(INT_CANB0, &INT_mainCAN_ISR);
    Interrupt_enable(INT_CANB0);


    //motor1 parameter setting(right)
    motor1.ID=MOTORA;
    motor1.inputPin1=A1;
    motor1.inputPin2=A2;
    motor1.pwmBase=MotorControl_BASE;
    motor1Handle=motorInit(&motor1,sizeof(motor1));
    //motor2 parameter setting(left)
    motor2.ID=MOTORB;
    motor2.inputPin1=B1;
    motor2.inputPin2=B2;
    motor2.pwmBase=MotorControl_BASE;


    motor1Handle=motorInit(&motor1,sizeof(motor1));

    motor2Handle=motorInit(&motor2,sizeof(motor2));

    imu1Handle=MPU6050init(&imu1,sizeof(imu1));

    vehicle1handle=HAL_vehicleInit(&vehicle1,sizeof(vehicle1));


    eqepMotorAHandle=HAL_quadratureEncoderInit(&eqepMotorA,sizeof(eqepMotorA));
    eqepMotorA.eqepBase=EQEP_motorA_BASE;


    eqepMotorBHandle=HAL_quadratureEncoderInit(&eqepMotorB,sizeof(eqepMotorB));
    eqepMotorB.eqepBase=EQEP_motorB_BASE;

    speedController_RightHandle=pidControllerInit(&speedController_Right,sizeof(speedController_Right));
    speedController_LeftHandle=pidControllerInit(&speedController_Left,sizeof(speedController_Left));

    CANHandle=canInit(&can1,sizeof(can1));
    usartHandle=cmdInit(&usartB,sizeof(usartB));

    checkAttendence();//Check sensor present
    setOffset(imu1Handle);
#ifdef Calibrate_Enable
    calibration(imu1Handle);
#endif



    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;


    while(1)
    {
        //falling detection
        HAL_fallDetection(vehicle1handle,imu1Handle);
        HAL_hoverDetection(vehicle1handle,imu1Handle);
        updateCAN(CANHandle,imu1Handle,vehicle1handle);
        cmdParse(usartHandle);
        comDispatch(usartHandle,vehicle1handle);
        speedRamp.targetValue=vehicle1.targetSpeed*0.001f;
    }
}

__interrupt void INT_mainController_ISR(void)
{

        IMURead(IMUADDR, 0x3B, 14, imu1.dataBuffer);
        MPU_dataProcessing(imu1Handle);
        complemenaryEuler(imu1Handle);
        //status_send(imu1.orientation.roll*MATH_R2D, imu1.orientation.pitch*MATH_R2D, imu1.orientation.yaw*MATH_R2D);

        //wheel speed calculation
        HAL_speedCalculation(eqepMotorAHandle);
        //First order low pass filter alpha=1/(1+2*pi*F_cutoff*Ts) 5Hz CUTOFF here to filter the speed.
        vehicle1.speedMSRight=eqepMotorA.speedMS*0.136f+vehicle1.speedMSRight*0.864f;
        //data_print(eqepMotorA.speedMS,vehicle1.speedMSRight);

        HAL_speedCalculation(eqepMotorBHandle);
        //First order low pass filter alpha=1/(1+2*pi*F_cutoff*Ts) 5Hz CUTOFF here to filter the speed.
        vehicle1.speedMSLeft=eqepMotorB.speedMS*0.136f+vehicle1.speedMSLeft*0.864f;
        //data_print(eqepMotorB.speedMS,vehicle1.speedMSLeft);
        vehicle1.vehicleDirection=(eqepMotorA.dir==0? -eqepMotorB.dir:eqepMotorA.dir);
        vehicle1.speedMS=(vehicle1.speedMSLeft+vehicle1.speedMSRight)*0.5f;

        //speed ramp
        RC_MACRO(speedRamp)
        if(vehicle1.status==PICKUP || FALLING==vehicle1.status)
         {
            motorSetDutyCycle(motor1Handle, 0.0f);
            motorSetDutyCycle(motor2Handle, 0.0f);
            motorSetStatus(motor1Handle,MOTOR_BRAKE);
            motorSetStatus(motor2Handle,MOTOR_BRAKE);
            motorRun(motor1Handle);
            motorRun(motor2Handle);
         }
        else
        {
           HAL_balanceControl(vehicle1handle,imu1Handle);
           HAL_positionHold(vehicle1handle);

           speedController_Left.refInput=((fabsf(speedRamp.setPoint)<0.03f)? 0 : speedRamp.setPoint)*1000;
           speedController_Left.fbValue=vehicle1.vehicleDirection*vehicle1.speedMS*1000;
           continuousP_Icontroller(speedController_LeftHandle,vehicle1handle);

           HAL_steeringControl(vehicle1handle,imu1Handle);

           motor1.dutyCycle=vehicle1.balanceKp*speedController_Left.out+vehicle1.balancePWM+vehicle1.steeringPWM;
           motor2.dutyCycle=vehicle1.balanceKp*speedController_Left.out+vehicle1.balancePWM-vehicle1.steeringPWM;

           HAL_vehicleRun(motor1Handle,motor2Handle);
        }
        //CAN transmission timing
        can1.timeBaseCounter1++;
        can1.timeBaseCounter2++;
        Interrupt_clearACKGroup(INT_mainController_INTERRUPT_ACK_GROUP);
}
__interrupt void INT_mainCAN_ISR(void)
{
    uint32_t status;
    status=CAN_getInterruptCause(mainCAN_BASE);
    if(CAN_INT_INT0ID_STATUS==status)
    {
        status=CAN_getStatus(mainCAN_BASE);
        if(((status & ~(CAN_STATUS_RXOK))!=CAN_STATUS_LEC_MSK) && ((status & ~(CAN_STATUS_RXOK))!=CAN_STATUS_LEC_NONE) )
        {
            can1.flagError=1;
        }
    }
    else if(status == 1)//received message object ID, not message ID
    {
        CAN_readMessage(mainCAN_BASE, 1, (uint16_t *)can1.rxBuffer);
        CAN_clearInterruptStatus(mainCAN_BASE, 1);
        can1.rxMsgCount++;
        can1.flagRxDone=1;
        can1.flagError=0;
    }
    CAN_clearGlobalInterruptStatus(mainCAN_BASE,CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

__interrupt void INT_BL_PIC_RX_ISR(void)
{
    SCI_readCharArray(BL_PIC_BASE, usartB.rawCMD, 8);
    usartB.flagNewcmd=1;

    SCI_clearOverflowStatus(BL_PIC_BASE);
    SCI_clearInterruptStatus(BL_PIC_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INT_BL_PIC_RX_INTERRUPT_ACK_GROUP);
}
/*
__interrupt void INT_EQEP_motorA_ISR(void)
{

    if((EQEP_getInterruptStatus(EQEP_motorA_BASE) & EQEP_INT_UNIT_TIME_OUT)!=0)
    {
        HAL_speedCalculation(eqepMotorAHandle);
    }

    //First order low pass filter alpha=1/(1+2*pi*F_cutoff*Ts) 5Hz CUTOFF here to filter the speed.
    vehicle1.speedMSRight=eqepMotorA.speedMS*0.24f+vehicle1.speedMSRight*0.76f;
    //data_print(speedRamp.setPoint,vehicle1.speedMSRight);
    //clear global flag!!
    EQEP_clearInterruptStatus(EQEP_motorA_BASE, EQEP_INT_UNIT_TIME_OUT | EQEP_INT_GLOBAL);
    Interrupt_clearACKGroup(INT_EQEP_motorA_INTERRUPT_ACK_GROUP);
}

__interrupt void INT_EQEP_motorB_ISR(void)
{

    if((EQEP_getInterruptStatus(EQEP_motorB_BASE) & EQEP_INT_UNIT_TIME_OUT)!=0)
    {
        HAL_speedCalculation(eqepMotorBHandle);
    }
    //update vehicle speed here
    //First order low pass filter alpha=1/(1+2*pi*F_cutoff*Ts) 5Hz CUTOFF here to filter the speed.
    vehicle1.speedMSLeft=eqepMotorB.speedMS*0.24f+vehicle1.speedMSLeft*0.76f;
    //data_print(speedRamp.setPoint,vehicle1.speedMSLeft);
    vehicle1.vehicleDirection=eqepMotorA.dir;
    vehicle1.speedMS=(vehicle1.speedMSLeft+vehicle1.speedMSRight)*0.5f;
    //clear global flag!!
    EQEP_clearInterruptStatus(EQEP_motorB_BASE, EQEP_INT_UNIT_TIME_OUT  | EQEP_INT_GLOBAL);
    Interrupt_clearACKGroup(INT_EQEP_motorB_INTERRUPT_ACK_GROUP);
}
*/
//
// End of File
//
