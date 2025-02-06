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
#include "PID_ElonM.h"
#include "ramp_Elon.h"
#include "DLOG_4CH_F.h"

#ifdef _FLASH
#pragma CODE_SECTION(INT_IMU_data_Ready_XINT_ISR, ".TI.ramfunc");
#pragma INTERRUPT(INT_IMU_data_Ready_XINT_ISR, {HP});
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

//speed control
//PID_CONTROLLER_t speedController={PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};

PIDController_t speedController_Right;
pidHandle speedController_RightHandle;

PIDController_t speedController_Left;
pidHandle speedController_LeftHandle;

//balance controller
PIDController_t balanceController;
pidHandle balanceHandle;

PIDController_t steeringPID;
pidHandle steeringHandle;



// Data log for debugging
float dbuffChan1[200],
      dbuffChan2[200],
      dbuffChan3[200],
      dbuffChan4[200],
      dlogChan1,
      dlogChan2,
      dlogChan3,
      dlogChan4;
DLOG_4CH_F dlog_4ch1;

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

    //data log setting
    DLOG_4CH_F_init(&dlog_4ch1);
    dlog_4ch1.input_ptr1=&dlogChan1;
    dlog_4ch1.input_ptr2=&dlogChan2;
    dlog_4ch1.input_ptr3=&dlogChan3;
    dlog_4ch1.input_ptr4=&dlogChan4;

    dlog_4ch1.output_ptr1=dbuffChan1;
    dlog_4ch1.output_ptr2=dbuffChan2;
    dlog_4ch1.output_ptr3=dbuffChan3;
    dlog_4ch1.output_ptr4=dbuffChan4;


    dlog_4ch1.size=200;
    dlog_4ch1.pre_scalar=7;// determined how long the buffer size points can represent
    dlog_4ch1.trig_value=0.01;
    dlog_4ch1.status=2;


    //motor1 parameter setting
    motor1.ID=MOTORA;
    motor1.inputPin1=A1;
    motor1.inputPin2=A2;
    motor1.pwmBase=MotorControl_BASE;
    motor1Handle=motorInit(&motor1,sizeof(motor1));
    //motor2 parameter setting
    motor2.ID=MOTORB;
    motor2.inputPin1=B1;
    motor2.inputPin2=B2;
    motor2.pwmBase=MotorControl_BASE;


    motor1Handle=motorInit(&motor1,sizeof(motor1));

    motor2Handle=motorInit(&motor2,sizeof(motor2));

    imu1Handle=MPU6050init(&imu1,sizeof(imu1));

    vehicle1handle=HAL_vehicleInit(&vehicle1,sizeof(vehicle1));
    //vehicle1.balanceKd=1.0f;

    balanceHandle=pidControllerInit(&balanceController,sizeof(balanceController));
    balanceController.ts=1.5e-3f;
    balanceController.kp=55.0f;
    balanceController.td=0.85f;

    eqepMotorA.eqepBase=EQEP_motorA_BASE;
    eqepMotorAHandle=HAL_quadratureEncoderInit(&eqepMotorA,sizeof(eqepMotorA));

    eqepMotorB.eqepBase=EQEP_motorB_BASE;
    eqepMotorBHandle=HAL_quadratureEncoderInit(&eqepMotorB,sizeof(eqepMotorB));

    speedController_RightHandle=pidControllerInit(&speedController_Right,sizeof(speedController_Right));
    speedController_LeftHandle=pidControllerInit(&speedController_Left,sizeof(speedController_Left));
    speedController_Right.iterm_Min=-200.0f;
    speedController_Right.iterm_Max=200.0f;
    speedController_Left.iterm_Min=-200.0f;
    speedController_Left.iterm_Max=200.0f;

    steeringHandle=pidControllerInit(&steeringPID, sizeof(steeringPID));


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
        HAL_proportionalSteering(vehicle1handle);
        if(CPUTimer_getTimerOverflowStatus(filterTimer_BASE))
        {

                RC_MACRO(speedRamp)

                //right wheel speed control
                speedController_Right.refInput=((fabsf(speedRamp.setPoint)<0.03f)? 0 : speedRamp.setPoint)*1000;
                speedController_Right.fbValue=eqepMotorA.dir*vehicle1.speedMSRight*1000;
               speedPIcontroller(speedController_RightHandle);
                //left wheel speed control
                speedController_Left.refInput=((fabsf(speedRamp.setPoint)<0.03f)? 0 : speedRamp.setPoint)*1000;
               speedController_Left.fbValue=-eqepMotorB.dir*vehicle1.speedMSLeft*1000;
               speedPIcontroller(speedController_LeftHandle);

                 // HAL_steeringControl(vehicle1handle,imu1Handle);

                    /*
                  speedController_Left.refInput=((fabsf(speedRamp.setPoint)<0.03f)? 0 : speedRamp.setPoint)*1000;
                   speedController_Left.fbValue=eqepMotorA.dir*vehicle1.speedMS*1000;
                   speedPIcontroller(speedController_LeftHandle);
                    */
                //HAL_balanceControl(vehicle1handle,imu1Handle);
               // balanceController.refInput=vehicle1.targetAngle;//Unit: Degree
               // balanceController.fbValue=imu1.orientation.roll*MATH_R2D-2.21f;
               // updateP_Dcontroller(balanceHandle);

               motor1.dutyCycle=vehicle1.balancePWM+speedController_Left.out+vehicle1.steeringPWM;
               motor2.dutyCycle=vehicle1.balancePWM+speedController_Left.out-vehicle1.steeringPWM;
                //filter testing code
               // vehicle1.balancePWM=balanceController.out-vehicle1.balanceKd*imu1.GX*MATH_R2D;
                //motor1.dutyCycle=vehicle1.balancePWM;
                //motor2.dutyCycle=vehicle1.balancePWM*0.91f;

                 //yaw rate control
                 //steeringPID.fbValue=imu1.GZ*MATH_R2D+2.8f;
                 //steeringPID.refInput=vehicle1.targetYawRate;
                // updatePIDcontroller(steeringHandle);
                //HAL_vehicleRun(motor1Handle,motor2Handle);
             //status_send(imu1.orientation.roll*MATH_R2D, imu1.orientation.pitch*MATH_R2D, imu1.orientation.yaw*MATH_R2D);


            CPUTimer_startTimer(filterTimer_BASE);
        }
    }
}


__interrupt void INT_IMU_data_Ready_XINT_ISR(void)
{

        IMURead(IMUADDR, 0x3B, 14, imu1.dataBuffer);
        MPU_dataProcessing(imu1Handle);
        complemenaryEuler(imu1Handle);
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
            HAL_vehicleRun(motor1Handle,motor2Handle);
        }
      //data log functions
     // dlogChan1=vehicle1.speedMS;
     // dlogChan2=steeringPID.out;
      //DLOG_4CH_F_FUNC(&dlog_4ch1);
      Interrupt_clearACKGroup(INT_IMU_data_Ready_XINT_INTERRUPT_ACK_GROUP);
}

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

//
// End of File
//
