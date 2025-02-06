#include "hal.h"

motorHandle motorInit(void *memory,const size_t memorySize)
{
    motorHandle handle;
    motor_t *obj;
    if(memorySize < sizeof(motor_t))
    {
        return (motorHandle)NULL;
    }
    handle = (motorHandle)memory;
    obj = (motor_t *)handle;
    obj->status = MOTOR_STOP;

    return handle;
}
vehicleHandle HAL_vehicleInit(void *memory,const size_t memorySize)
{
    vehicleHandle handle;
    vehicle_t *obj;
    if(memorySize < sizeof(vehicle_t))
    {
        return (vehicleHandle)NULL;
    }
    handle = (vehicleHandle)memory;
    obj = (vehicle_t *)handle;
    obj->status = NORMALL;
    obj->balanceKp=700.0f;
    obj->balanceKd=160.3f;
    obj->outMax=4750.0f;
    obj->outMin=-4750.0f;
    obj->targetAngle=0.50f;
    obj->steeringKd=0.0f;
    obj->steeringKd=0.0f;
    obj->steeringPWM=0.0f;
    obj->targetYawRate=0.0f;//unit:deg/s
    obj->speedMSLeft=0.0f;
    obj->speedMSRight=0.0f;
    obj->flag_turning=0;//left -1 right 1
    return handle;
}
quadratureHandle HAL_quadratureEncoderInit(void *memory,const size_t memorySize)
{

    quadratureHandle handle;
    quadratureEncoder_t *obj;
    if(memorySize < sizeof(quadratureEncoder_t))
    {
        return (quadratureHandle)NULL;
    }
    handle = (quadratureHandle)memory;
    obj = (quadratureEncoder_t *)handle;
    obj->unitTimerFreq=100;
    obj->dir=0;
    obj->freq=0;
    obj->preCounter=0;
    obj->speedRPM=0.0f;
    obj->speedMS=0.0f;
    return handle;

}
void HAL_speedCalculation(quadratureHandle handle)
{
    quadratureEncoder_t *obj=(quadratureEncoder_t *)handle;
    uint32_t temp=0,newCounter=0;

    newCounter=EQEP_getPositionLatch(obj->eqepBase);
    temp=newCounter;

    obj->dir=EQEP_getDirection(obj->eqepBase);

    // Calculate the delta counts in unit timer  10ms

    if(obj->dir>0)
    {
        if(newCounter>=obj->preCounter)
            newCounter-=obj->preCounter;
        else
            newCounter+=0xFFFFFFFF-obj->preCounter;
    }
    else
    {
        if(newCounter<=obj->preCounter)
            newCounter=obj->preCounter-newCounter;
        else
            newCounter=(0xFFFFFFFF-newCounter)+obj->preCounter;
    }

    obj->preCounter=temp;

    obj->freq=newCounter*obj->unitTimerFreq;
    obj->speedRPM=WHEELRPM_SF*obj->freq;
    obj->speedMS=WHEELMS_SF*obj->freq;


}
void HAL_balanceControl(vehicleHandle vehiclehandle,IMUHandle imuhandle)
{
    vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    MPU6050_T *obj_IMU=( MPU6050_T *) imuhandle;
    float32_t result=0.0f;
    //subtract mechanical zero point roll angle and angular speed offset
    obj_V->fbAngle=obj_IMU->orientation.roll*MATH_R2D-2.21f;
    result=(obj_V->targetAngle-obj_V->fbAngle)*0.1*obj_V->balanceKp+obj_V->balanceKd*(0-obj_IMU->GX*MATH_R2D)*0.1;
    obj_V->balancePWM=_constrain(result,obj_V->outMin,obj_V->outMax);

}
void HAL_steeringControl(vehicleHandle vehiclehandle,IMUHandle imuhandle)
{
    vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    MPU6050_T *obj_IMU=( MPU6050_T *) imuhandle;
    float result=0,error=0;
    obj_V->fbYawRate=obj_IMU->GZ*MATH_R2D;
    error=obj_V->targetYawRate-obj_V->fbYawRate;
    result=error*obj_V->steeringKp+obj_V->steeringKd*(error-obj_V->lastError);
    obj_V->lastError=error;
    obj_V->steeringPWM=_constrain(result,obj_V->outMin,obj_V->outMax);

}

void HAL_proportionalSteering(vehicleHandle vehiclehandle)
{
    vehicle_t *obj= (vehicle_t *)vehiclehandle;
    if(obj->flag_turning==-1)
    {
        obj->steeringPWM=TURNINGPWMDEF;
    }
    else if(obj->flag_turning==1)
    {
        obj->steeringPWM=-TURNINGPWMDEF;
    }
    else
    {
        obj->steeringPWM=0;
    }
}
void HAL_fallDetection(vehicleHandle vehiclehandle,IMUHandle imuhandle)
{
    vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    MPU6050_T *obj_IMU=( MPU6050_T *) imuhandle;
    if(fabsf(obj_IMU->orientation.roll*MATH_R2D)>60.0f)
    {
        obj_V->status=FALLING;
    }
}

void HAL_hoverDetection(vehicleHandle vehiclehandle,IMUHandle imuhandle)
{
    vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    MPU6050_T *obj_IMU=( MPU6050_T *) imuhandle;
    if(obj_V->speedMS>0.75f && fabsf(obj_IMU->orientation.roll)<0.79f)
    {
        obj_V->status=PICKUP;
    }

}
void motorSetStatus(motorHandle handle,motorState_t status)
{
    motor_t *obj =(motor_t *) handle;
    obj->status=status;
}

void motorSetDutyCycle(motorHandle handle,float32_t thresHold)
{
    motor_t *obj =(motor_t *) handle;

    obj->compareValue=fabsf(thresHold);
    if(MOTORA==obj->ID)
    {
        EPWM_setCounterCompareValue(obj->pwmBase, EPWM_COUNTER_COMPARE_A, obj->compareValue);
    }
    else if(MOTORB==obj->ID)
    {
        EPWM_setCounterCompareValue(obj->pwmBase, EPWM_COUNTER_COMPARE_B, obj->compareValue);
    }
}

void motorRun(motorHandle handle)
{
    motor_t *obj =(motor_t *) handle;

    switch(obj->status)
    {
        case MOTOR_STOP:
            //stop the motor
           EPWM_clearTripZoneFlag(obj->pwmBase, EPWM_TZ_FLAG_OST);
           GPIO_writePin(obj->inputPin1, 0);
           GPIO_writePin(obj->inputPin2, 0);

            break;
        case MOTOR_FORWARD:
            //run the motor in forward direction
            EPWM_clearTripZoneFlag(obj->pwmBase, EPWM_TZ_FLAG_OST);
            GPIO_writePin(obj->inputPin1, 0);
            GPIO_writePin(obj->inputPin2, 1);
            break;
        case MOTOR_BACKWARD:
            //run the motor in backward direction
            EPWM_clearTripZoneFlag(obj->pwmBase, EPWM_TZ_FLAG_OST);
            GPIO_writePin(obj->inputPin1, 1);
            GPIO_writePin(obj->inputPin2, 0);
            break;
        case MOTOR_BRAKE:
            //brake the motor
            GPIO_writePin(obj->inputPin1, 1);
            GPIO_writePin(obj->inputPin2, 1);
            EPWM_forceTripZoneEvent(obj->pwmBase, EPWM_TZ_FORCE_EVENT_OST);
            break;
        default:
            break;
    }
}
void HAL_vehicleRun(motorHandle handle1,motorHandle handle2)
{
        motor_t *obj_motor1 =(motor_t *) handle1;
        motor_t *obj_motor2 =(motor_t *) handle2;

        obj_motor1->dutyCycle=_constrain(obj_motor1->dutyCycle,-4375,4375);
        obj_motor2->dutyCycle=_constrain(obj_motor2->dutyCycle,-4375,4375);

        if(obj_motor1->dutyCycle>=0.0f)
        {
            motorSetStatus(handle1,MOTOR_FORWARD);
        }
        else
        {
            motorSetStatus(handle1,MOTOR_BACKWARD);
        }

        if(obj_motor2->dutyCycle>=0.0f)
        {
            motorSetStatus(handle2,MOTOR_FORWARD);
        }
        else
        {
            motorSetStatus(handle2,MOTOR_BACKWARD);
        }
        motorSetDutyCycle(handle1, obj_motor1->dutyCycle);
        motorSetDutyCycle(handle2, obj_motor2->dutyCycle);
        motorRun(handle1);
        motorRun(handle2);
}
