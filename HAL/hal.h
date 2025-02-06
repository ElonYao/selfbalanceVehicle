#ifndef _HAL_SELBALANCE
#define _HAL_SELBALANCE


#ifdef __cplusplus
extern "C" {
#endif

#include "driverlib.h"
#include <string.h>
#include "board.h"
#include "IMU6050_Elon.h"

#define _constrain(amt,min,high)  ((amt>high)? high : ((amt<min)? min : amt))
#define WHEELRPM_SF 0.00048828f // 60/(1024*4*30)
#define WHEELMS_SF 2.0453e-6f // pi*0.08/(1024*4*30)
#define TURNINGPWMDEF 250
typedef enum
{
    FALLING=0,
    NORMALL,
    FORWARD,
    BACKWARD,
    PICKUP,
    LEFTTURN,
    RIGHNTURN,
    BRAKE,
    STOP
}vehicleStatus_t;

typedef struct _vehicle_
{
    vehicleStatus_t status;
    float32_t targetSpeed;
    float32_t speedMS;
    float32_t speedMSLeft;
    float32_t speedMSRight;
    int16_t vehicleDirection;
    //balance control
    float32_t fbAngle;
    float32_t targetAngle;
    float32_t balanceKp;
    float32_t balanceKd;
    float32_t outMax;
    float32_t outMin;
    float32_t balancePWM;

    //steering control
    float32_t fbYawRate;
    float32_t targetYawRate;
    float32_t steeringKp;
    float32_t steeringKd;
    float32_t steeringPWM;
    float32_t lastError;
    int16_t flag_turning;

}vehicle_t;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
}motorState_t;

typedef enum
{
    MOTORA=0,
    MOTORB
}motorID_t;
typedef struct _motor_
{

    uint32_t pwmBase;
    motorID_t ID;
    motorState_t status;
    uint16_t speed;
    float32_t dutyCycle;
    uint16_t compareValue;//compare value for pwm
    uint32_t inputPin1;
    uint32_t inputPin2;
} motor_t;

typedef struct _eqep_
{
   uint16_t unitTimerFreq;
   uint32_t eqepBase;
   uint32_t preCounter;
   int32_t freq;
   float32_t speedRPM;
   float32_t speedMS;
   int16_t dir;

}quadratureEncoder_t;


typedef struct _motor_ *motorHandle;
typedef struct _vehicle_ *vehicleHandle;
typedef struct _eqep_ *quadratureHandle;

motorHandle motorInit(void *memory,const size_t memorySize);
void motorSetStatus(motorHandle handle,motorState_t status);
void motorSetDutyCycle(motorHandle handle,float32_t dutyCycle);
void motorRun(motorHandle handle);

vehicleHandle HAL_vehicleInit(void *memory,const size_t memorySize);
void HAL_fallDetection(vehicleHandle vehiclehandle,IMUHandle imuhandle);
void HAL_hoverDetection(vehicleHandle vehiclehandle,IMUHandle imuhandle);
void HAL_balanceControl(vehicleHandle vehiclehandle,IMUHandle imuhandle);
void HAL_steeringControl(vehicleHandle vehiclehandle,IMUHandle imuhandle);
void HAL_proportionalSteering(vehicleHandle vehiclehandle);
void HAL_vehicleRun(motorHandle handle1,motorHandle handle2);
quadratureHandle HAL_quadratureEncoderInit(void *memory,const size_t memorySize);
void HAL_speedCalculation(quadratureHandle handle);




#ifdef __cplusplus
}
#endif

#endif
