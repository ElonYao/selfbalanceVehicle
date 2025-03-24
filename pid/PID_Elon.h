#ifndef _PID_ELON_H
#define _PID_ELON_H


/*-----------------------------------Revision history--------------------------------*/
/*V0.1 ---First released on 4th July 2024--------------------------------------------*/
/*V0.2 ---Release on 8th Nov 2025----------------------------------------------------*/
/*Copy right by Elon Yao------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif
#include <string.h>
#include "hal.h"

typedef struct _pidcontroller_
{
    unsigned int flag_AntiWindUp;
    float kp;
    float ti;
    float inv_ti;
    float td;
    float ts;
    float half_ts;
    unsigned int N;// derivative iterm low pass filter bandwith(default 100)
    float I_term;
    float D_term;
    float D_term_coe1;//=2.0f*N/(2.0f+N*Ts)
    float D_term_coe2;//=(2.0f-N*Ts)/(2.0f+N*Ts)
    float iterm_Max;
    float iterm_Min;
    float outMax;
    float outMin;
    float lastError;
    float lastMeasure;
    float refInput;
    float fbValue;
    float temp;//for observing the target value range
    float out;

}PIDController_t;

typedef struct _pidcontroller_ *pidHandle;

pidHandle pidControllerInit(void *pMemory, const size_t numBytes);

float discreteP_Icontroller(pidHandle handle,vehicleHandle vehiclehandle);
float continuousP_Icontroller(pidHandle handle,vehicleHandle vehiclehandle);
float updateP_Dcontroller(pidHandle handle);
float speedPIcontroller(pidHandle handle);
float speedPIcontroller2(pidHandle handle,vehicleHandle vehiclehandle);
float speedPIcontroller3(pidHandle handle);
void setKp(pidHandle handle,float kp);
void setKi(pidHandle handle,float ki);
void setKd(pidHandle handle,float kd);
void setTs(pidHandle handle,float ts);
void setOutputLimt(pidHandle handle,float max,float min);


#ifdef __cplusplus
}
#endif
#endif

