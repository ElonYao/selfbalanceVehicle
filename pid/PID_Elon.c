#include "PID_Elon.h"

pidHandle pidControllerInit(void *pMemory, const size_t numBytes)
{
    pidHandle handle;
    PIDController_t * obj;
    
    if(numBytes < sizeof(PIDController_t))
    {
        return((pidHandle)NULL);
    }

    // assign the handle
    handle = (pidHandle)pMemory;

    // assign the object
    obj = (PIDController_t *)handle;
    //variables initialization section
    obj->N=100;//only valid for derivative term
    obj->flag_AntiWindUp=1;
    obj->kp=-0.039f;
    obj->ti=-0.00025f;
    //obj->inv_ti=1.0f/obj->ti;
    obj->td=0.0f;
    obj->ts=0.005f;
    obj->half_ts=0.5*obj->ts;
    obj->I_term=0.0f;
    obj->D_term=0.0f;
    obj->D_term_coe1=(2.0f*obj->N)/(2.0f+obj->N*obj->ts);
    obj->D_term_coe2=(2.0f-obj->N*obj->ts)/(2.0f+obj->N*obj->ts);
    obj->iterm_Max=100.0f;
    obj->iterm_Min=-100.0f;
    obj->outMax=30.0f;
    obj->outMin=-30.0f;
    obj->lastError=0.0f;
    obj->lastMeasure=0.0f;
    obj->out=0.0f;
    return handle;

}
//derived on discrete PI controller P+I*Ts/(z-1)
float discreteP_Icontroller(pidHandle handle,vehicleHandle vehiclehandle)
{
   PIDController_t * obj = (PIDController_t *) handle;
   vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    //get the error
    float error=obj->refInput-obj->fbValue;
    float result=0.0f;

    //Integrator part
    obj->I_term=obj->lastError*obj->ti*obj->ts+obj->I_term;

    if(obj->flag_AntiWindUp)
    {
        obj->I_term=((obj->I_term<obj->iterm_Min)? obj->iterm_Min : ((obj->I_term>obj->iterm_Max)? obj->iterm_Max : obj->I_term));
    }
    result=-error*obj->kp-obj->I_term-obj_V->positionError;

    result=((result<obj->outMin)? obj->outMin : ((result>obj->outMax)? obj->outMax : result));
    obj->out=result;
    obj->lastError=error;
    return result;
}

//derived on continuous PI controller P+I/s with tustin method
float continuousP_Icontroller(pidHandle handle,vehicleHandle vehiclehandle)
{
   PIDController_t * obj = (PIDController_t *) handle;
   vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    //get the error
    float error=obj->refInput-obj->fbValue-(obj_V->positionError*obj_V->positionCoeff);
    float result=0.0f;

    //Integrator part
    obj->I_term+=obj->half_ts*obj->ti*(error+obj->lastError);

    if(obj->flag_AntiWindUp)
    {
        obj->I_term=((obj->I_term<obj->iterm_Min)? obj->iterm_Min : ((obj->I_term>obj->iterm_Max)? obj->iterm_Max : obj->I_term));
    }

    result=error*obj->kp+obj->I_term;

    result=_constrain(result,obj->outMin,obj->outMax);

    obj->out=result;
    obj->lastError=error;
    return result;
}
float updateP_Dcontroller(pidHandle handle)
{
      PIDController_t * obj = (PIDController_t *) handle;
     //get the error
     float error=obj->refInput-obj->fbValue;
     float result=0.0f;

     //derivative part
      obj->D_term=1/obj->ts*obj->td*(error-obj->lastError);

     result=error*obj->kp+obj->D_term;

     result=((result<obj->outMin)? obj->outMin : ((result>obj->outMax)? obj->outMax : result));
     obj->out=result;
     obj->lastError=error;
     return result;
}
//MW method
float speedPIcontroller(pidHandle handle)
{
    PIDController_t * obj = (PIDController_t *) handle;
    float error=obj->refInput+obj->fbValue;

    obj->I_term+=error * obj->ti;
    if(obj->flag_AntiWindUp)
    {
        obj->I_term=((obj->I_term<obj->iterm_Min)? obj->iterm_Min : ((obj->I_term>obj->iterm_Max)? obj->iterm_Max : obj->I_term));
    }
    obj->out=-obj->kp*error-obj->I_term;
    obj->out=_constrain(obj->out,obj->outMin,obj->outMax);
    return obj->out;
}
float speedPIcontroller2(pidHandle handle,vehicleHandle vehiclehandle)
{
    PIDController_t * obj = (PIDController_t *) handle;
    vehicle_t *obj_V= (vehicle_t *)vehiclehandle;
    float error=0.0f;
    error=obj->refInput-obj->fbValue-obj_V->positionError*0.05f;
    obj->I_term=(error+obj->lastError)*obj->ti;

    if(obj->flag_AntiWindUp)
    {
        obj->I_term=((obj->I_term<obj->iterm_Min)? obj->iterm_Min : ((obj->I_term>obj->iterm_Max)? obj->iterm_Max : obj->I_term));
    }
    obj->out=obj->kp*error+obj->I_term;
    obj->out=((obj->out<-20.0f)? -20.0f : ((obj->out>20.0f)? 20.0f : obj->out));
    obj_V->targetAngle=obj->out;
    obj->lastError=error;
    return obj->out;

}
//tark method
float speedPIcontroller3(pidHandle handle)
{
    PIDController_t * obj = (PIDController_t *) handle;
    float error,errorUpdate;
    errorUpdate=obj->fbValue-obj->refInput;
    error=obj->lastError*0.8f+errorUpdate*0.2f;

    obj->I_term+=error;

    if(obj->flag_AntiWindUp)
    {
        obj->I_term=((obj->I_term<obj->iterm_Min)? obj->iterm_Min : ((obj->I_term>obj->iterm_Max)? obj->iterm_Max : obj->I_term));
    }
    obj->out=obj->kp*error+obj->ti*obj->I_term;
    obj->out=_constrain(obj->out,obj->outMin,obj->outMax);
    obj->lastError=error;
    return obj->out;
}
void setKp(pidHandle handle,float kp)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->kp=kp;
}
void setKi(pidHandle handle,float ki)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->ti=ki;
    obj->inv_ti=1.0f/ki;
}
void setKd(pidHandle handle,float kd)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->td=kd;
}
void setTs(pidHandle handle,float ts)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->ts=ts;
    obj->half_ts=0.5*ts;
    obj->D_term_coe1=(2.0f*obj->N)/(2.0f+obj->N*obj->ts);
    obj->D_term_coe2=(2.0f-obj->N*obj->ts)/(2.0f+obj->N*obj->ts);
}

void setOutputLimt(pidHandle handle,float max,float min)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->outMax=max;
    obj->outMin=min;
}
