
#ifndef __RAMP_ELON_H__
#define __RAMP_ELON_H__

//V2 ramp add the acceleration control

typedef struct { float    targetValue; 	// Input: Target input
				 float    maxDelta;	//define the maximum acceleration step
				 float    lowLimit;	// Minimum limit
				 float    highLimit;	// Maximum limit
				 float    setPoint;	// ramp output
				 float	  error;			// Variable: Temp variable
				 float    maxAccel_mpss; // Maximum acceleration 0.25m/ss
		  	   } rampControl_t;


#define RAMPCTL_DEFAULTS {  0.0f, 		 \
                            0.00125f,/*maxAccel_mpss *ctrlPeriod_sec*/	 \
                           -0.5f, \
                           0.5f, \
                            0.0f, \
                          	0.0f,\
                          	0.25f\
                   		  }



#define RC_MACRO(m)\
    m.error = m.targetValue - m.setPoint;\
	m.setPoint +=__fsat(m.error, m.maxDelta, -m.maxDelta);\
	m.setPoint=__fsat(m.setPoint, m.highLimit, m.lowLimit);	\

#endif //
