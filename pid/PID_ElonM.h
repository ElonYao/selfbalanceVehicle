#ifndef __PID_E_H__
#define __PID_E_H__

typedef struct {  float  Ref;   			// Input: reference set-point
				  float  Fbk;   			// Input: feedback
				  float  Out;   			// Output: controller output 
				  float  c1;   			// cutoff frequency in  Hz
				  float  c2;   			// =1-c1*Ts
				} PID_TERMINALS;
				// note: c1 & c2 placed here to keep structure size under 8 words

typedef struct {  float  Kr;				// Parameter: reference set-point weighting 
				  float  Kp;				// Parameter: proportional loop gain
				  float  Ki;			    // Parameter: integral gain
				  float  Kd; 		        // Parameter: derivative gain
				  float  Km; 		        // Parameter: derivative weighting
				  float  Umax;			// Parameter: upper saturation limit
				  float  Umin;			// Parameter: lower saturation limit
				} PID_PARAMETERS;

typedef struct {  float  up;				// Data: proportional term
				  float  ui;				// Data: integral term
				  float  ud;				// Data: derivative term
				  float  v1;				// Data: pre-saturated controller output
				  float  i1;				// Data: integrator storage: ui(k-1)
				  float  d1;				// Data: differentiator storage: ud(k-1)
				  float  d2;				// Data: differentiator storage: d2(k-1) 
				  float  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
				} PID_DATA;


typedef struct {  PID_TERMINALS	term;
				  PID_PARAMETERS param;
				  PID_DATA		data;
				} PID_CONTROLLER_t;

/*-----------------------------------------------------------------------------
Default initial values for the PID objects
-----------------------------------------------------------------------------*/                     

#define PID_TERM_DEFAULTS {				\
						   0, 			\
                           0, 			\
                           0, 			\
                           1, 			\
						   0 			\
              			  }

#define PID_PARAM_DEFAULTS {			\
                           1.0,	\
                           5.1f, /*Proportional*/	\
                           0.0f,/*ki*/	\
                           0.0f,	/*kd*/\
                           1.0,	\
                           4735.0f,	\
                           -4735.0f	\
              			  }

#define PID_DATA_DEFAULTS {			    \
                           0.0,	\
                           0.0, 	\
                           0.0,	\
                           0.0,	\
                           0.0, 	\
                           0.0,	\
                           0.0,	\
                           1.0 	\
              			  }


/*------------------------------------------------------------------------------
 	PID Macro Definition
------------------------------------------------------------------------------*/

#define PID_MACRO(m)																				\
																									\
	/* proportional term */ 																		\
	m.data.up = (m.param.Kr*m.term.Ref) - m.term.Fbk;	        									\
																									\
	/* integral term */ 																			\
	m.data.ui =(m.param.Ki*(m.data.w1*(m.term.Ref - m.term.Fbk))) + m.data.i1;						\
	m.data.i1 = m.data.ui;																			\
																									\
	/* derivative term */ 																			\
	m.data.d2 = (m.param.Kd*(m.term.c1*((m.term.Ref*m.param.Km) - m.term.Fbk))) - m.data.d2;	\
	m.data.ud = m.data.d2 + m.data.d1;																\
	m.data.d1 = m.data.ud*m.term.c2;														\
																									\
	/* control output */ 																			\
	m.data.v1 = m.param.Kp*(m.data.up + m.data.ui + m.data.ud);							\
	m.term.Out= ((m.data.v1<m.param.Umin)? m.param.Umin : ((m.data.v1>m.param.Umax)? m.param.Umax : m.data.v1));										\
	m.data.w1 = (m.term.Out == m.data.v1) ? 1.0f : 0.0f;									\
	
#endif //

