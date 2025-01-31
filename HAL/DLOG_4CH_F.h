#ifndef DLOG_4CH_F_H
#define DLOG_4CH_F_H

//*********** Structure Definition ********//
typedef struct{
	float *input_ptr1;
	float *input_ptr2;
	float *input_ptr3;
	float *input_ptr4;
	float *output_ptr1;
	float *output_ptr2;
	float *output_ptr3;
	float *output_ptr4;
	float prev_value;
	float trig_value;
	int status;
	int pre_scalar;
	int skip_count;
	int size;
	int count;
}DLOG_4CH_F;

//*********** Function Declarations *******//
void DLOG_4CH_F_init(DLOG_4CH_F *v);
void DLOG_4CH_F_FUNC(DLOG_4CH_F *v);

//*********** Macro Definition ***********//
#define DLOG_4CH_F_MACRO(v)											\
	switch(v.status)												\
	{																\
	case 1: /* wait for trigger*/									\
		if(*v.input_ptr1>v.trig_value && v.prev_value<v.trig_value)	\
		{															\
			/* rising edge detected start logging data*/			\
			v.status=2;												\
		}															\
		break;														\
	case 2:															\
		v.skip_count++;												\
		if(v.skip_count==v.pre_scalar)								\
		{															\
			v.skip_count=0;											\
			v.output_ptr1[v.count]=*v.input_ptr1;					\
			v.output_ptr2[v.count]=*v.input_ptr2;					\
			v.output_ptr3[v.count]=*v.input_ptr3;					\
			v.output_ptr4[v.count]=*v.input_ptr4;					\
			v.count++;												\
			if(v.count==v.size)										\
			{														\
				v.count=0;											\
				v.status=1;											\
			}														\
		}															\
		break;														\
	default:														\
		v.status=0;													\
	}																\
	v.prev_value=*v.input_ptr1;										\


#endif /* DLOG_4CH_F_H_ */
