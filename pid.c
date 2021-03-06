#include <stdlib.h>
#include <stdint.h>

#include "pid.h"

double	pid_controller(const double* input, const double* feedback, pid_data_t* pid)
{
	double error;
	double ret, tmp;
	double p_term,i_term,d_term;

	error = *input - *feedback;

	// P
	if (error > pid->ErrorMax)
	{
		p_term = pid->ErrorMax;
	}
	else if (error < -pid->ErrorMax)
	{
		p_term = -pid->ErrorMax;
	}else{
		p_term = pid->Kp*error;
	}

	// I
	tmp = pid->IntSum + pid->Ki*error;
	if (tmp > pid->IntSumMax)
	{
		i_term	= pid->IntSumMax;
		pid->IntSum = pid->IntSumMax;
	}
	else if (tmp < -pid->IntSumMax)
	{
		i_term = -pid->IntSumMax;
		pid->IntSum = -pid->IntSumMax;
	}else{
		pid->IntSum = tmp;
		//i_term = pid->Ki * pid->IntSum;
		i_term = pid->IntSum;
	}

	// D
	d_term = pid->Kd * (*feedback - pid->FeedbackLast);

	// Save last feedback
	pid->FeedbackLast = *feedback;

	// Sum it all
	ret = (p_term + i_term + d_term);

	if(ret > pid->ErrorMax)
	{
		ret = pid->ErrorMax;
	}
	else if(ret < -pid->ErrorMax)
	{
		ret = -pid->ErrorMax;
	}

	return ret;
}
