#ifndef PID_H_
#define PID_H_

typedef	struct pid_data
{
	double	Kp;
	double	Ki;
	double	Kd;
	double	IntSum;
	double	IntSumMax;
	double	FeedbackLast;
	double	ErrorMax;
} pid_data_t;

double		pid_controller		(const double*, const double*, pid_data_t*);

#endif /* PID_H_ */
