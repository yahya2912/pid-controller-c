/* pid.c */

#include"pid.h"

void PID_init(PID_params *param, float Kp, float Ki, float Kd, float output_min, float output_max)
{
	param->integral = 0.0;
	param->prev_error = 0.0;
	param->Kp = Kp;
	param->Ki = Ki;
	param->Kd = Kd;
	param->output_min = output_min;
	param->output_max = output_max;
}

float PID_update(PID_params *param, float setpoint, float meas, float dt)
{
	float curr_error;
       	float derivative;
	float output;

	curr_error = setpoint - meas;

	param->integral += curr_error * dt;

	if (param->integral < param->output_min)
		param->integral = param->output_min;
 	if (param->integral > param->output_max)
		param->integral = param->output_max;

	derivative = (curr_error - param->prev_error) / dt;

	output = (param->Kp * curr_error) + (param->Ki * param->integral) + (param->Kd * derivative);

	if (output < param->output_min)
		output = param->output_min;
 	if (output > param->output_max)
		output = param->output_max;

	param->prev_error = curr_error;

	return output;
}
