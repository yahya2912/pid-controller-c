#ifndef PID_H
#define PID_H

/* PID controller - to be implemented */

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float prev_error;
	float output_min;
	float output_max;
} PID_params;

void PID_init(PID_params *param, float Kp, float Ki, float Kd, float output_min, float output_max);

float PID_update(PID_params *param, float setpoint, float meas, float dt);

#endif /* PID_H */
