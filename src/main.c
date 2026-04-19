/* main.c */
#include <stdio.h>
#include "pid.h"

int  main(void)
{
	PID_params param_1;
	float setpoint = 25.0; /* target value: 25 deg temp */
	float measurement = 19.0; /*starting temp value is 19 deg*/
	float dt = 0.01; // time step
	float output;

	PID_init(&param_1, 2.5, 0.5, 0.0, -100.0, 100.0);

	for (int i = 0; i < 200; ++i)
	{
		output = PID_update(&param_1, setpoint, measurement, dt);
		measurement += output * dt;

		printf("Step %d | Measurement: %.2f | Output: %.2f\n", i, measurement, output);
	
	}

	return 0;
}
