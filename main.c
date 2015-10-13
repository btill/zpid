/**********************************************************************************************
main.c
Author: Brendan Till

This serves as an example use-case for the zpid and zblock modules.
A simulation is set up as follows:
	- Controller
	- First-order plant model
	- Step input, first stepping up to a set value, then stepping back down to zero
	- 0.001s sample rate; 5 seconds total duration
	- Output of the plant model is printed

To compile and run:
$ gcc main.c zpid.c zblock.c -std=c99 -lm -Wall
$ ./a.out > output.txt
 **********************************************************************************************/

#include <stdio.h>
#include "zpid.h"
#include "zblock.h"

int main(void)
{
	/// simulation settings and initialization
	double Ts = 0.001;
	double T_sim = 5;

	double y_target = 0.0;
	double y_actual = 0.0;
	double u_control = 0.0;

	double y_step = 100.0;
	double T_step_1 = 0.1;
	double T_step_2 = 2.0;
	
	// create new controller
	zpidParams p = {
		.Kp = 1.0,
		.Ki = 5.0,
		.Kd = 1.0,
		.Kb = 0.1,
		.N = 100.0,
		.output_min = 0.0,
		.output_max = 100.0,
		.Ts = Ts,
		.int_init = 0.0,
		.deriv_init = 0.0,
		.int_method = Z_METHOD_TRAP,
		.deriv_method = Z_METHOD_TRAP
	};
	zpid C = zpidCreate(p);

	// create plant model as a first order system with specified time constant
	double tau = 0.1;
	zblock G = zblockCreate(0.0, Ts, tau, Z_TYPE_FIRSTORDER, Z_METHOD_STD);

	// loop init
	double num_ticks = T_sim / Ts + 1;
	double step_1_ticks = T_step_1 / Ts;
	double step_2_ticks = T_step_2 / Ts;

	// loop
	for (int i = 1; i <= num_ticks; i++) {

		// step input
		if (i >= step_1_ticks)
			y_target = y_step;
		if (i >= step_2_ticks)
			y_target = 0.0;

		// update PID
		u_control = zpidUpdate(C, y_target, y_actual);

		// update plant
		y_actual = zblockForwardStep(G, u_control);

		// print plant model output
		printf("%2.6f\n", y_actual);

	}

	// clean-up
	zblockDestroy(G);
	zpidDestroy(C);

	return 0;
}
