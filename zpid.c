/**********************************************************************************************
zpid.c
Author: Brendan Till

Notes:
- BACKPROP_LOOP_TOL and BACKPROP_MAX_ITERS worked for the cases I tested. May need to be tweaked.
- zpid_t, an incomplete structure type in zpid.h, is completed here. This was done to hide the
	structure's contents from clients, to avoid corruption.
 **********************************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "zpid.h"

#define BACKPROP_LOOP_TOL 0.001
#define BACKPROP_MAX_ITERS 1000

typedef struct {
	double target;
	double actual;
	double error;
	double P_out;
	double I_out;
	double D_out;
	double B_out;
	double int_out;
	double deriv_out;
	double sum_I;
	double sum_sat;
	double sat_in;
	double output;
	zblock integrator;
	zblock derivative;
} zpidSignals;

// incomplete type in header that is now completed here in source
struct zpid_t {
	zpidParams p;
	zpidSignals s;
	int ticks;
};

// static function prototypes
static double saturate(double u, double output_min, double output_max);

zpid zpidCreate(zpidParams p)
{
	zpid C = malloc(sizeof(struct zpid_t));
	
	C->p = p;
	C->ticks = 0;
	C->s.integrator = zblockCreate(C->p.int_init, C->p.Ts, 1.0, Z_TYPE_INT, C->p.int_method);
	C->s.derivative = zblockCreate(C->p.deriv_init, C->p.Ts, C->p.N, Z_TYPE_DER, C->p.deriv_method);

	return C;
}

void zpidDestroy(zpid C)
{
	free(C);
}

void zpidSetParams(zpid C, zpidParams p)
{
	C->p = p;
}

zpidParams zpidGetParams(zpid C)
{
	return C->p;
}

double zpidUpdate(zpid C, double target, double actual)
{
	// error
	C->s.target = target;
	C->s.actual = actual;
	C->s.error = C->s.target - C->s.actual;

	// PID gains
	C->s.P_out = C->p.Kp * C->s.error;
	C->s.I_out = C->p.Ki * C->s.error;
	C->s.D_out = C->p.Kd * C->s.error;

	// derivative block
	C->s.deriv_out = zblockForwardStep((C->s.derivative), C->s.D_out);

	// integral with backpropagation
	// - the do-while loop is here to handle the algebraic loop caused by the back-propagation feature
	// - a hardcoded tolerance and maximum number of iterations are in place
	double B_out_last;
	int num_backprop_iters = 0;
	do {
		B_out_last = C->s.B_out;

		// back step integrator after first pass
		if (num_backprop_iters > 0)
			zblockBackStep((C->s.integrator));

		// step forward
		C->s.int_out = zblockForwardStep((C->s.integrator), C->s.I_out + B_out_last);

		// total controller output before saturation block
		C->s.sat_in = C->s.P_out + C->s.int_out + C->s.deriv_out;

		// saturation and back prop
		C->s.output = saturate(C->s.sat_in, C->p.output_min, C->p.output_max);
		C->s.sum_sat = C->s.output - C->s.sat_in;
		C->s.B_out = C->p.Kb * C->s.sum_sat;

		num_backprop_iters++;

	} while (fabs(C->s.B_out - B_out_last) > BACKPROP_LOOP_TOL && num_backprop_iters < BACKPROP_MAX_ITERS);

	C->ticks++;

	return C->s.output;
}

static double saturate(double u, double output_min, double output_max)
{
	double y;

	if (u > output_max)
		y = output_max;
	else if (u < output_min)
		y = output_min;
	else
		y = u;

	return y;
}
