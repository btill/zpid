/**********************************************************************************************
zblock.c
Author: Brendan Till

Notes:
- zblock_t, an incomplete structure type in zblock.h, is completed here. This was done to hide the
	structure's contents from clients, to avoid corruption.
 **********************************************************************************************/

#include <stdlib.h>
#include "zblock.h"

// incomplete type in header that is now completed here in source
struct zblock_t {
	double uk; 				// input, time k
	double yk; 				// output, time k
	double xk; 				// state, time k
	double xk1;				// state, time k+1
	double x0; 				// intial state, time 0
	double Ts; 				// sample time [s]
	double K;  				// gain
	zblockType type;   		// block type (Z_TYPE_)
	zblockMethod method;	// discrete step method (Z_METHOD_)
};

// static function prototypes
static double forwardStepIntegration(zblock D, double uk);
static double forwardStepDerivative(zblock D, double uk);
static double forwardStepFirstOrder(zblock D, double uk);

zblock zblockCreate(double x0, double Ts, double K, zblockType type, zblockMethod method)
{
	zblock D = malloc(sizeof(struct zblock_t));

	D->uk = 0.0;
	D->yk = 0.0;
	D->xk = 0.0;
	D->xk1 = x0;
	D->x0 = x0;
	D->Ts = Ts;
	D->K = K;
	D->type = type;
	D->method = method;

	return D;
}

void zblockDestroy(zblock D)
{
	free(D);
}

double zblockForwardStep(zblock D, double uk) 
{
	double yk;
	switch (D->type) {
		case Z_TYPE_INT:
			yk = forwardStepIntegration(D, uk);
			break;
		case Z_TYPE_DER:
			yk = forwardStepDerivative(D, uk);
			break;
		case Z_TYPE_FIRSTORDER:
			yk = forwardStepFirstOrder(D, uk);
			break;
		default:
			yk = 0.0;
			break;
	}
	return yk;
}

void zblockBackStep(zblock D)
{
	D->xk1 = D->xk;
}

static double forwardStepIntegration(zblock D, double uk)
{
	D->uk = uk;
	D->xk = D->xk1;

	switch (D->method) {
		case Z_METHOD_FE:
			D->yk = D->xk;
			D->xk1 = D->xk + D->K * D->Ts * D->uk;
			break;
		case Z_METHOD_BE:
			D->yk = D->xk + D->K * D->Ts * D->uk;
			D->xk1 = D->yk;
			break;
		case Z_METHOD_TRAP:
			D->yk = D->xk + D->K * D->Ts / 2.0 * D->uk;
			D->xk1 = D->yk + D->K * D->Ts / 2.0 * D->uk;
			break;
		default:
			D->yk = D->uk;
			D->xk1 = D->xk;
			break;
	}
	return D->yk;
}

static double forwardStepDerivative(zblock D, double uk)
{
	D->uk = uk;
	D->xk = D->xk1;

	// K here is the derivative filter gain, N (rdefined for clarity)
	double N = D->K;

	switch (D->method) {
		case Z_METHOD_STD:
			D->yk = (1.0 / D->Ts) * D->uk + D->xk;
			D->xk1 = (-1.0 / D->Ts) * D->uk;
			break;
		case Z_METHOD_FE:
			D->yk = D->xk + N * D->uk;
			D->xk1 = (1.0 - N * D->Ts) * D->yk - N * D->uk;
			break;
		case Z_METHOD_BE:
			D->yk = N / (1.0 + N * D->Ts) * D->uk + D->xk;
			D->xk1 = (D->yk - N * D->uk) / (1.0 + N * D->Ts);
			break;
		case Z_METHOD_TRAP:
			D->yk = 2.0 * N / (2.0 + N * D->Ts) * D->uk + D->xk;
			D->xk1 = ((1.0 - N * D->Ts / 2.0) * D->yk - N * D->uk) / (1.0 + N * D->Ts / 2.0);
			break;
		default:
			D->yk = D->uk;
			D->xk1 = D->xk;
			break;
	}
	return D->yk;
}

static double forwardStepFirstOrder(zblock D, double uk)
{
	D->uk = uk;
	D->xk = D->xk1;

	// K here is a first order time constant, tau (redefined for clarity)
	double tau = D->K;

	D->yk = D->Ts / (tau + D->Ts) * D->uk + D->xk;
	D->xk1 = tau / (tau + D->Ts) * D->yk;

	return D->yk;
}
