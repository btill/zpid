/**********************************************************************************************
zpid.h
Author: Brendan Till

zpid is a simple module written in C for performing discrete PID control loop calculations.
See README for overview, and main.c for sample usage

Summary of functionality:
	Initialization and parameter definition:
		zpidCreate
		zpidSetParams
		zpidGetParams
	Controller update:
		zpidUpdate
	Clean-up:
		zpidDestroy

Fundamental data types:
	zpidParams
		- A structure used for configuring the controller
		- Passed by value, since it's only used for configuration
		- See "zblock.h" header for definition of zblockMethod
		- Example usage:
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
	zpid
		- A pointer to a structure used for internal (hidden) controller calculations
		- Definition as pointer is explicit, so no need to use address operator (&) when 
			passing as argument to a function
		- Example usage:
			// create controller, given defined parameter structure p)
			zpid C = zpidCreate(p);

			// update controller, given target and actual (feedback) values
			u_control = zpidUpdate(C, y_target, y_actual);

			// clean-up
			zpidDestroy(C);

Dependencies:
	- "zblock.h", a lower-level module handling discrete-time calculations

 **********************************************************************************************/
#ifndef ZPID_H
#define ZPID_H

#include "zblock.h" // included here to specify zblockMethod type in zpidParams structure

/*
 * zpidParams structure definition
 * 	This structure is passed to zpidCreate and zpidSetParams, and returned by zpidGetParams
 *  It contains all the tunable controller parameters
 *  See "zblock.h" header for definition of zblockMethod
 */
typedef struct {
	double Kp;						// proportional gain
	double Ki;						// integral gain
	double Kd;						// derivative gain
	double Kb;						// back-propogation gain (integral anti-windup)
	double N;						// derivative filter parameter
	double output_min;				// output minimum (saturation block)
	double output_max;				// output maximum (saturation block)
	double Ts;						// sample time [seconds]
	double int_init;				// integral calculation initial condition
	double deriv_init;				// derivative calculation initial condition
	zblockMethod int_method;		// integral solver method
	zblockMethod deriv_method;		// derivative solver method
} zpidParams;

/* 
 * Type 'zpid' is a pointer to a structure with tag 'zpid_t'
 * This is an incomplete type definition, in order to hide the contents of the structure
 * 	from clients (to avoid corruption)
 * The structure definition is completed in zpid.c
 * zpid is the fundamental type passed to most functions in this module
 */
typedef struct zpid_t *zpid;

/**********************************************************************************************
zpidCreate    Creates a new controller given a set of parameters
	inputs:	
		zpidParams p : Controller parameter structure
	outputs:
		zpid : A pointer to a zpid_t structure, whos internals are hidden
 **********************************************************************************************/
zpid zpidCreate(zpidParams p);

/**********************************************************************************************
zpidSetParams    Sets parameters for an existing controller
	inputs:	
		zpid C : Pointer to a zpid_t structure. Contents are modified by this call.
		zpidParams p : Controller parameter structure to be written
	outputs:
		none
 **********************************************************************************************/
void zpidSetParams(zpid C, zpidParams p);

/**********************************************************************************************
zpidGetParams    Gets parameters from an existing controller
	inputs:	
		zpid C : Pointer to a zpid_t structure
	outputs:
		zpidParams p : Controller parameter structure
 **********************************************************************************************/
zpidParams zpidGetParams(zpid C);

/**********************************************************************************************
zpidUpdate   Steps the controller forward in time, given a target value and actual value
	inputs:	
		zpid C : Pointer to a zpid_t structure. Contents are modified by this call.
		double target : Target value for controller
		double actual : Actual value, typically from feedback
	outputs:
		double : Controller output value, typically sent to an actuator
 **********************************************************************************************/
double zpidUpdate(zpid C, double target, double actual);

/**********************************************************************************************
zpidDestroy   Frees memory previously held by controller
	inputs:	
		zpid C : Pointer to a zpid_t structure. Contents are destroyed by this call.
	outputs:
		none
 **********************************************************************************************/
void zpidDestroy(zpid C);

#endif /* ZPID_H */