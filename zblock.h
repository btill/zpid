/**********************************************************************************************
zblock.h
Author: Brendan Till

zblock is module that handles simple discrete-time systems and their solutions in time.
Integral, derivative, and first order systems are currently supported, with various
solver options available (e.g. forward euler, backward euler, trapezoidal) as well as
derivative filtering.

Summary of functionality:
	Initialization and parameter definition:
		zblockCreate
	Update:
		zblockForwardStep
		zblockBackStep
	Clean-up:
		zblockDestroy

Fundamental data types:
	zblock
		- A pointer to a structure used for internal (hidden) calculations
		- Definition as pointer is implicit, so no need to use address operator (&) when 
			passing as argument to a function
		- Example usage:
			// create first order system
			zblock G = zblockCreate(0.0, 0.001, 0.1, Z_TYPE_FIRSTORDER, Z_METHOD_STD);
			
			// update
			y_actual = zblockForwardStep(G, u_control);

			//destroy
			zblockDestroy(G);

Dependencies:
	- None

 **********************************************************************************************/
#ifndef ZBLOCK_H
#define ZBLOCK_H

// enums for block type and solver method
typedef enum {Z_TYPE_INT, Z_TYPE_DER, Z_TYPE_FIRSTORDER} zblockType;
typedef enum {Z_METHOD_STD, Z_METHOD_FE, Z_METHOD_BE, Z_METHOD_TRAP} zblockMethod;

/*  
 * Type 'zblock' is a pointer to a structure with tag 'zblock_t'
 * This is an incomplete type definition, in order to hide the contents of the structure
 * 	from clients (to avoid corruption)
 * The structure is fully defined in zblock.c 
 */
typedef struct zblock_t *zblock;

/**********************************************************************************************
zblockCreate    Creates a new block given set of parameters
	inputs:	
		double x0 : initial condition for internal state
		double Ts : sample time in [sec]
		double K : gain
			For an integral block, this is a multiplier on the output
			For a derivative block, this is a filtering parameter. As N -> inf, behavior approaches
				ideal unfiltered derivative
			For a first order block, this is a time constant in [sec]
		zblockType type : 
			Z_TYPE_INT: Integral
			Z_TYPE_DER: Derivative
			Z_TYPE_FIRSTORDER: First order system
		zblockMethod method :
			Z_METHOD_STD: Standard, used for a dervative block type only
			Z_METHOD_FE: Forward Euler. Best for small sampling times. Large sampling times can lead to instability.
			Z_METHOD_BE: Backward Euler. Guarenteed stability if corresponding continous-time system is stable.
			Z_METHOD_TRAP: Trapezoidal. Closet match in the frequency domain to the corresponding continuous-time
				system. Guarenteed stability if corresponding continous-time system is stable. 
	outputs:
		zblock : A pointer to a zblock_t structure, whos internals are hidden
 **********************************************************************************************/
zblock zblockCreate(double x0, double Ts, double K, zblockType type, zblockMethod method);

/**********************************************************************************************
zblockForwardStep    Steps discrete system forward in time, given an input
	inputs:	
		zblock D : A pointer to a zblock_t structure. Contents are modified by this call.
		double uk : Input to system at time current time step k
	outputs:
		double : Output at current time step k
 **********************************************************************************************/
double zblockForwardStep(zblock D, double uk);

/**********************************************************************************************
zblockBackStep    Steps discrete system backwards in time
	inputs:	
		zblock D : A pointer to a zblock_t structure. Contents are modified by this call.
	outputs:
		none
 **********************************************************************************************/
void zblockBackStep(zblock D);

/**********************************************************************************************
zblockDestroy    Frees up memory held by zblock
	inputs:	
		zblock D : A pointer to a zblock_t structure. Contents are destroyed by this call
	outputs:
		none
 **********************************************************************************************/
void zblockDestroy(zblock D);

#endif /* ZBLOCK_H */