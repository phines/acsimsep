
#ifndef POWER_FLOW_H
#define POWER_FLOW_H

#include "../matrix/matrix.hpp"
#include "PowerGlobals.hpp"
#include <complex>

/// options for the power flow solver
struct PowerFlowOptions
{
	double convergence_eps;
	int max_iterations;
	int print_level;
	bool flat_start;
	int max_line_search_iterations;
	// defaults
	PowerFlowOptions();
};

/// This routine solves the inner loop of the standard Newton-Raphson power flow
///  Returns true if the algorithm converged to a solution
bool SolvePowerFlow( Sparse_cx        &Ybus,     //the Ybus (network admittance) matrix
                     bus_type_e       *BusTypes, // a vector of bux types
                     DenseVector_cx   &Sbus,     // a vector with the bus power injections
					 DenseVector_cx   &V,        // a vector with the bus voltages
					 DenseVector_cx   &Mismatch, // the output mismatch vector
					 PowerFlowOptions options=PowerFlowOptions() );

#endif
