#ifndef POWER_OPTIONS_H
#define POWER_OPTIONS_H

#include "../utilities/options.hpp"

/// PowerOptions.hpp
/// Defines an options structure that is used in class PowerSystem

enum print_level_e { NO_PRINT=0, MINIMAL=1, VERBOSE=2 };

class power_options_t : public options_t
{
	public:
		/// set the default options
		power_options_t();
		/// the maximum number of newton step iterations to use in power flow
		unsigned max_pf_iterations;
		/// the convergence epsilon for the newton power flow
		double pf_eps;
		/// the amound of printing to do. Can be either NO_PRINT, MINIMAL, or VERBOSE
		///  though none of the levels really print that much stuff.
		print_level_e print_level;
		/// the system frequency
		double baseFrequency;
		/// the time step size for the MPC problem in seconds. Default is 0.5 seconds.
		double MPC_delta_t;
		/// A threshold that tells the MPC solve when to include branches in the problem. 
		/// When MPC_br_thresh=0.5, variables will be included when the branch is at or above
		/// 50% of its limit.  Default is 0.8.
		double MPC_br_thresh;
		/// The discont rate used in the MPC problem formulation
		double MPC_discount_rate;
		/// The rate at which violations should be reduced during the solution process.
		/// The default is 0.05, which means that the violation will be reduced by 5% at each
		/// iteration.  For example if the branch limit is 100 and the current flow is 110, the 
		/// default reduction_rate will make the problem try to reduce the flow by 5 at each time step.
		double MPC_reduction_rate;
		/// The cost assigned to persistent overcurrents in the MPC problem
		double MPC_current_cost;
		/// The cost assigned to persistent under/overvoltages in the MPC problem
		double MPC_voltage_cost;
		/// the cost of changing a voltage set point
		double MPC_voltage_change_cost;
		/// the 
		double MPC_branch_include_thresh;
		/// 
		bool retry_power_flow;
};

#endif
