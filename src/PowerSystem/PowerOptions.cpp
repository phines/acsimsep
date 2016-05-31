#include "PowerOptions.hpp"

power_options_t::power_options_t()
{
	max_pf_iterations = 20;
	pf_eps = 1e-9;
	print_level = NO_PRINT;
	baseFrequency = 60;
	MPC_delta_t = 0.5;
	MPC_discount_rate = 0.1;
	MPC_reduction_rate = 0.05;
	MPC_voltage_cost = 1e6;
	MPC_current_cost = 1e9;
	MPC_voltage_change_cost = 1000/0.01;
	MPC_branch_include_thresh = 0.6;
	MPC_br_thresh = MPC_branch_include_thresh;
	retry_power_flow = false;
	read_file( "PowerSystem.opt", false );
}
