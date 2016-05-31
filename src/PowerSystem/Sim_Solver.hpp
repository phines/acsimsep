#ifndef SIM_SOLVER_H
#define SIM_SOLVER_H

#define DEFAULT_STEP_SIZE 0.01
#define NO_STEP_SIZE -1

#include <cstdio>
#include <string>
#include "../utilities/RNG.hpp"

//#include "PowerSystem.hpp"
typedef class PowerSystem;
typedef class event_t;

/** class Sim_Solver
 * The Sim_Solver class uses the "event" data in the given PowerSystem class to run a time
 * domain simulation of the network using a Quasi-steady-state AC power flow.
*/
class Sim_Solver
{
	public:
		//// methods:
		/// constructor
		Sim_Solver() { t_=-1; k_=-1; dt_=DEFAULT_STEP_SIZE; t_finish_=0; nextEvent_=NULL; PS_=NULL; log_=NULL; };
		/// destructor
		~Sim_Solver() { if (log_!=NULL) fclose(log_); }
		/// init - initializes the simulation
		bool init( PowerSystem *PS, const char *logFileName=NULL );
		/// step - takes one time step. Return false when finished, or if a problem occurs
		/// @param dt provides the time step to use.  If the time step is negative, the step size
		///   given in the event data is used
		/// @param msg - (output) gives a description of what happened
		bool step( double dt, std::string &msg );
		/// a default version of step
		bool step( double dt=NO_STEP_SIZE ) { std::string tmp; return step(dt, tmp); }
		/// log - write the current simulation results to the log
		bool log();
		/// run the full simulation
		bool run ( PowerSystem *PS, char *logFileName=NULL );
		/// runtime
		double runtime() { return (t_finish_ - t_0_); };
		/// t
		double t() { return t_; };
		/// make a note for any components that are stressed
		int note_stressed_components( std::string &note );
		
	private:
		//// local variables
		double t_;  ///< the current time in the simulation
		int    k_;  ///< the current time step number
		double t_0_; ///< the start time of the simulation
		double t_finish_; ///< quitting time
		double dt_; ///< the current step size
		event_t *nextEvent_; ///< a pointer to the next event in the event structure
		PowerSystem *PS_; ///< a pointer to the actual power system data
		FILE *log_; ///< a log in which to store the simulation results
		//// local functions
		/// calculate the discrete dynamics
		void discrete_step();
		/// calculate the continuous dynamics
		void continuous_step();
		/// implement the specified event
		bool implement( int type, int location, double amount, std::string &msg );
		/// write a header to the log file
		void log_header();
		/// write a file with some information about this case
		void print_case_info( const char *logFileName );
		/// A random number generator:
		RNG rng_;
};

#endif
