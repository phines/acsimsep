#ifndef EVENT_H
#define EVENT_H

#include <string>
#include "../myxml/myxml.hpp"
#include "PowerElement.hpp"
#define EVENT_ALL_LOCATIONS -99
#define EVENT_EPS 1e-12

enum event_type_e {EVENT_START=0, NO_EVENT_TYPE=1, EVENT_FINISH=1000, // admin opps, (finish gets a large number to make sure it sorts to the end
                   REMOVE_BRANCH=10, REMOVE_GEN=11, REMOVE_LOAD=12, RESTORE_BRANCH=13, RESTORE_GEN=14, RESTORE_LOAD=15, // binary opps
                   INCREASE_GEN=20,  DECREASE_GEN=21,  SCALE_GEN=22,  PERTURB_GEN=23,  // continuous opps on gen
                   INCREASE_LOAD=25, DECREASE_LOAD=26, SCALE_LOAD=27, PERTURB_LOAD=28, // continuous opps on load
                   RUN_OPF=30, RUN_SCOPF=31, RUN_PF=32, RUN_SE=33, RUN_MPC=34, // global operator-initiated opps
                   CHANGE_STEP_SIZE=40
                   }; 

/// struct event_t
///  A simple structure / class to hold event data
///  that can be used to represent external events
///  for a simulation.
///  The following events are feasible:
///    start  - begin the simulation with t_0 = time
///    finish - finish the simulation at the specified time
///    remove_branch  - remove branch number=location at time
///    remove_gen     - remove gen number=location at time
///    remove_load    - remove load number=location at time
///    restore_branch - remove branch number=location at time
///    restore_gen    - remove gen number=location at time
///    restore_load   - remove load number=location at time
///    increase_gen   - increase gen number=location by amount
///    increase_load  - increase load number=location by amount (P and Q will be increased proportionally)
///    decrease_gen   - decrease gen number=location by amount
///    decrease_load  - decrease load number=location by amount (P and Q will be increased proportionally)
///    scale_load     - multiply the load by the fraction (scalar) given in amount
///    perturb_load   - multiply the load by a normally distributed random number with sigma=amount, and mu=1
///    run_opf        - run an optimal power flow routine, and re-dispatch generation
///    run_pf         - run a power flow
///    run_scopf      - run security constrained power flow (doesn't do anything currently)
///    run_se         - run a state estimation (doesn't do anything)
///    run_mpc        - run an MPC curtailment problem (doesn't do anything currently)
///
/// In most of the above cases location can be set to the constant ALL_LOCATIONS to 
class event_t : public power_element_t {
	public: 
		// data members:
		event_type_e type;     ///< the event type
		double       time;     ///< the time at which the event occurs
		int          location; ///< an integer indicating the location of the event
		double       amount;   ///< a scalar indicating the magnitude of the event (only relevent for some events)
		bool         implemented; ///< set to true after the event has been implemented in a simulation
		//// function members:
		/// constructor
		event_t() { type=NO_EVENT_TYPE; time=0; location=EVENT_ALL_LOCATIONS; amount=0; implemented=false; };
		/// read event data from an xml node
		bool read( MyXmlNode & event_node );
		/// write event data to an xml formatted string
		bool write( std::string &data );
		/// print
		void print();
		//// comparison operators...
		/// operator< - events compare to one another based upon their time and type:
		bool operator< (const event_t &other) const { if (*this==other) return type<other.type; else return time<other.time; };
		/// operator> - events compare to one another based upon their time and type:
		bool operator> (const event_t &other) const { if (*this==other) return type>other.type; else return time>other.time; };
		/// operator== - events compare to one another based upon their time:
		bool operator==(const event_t &other) const { return fabs(time-other.time)<EVENT_EPS; };
};

typedef element_list_t<event_t> event_list_t;

#endif

