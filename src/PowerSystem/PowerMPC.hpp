#ifndef POWER_MPC_H
#define POWER_MPC_H

#include <vector>
#include <set>
#include <map>
#include "../matrix/matrix.hpp"

typedef class MPC_Solver;
typedef class PowerSystem;
typedef struct bus_t;
typedef struct gen_t;
typedef struct load_t;
typedef struct branch_t;

// POWER_MPC_MAX_K is the maximum number of control steps that will be taken
#define POWER_MPC_MAX_K 10

/// PowerMPC
/// A simple class to contain the power system MPC solver code.
class PowerMPC {
	public:
		/// constructor
		PowerMPC();
		/// Solve the MPC problem
		/// @param PS    (input) is the power system object
		/// @param dPg   (output) is a matrix of changes to generator real output power
		/// @param dPd   (output) is a matrix of changes to demand (Q changes proportionately)
		/// @param dVg   (output) is a matrix of changes to generator voltage set points
		/// @param dVmag (output) gives predicted changes to bus voltages
		/// @param dImag (output) gives predicted changes to branch currents
		/// @param dTheta_slack (output) gives predicted changes to the slack bus phasor
		bool solve( PowerSystem &PS, Sparse &dPd, Sparse &dPg, Sparse &dVg, SparseVector &dVmag, SparseVector &dImag, double &dTheta_slack );
		
		/// a simpler version of the above for compatibility
		bool solve( PowerSystem &PS, Sparse &dPd, Sparse &dPg, Sparse &dVg );
		
		/// Solve the MPC problem using the old formulation
		/// @param PS  (input) is the power system object
		/// @param dPg (output) is a matrix of changes to generator real output power
		/// @param dPd (output) is a matrix of changes to demand (Q changes proportionately)
		bool solve_old ( PowerSystem &PS, Sparse &dPg, Sparse &dPd );
		
	private:
		//// local functions:
		/// set the control variable limits and costs
		void set_control_vars ( DenseVector & u_min,   DenseVector & u_max,
								DenseVector & du_min,  DenseVector & du_max,
								DenseVector & c_u_inc, DenseVector & c_u_dec,
								Dense & u );
		/// set the state variable limits and costs
		void set_state_vars( Dense & x_min,  Dense & x_max, DenseVector & c_x_low, DenseVector & c_x_high, Dense & x );
		/// build the dynamic constraints x_{k+1} = A x_k + B u_k
		void build_dynamic_cons ( Sparse &A, Sparse &B, Sparse &W );
		/// calculate the size of the problem
		///  returns K
		///  sets nBus_, nBranch_, nLoad_, nGen_, bus_ix_, gen_ix_, load_ix_, branch_ix_
		unsigned set_problem_size ( PowerSystem &PS );
		/// solve a reduced version of the problem
		bool solve_reduced_problem( MPC_Solver &mpc );
		/// print the predicted state/output variables
		void print_predictions( MPC_Solver &mpc );
		/// extract the solution to the problem
		void extract_solution( Dense &du, Sparse &dPg, Sparse &dPd, Sparse &dVg );
		/// extract the predictions from the state vector
		void extract_predictions( Dense &x, Sparse &dVg, SparseVector &dVmag, SparseVector &dImag, double &dTheta_slack );
		/// check to see if i is a gen bus (if there is a locally controlable gen at the bus)
		bool is_gen_bus( int i );
		/// check to see if i is a load bus (if there is a locally controlable load at the bus)
		bool is_load_bus( int i );
		/// check to see if i is a pq bus
		bool is_pq_bus( int i );
		/// find the control variables connected to the given set of bus numbers
		/// @param bus_numbers is a set of actual bus NUMBERS not the local indeces
		/// @param control_vars (output) gives the set of control variables at the given nodes
		void find_control_vars( std::set<int> & bus_numbers, std::set<int> & control_vars );
				
		//// Return references to the bus elements
		/// return a reference to the i'th bus element in the local data sub-set
		inline bus_t& bus(int i);
		/// return a reference to the i'th bus element in the local data sub-set
		inline gen_t& gen(int i);
		/// return a reference to the i'th bus element in the local data sub-set
		inline load_t& load(int i);
		/// return a reference to the i'th bus element in the local data sub-set
		inline branch_t& branch(int i);
		// some local variables
		PowerSystem *PS_;
		double   baseMVA_;
		unsigned nBus_;    ///< the number of buses in the local problem
		unsigned nBranch_; ///< the number of branches in the local problem
		unsigned nLoad_;   ///< the number of loads in the local problem
		unsigned nGen_;    ///< the number of gens in the local problem
		unsigned K_;       ///< the size of the MPC problem
		unsigned ref_ix_;  ///< the reference bus ix
		unsigned ref_gen_; ///< the index of the reference bus generator
		Dense    Vmin_;    ///< record the minimum voltage for each time step
		Dense    Imax_;    ///< record the max branch current for each time step
		// indeces
		std::vector<unsigned> bus_ix_;     ///< the list of buses in the local problem
		std::set<int> bus_near_limit_set_; ///< a list of buses that are near their limits
		std::vector<unsigned> gen_ix_;     ///< the list of gens in the local problem
		std::vector<unsigned> load_ix_;    ///< the list of loads in the local problem
		std::vector<unsigned> branch_ix_;  ///< the list of branches in the local problem
		// lists of local element bus indexes
		std::vector<unsigned> local_bus_ix_;      ///< used to convert the full system's bus indeces to the local bus indeces
		std::vector<unsigned> gen_at_bus_ix_;     ///< the gen at the given bus index
		std::vector<unsigned> gen_bus_;           ///< the list of generator buses
		std::vector<unsigned> non_gen_at_bus_ix_; ///< the non gen at the given bus index
		std::vector<unsigned> load_at_bus_ix_;    ///< the load at the given bus index
		std::map< int, std::set<int> > control_vars_at_bus_; ///< a record of which control vars are at each bus

		//// OPTION variables
		double bus_include_thresh_;
		double branch_include_thresh_;
		double discount_rate_;
		double voltage_inc_rate_;
		double current_dec_rate_;
		double voltage_stress_cost_;
		double current_stress_cost_;
		double slack_bus_cost_;
		double dVg_max_; ///< maximum absolute change in Vg per time step
		double dPd_max_; ///< maximum absolute change in Pd per time step
		double dPg_max_; ///< maximum absolute change in Pg per time step
		double voltage_change_cost_;
		double current_margin_;
		double voltage_margin_;
		bool   reduce_; 
		int    reduce_distance_; ///< tells us how far the stress var discounting will extend
		bool   use_stress_costs_;
		int    print_level_;
		int    K_max_;
		
};

#endif

