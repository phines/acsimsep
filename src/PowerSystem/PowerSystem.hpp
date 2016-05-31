#ifndef POWER_H
#define POWER_H

#include <vector>
#include <string>
#include <set>

#include "../utilities/graph.hpp"
#include "../matrix/matrix.hpp"
#include "../myxml/myxml.hpp"
#include "bus.hpp"
#include "branch.hpp"
#include "gen.hpp"
#include "load.hpp"
#include "event.hpp"
#include "PowerOptions.hpp"
#include "Sim_Solver.hpp"

/// \mainpage Paul Hines' Power System simulation software
///  The PowerSystem class provides some basic functionality for simulating bulk electrical power networks.
///  The doxygen pages should provide some indication of how to use the class. Better documentation
///  will be provided in the future (perhaps, I hope...).
class PowerSystem {
	public:
		PowerSystem(); ///< constuctor
		PowerSystem(const char *filename) { PowerSystem(); read_file(filename,POWER_XML); }
		~PowerSystem(); ///< destructor
		/// calculate the voltage magnitudes, angles, etc using a Newton Power Flow algorithm
		bool runPowerFlow( bool use_load_shedding_power_flow=false );
		/// new power flow algorithm
		bool newPowerFlow( bool use_load_shedding_power_flow=false );
		/// runSimulation - run the simulation specified in the event structure
		bool runSimulation() { return Sim_.run(this); }
		/// sim_init - initialize a simulation
		bool sim_init( char *logFileName=NULL ) { return Sim_.init(this, logFileName); }
		/// sim_step() - take one step toward the finish of a simulation
		/// @param dt  - the step size
		/// @param msg - (output) a text description of what happened
		bool sim_step( double dt, std::string &msg ) { return Sim_.step(dt,msg); }
		bool sim_step();
		/// sim_runtime() - return the total time that the simulation will run
		double sim_runtime() { return Sim_.runtime(); }
		/// sim_t() - return the current time in the simultion
		double sim_t() { return Sim_.t(); };
		/// SolveOPF - Solves an Optimal Power Flow problem using Ipopt/Bonmin, implements the results in the data structure
		bool SolveOPF();
		/// Solve the MPC problem using the old formulation
		/// @param dPg (output) is a matrix of changes to generator real output power
		/// @param dPd (output) is a matrix of changes to demand (Q changes proportionately)
		bool SolveMPC( Sparse &dPg, Sparse &dPd );
		/// Solve the MPC problem using the new formulation
		/// @param dPg (output) is a matrix of changes to generator real output power
		/// @param dPd (output) is a matrix of changes to demand (Q changes proportionately)
		/// @param dVg (output) is a matrix of changes to demand (Q changes proportionately)
		bool SolveMPC( Sparse &dPd, Sparse &dPg, Sparse &dVg );
		/// with output vars:
		bool SolveMPC( Sparse &dPd, Sparse &dPg, Sparse &dVg, SparseVector &dVmag, SparseVector &dImag, double &dTheta_slack );
		/// control - implements the control actions calculated by SolveMPC
		bool control( Sparse &dPd, Sparse &dPg, Sparse &dVg );
		/// adjust state vars
		void adjust_state_vars( SparseVector &dVmag, SparseVector &dImag );
		/// removeBranch takes a branch out of service (updates the Ybus matrix)
		void removeBranch( int branch_number );
		/// restoreBranch puts a branch back in service (updates the Ybus matrix)
		void restoreBranch( int branch_number );
		/// check to see if there are multiple islands within the network
		bool is_islanded() { return not PowerGraph_.is_fully_connected(); }
		/// check to see if the system has stressed components (low voltages, high currents)
		bool is_stressed();
		//// functions for reading and writing data:
		/// read power network data from a file
		bool read_file( const std::string & FileName, data_format_e format=POWER_XML ); ///< read data from FileName
		/// write power network data to a file
		bool write_file( const char* FileName, data_format_e format=POWER_XML ); ///< write data to  FileName
		/// read power network data from a string
		bool read( const std::string& data, data_format_e format=POWER_XML );
		/// write power network data to a string
		bool write( std::string& data, data_format_e format=POWER_XML );
		/// write some of the more important and recent state variable data.
		///  Any state data that is more recent than threshold time will be added to the model
		void write_recent_data( std::string &data, double threshold_time );
		//// functions that return the number of elements in the network
		unsigned nBus()    const { return bus.size();    } ///< return the number of buses in the network
		unsigned nGen()    const { return gen.size();    } ///< return the number of generators in the network
		unsigned nLoad()   const { return load.size();   } ///< return the number of loads in the network
		unsigned nBranch() const { return branch.size(); } ///< return the number of branches in the network
		unsigned nEvent()  const { return event.size();  } ///< return the number of simulation events in the network model
		Sparse_cx& Ybus()  { return Ybus_; }; ///< return a reference to the Ybus matrix
		DenseVector_cx& Sbus()  { return Sbus_; }; ///< return a reference to the complex bus injections
		Dense&     PTDF_loads() { return PTDF_loads_; }
		Dense&     PTDF_gens()  { return PTDF_gens_; }
		Sparse&    Jac()   { return Jac_;  }; ///< return a reference to the current Jacobian
		unsigned   ref()   { return bus(refNo_).ix(); } ///< return the index (not the number) of the reference (swing, slack) bus
		int        refNo() { return refNo_; }           ///< return the ref. bus number
		/// return the number of generators that are on
		unsigned nGenOn() {
			unsigned count=0;
			for(unsigned i=0;i<nGen();i++) if( gen[i].Pg>0 and gen[i].status==ON ) count++;
			return count;
		}
		/// return the number of loads that are on
		unsigned nLoadOn() {
			unsigned count=0;
			for(unsigned i=0;i<nLoad();i++) if( std::abs(load[i].Pd)>1e-3 and load[i].status==ON ) count++;
			return count;
		}
		/// reset clears out all of the data in the network
		void reset() { branch.clear(); bus.clear(); gen.clear(); load.clear(); baseMVA=100; };
		bool check();  ///< performs a simple check on the data.  Returns true if things look OK.
		///< Write a human-readable form of the network data to @param output
		void print( std::string &output ) const;
		void print() const;  ///< print the power network data to stdio
		/// update the network data (Ybus, PTDF, etc)
		bool update();
		/// update the bus types
		void updateBusTypes();
		/// update the index variables
		void updateIndexVars();
		/// update the PowerGraph
		void updatePowerGraph();
		
		void clear();  ///< clear out the power network data
		void invalidate() { Ybus_valid_=false; PTDF_valid_=false; }
		/// neighbor_sets -- calculates a set of neighbor bus numbers extending out r1 and r2 from bus_no
		void neighbor_sets( int bus_no , unsigned r1, unsigned r2, std::vector< std::set<int> > &neighbors );
		/// sererate the power system into connected sets of buses
		/// @return the number of islands found (0 means that only the primary connected component was found)
		/// @param islands (output) a vector of power systems on which to run the problem.
		///  this output is only valid when the
		int findIslands( std::vector< PowerSystem > &islands );
		/// take a subset of the input power system and put it into the current model
		void subset( PowerSystem &InputSys, std::set<int> &busNos );
		/// return a reference to the power graph object
		Graph& PowerGraph() { return PowerGraph_; }
		/// set the voltages to 1.0 /_ 0.0
		void set_flat_start();
		/// set the voltages, currents, powers to zero
		void set_blackout();
		/// read data from another power system to the local one
		void read( PowerSystem & ps );
		/// scale the power system load and gen by the specified factor
		void scale( double scale_factor );
		
		/// build the power flow jacobian
		///  @param Jac is the output
		///  @param x is a vector of voltage angles (radians) and magintudes: [Vang;Vmag]
		///  @param row_index gives the set of Jac rows that should be calculated
		///  @param col_index gives the set of Jac cols that should be calculated
		bool MakePowerFlowJac( Sparse &Jac, DenseVector &x=EmptyDenseVector_,
		                       std::vector<unsigned> &row_index=EmptyIntVector_,
		                       std::vector<unsigned> &col_index=EmptyIntVector_ ) ;
		int print_violations( std::string & output, BranchRateNo_e rate=RATE_B );
		int print_violations();
		/// change the reference bus
		void change_ref_bus( int busNo );
		bool calcYbus( bool force_update=false );  ///< update the Ybus matrix
		//// public data members:
		bus_list_t      bus;    ///< the array of bus data structures (see bus.h for details)
		gen_list_t      gen;    ///< the array of gen data structures (see gen.h for details)
		branch_list_t   branch; ///< the array of branch data structures (see branch.h for details)
		load_list_t     load;   ///< the array of load data structures (see load.h for details)
		event_list_t    event;  ///< the array of simulation event structures (see event.h for details)
		double          baseMVA; ///< the per unit MVA base used for the system
		double          baseFrequency; ///< the base frequency of the system (probably 50 or 60; Hz is implied)
		std::string     description;   ///< a short description of the network
		power_options_t options;       ///< some options.  See power_options.h for details.
		std::string     dataFileName;  ///< the name of the data file used as an input
	private:
		Sparse_cx Ybus_; ///< the Ybus matrix
		Sparse_cx Yf_;   ///< a matrix for calculating from end branch flows
		Sparse_cx Yt_;   ///< a matrix for calculating to end branch flows
		Sparse_cx Ybr_;  ///< a matrix for calculating branch flows without getting particularly accurate results for either end
		Sparse    Jac_;  ///< a matrix to hold the system Jacobian for power-flow calculations
		DenseVector_cx Sbus_; ///< the vector of complex bus injections
		DenseVector_cx V_;    ///< a local vector of complex voltages
		static DenseVector EmptyDenseVector_; ///< a temporary dense vector object 
		static std::vector<unsigned> EmptyIntVector_;   ///< a temporary unsigned int vector object
		double ufls_factor_; ///< used for the under-frequency load shedding version of the power flow
		int nIslands_; ///< after an update, indicates the number of islands in the system (0 -> fully connected, 1->2 subgraphs...)
		//// the Power Transfer Distribution Factor matrices (one standard, one for loads, one for gens)
		Dense PTDF_;       ///< the Power Transfer Distribution Factor matrix
		Dense PTDF_loads_; ///< (nBranch X nLoad )
		Dense PTDF_gens_;  ///< (nBranch X nGen  )
		int   refNo_;     ///< the reference bus number
		bool  Ybus_valid_; ///< a flag to indicate whether the Ybus matrix is valid or not
		bool  PTDF_valid_; ///< a flag to indicate whether the PTDF matrix is valid or not
		//// Solver class members
		//PF_Solver   PF_;   ///< a class which contains power flow related calculation stuff
		Sim_Solver  Sim_;  ///< a class which holds dynamic simulation stuff
		//SMP_Solver  SMP_;  ///< a class which solves a Stress Management Problem using MPC_Solver
		Graph PowerGraph_; ///< a graph representation of the network
		bool is_blackout_; ///< indicate if the entire system is a blackout
		
		// private function members
		bool calcSbus( DenseVector_cx & Sbus );  ///< update the Sbus vector
		bool calcSbus();  ///< update the Sbus vector
		bool calcPTDF();  ///< update the PTDF matrix
		bool calcV( DenseVector_cx &V ); ///< valculate the voltages and put them in V
		/// calculate the mismatch
		double calcMismatch( DenseVector&, DenseVector&, const std::vector<unsigned>& = std::vector<unsigned>(0) );
		/// calculate the power flows
		bool calcFlows();
		void insert( bus_t    ); ///< inserts a bus into the model
		void insert( gen_t    ); ///< inserts a gen into the model
		void insert( branch_t ); ///< inserts a branch into the model
		void insert( load_t   ); ///< inserts a load into the model
		void insert( event_t  ); ///< inserts an event into the model
		/// a simple error function
		void error( const char *msg ) const { printf("%s\n", msg); throw msg; };
		/// a conditional error function
		void err_if( bool condition, const char* msg="error") const { if(condition) error(msg); };
		/// read data in matpower format
		bool read_matpower( const std::string & data );
		/// read data in xml format
		bool read_xml( const std::string & data );
		/// read data in xml format
		bool read_xml( MyXmlDocument & doc );
		/// read data from a file in xml format
		bool read_xml_file( const std::string & filename );
		/// read data from a matpower file
		bool read_matpower_file( const std::string & filename );
		/// write data in matpower format
		bool write_matpower( std::string &output );
		/// write data in xml format
		bool write_xml( std::string &output );
		/// power_flow_finish - some simple calculations to perform after solving the power flow equations
		void power_flow_finish( DenseVector& x, DenseVector& mismatch, const std::vector<unsigned>& index );
};

#endif
