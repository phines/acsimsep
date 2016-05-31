#include "PowerMPC.hpp"
#include "PowerSystem.hpp"
#include "../MPC_Solver/MPC_Solver.hpp"

#define LARGE_INT 91919191
#define EPS      1e-3
#define MIN_SPAN 0.04
#define K_MAX    10

using namespace std;

/// constructor
PowerMPC::PowerMPC() {
	PS_=NULL;
	// default options
	bus_include_thresh_    = 0.01;
	branch_include_thresh_ = 0.9;
	discount_rate_         = 0.1;
	voltage_inc_rate_      = 0.01;
	current_dec_rate_      = 0.10;
	voltage_stress_cost_   = 1e6;
	current_stress_cost_   = 1e4;
	slack_bus_cost_        = 1e6;
	dVg_max_               = 0.01;
	dPd_max_               = 20.0;
	dPg_max_               = 20.0;
	voltage_change_cost_   = 2e4;
	current_margin_        = 0.5;
	voltage_margin_        = 0.0001;
	reduce_                = true;
	reduce_distance_       = 3;
	use_stress_costs_      = true;
	print_level_           = 0;
	K_max_                 = 1;//POWER_MPC_MAX_K;
}

/// a simple error function
inline static void error(const char *msg) {
	printf("%s\n", msg);
	throw( msg );
}

/// a conditional error function
inline static void err_if(bool condition, const char* msg="error") {
	if(condition) error(msg);
}

/// return the larger of the inputs
inline static double maximum ( const double &a, const double &b ) {
	return a>b ? a : b;
}

/// return the smaller of the inputs
inline static unsigned minimum ( const unsigned &a, const unsigned &b ) {
	return a<b ? a : b;
}

/// return the larger of the inputs
inline static unsigned maximum ( const unsigned &a, const unsigned &b ) {
	return a>b ? a : b;
}

/// return the smaller of the inputs
inline static double minimum ( const double &a, const double &b ) {
	return a<b ? a : b;
}

/// an index used to keep track of stuff within the matrices
struct PowerMPC_Index {
	struct u_index {
		vector<int> Pg;
		vector<int> Vg;
		vector<int> LoadFactor;
		vector<int> Vmag;
		int n;
	} u;
	struct x_index {
		vector<int> theta;
		vector<int> Vmag_pq;
		vector<int> Vmag;
		vector<int> Imag;
		int n;
	} x;
	struct y_index {
		// nothing here for now
		int n;
	} y;
	
	struct w_index {
		// nothing here for now;
		int n;
	} w;
	/// initialize the index variables
	void init( unsigned nBus, set<int> &pq_set, vector<unsigned> & gen_bus, unsigned nLoad, unsigned nBranch ) {
		unsigned i, u_no=0, x_no=0;
		unsigned nGen = gen_bus.size();
		set<int>::iterator iter;
		/// u
		u.Pg.resize(nGen,LARGE_INT);
		u.Vg.resize(nGen,LARGE_INT);
		u.LoadFactor.resize(nLoad,LARGE_INT);
		u.Vmag.resize(nBus,LARGE_INT);
		// Pg
		for (i=0;i<nGen;i++) {
			u.Pg[i] = u_no;
			u_no++;
		}
		// Vg
		for (i=0;i<nGen;i++) {
			u.Vg[i] = u_no;
			u.Vmag[gen_bus[i]] = u_no;
			u_no++;
		}
		// LoadFactor
		for (i=0;i<nLoad;i++) {
			u.LoadFactor[i] = u_no;
			u_no++;
		}
		u.n = u_no;
		/// x
		x.theta  .resize(nBus,         LARGE_INT);
		x.Vmag_pq.resize(pq_set.size(),LARGE_INT);
		x.Vmag   .resize(nBus,         LARGE_INT);
		x.Imag   .resize(nBranch,      LARGE_INT);
		// theta
		for (i=0;i<nBus;i++) {
			x.theta[i] = x_no;
			x_no++;
		}
		// Vmag
		iter = pq_set.begin();
		for (i=0;i<pq_set.size();i++) {
			x.Vmag_pq[i] = x_no;
			x.Vmag[*iter] = x_no;
			x_no++;
			iter++;
		}
		// Imag
		for (i=0;i<nBranch;i++) {
			x.Imag[i] = x_no;
			x_no++;
		}
		x.n = x_no;
		w.n = 0;
		y.n = 0;
		return;
	}
} ix;

bool PowerMPC::solve( PowerSystem &PS, Sparse &dPd, Sparse &dPg, Sparse &dVg ) {
	SparseVector dVmag, dImag;
	double dTheta_slack;
	return PowerMPC::solve( PS, dPd, dPg, dVg, dVmag, dImag, dTheta_slack );
}

/// solve the mpc problem.  This function assumes that ::update() was called before calling this function
///  and that there are no sub-islands within the system.
///  note that it is the responsibility of PowerSystem to resize the outputs and set them to zero
bool PowerMPC::solve( PowerSystem &PS, Sparse &dPd, Sparse &dPg, Sparse &dVg, SparseVector &dVmag, SparseVector &dImag, double &dTheta_slack ) {
	// variables
	MPC_Solver mpc;
	unsigned i, nU, nX, nY, nW;
	bool dataOK=true;
	bool success=true;
	set<int> pq_list;
	
	// 
	PS.invalidate();
	PS.update();
	
	// check for cases for which we should return and do nothing
	for( i=0; i<PS.nBus(); i++ ) {
		if( PS.bus[i].Vmag < 0.1 ) return true;
	}

	// this function assumes that the network was updated before calling
	// make sure that the PS_ pointer is recorded
	if (PS_!=NULL) {
		printf("Attempts to reuse an existing PowerMPC structure cause errors.\n");
		return false;
	}
	PS_ = &PS;
	// calculate the size the problem (in time)
	K_ = set_problem_size( PS );
	// if K==0 optimal action is to do nothing
	if ( K_<=0 ) {
		//printf("No violations, therefore no action.\n");
		return true;
	}
	dTheta_slack = 0;
	// do a couple checks on the data
	if ( nGen_<2 || nLoad_<2 || nBus_<3 ) {
		//printf("nBus: %d %d, nGen: %d %d, nLoad: %d %d\n", PS.nBus(), nBus_, PS.nGen(), nGen_, PS.nLoad(), nLoad_);
		return false; // not enough data to do anything interesting.
	}
	err_if( gen_bus_.size()!=nGen_, "gen_bus_ and nGen_ should be equal.");
	// build the pq_list
	for(i=0;i<nBus_;i++) {
		if( PS.bus[i].type==PQ ) pq_list.insert(i);
	}
	// initialize the index variables and the mpc problem
	ix.init( nBus_, pq_list, gen_bus_, nLoad_, nBranch_ );
	nU = ix.u.n;
	nX = ix.x.n;
	nY = ix.y.n;
	nW = ix.w.n;
	//printf("MPC problem size is: nU=%d, nX=%d, nY=%d, K=%d.\n", nU, nX, nY, K_);
	mpc.set_problem_size( nU, nX, nY, K_ );
	mpc.set_use_du();
	// set limits on control variables
	set_control_vars( mpc.u_min, mpc.u_max, mpc.du_min, mpc.du_max, mpc.c_u, mpc.c_u_dec, mpc.u );
	// set limits on state variables (also sets x0)
	set_state_vars( mpc.x_min, mpc.x_max, mpc.c_x_low, mpc.c_x_high ,mpc.x );
	// build the x_{k+1} = x_k + B*u_k constraints
	build_dynamic_cons( mpc.A, mpc.B, mpc.W );
	mpc.set_A_both_sides();
	// set the discount rate
	mpc.rho() = 1 - PS.options.MPC_discount_rate;
	
	// try and solve the problem
	if ( reduce_ ) {
		success = solve_reduced_problem( mpc );
	} else {
		success = mpc.solve();
	}
	// get the solution
	if ( success ) {
		extract_solution( mpc.du, dPd, dPg, dVg );
		extract_predictions( mpc.x, dVg, dVmag, dImag, dTheta_slack );
		if ( print_level_>1 ) {
			print_predictions( mpc );
		}
		// check for a strange solution with 1 or fewer non-zeros
		if( dPd.nnz() + dPg.nnz() + dVg.nnz() == 1 ) {
			if( print_level_>0 ) {
				printf( "Strange result from PowerMPC::solve.  dTheta_slack = %g\n", dTheta_slack );
			}
		}
	} else {
		if( print_level_>0 ) {
			printf("Infeasible MPC problem.\n");
		}
	}
	return success;
}

void PowerMPC::extract_predictions( Dense &x, Sparse &dVg, SparseVector &dVmag, SparseVector &dImag, double &dTheta_slack ) {
	unsigned i, bus_ix;
	set<int>::iterator iter;
	/// the slack bus theta
	dTheta_slack = x( ix.x.theta[ref_ix_], 1 ) - x( ix.x.theta[ref_ix_], 0 );
	/// fill in the voltages
	for( iter=bus_near_limit_set_.begin(); iter!=bus_near_limit_set_.end(); iter++ ) {
		bus_ix = *iter;
		dVmag.set( bus(bus_ix).number , x( ix.x.Vmag[bus_ix], 1 ) - x( ix.x.Vmag[bus_ix], 0 ) );
	}
	/// fill in the currents
	for( i=0; i<nBranch_; i++ ) {
		dImag.set( branch(i).number, x( ix.x.Imag[i], 1 ) - x( ix.x.Imag[i], 0 ) ) / baseMVA_;
	}
	return;
}

bool PowerMPC::solve_reduced_problem( MPC_Solver &mpc ) {
	unsigned i, ix_x, bi;
	Dense A_hat;
	MPC_Solver mpc_reduced;
	set<int> x_subset;
	set<int> bus_set;
	set<int> all_buses;
	set<int> input_set;
	set<int>::iterator bus_iter;
	vector< set<int> > control_subsets( mpc.n_x() );
	bool success;
	
	/// calculate the control subsets
	// for voltages
	for ( bus_iter=bus_near_limit_set_.begin(); bus_iter!=bus_near_limit_set_.end(); bus_iter++) {
		bi = *bus_iter;
		ix_x = ix.x.Vmag[ bi ];
		x_subset.insert( ix_x );
		bus_set.clear();
		PS_->PowerGraph().find_neighbors( bus(bi).number, reduce_distance_, bus_set, false );
		find_control_vars( bus_set , control_subsets[ix_x] );
		all_buses = set_union( all_buses, bus_set );
	}
	// for currents
	for (i=0;i<nBranch_;i++) {
		ix_x = ix.x.Imag[i];
		x_subset.insert(ix_x);
		input_set.clear();
		input_set.insert( branch(i).fromBusNo );
		input_set.insert( branch(i).toBusNo );
		bus_set.clear();
		PS_->PowerGraph().find_neighbors( input_set, reduce_distance_, bus_set, false );
		find_control_vars( bus_set, control_subsets[ix_x] );
		all_buses = set_union( all_buses, bus_set );
	}
	// and the slack bus variable
	ix_x = ix.x.theta[ref_ix_];
	x_subset.insert(ix_x);
	find_control_vars( all_buses, control_subsets[ix_x] );
	// solve the reduced problem:
	success = mpc.solve_reduced_problem( control_subsets );
	
	return success;
}

// a local variable to be used with find_control_vars
void PowerMPC::find_control_vars( std::set<int> & bus_numbers, std::set<int> & control_vars )
{
	unsigned i;
	set<int>::iterator iter;
	// initialize the table if needed
	if (control_vars_at_bus_.size()==0) {
		// add load vars
		for(i=0;i<nLoad_;i++) {
			control_vars_at_bus_[load(i).busNo].insert( ix.u.LoadFactor[i] );
		}
		// add gen vars
		for(i=0;i<nGen_;i++) {
			control_vars_at_bus_[gen(i).busNo].insert( ix.u.Pg[i] );
			control_vars_at_bus_[gen(i).busNo].insert( ix.u.Vg[i] );
		}
	}
	// fill in the control_vars variable
	control_vars.clear();
	for( iter=bus_numbers.begin(); iter!=bus_numbers.end(); iter++ ) {
		control_vars = set_union( control_vars, control_vars_at_bus_[*iter] );
	}
}

/// extract the solution
void PowerMPC::extract_solution( Dense &du, Sparse &dPd, Sparse &dPg, Sparse &dVg )
{
	unsigned i, k;
	double value;
	// go through du and extract the solution
	for (k=0;k<K_;k++) {
		// load values
		for (i=0;i<nLoad_;i++) {
			value = du( ix.u.LoadFactor[i], k );
			value *= load(i).Pd;
			dPd.set( load(i).number, k, value );
		}
		// gen power values
		for (i=0;i<nGen_;i++) {
			value = du(ix.u.Pg[i],k);
			dPg.set( gen(i).number, k, value );
		}
		// gen voltage values
		for (i=0;i<nGen_;i++) {
			value = du(ix.u.Vg[i],k);
			dVg.set( gen(i).number, k, value );
		}
	}
}

/// calculate the length of the time horizonn and some other variables
///  This should be the only function that uses the PS structure directly
unsigned PowerMPC::set_problem_size( PowerSystem &PS )
{
	unsigned i, k, bi;
	unsigned K, K_voltage=0, K_current=0;
	unsigned nI_Viols=0, nV_Viols=0;
	double Vmag, Vmin, Vmax;
	double Imag, Imax, Imax_extreme, Inorm;
	double step;
	bool   status;
		
	baseMVA_ = PS.baseMVA;
	local_bus_ix_.resize(PS.nBus());
	// prepare bus data stuff
	i=0;
	for (bi=0;bi<PS.nBus();bi++) {
		bus_ix_.push_back(bi); // DO NOT SUBSET BUS LIST HERE!
		local_bus_ix_[bi] = i;
		if (PS.bus[bi].is_ref()) {
			ref_ix_ = i;
		}
		i++;
	}
	nBus_ = bus_ix_.size();
	// set gen_ix_, 
	for (i=0;i<PS.nGen();i++) {
		//status = PS.gen[i].status==ON and PS.gen[i].Pmax>(PS.gen[i].Pmin+EPS) and PS.gen[i].Pg>1.0;
		bool voltages_close = abs( PS.bus(PS.gen[i].busNo).Vmag - PS.gen[i].Vref )<0.001;
		status = PS.gen[i].status==ON and PS.gen[i].Pg>1.0 and voltages_close;
		if ( status ) {
			gen_ix_.push_back(i);
			gen_bus_.push_back(PS.gen[i].bi);
			if (PS.gen[i].bi==ref_ix_) {
				ref_gen_ = gen_ix_.size()-1;
			}
		}
	}
	// set load_ix_
	for (i=0;i<PS.nLoad();i++) {
		status = PS.load[i].Pd>1.0 and PS.load[i].status==ON;
		if ( status ) {
			load_ix_.push_back(i);
		}
	}
	// count the branch violations and set branch_ix_
	for (i=0;i<PS.nBranch();i++) {
		Inorm = PS.branch[i].Inorm();
		if ( Inorm > 1.0 ) nI_Viols++;
		if ( Inorm > branch_include_thresh_ ) {
			branch_ix_.push_back(i);
		}
	}
	
	// set nBus_, nBranch_, nLoad_, nGen_,
	nBus_    = bus_ix_.size();
	nGen_    = gen_ix_.size();
	nLoad_   = load_ix_.size();
	nBranch_ = branch_ix_.size();
	
	// set up the gen load lookup vectors
	gen_at_bus_ix_. resize(PS.nBus(),LARGE_INT);
	load_at_bus_ix_.resize(PS.nBus(),LARGE_INT);
	// fill in the gen references
	for (i=0;i<nGen_;i++) {
		gen_at_bus_ix_[gen(i).bi] = i;
	}
	// figure out the load locs // maybe a problem here if there is a bus subset
	for (i=0;i<nLoad_;i++) {
		load_at_bus_ix_[load(i).bi] = i;
	}
	
	// record the voltage limits
	Vmin_.resize(nBus_,K_MAX,0);
	for (i=0;i<nBus_;i++) {
		// collect bus vars
		Vmag = bus(i).Vmag;
		Vmin = bus(i).Vmin * double(Vmag>0.001);
		Vmax = bus(i).Vmax;
		if (Vmag<Vmin) Vmin += voltage_margin_;
		
		// record the bus if it is near its limit (but not zero)
		if ( Vmag>0.001 and Vmag<(Vmin+bus_include_thresh_) and PS.bus[i].is_pq() ) {
			bus_near_limit_set_.insert(i);
		}
		// record the number of violations
		if ( Vmag>Vmax or Vmag<Vmin ) nV_Viols++;
		// fill the whole thing in with Vmin just to be careful
		for(k=0;k<K_MAX;k++) Vmin_(i,k) = Vmin;
		// if the voltage is excessive, make a gradual decrease
		if (Vmag<Vmin) {
			k = 0;
			Vmin_(i,k) = Vmag;
			while ( Vmin_(i,k)<Vmin and k<K_MAX-1 ) {
				k++;
				Vmin_(i,k) = minimum(Vmin,Vmin_(i,k-1)+voltage_inc_rate_);
			}
			K_voltage = maximum(k,K_voltage);
		}
	}
	// figure out the branch limits
	Imax_.resize(nBranch_,K_MAX,0);
	for (i=0;i<nBranch_;i++) {
		// collect Imag vars
		Imag = branch(i).Imag()*baseMVA_;
		Imax = branch(i).rateB - current_margin_;
		Imax_extreme = branch(i).rateC;
		Inorm = Imag/Imax;
		step = Imax*current_dec_rate_;
		// fill in the Imax_
		for(k=0;k<K_MAX;k++) Imax_(i,k) = Imax;
		// fill in the 
		if (Imag>Imax) {
			k=0;
			Imax_(i,k) = Imag + EPS;
			k=1;
			Imax_(i,k) = minimum( Imax_extreme, maximum(Imax,Imag-step) );
			while ( Imax_(i,k)>Imax and k<K_MAX-1 ) {
				k++;
				Imax_(i,k) = maximum(Imax,Imax_(i,k-1)-step);
			}
			K_current = maximum(k,K_current);
		}
	}
	// calculate K
	K = maximum( K_voltage, K_current );
	return minimum( K, K_max_ );
}

/// return a reference to the i'th bus element in the local data sub-set
inline bus_t& PowerMPC::bus(int i) {
	err_if( i>=(int)nBus_ or i<0, "Out-of-bounds access attempt in PowerMPC::bus(int i)");
	return PS_->bus[bus_ix_[i]];
}

/// return a reference to the i'th bus element in the local data sub-set
inline gen_t& PowerMPC::gen(int i) {
	err_if( i>=(int)nGen_ or i<0, "Out-of-bounds access attempt in PowerMPC::gen(int i)");
	return PS_->gen[gen_ix_[i]];
}

/// return a reference to the i'th bus element in the local data sub-set
inline load_t& PowerMPC::load(int i) {
	err_if( i>=(int)nLoad_ or i<0, "Out-of-bounds access attempt in PowerMPC::load(int i)");
	return PS_->load[load_ix_[i]];
}

/// return a reference to the i'th bus element in the local data sub-set
inline branch_t& PowerMPC::branch(int i) {
	err_if( i>=(int)nBranch_ or i<0, "Out-of-bounds access attempt in PowerMPC::branch(int i)");
	return PS_->branch[branch_ix_[i]];
}

/// set limits and costs for control variables
void PowerMPC::set_control_vars( DenseVector & u_min,   DenseVector & u_max,
                                 DenseVector & du_min,  DenseVector & du_max,
                                 DenseVector & c_u_inc, DenseVector & c_u_dec, Dense & u )
{
	// vars
	unsigned i, ix_u;
	double   Pg, Vg, Pd;
	
	// Pg
	for (i=0;i<nGen_;i++) {
		Pg = gen(i).Pg*gen(i).status;
		ix_u = ix.u.Pg[i];
		// u0
		u(ix_u,0) = Pg;
		// gen limits
		u_min[ix_u]  = 0.0;
		u_max[ix_u]  = Pg;
		// could use ramp rates below, but for now:
		du_min[ix_u]  = -dPg_max_;
		du_max[ix_u]  = 0.0;
		// costs
		if (du_max[ix_u] > 0.0) {
			c_u_inc[ix_u] = gen(i).RUC();
			c_u_dec[ix_u] = gen(i).RDC();
		} else {
			c_u_inc[ix_u] = -gen(i).RDC();
		}
	}
	// Vg
	for (i=0;i<nGen_;i++) {
		Vg = gen(i).Vref;
		ix_u = ix.u.Vg[i];
		// u0
		u(ix_u,0) = Vg;
		// limits
		u_min [ix_u] = bus(gen(i).bi).Vmin;
		u_max [ix_u] = bus(gen(i).bi).Vmax;
		du_min[ix_u] = -dVg_max_;
		du_max[ix_u] = +dVg_max_;
		// costs
		c_u_inc[ix_u] = voltage_change_cost_;
		c_u_dec[ix_u] = voltage_change_cost_;
	}
	// LoadFactor
	for (i=0;i<nLoad_;i++) {
		Pd   = load(i).Pd;
		ix_u = ix.u.LoadFactor[i];
		
		// u0
		u(ix_u,0) = 1.0;
		// limits
		du_min[ix_u] = -dPd_max_/Pd; // ratio of limit to total gives the LoadFactor limit
		du_max[ix_u] =  0.0; // can be constrained less here
		u_min [ix_u] =  0.0;
		u_max [ix_u] =  1.0;
		// costs
		c_u_inc[ix_u] = -load(i).value*Pd;
	}
}

/// set state variables limits and costs
///   The limits here are the physical limits, not stress limits.
void PowerMPC::set_state_vars( Dense &x_min, Dense &x_max, DenseVector &c_x_low, DenseVector &c_x_high, Dense &x )
{
	// vars
	unsigned i, k, ix_x;
	double   theta, Imag, Imax, Vmag, Vmin, Vmax;
	// set limits
	// Vmag
	for (i=0;i<nBus_;i++) {
		// deal with voltage magnitude
		if ( bus(i).is_pq() ) { // x limits only apply to pq buses
			ix_x = ix.x.Vmag[i];
			err_if(ix_x==LARGE_INT,"Problem with the index in PowerMPC.");
			
			Vmag = bus(i).Vmag;
			Vmin = bus(i).Vmin;
			Vmax = bus(i).Vmax;
			// x0
			x(ix_x,0) = Vmag;
			// costs are only set for stressed buses...
			if ( Vmag<Vmin && use_stress_costs_ ) {
				c_x_low[ix_x]  = voltage_stress_cost_;
			}
			// limits are only set for buses that are near
			if ( Vmag < Vmin+bus_include_thresh_ or Vmag > Vmax-bus_include_thresh_ ) {
				for (k=0;k<=K_; k++) {
					x_min(ix_x,k) = Vmin_(i,k);
					x_max(ix_x,k) = Vmax;
				}
			}
		}
		// deal with voltage angles
		// Since we are only concerned with changes we just set all theta's to zero
		theta = 0.0;//bus(i).theta();
		//if ( is_unknown(theta) ) theta=0;
		ix_x = ix.x.theta[i];
		x(ix_x,0) = theta;
		if ( bus(i).is_ref() ) {
			c_x_low [ix_x] = slack_bus_cost_;
			c_x_high[ix_x] = slack_bus_cost_;
			for (k=0; k<=K_; k++) {
				x_min(ix_x,k) = x_max(ix_x,k) = theta;
			}
		}
	}
	// current magnitude
	for(i=0;i<nBranch_;i++) {
		Imag = branch(i).Imag()*baseMVA_;
		Imax = branch(i).rateB;
		if (Imag>Imax) Imax -= current_margin_;
		ix_x = ix.x.Imag[i];
		
		x(ix_x,0) = Imag;
		if (Imag>Imax && use_stress_costs_) {
			c_x_high[ix_x] = current_stress_cost_;
		}
		for (k=0; k<=K_; k++) {
			x_max(ix_x,k) = Imax_(i,k);
			x_min(ix_x,k) = 0;
		}
	}
}

/// build the dynamic constraints
void PowerMPC::build_dynamic_cons ( Sparse &A, Sparse &B, Sparse &W ) {
	unsigned i, ix_x, ix_u;
	double dImag_dVf, dImag_dVt, dImag_dDelta;
	unsigned bi, tbi, fbi;
	unsigned ix_Imag;
	unsigned Jrow, Jcol, bi_row, bi_col;
	double value;
	Sparse Jac;
		
	/// Imag+ = Imag + dImag_dVf * dVf + dImag_dVt * dVt + dImag_dDelta * (dTheta_F - dTheta_T)
	for (i=0;i<nBranch_;i++) {
		// get the derivatives:
		branch(i).dImag_dx(dImag_dVf, dImag_dVt, dImag_dDelta);
		// get the indeces needed
		tbi = local_bus_ix_[branch(i).tbi];
		fbi = local_bus_ix_[branch(i).fbi];
		ix_Imag    = ix.x.Imag[i];
		// stuff for the from end voltage
		if ( gen_at_bus_ix_[fbi]<LARGE_INT ) {
			B.set( ix_Imag, ix.u.Vg[gen_at_bus_ix_[fbi]], dImag_dVf*baseMVA_ );
		} else if ( bus(fbi).is_pq() ) {
			A.set( ix_Imag, ix.x.Vmag[fbi], -dImag_dVf*baseMVA_ );
		}
		// stuff for the to end voltage
		if ( gen_at_bus_ix_[tbi]<LARGE_INT ) {
			B.set( ix_Imag, ix.u.Vg[gen_at_bus_ix_[tbi]], dImag_dVt*baseMVA_ );
		} else if ( bus(tbi).is_pq() ) {
			A.set( ix_Imag, ix.x.Vmag[tbi], -dImag_dVt*baseMVA_ );
		}
		// voltage angle stuff
		A.set(ix_Imag, ix.x.theta[fbi], -dImag_dDelta*baseMVA_);
		A.set(ix_Imag, ix.x.theta[tbi], +dImag_dDelta*baseMVA_);
		
 		// set up the diagonal element
		A.set(ix_Imag, ix_Imag, 1);
	}
	
	// jacobian related stuff
	// set gen contributions
	for (i=0;i<nGen_;i++) {
		// add in the contribution from Pg changes
		ix_x = ix.x.theta[gen(i).bi];
		ix_u = ix.u.Pg[i];
		B.set( ix_x, ix_u, 1/baseMVA_ );
	}
	// add the P/Q effects from loads
	for(i=0;i<nLoad_;i++) {
		bi = local_bus_ix_[load(i).bi];
		ix_u = ix.u.LoadFactor[i];
		// P portion
		B.set ( ix.x.theta[bi], ix_u, -load(i).Pd/baseMVA_ );
		if ( is_pq_bus(bi) ) {
			B.set ( ix.x.Vmag[bi], ix_u, -load(i).Qd/baseMVA_ );
		}
	}
	// put the elements of the Jacobian in the A matrix, copying the following matlab lines:
	/*
	% slack bus P row
	mpc.A(Prows(s),ix_x.theta)      =  dP_dTheta(s,:);        % done
	mpc.A(Prows(s),ix_x.Vmag_pq)    =  dP_dVmag(s,pq);        % done
	mpc.B(Prows(s),ix_u.Vg)         = -dP_dVmag(s,gen_bus);   % done
	% non slack P rows
	mpc.A(Prows(ns),ix_x.theta(ns)) =  dP_dTheta(ns,ns);      % done
	mpc.A(Prows(ns),ix_x.Vmag(pq))  =  dP_dVmag(ns,pq);       % done
	% non-gen Q rows
	mpc.A(Qrows(pq),ix_x.theta(ns)) =  dQ_dTheta(pq,ns);      % 
	mpc.A(Qrows(pq),ix_x.Vmag(pq))  =  dQ_dVmag(pq,pq);       % done
	% effects from Vg changes
	mpc.B(Prows(ns),ix_u.Vg)        = -dP_dVmag(ns,gen_bus);  % done
	mpc.B(Qrows(pq),ix_u.Vg)        = -dQ_dVmag(pq,gen_bus);  % done
	*/
	PS_->MakePowerFlowJac( Jac );
	Jac.reset_next();
	while ( Jac.get_next(Jrow, Jcol, value) ) {
		bi_row = Jrow%nBus_;
		bi_col = Jcol%nBus_;
		
		if (Jcol<nBus_) { // dTheta
			if ( Jrow<nBus_ ) { // dP
				if ( bus(bi_row).is_ref() or !bus(bi_col).is_ref() ) {
					A.set( ix.x.theta[bi_row], ix.x.theta[bi_col], value );
				}
			} else if ( is_pq_bus(bi_row) and !bus(bi_col).is_ref() ) { // dQ
				A.set( ix.x.Vmag[bi_row], ix.x.theta[bi_col], value );
			}
		} else { // dVmag
			if ( is_gen_bus(bi_col) ) { // if this is a gen bus, the values go in the B matrix
				if ( Jrow<nBus_ ) {   // and this is a P row
					B.set( ix.x.theta[bi_row], ix.u.Vg[gen_at_bus_ix_[bi_col]], -value );
				} else if ( is_pq_bus(bi_row) ) { // if a gen bus and a Q row and 
					B.set( ix.x.Vmag[bi_row], ix.u.Vg[gen_at_bus_ix_[bi_col]], -value );
				}
			} else if ( is_pq_bus(bi_col) ) {
				if ( Jrow<nBus_ ) {   // and this is a P row
					A.set( ix.x.theta[bi_row], ix.x.Vmag[bi_col], value );
				} else if ( is_pq_bus(bi_row) ) { // this is a Q row and a PQ bus
					A.set( ix.x.Vmag[bi_row], ix.x.Vmag[bi_col], value );
				}
			}
		}
	}
}

//// is functions
/// check to see if the bus is a gen bus (if there is a locally controlable gen at the bus)
bool PowerMPC::is_gen_bus ( int bus_i )  { return gen_at_bus_ix_[bus_i]<nGen_; }
/// check to see if the bus is a load bus (if there is a locally controlable load at the bus)
bool PowerMPC::is_load_bus ( int bus_i ) { return load_at_bus_ix_[bus_i]<nLoad_; }
/// check to see if the bus is a pq bus
bool PowerMPC::is_pq_bus ( int bus_i )   { return bus(bus_i).is_pq(); }

/// not working right now
void PowerMPC::print_predictions(MPC_Solver &mpc) {
	unsigned i, ix_Imag;
	/*
	// print the voltage vars
	unsigned ix_theta = x_index(THETA);
	unsigned ix_Vmag  = x_index(V_MAG);
	printf("Vmag_theta = [...\n");
	for (i=0;i<nBus_;i++) {
		printf("  %10g %10g %10g %10g %10g;\n", 
			   mpc.x(ix_Vmag,0), mpc.x(ix_Vmag,1), mpc.y_min(y_index(V_MAG,i),1), mpc.x(ix_theta,0)*180/PI, mpc.x(ix_theta,1)*180/PI );
		ix_Vmag++;
		ix_theta++;
	}
	printf("]\n");
	*/
	// print the branch vars
	printf("Imag_0_1_max = [...\n");
	for (i=0;i<nBranch_;i++) {
		if (branch(i).Inorm()>1) {
			ix_Imag = ix.x.Imag[i];
			printf("  %10g %10g %10g;\n", mpc.x(ix_Imag,0), mpc.x(ix_Imag,1), mpc.x_max(ix_Imag,1) );
		}
		ix_Imag++;
	}
	printf("]\n");
}

