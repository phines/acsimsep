#ifndef POWER_SOLVER_HPP
#define POWER_SOLVER_HPP

#include <complex>
#include <vector>
#include "PowerSystem.hpp"

// Bonmin/Ipopt includes
#include "BonTMINLP.hpp"
#include "BonOsiTMINLPInterface.hpp"
#include "BonIpoptSolver.hpp"
#include "BonCbc.hpp"

using namespace Ipopt;
using namespace Bonmin;

enum PowerVariableType_e { VOLTAGE, GENERATION, DEMAND, CURRENT_F, CURRENT_T, BRANCH_STATUS, GEN_STATUS, DEMAND_STATUS, N_X };
enum PowerConstraintType_e { OBJECTIVE, GEN_LIMITS, DEMAND_LIMITS, DEMAND_GEN_EQUALITY,
                             BUS_INJECTION_MISMATCH, VOLTAGE_MAGNITUDE, VOLTAGE_REFERENCE,
                             BRANCH_CURRENT_EQ, BRANCH_CURRENT_EQ_F=BRANCH_CURRENT_EQ, BRANCH_CURRENT_EQ_T, BRANCH_CURRENT_MAG,
                             N_CON, NNZ_JAC, NNZ_HESS };
enum cx_type_e  { REAL=0, IMAG=1 };
enum ProblemType_e { PF, OPF, ED, SCOPF, SE, MPC };

// define a flag for input info
#define NO_DEMAND_STATUS 1
#define POWER_SOLVER_INF 1e19

/** PowerSolver class
  *  PowerSolver is a base class for use with power system optimization problems
  *  The variable order is defined by the x_index function.
  *  By default, PowerSolver is used to solve a simple economic dispatch problem, with a single 
  *   complex equality constraint: Sum(Generation_i) = sum(Demand_i)
  */
class PowerSolver : public TMINLP
{
	public:
		/// constructor
		PowerSolver();
		/// Initialize the data structures from a power system structure
		PowerSolver ( PowerSystem *PS,  ProblemType_e problem=OPF, BranchRateNo_e rate=RATE_A );
		/// destructor
		virtual ~PowerSolver() {}
		
		// no copy constructor
		/// set all of the data from a PowerSystem structure
		void set_all ( PowerSystem *PS, ProblemType_e problem=OPF, BranchRateNo_e rate=RATE_A );
		/// set the problem size
		void set_problem_size ( int nBus, int nBranch, int nDemand, int nGen );
		
		/// Bonmin/Ipopt functions
		//@{
		/// get the variable types. Valid types are: (INTEGER, BINARY, CONTINUOUS)
		virtual bool get_var_types(Index n, VariableType* var_types);
		/// get the constraint types. Valid types are: (LINEAR, NON_LINEAR)
		virtual bool get_constraints_linearity(Index m, Ipopt::TNLP::LinearityType* const_types);
		/// get the problem dimensions
		virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style);
		/// get the bounds information
		virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u);
		/// get the start point
		virtual bool get_starting_point(Index n, bool init_x, Number* x, 
		                                         bool init_z, Number* z_L, Number* z_U, Index m, 
		                                         bool init_lambda, Number* lambda);
		virtual bool get_starting_point(Index n, bool init_x, Number* x, Index m, bool init_lambda, Number* lambda);
		/// calculate the objective function
		virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);
		/// evaluate the gradient of the objective function
		virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);
		/// calculate the output of the constraint functions
		virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);
		/// calculate the Jacobian of the constraint functions
		virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
		                        Index m, Index nele_jac, Index* iRow, Index *jCol,
		                        Number* values);
		/// calculate the Hessian of the LaGrangian
		virtual bool eval_h(Index n, const Number* x, bool new_x,
		                    Number obj_factor, Index m, const Number* lambda,
		                    bool new_lambda, Index nele_hess, Index* iRow,
		                    Index* jCol, Number* values);
		/// do some clean up after the optimization has finished
		virtual void finalize_solution(SolverReturn status,
		                               Index n, const Number* x, const Number* z_L, const Number* z_U,
		                               Index m, const Number* g, const Number* lambda,
		                               Number obj_value);
		/// sosConstraints does nothing
		virtual const SosInfo * sosConstraints() const{return NULL;}
		/// branchingInfo does nothing
		virtual const BranchingInfo* branchingInfo() const{return NULL;}
		//@}
	protected:
		//// data members
		PowerSystem *PS_; ///< a pointer to the power system structure

		// variables related to the problem size:
		int nBus_;    ///< the number of busses in the network
		int nGen_;    ///< the number of gens in the network
		int nDemand_; ///< the number of loads/shunt/demands in the network
		int nBranch_; ///< the number of branches in the network
		int nX_;      ///< the size of the decision vector
		// system variables:
		double baseMVA_; ///< the system base MVA
		int RefBus_; ///< the system reference bus (voltage angle will be fixed at 0 )
		
		// a local copy of the decision vector variables
		std::vector<Number> x_;     ///< this is the variable that acutually holds the data
		std::vector<Number> x_min_; ///< the variable that holds the upper limits
		std::vector<Number> x_max_; ///< the variable that holds the lower limits
		
		// static bus voltage data
		std::vector<Number> Vmin_; ///< min voltage magnitude
		std::vector<Number> Vmax_; ///< max voltage magnitude
		
		// static generation data
		std::vector< std::complex<Number> > Gmin_; ///< min generator output while on
		std::vector< std::complex<Number> > Gmax_; ///< max generator output while on
		std::vector<int>    GenLocs_; ///< generator locations
		std::vector<Number> QC_; ///< generator quadratic cost
		std::vector<Number> MC_; ///< generator marginal cost
		std::vector<Number> FC_; ///< generator fixed (operating) cost
		
		// static demand data
		std::vector< std::complex<Number> > D_; ///< the current demand (P * jQ)
		std::vector<int>    DemandLocs_; ///< demand locations
		std::vector<Number> DemandCost_; ///< the costs associated with reducing the demand
		
		// static branch data
		std::vector<int> F_;  ///< the from end bus index of the branches
		std::vector<int> T_;  ///< the from end bus index of the branches
		std::vector< std::complex<Number> > y_FF_; ///< the from end branch admittance
		std::vector< std::complex<Number> > y_TT_; ///< the to end branch admittance
		std::vector< std::complex<Number> > y_FT_; ///< the from-to branch admittance
		std::vector< std::complex<Number> > y_TF_; ///< the to-from branch admittance
		std::vector<Number> BranchLimits_; ///< the branch capacity limit to use during the problem
		
		// Constraint flags:
		bool UseGenLimits_;
		bool UseDemandLimits_;
		bool UseDemandGenEquality_;
		bool UseBusInjectionMismatch_;
		bool UseVoltageMagnitude_;
		bool UseBranchCurrentEquality_;
		bool UseBranchCurrentMagnitude_;
		
		/// functions used by the solvers to determine the locations of data within certain vectors:
		//@{
		/// x_index returns an index into the x vector that corresponds to the specified variable
		int x_index(PowerVariableType_e var_type, int i=0, cx_type_e cxType=REAL);
		/// g_index returns an index into the constraint function (g) output vector
		/// \param con_type specifies which constraint, or N_CON to get the number of constraints in the problem.
		/// \param con_no specifies an index into the constraint
		/// \param cxType specifies whether the real (REAL) or imaginary (IMAG) portion of the constraint is wanted
		/// \return is the index into g, or if the constraint is not currently in use the result is -1.
		///  Can also return the number of constraints in the problem if N_CON is passed to con_type
		int g_index(PowerConstraintType_e con_type, int con_no=0, cx_type_e cxType=REAL );
		/// jac_index returns an index into the triplet form Jacobian vector corresponding to the specified constraint type.
		///  can also return the number of non-zeros in the jacobian, by passing NNZ_JAC
		int jac_index(PowerConstraintType_e con_type);
		/// hess_index returns an index into the triplet form Hessian vector corresponding to the specified constraint type
		///  (in this case, constraint type can include OBJECTIVE)
		///  Can also return the number of non-zeros in the hess, by passing NNZ_HESS
		int hess_index(PowerConstraintType_e con_type);
		//@}
		
		/// objective function functions
		//@{
		///    compute the dispatch cost given G and GenStatus
		Number GenDispatchCost ( const std::complex<Number> *G, const Number *GenStatus );
		///    calculate the gradient of the dispatch cost
		void   GenDispatchGrad ( const std::complex<Number> *G, const Number *GenStatus, std::complex<Number> *dCost_dG, Number *dCost_dStatus );
		///    calculate the GenDispatchHess values
		void   GenDispatchHess ( const std::complex<Number> *G, Number *HessValues, Number multiplier );
		///    calculate the GenDispatchHess Pattern
		void   GenDispatchHessPattern ( int *rows, int *cols );
		/// return the number of non-zeros in the gen dispatch hess
		int    GenDispatchHess_nnz();
		/// calculate the cost of reducing the demand according to D = D0 * DemandFactor
		Number DemandReductionCost ( const Number *DemandFactor, const Number *DemandStatus );
		/// calculate the cost of gradiant of the demand reduction cost
		void   DemandReductionGrad ( const Number *DemandFactor, const Number *DemandStatus, // inputs
		                             Number *dCost_dDemandFactor, Number *dCost_dStatus );   // outputs
		
		int ObjectiveHess_nnz() { return GenDispatchHess_nnz(); /*temporary*/ }
		// need to have a single set of functions for objectives 
		//@}
		
		/// Bus injection constraint functions
		//@{
		/// Bus Injection constraint
		void BusInjectionMismatch ( const std::complex<Number> *G, const Number *DemandFactor, 
		                            const std::complex<Number> *I_F, const std::complex<Number> *I_T,
		                            const std::complex<Number> *V, std::complex<Number> *Mismatch );
		/// Bus injection Jacobian (linear)
		void BusInjectionJac ( const std::complex<Number> *G, const Number *DemandFactor,
		                       const std::complex<Number> *I_F, const std::complex<Number> *I_T,
		                       const std::complex<Number> *V,  Number *JacValues );
		/// Bus injection Jacobian sparsity pattern
		void BusInjectionJacPattern ( int RowStartIndex, int *rows, int *cols );
		/// Bus injection Jacobian number of non-zeros
		int BusInjectionJac_nnz();
		/// calculate the bus injection Hessian
		void BusInjectionHess ( const Number *multipliers, Number *HessValues );
		/// calculate the non-zero pattern of the Hessian
		void BusInjectionHessPattern ( int *rows, int *cols );
		/// 
		int  BusInjectionHess_nnz();
		//@}
		
		/// Demand / Gen equality functions for simple problems and debugging
		//@{
		/// calculate the simple mismatch (the result is a single complex variable--Mismatch)
		void DemandGenEquality ( const std::complex<Number> *G, const Number *DemandFactor, std::complex<Number> *Mismatch );
		///  calculate the Jac
		void DemandGenEqualityJac ( const std::complex<Number> *G, const Number *DemandFactor, Number *JacValues );
		///  calculate the Jac pattern
		void DemandGenEqualityJacPattern ( int RowStartIndex, int *rows, int *cols );
		///  calculate the Jac number of non-zeros
		int  DemandGenEqualityJac_nnz();
		//@}
		
		/// Voltage magnitude functions
		//@{
		/// calculate the voltage magnitudes
		void VoltageMagnitude    ( const std::complex<Number> *V, Number *Vmag );
		/// calculate the voltage magnitude Jacobian
		void VoltageMagnitudeJac ( const std::complex<Number> *V, Number *JacValues );
		/// calculate the voltage magnitude Jacobian sparsity pattern
		void VoltageMagnitudeJacPattern ( int RowStartIndex, int *rows, int *cols );
		/// voltage magnitude Jacobian number of non-zeros
		int  VoltageMagnitudeJac_nnz();
		/// calculate the voltage magnitude Hessian
		void VoltageMagnitudeHess ( const std::complex<Number> *V, const Number* multipliers,  Number *HessValues );
		/// calculate the voltage magnitude Hessian sparsity pattern
		void VoltageMagnitudeHessPattern ( int *rows, int *cols );
		/// voltage magnitude Hessian number of non-zeros
		int  VoltageMagnitudeHess_nnz();
		//@}
		
		/// Generator output limit constraint functions
		//@{
		/// generator output constraint result
		void GenLimits    ( const std::complex<Number> *G, const Number *GenStatus, std::complex<Number> *Output );
		/// generator output constraint Jacobian (linear, so no inputs)
		void GenLimitsJac ( Number *JacValues );
		/// generator output constraint Jacobian pattern
		void GenLimitsJacPattern ( int RowStartIndex, int *rows, int *cols );
		/// gen limits Jac number of non-zeros
		int  GenLimitsJac_nnz();
		//@}
		
		/// Demand output limit functions
		//@{
		/// calculate the demand limits
		void DemandLimits ( const Number *DemandFactor, const Number *DemandStatus, Number *Output );
		/// calculate the demand limits Jacobian (linear)
		void DemandLimitsJac ( Number *dDemand_dD_values );
		/// calculate the demand limits Jacobian pattern
		void DemandLimitsJacPattern ( int RowStartIndex, int *rows, int *cols );
		/// demand limits Jac number of non-zeros
		int  DemandLimitsJac_nnz();
		//@}
		
		/// Branch current equality constraint functions
		//@{
		/// Branch current equality constraints
		void BranchCurrentEquality ( const std::complex<Number> *V,   const Number *BranchStatus,
		                             const std::complex<Number> *I_F, const std::complex<Number> *I_T, 
		                             std::complex<Number> *Error_F,   std::complex<Number> *Error_T );
		/// Branch current equality constraints Jacobian (not linear)
		void BranchCurrentEqualityJac (const std::complex<Number> *V, const Number *BranchStatus, Number *JacValues );
		/// Branch current equality constraints Jacobian pattern
		void BranchCurrentEqualityJacPattern ( int RowStartIndex, int *rows, int *cols );
		/// Branch current equality constraints Jacobian number of non-zeros
		int  BranchCurrentEqualityJac_nnz();
		/// Branch current equality constraints Hessian
		void BranchCurrentEqualityHess (const std::complex<Number> *V, const Number *BranchStatus, const Number *multipliers, Number *HessValues );
		/// Branch current equality constraints Hessian pattern
		void BranchCurrentEqualityHessPattern ( int *rows, int *cols );
		/// Branch current equality constraints Hessian number of non-zeros
		int  BranchCurrentEqualityHess_nnz();
		//@}
		
		/// Branch current magnitude constraint functions
		//@{
		///  Branch current inequality (magnitude) constraints
		void BranchCurrentMagnitude ( const std::complex<Number> *I_F, const std::complex<Number> *I_T, Number *Imag );
		///  Branch current inequality constraints Jacobian (not linear)
		void BranchCurrentMagnitudeJac ( const std::complex<Number> *I_F, const std::complex<Number> *I_T, Number *dImag_dI );
		///  Branch current inequality constraints Jacobian pattern
		void BranchCurrentMagnitudeJacPattern ( int RowStartIndex, int *rows, int *cols );
		///  Branch current inequality constraints Jacobian number of non-zeros
		int  BranchCurrentMagnitudeJac_nnz();
		///  Branch current inequality constraints Hessian (not linear)
		void BranchCurrentMagnitudeHess ( const std::complex<Number> *I_F, const std::complex<Number> *I_T,
		                                  const Number* multipliers, Number *HessValues );
		///  Branch current inequality constraints Hessian pattern
		void BranchCurrentMagnitudeHessPattern ( int *rows, int *cols );
		///  Branch current inequality constraints Hessian number of non-zeros
		int  BranchCurrentMagnitudeHess_nnz();
		//@}
};

//// Garbage:
/*
/// set the bus voltage variables
void set_vovirtual bool get_nlp_info(int& n, int& m, int& nnz_jac_g,
												int& nnz_h_lag, TNLP::IndexStyleEnum& index_style);ltage ( int nBus, std::complex<Number> *V, Number *Vmag_max, Number *Vmag_min ) {
	nBus_=nBus; V_=V; Vmax_=Vmag_max; Vmin_=Vmag_min; 
}

/// set branch data
void set_branch_data ( int nBranch, int *F, int *T, 
												std::complex<Number> *y_FF, std::complex<Number> *y_TT, 
												std::complex<Number> *y_FT, std::complex<Number> *y_TF,
												bool *BranchStatus, Number *BranchLimits ) {
	nBranch_=nBranch; F_=F; T_=T; y_FF_=y_FF; y_TT_=y_TT; y_FT_=y_FT; y_TF_=y_TF; 
	BranchStatus_=BranchStatus; BranchLimits_=BranchLimits;
}

/// set generator data
void set_gen_data ( int nGen, int *GenLocations,
										std::complex<Number> *G, std::complex<Number> *G_min, std::complex<Number> *G_max,
										bool *GenStatus, Number *FixedCost, Number *MarginalCost, Number *QuadCost=NULL) {
	nGen_=nGen; GenLocs_=GenLocations; G_=G; Gmin_=G_min; Gmax_=G_max; 
	GenStatus_=GenStatus; FC_=FixedCost; MC_=MarginalCost; QC_=QuadCost;
}

/// set demand data
void set_demand_data ( int nDemand, int *DemandLocations,
												std::complex<Number> *D, std::complex<Number> *D_max, Number *DemandReductionCost ) {
	nDemand_=nDemand; DemandLocs_=DemandLocations; D_=D; Dmax_=D_max; DemandCost_=DemandReductionCost;
}
*/

#endif

