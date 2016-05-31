#include "PowerSolver.hpp"
#include "BonTMINLP.hpp"

using namespace std;
using namespace Ipopt;

typedef std::complex<Number> cx;

/// constructor implementation
PowerSolver::PowerSolver()
{
	nX_=0;
	// initialize the problem for solving an economic dispatch problem, with a simple equality constraint
	UseGenLimits_=true;
	UseDemandLimits_=true;
	UseDemandGenEquality_=true;
	UseBusInjectionMismatch_=false;
	UseBranchCurrentEquality_=true;
	UseBranchCurrentMagnitude_=true;
	UseVoltageMagnitude_=true;
}
// constructor number 2 implementation
PowerSolver::PowerSolver ( PowerSystem *PS, ProblemType_e problem, BranchRateNo_e rate )
{
	nX_=0;
	set_all(PS, problem, rate);
}

/*
/// a simple function to print the contents of a vector
static void print_vector(vector<Number> x, const char *name=NULL)
{
	if (name==NULL)
		printf("Vector = [...\n");
	else
		printf("%s = [...\n",name);
	for (unsigned i=0; i<x.size(); i++)
	{
		printf("%.10g;\n",x[i]);
	}
	printf("]\n");
}
*/

/// The x vector has the following form:
///  Voltage (2 x Number)
///  Generation output (2 x Number)
///  Demand Factor (1 x Number)
///  Current on from end of the branch (2 x Number)
///  Current on to end of the branch   (2 x Number)
///  Generator status (1 x Number)
///  Branch status    (1 x Number)
///  Demand status    (1 x Number) (not used yet)
int PowerSolver::x_index(PowerVariableType_e var_type, int i, cx_type_e cxType)
{
	switch (var_type)
	{
		case GENERATION:     return                                                              + i*2 + cxType;
		case DEMAND:         return nGen_*2                                                      + i;
		case VOLTAGE:        return nGen_*2 + nDemand_                                           + i*2 + cxType;
		case CURRENT_F:      return nGen_*2 + nDemand_ + nBus_*2                                 + i*2 + cxType;
		case CURRENT_T:      return nGen_*2 + nDemand_ + nBus_*2 + nBranch_*2                    + i*2 + cxType;
		case GEN_STATUS:     return nGen_*2 + nDemand_ + nBus_*2 + nBranch_*4                    + i;
		case BRANCH_STATUS:  return nGen_*2 + nDemand_ + nBus_*2 + nBranch_*4 + nGen_            + i;
		case DEMAND_STATUS:  return nGen_*2 + nDemand_ + nBus_*2 + nBranch_*4 + nGen_ + nBranch_ + i;
		case N_X:            return nGen_*2 + nDemand_ + nBus_*2 + nBranch_*4 + nGen_ + nBranch_ + nDemand_;
	}
	printf("Warning: unhandled input to x_index\n");
	return -1;
}

/// g_index returns an index into the constraint function (g) output vector

int PowerSolver::g_index(PowerConstraintType_e con_type, int con_no, cx_type_e cxType )
{
//OBJECTIVE, GEN_LIMITS, DEMAND_LIMITS, DEMAND_GEN_EQUALITY, BUS_INJECTION_MISMATCH, VOLTAGE_MAGNITUDE, BRANCH_CURRENT,
//                             N_CON, NNZ_JAC, NNZ_HESS
	int result=0;
	
	// gen limits
	if (con_type==GEN_LIMITS && UseGenLimits_) return result + con_no*2 + cxType;
	result += UseGenLimits_*nGen_*2;
	
	// demand limits (only one per demand
	if (con_type==DEMAND_LIMITS && UseDemandLimits_) return result + con_no + cxType;
	result += UseDemandLimits_*nDemand_;
	
	// demand gen equality
	if (con_type==DEMAND_GEN_EQUALITY && UseDemandGenEquality_) return result + cxType;
	result += UseDemandGenEquality_*2;
	
	// bus injection constraint
	if (con_type==BUS_INJECTION_MISMATCH && UseBusInjectionMismatch_) return result + con_no*2 + cxType;
	result += UseBusInjectionMismatch_*nBus_*2;
	
	// bus voltage constraint
	if (con_type==VOLTAGE_MAGNITUDE && UseVoltageMagnitude_) return result + con_no;
	result += UseVoltageMagnitude_*nBus_;
	
	// branch current equality constraints (From end)
	if (con_type==BRANCH_CURRENT_EQ_F && UseBranchCurrentEquality_) return result + con_no*2 + cxType;
	result += UseBranchCurrentEquality_*nBranch_*2;
	
	// branch current equality constraints (To end)
	if (con_type==BRANCH_CURRENT_EQ_T && UseBranchCurrentEquality_) return result + con_no*2 + cxType;
	result += UseBranchCurrentEquality_*nBranch_*2;
	
	// branch current magnitude constraints
	if (con_type==BRANCH_CURRENT_MAG && UseBranchCurrentMagnitude_) return result + con_no;
	result += UseBranchCurrentMagnitude_*nBranch_;

	// we should now have the number of constraints in result
	if (con_type==N_CON) return result;
	
	// in all other cases we should return -1
	return -1;
}
/// jac_index returns an index into the triplet form Jacobian vectors corresponding to the specified constraint type.
int PowerSolver::jac_index(PowerConstraintType_e con_type)
{
	int result = 0;
	
	if (con_type==GEN_LIMITS && UseGenLimits_) return result;
	result += UseGenLimits_*GenLimitsJac_nnz();
	
	if (con_type==DEMAND_LIMITS && UseDemandLimits_) return result;
	result += UseDemandLimits_*DemandLimitsJac_nnz();
	
	if (con_type==DEMAND_GEN_EQUALITY && UseDemandGenEquality_) return result;
	result += UseDemandGenEquality_*DemandGenEqualityJac_nnz();
	
	if (con_type==BUS_INJECTION_MISMATCH && UseBusInjectionMismatch_) return result;
	result += UseBusInjectionMismatch_*BusInjectionJac_nnz();
	
	if (con_type==VOLTAGE_MAGNITUDE && UseVoltageMagnitude_) return result;
	result += UseVoltageMagnitude_*VoltageMagnitudeJac_nnz();
	
	// BRANCH_CURRENT_EQ is the same as BRANCH_CURRENT_EQ_F
	if (con_type==BRANCH_CURRENT_EQ && UseBranchCurrentEquality_) return result;
	result += UseBranchCurrentEquality_*BranchCurrentEqualityJac_nnz();
	
	if (con_type==BRANCH_CURRENT_MAG && UseBranchCurrentMagnitude_) return result;
	result += UseBranchCurrentMagnitude_*BranchCurrentMagnitudeJac_nnz();

	// if we want the number of non-zeros in the Jac do:
	if (con_type==NNZ_JAC) return result;
	
	// otherwise just return -1
	return -1;
}
/// hess_index returns an index into the triplet form Hessian vectors corresponding to the specified constraint type
int PowerSolver::hess_index(PowerConstraintType_e con_type)
{
	int result = 0;
	
	if (con_type==OBJECTIVE) return result;
	result += ObjectiveHess_nnz();
	
	if (con_type==GEN_LIMITS && UseGenLimits_) return result;
	// no hess elements so no need to add anything
	
	if (con_type==DEMAND_LIMITS && UseDemandLimits_) return result;
	// no hess elements so no need to add anything
	
	if (con_type==DEMAND_GEN_EQUALITY && UseDemandGenEquality_) return result;
	// no hess elements so no need to add anything
	
	if (con_type==BUS_INJECTION_MISMATCH && UseBusInjectionMismatch_) return result;
	result += UseBusInjectionMismatch_*BusInjectionHess_nnz();
	
	if (con_type==VOLTAGE_MAGNITUDE && UseVoltageMagnitude_) return result;
	result += UseVoltageMagnitude_*VoltageMagnitudeHess_nnz();
	
	if (con_type==BRANCH_CURRENT_EQ && UseBranchCurrentEquality_) return result;
	result += UseBranchCurrentEquality_*BranchCurrentEqualityHess_nnz();
	
	if (con_type==BRANCH_CURRENT_MAG && UseBranchCurrentMagnitude_) return result;
	result += UseBranchCurrentMagnitude_*BranchCurrentMagnitudeHess_nnz();	
	
	// if we want the number of non-zeros in the hess do:
	if (con_type==NNZ_HESS) return result;
	
	// otherwise just return -1
	return -1;
}

/// set the problem size, initialize x_min_/x_max_ to +/- INF
void PowerSolver::set_problem_size ( int nBus, int nBranch, int nDemand, int nGen )
{
	int nX, i;
	
	// reset the size variables
	nBus_=nBus;
	nBranch_=nBranch;
	nDemand_=nDemand;
	nGen_=nGen;
	// figure out how many variables in the problem
	nX = x_index(N_X);
	
	// if the problem size has changed, reallocate memory
	if (nX != nX_)
	{
		nX_=nX;
		// allocate new memory for x
		x_.resize(nX_);
		x_min_.resize(nX_);
		x_max_.resize(nX_);
		// just resize all of the other variables
		// voltages
		Vmin_.resize(nBus_);
		Vmax_.resize(nBus_);
		// gen data
		GenLocs_.resize(nGen_);
		QC_.resize(nGen_);
		MC_.resize(nGen_);
		FC_.resize(nGen_);
		Gmax_.resize(nGen_);
		Gmin_.resize(nGen_);
		// demand data
		D_.resize(nDemand_);
		DemandLocs_.resize(nDemand_);
		DemandCost_.resize(nDemand_);
		// branch data
		F_.resize(nBranch_);
		T_.resize(nBranch_);
		y_FF_.resize(nBranch_);
		y_TT_.resize(nBranch_);
		y_TF_.resize(nBranch_);
		y_FT_.resize(nBranch_);
		BranchLimits_.resize(nBranch_);
	}
	// initialize xmin xmax so we don't forget to do it later
	for (i=0; i<nX_; i++) {
		x_min_[i] = -POWER_SOLVER_INF;
		x_max_[i] = +POWER_SOLVER_INF;
	}
}

/// set all of the data from a PowerSystem object
void PowerSolver::set_all( PowerSystem *PS,  ProblemType_e problem, BranchRateNo_e rate )
{
	int i, ix;
	cx y_ff, y_ft, y_tt, y_tf;
	cx *V, *G, *I_F, *I_T, *Gmin, *Gmax;
	Number *DemandFactor, *BranchStatus, *GenStatus, *DemandStatus;

	PS_=PS;	
	// set the problem size
	set_problem_size( PS_->nBus(), PS_->nBranch(), PS_->nLoad(), PS_->nGen() );
	// initialize pointers to the data items
	G   = (cx*) &x_[x_index(GENERATION)];
	V   = (cx*) &x_[x_index(VOLTAGE)];
	I_F = (cx*) &x_[x_index(CURRENT_F)];
	I_T = (cx*) &x_[x_index(CURRENT_T)];
	DemandFactor = &x_[x_index(DEMAND)];
	BranchStatus = &x_[x_index(BRANCH_STATUS)];
	GenStatus    = &x_[x_index(GEN_STATUS)];
	DemandStatus = &x_[x_index(DEMAND_STATUS)];
	Gmax = (cx*) &x_max_[x_index(GENERATION)];
	Gmin = (cx*) &x_min_[x_index(GENERATION)];
	
	// x_min/x_max get set to +/- inf automatically
	// update the PS data
	PS_->update();
	// set the base MVA
	baseMVA_ = PS_->baseMVA;
	
	// set the voltage data
	for (i=0; i<nBus_; i++) {
		V[i] = PS_->bus[i].V();
		Vmin_[i] = PS_->bus[i].Vmin;
		Vmax_[i] = PS_->bus[i].Vmax;
	}
	// set the reference bus constraint
	RefBus_ = (int) PS_->ref();
	ix = x_index(VOLTAGE,RefBus_,IMAG);
	x_[ix]=x_min_[ix]=x_max_[ix]=0;
	// set the branch data
	for (i=0; i<nBranch_; i++) {
		// set the branch to / from data
		F_[i] = PS_->branch[i].fbi;
		T_[i] = PS_->branch[i].tbi;
		// set the branch admittance data
		PS_->branch[i].calc_admittances( y_ft, y_tf, y_ff, y_tt, true );
		y_FT_[i] = y_ft;
		y_TF_[i] = y_tf;
		y_FF_[i] = y_ff;
		y_TT_[i] = y_tt;
		// and the limits
		BranchLimits_[i] = PS_->branch[i].rate(rate);
		// and the status
		BranchStatus[i] = 1;//(Number) PS_->branch[i].status();
		// and the currents
		I_F[i] = PS_->branch[i].If;
		I_T[i] = PS_->branch[i].It;
		// and the branch status limits
		ix = x_index(BRANCH_STATUS,i);
		printf("Branch status debug\n");
		x_min_[ix]=1;
		x_max_[ix]=1;
	}
	// set the demand data
	for (i=0; i<nDemand_; i++) {
		// locations
		DemandLocs_[i] = PS_->load[i].bi;
		// actual demand
		D_[i] = PS_->load[i].Sd();
		// the demand status and factor variables
		DemandFactor[i] = (Number) PS_->load[i].status;
		DemandStatus[i] = (Number) PS_->load[i].status;
		// the demand reduction cost:
		DemandCost_[i] = PS_->load[i].value;
		// limits (!!!!!!debug!!!!!!)
		printf("demand debug\n");
		ix = x_index(DEMAND,i);
		x_min_[ix] = 1; // this is the demand factor
		x_max_[ix] = 1;//DemandFactor_[i]; // this is the demand factor (debug)
		ix = x_index(DEMAND_STATUS,i);
		x_min_[ix] = 1;
		x_max_[ix] = 1;//DemandStatus_[i]; (debug)
	}
	
	// set the gen data
	for (i=0; i<nGen_; i++) {
		// locations
 		GenLocs_[i] = PS_->gen[i].bi;
		// status variable
		GenStatus[i] = (Number) PS_->gen[i].status;
		// costs
		FC_[i] = PS_->gen[i].cost.a();
		MC_[i] = PS_->gen[i].cost.b();
		QC_[i] = PS_->gen[i].cost.c();
		// output
		G[i] = PS_->gen[i].Sg();
		// limits
		Gmax[i] = Gmax_[i] = PS_->gen[i].Smax();
		Gmin[i] = cx( 0, PS_->gen[i].Qmin );
		Gmin_[i] = PS_->gen[i].Smin();
		printf("gen status debug\n");
		ix = x_index(GEN_STATUS,i);
		x_min_[ix] = 1;
		x_max_[ix] = 1;
	}
	// set the variables related to the problem formulation
	switch (problem)
	{
		case ED:
			UseGenLimits_=true;
			UseDemandLimits_=true;
			UseDemandGenEquality_=true;
			UseBusInjectionMismatch_=false;
			UseBranchCurrentEquality_=false;
			UseBranchCurrentMagnitude_=false;
			UseVoltageMagnitude_=false;
			break;
		case OPF:
			UseGenLimits_=true;
			UseDemandLimits_=true;
			UseDemandGenEquality_=false;
			UseBusInjectionMismatch_=true;
			UseBranchCurrentEquality_=true;
			UseBranchCurrentMagnitude_=true;
			UseVoltageMagnitude_=true;
			break;
		case SE:
			assert(false);
		case MPC:
			assert(false);
		default:
			assert(false);
	}
}

//// Cost/objective functions

// GenDispatchCost
Number PowerSolver::GenDispatchCost ( const cx *G, const Number *GenStatus )
{
	int g;
	Number Pg, cost=0;
	
	// generator costs:
	for (g=0;g<nGen_;g++)
	{
		Pg = G[g].real();
		cost += QC_[g]*pow(Pg,2) + MC_[g]*Pg + FC_[g]*GenStatus[g];
	}
	return cost;
}

void PowerSolver::GenDispatchGrad ( const cx *G, const Number *GenStatus, cx *dCost_dG, Number *dCost_dStatus )
{
	Number Pg;
	int g;
	
	for (g=0; g<nGen_; g++ )
	{
		Pg = G[g].real();
		dCost_dG[g] = cx( 2*Pg*QC_[g] + MC_[g], 0 );
		//dCost_dG[g].imag() = 0; this is done elsewhere
		dCost_dStatus[g]   = FC_[g];
	}
}

/// calculate the GenDispatchHess values
void PowerSolver::GenDispatchHess ( const std::complex<Number> *G, Number *HessValues, Number multiplier )
{
	int g;
	// one per gen
	for (g=0;g<nGen_;g++) {
		HessValues[g] = 2*QC_[g]*multiplier;
	}
}
/// calculate the Gen dispatch Hessian pattern
void PowerSolver::GenDispatchHessPattern ( int *rows, int *cols )
{
	int g, start;
	// one per gen
	start = x_index(GENERATION);
	for (g=0;g<nGen_;g++) {
		rows[g] = cols[g] = start + g*2; // gives the index to the real element of G
	}
}
/// return the number of non-zeros in the gen dispatch hess
int PowerSolver::GenDispatchHess_nnz() {
	return nGen_;
}

/// stuff for demand reduction cost:

/// demand reduction cost objective
Number PowerSolver::DemandReductionCost ( const Number *DemandFactor, const Number *DemandStatus )
{
	Number Pd0, cost;
	int d;
	
	// demand costs:
	cost=0;
	for (d=0;d<nDemand_;d++)
	{
		Pd0 = D_[d].real();
		cost += (DemandCost_[d] * Pd0 * (1-DemandFactor[d]) );
	}
	return cost;
}

void PowerSolver::DemandReductionGrad ( const Number *DemandFactor, const Number *DemandStatus, // inputs
                                        Number *dCost_dDemandFactor, Number *dCost_dStatus )   // ouptuts
{
	int d;
	
	for (d=0;d<nDemand_;d++) {
		//cost += DemandCost_[d] * Pd0 * (1-DemandFactor[d]);
		dCost_dDemandFactor[d] = -DemandCost_[d] * D_[d].real();
	}
}

//// Constraints:
/// Bus Injection constraint
void PowerSolver::BusInjectionMismatch ( const cx *G, const Number *DemandFactor, const cx *I_F, const cx *I_T, const cx *V, cx *Mismatch )
{
	int i, g, d, br;
	cx If, It, Vf, Vt;
	
	// make sure we start at zero
	for (i=0; i<nBus_; i++) {
		Mismatch[i]=0;
	}
	// calculate the injections from Generators
	for ( g=0; g<nGen_; g++ ) {
		Mismatch [ GenLocs_[g] ] += G[g];
	}
	// calculate the injections from Demands
	for ( d=0; d<nDemand_; d++ ) {
		Mismatch[ DemandLocs_[d] ] -= D_[d] * DemandFactor[d];
	}
	// calculate the injections from Branches
	for ( br=0; br<nBranch_; br++ ) {
		If = I_F[br]; Vf = V[F_[br]];
		It = I_T[br]; Vt = V[T_[br]];
		Mismatch[ F_[br] ] -= Vf * conj(If);
		Mismatch[ T_[br] ] -= Vt * conj(It);
	}
}
/// Bus injection Jacobian (linear, so inputs not needed)
void PowerSolver::BusInjectionJac ( const cx *G, const Number *DemandFactor, const cx *I_F, const cx *I_T, const cx *V, Number *JacValues )
{
	int start, i, nz=0;
	cx If, It, Vf, Vt;
	
	// dMismatch / dG
	for (i=0;i<nGen_;i++) {
		JacValues[nz] = 1; nz++; // dReal / dP
		JacValues[nz] = 1; nz++; // dImag / dQ
	}
	// dMismatch / dDF
	for (i=0;i<nDemand_;i++) {
		JacValues[nz] = -D_[i].real(); nz++; // dReal / dP
		JacValues[nz] = -D_[i].imag(); nz++; // dImag / dQ
	}
	// dMismatch / dI_F
	// dMismatch / dI_T
	for ( i=0; i<nBranch_; i++ ) {
		If = I_F[i]; Vf = V[F_[i]];   
		It = I_T[i]; Vt = V[T_[i]];
		// dReal Mismatch at the From bus
		JacValues[nz] = -Vf.real(); nz++; // dReal / dRe (If)
		JacValues[nz] = -Vf.imag(); nz++; // dReal / dIm (If)
		// dImag Mismatch at the From bus
		JacValues[nz] = -Vf.imag(); nz++; // dImag / dRe (If)
		JacValues[nz] = +Vf.real(); nz++; // dImag / dIm (If)
		// dReal Mismatch at the To bus
		JacValues[nz] = -Vt.real(); nz++; // dReal / dRe (It)
		JacValues[nz] = -Vt.imag(); nz++; // dReal / dIm (It)
		// dImag Mismatch at the To bus
		JacValues[nz] = -Vt.imag(); nz++; // dImag / dRe (It)
		JacValues[nz] = +Vt.real(); nz++; // dImag / dIm (It)
		// dMismatch / dV (2*nBus elements)
		
	}

	//	zero out the values
	for (i=0;i<nBus_*4;i++) {
		JacValues[nz+i]=0;
	}
	for (i=0;i<nBranch_;i++) {
		// add the contribution on the From end
		start = nz + F_[i]*4;
		If = I_F[i];
		// dReal / dReal(V)
		JacValues[start+0] -=  If.real();
		// dReal / dImag(V)
		JacValues[start+1] -=  If.imag();
		// dImag / dReal(V)
		JacValues[start+2] -= -If.imag();
		// dImag / dImag(V)
		JacValues[start+3] -=  If.real();
		
		// add the contribution on the To end
		start = nz + T_[i]*4;
		It = I_T[i];
		// dReal / dReal(V)
		JacValues[start+0] -=  It.real();
		// dReal / dImag(V)
		JacValues[start+1] -=  It.imag();
		// dImag / dReal(V)
		JacValues[start+2] -= -It.imag();
		// dImag / dImag(V)
		JacValues[start+3] -=  It.real();
	}
}
/// Bus injection Jacobian sparsity pattern
void PowerSolver::BusInjectionJacPattern ( int RowStartIndex, int *rows, int *cols )
{
	int i, d, br, row, col, nz=0;
	int col_V, col_I;
	int startCol, startCol_F, startCol_T, startCol_V;
	// this matrix has nBus*2 rows, and nX columns.
	
	// calculate the jac elements for Generators
	//  (there are 2*nGen elements for generators)
	startCol = x_index ( GENERATION );
	for ( i=0; i<nGen_; i++ ) {
		row = RowStartIndex + GenLocs_[i]*2;
		col = startCol + i*2;
		// fill in the values
		rows[nz] = row;   cols[nz] = col;   nz++;
		rows[nz] = row+1;	cols[nz] = col+1;	nz++;		
	}
	// calculate the injections from Demands
	startCol = x_index ( DEMAND );
	for ( d=0; d<nDemand_; d++ ) {
		//Mismatch[ DemandLocs_[d] ] -= D[d];
		row = RowStartIndex + DemandLocs_[d]*2;
		col = startCol + d;
		// fill in the values
		rows[nz] = row;   cols[nz] = col; nz++;
		rows[nz] = row+1;	cols[nz] = col; nz++;// same column because only one demand variable
	}
	// calculate the injections from Branches
	startCol_F = x_index (CURRENT_F);
	startCol_T = x_index (CURRENT_T);
	startCol_V = x_index (VOLTAGE);
	for ( br=0; br<nBranch_; br++ ) {
		// work on From end
		row   = RowStartIndex + F_[br]*2;
		col_I = x_index(CURRENT_F,br);   //startCol_F    + br*2;
		col_V = x_index(VOLTAGE,F_[br]); //startCol_V    + F_[br]*2;
		
		// dReal Mismatch at the From bus
		// dReal / dRe (If)
		rows[nz] = row;	cols[nz] = col_I; nz++;
		// dReal / dIm (If)
		rows[nz] = row;	cols[nz] = col_I+1; nz++;
		// dImag Mismatch at the From bus
		// dImag / dRe (If)
		rows[nz] = row+1;	cols[nz] = col_I; nz++;
		// dImag / dIm (If)
		rows[nz] = row+1;	cols[nz] = col_I+1; nz++;
		
		// work on To end
		row   = RowStartIndex + T_[br]*2;
		col_I = x_index(CURRENT_T,br);  //startCol_F    + br*2;
		col_V = x_index(VOLTAGE,T_[br]);//startCol_V    + F_[br]*2;
		// dReal Mismatch at the To bus
		// dReal / dRe (It)
		rows[nz] = row;	cols[nz] = col_I; nz++;
		// dReal / dIm (It)
		rows[nz] = row;	cols[nz] = col_I+1; nz++;
		// dImag Mismatch at the To bus
		// dImag / dRe (It)
		rows[nz] = row+1;	cols[nz] = col_I; nz++;
		// dImag / dIm (It)
		rows[nz] = row+1;	cols[nz] = col_I+1; nz++;
	}
	for (i=0;i<nBus_;i++) {
		col_V = startCol_V + 2*i;
		// dReal / dReal(V)
		rows[nz] = RowStartIndex + 2*i;   cols[nz] = col_V;   nz++;
		// dReal / dImag(V)
		rows[nz] = RowStartIndex + 2*i;   cols[nz] = col_V+1; nz++;
		// dImag / dReal(V)
		rows[nz] = RowStartIndex + 2*i+1; cols[nz] = col_V;   nz++;
		// dImag / dImag(V)
		rows[nz] = RowStartIndex + 2*i+1; cols[nz] = col_V+1; nz++;
	}
}
/// Bus injection Jacobian number of non-zeros
int PowerSolver::BusInjectionJac_nnz() {
	return 2*nGen_ + 2*nDemand_ + 8*nBranch_ + 4*nBus_;
}
void PowerSolver::BusInjectionHess ( const Number *multipliers, Number *HessValues )
{
	int i, nz=0;
	Number multiplier;
		
	for (i=0;i<nBranch_;i++) {
		// From end real valued constraint
		multiplier = multipliers[F_[i]*2];
		// dReal / d Re(Vf) / d Re(If)
		HessValues[nz] = -multiplier; nz++;
		// dReal / d Im(Vf) / d Im(If)
		HessValues[nz] = -multiplier; nz++;
		
		// From end imag valued constraint
		multiplier = multipliers[F_[i]*2+1];
		// dImag / d Re(Vf) / d Im(If)
		HessValues[nz] = +multiplier; nz++;
		// dImag / d Im(Vf) / d Re(If)
		HessValues[nz] = -multiplier; nz++;
		
		// To end real valued constraint
		multiplier = multipliers[T_[i]*2];
		// dReal / d Re(Vt) / d Re(It)
		HessValues[nz] = -multiplier; nz++;
		// dReal / d Im(Vt) / d Im(It)
		HessValues[nz] = -multiplier; nz++;
		
		// To end imag valued constraint
		multiplier = multipliers[T_[i]*2+1];
		// dImag / d Re(Vt) / d Im(It)
		HessValues[nz] = +multiplier; nz++;
		// dImag / d Im(Vt) / d Re(It)
		HessValues[nz] = -multiplier; nz++;
	}
}
void PowerSolver::BusInjectionHessPattern ( int *rows, int *cols ) 
{
	int i, nz=0;
	int ix_I, ix_V;
	
	for (i=0;i<nBranch_;i++) {
		// get the index values for the from end
		ix_I = x_index(CURRENT_F,i);
		ix_V = x_index(VOLTAGE,F_[i]);
		// From end real valued constraint
		// dReal / d Re(Vf) / d Re(If)
		rows[nz]=ix_I; cols[nz]=ix_V; nz++;
		// dReal / d Im(Vf) / d Im(If)
		rows[nz]=ix_I+1; cols[nz]=ix_V+1; nz++;
		
		// From end imag valued constraint
		// dImag / d Re(Vf) / d Im(If)
		rows[nz]=ix_I+1; cols[nz]=ix_V; nz++;
		// dImag / d Im(Vf) / d Re(If)
		rows[nz]=ix_I; cols[nz]=ix_V+1; nz++;
		
		// get the index values for the To end
		ix_I = x_index(CURRENT_T,i);
		ix_V = x_index(VOLTAGE,T_[i]);
		// To end real valued constraint
		// dReal / d Re(Vt) / d Re(It)
		rows[nz]=ix_I; cols[nz]=ix_V; nz++;
		// dReal / d Im(Vt) / d Im(It)
		rows[nz]=ix_I+1; cols[nz]=ix_V+1; nz++;
		
		// To end imag valued constraint
		// dImag / d Re(Vt) / d Im(It)
		rows[nz]=ix_I+1; cols[nz]=ix_V; nz++;
		// dImag / d Im(Vt) / d Re(It)
		rows[nz]=ix_I; cols[nz]=ix_V+1; nz++;
	}
}
int PowerSolver::BusInjectionHess_nnz() {
	return 8*nBranch_;
}
/// calculate the voltage magnitudes
void PowerSolver::VoltageMagnitude ( const cx *V, Number *VmagSquared )
{
	for (int i=0; i<nBus_; i++) {
		VmagSquared[i] = pow(V[i].real(),2) + pow(V[i].imag(),2);
	}
}
/// calculate the voltage magnitude Jacobian
void PowerSolver::VoltageMagnitudeJac ( const cx *V, Number *JacValues )
{
	int i, nz=0;
	
	for (i=0; i<nBus_; i++) {
		JacValues[nz] = 2*V[i].real();
		nz++;
		JacValues[nz] = 2*V[i].imag();
		nz++;
	}
}
/// calculate the voltage magnitude Jacobian sparsity pattern
void PowerSolver::VoltageMagnitudeJacPattern ( int RowStartIndex, int *rows, int *cols )
{
	int startCol = x_index(VOLTAGE);
	int i, nz=0;
	for (i=0; i<nBus_; i++)
	{
		// for the real part
		rows[nz] = RowStartIndex + i;
		cols[nz] = startCol + 2*i;
		nz++;
		// for the imaginary part
		rows[nz] = RowStartIndex + i;
		cols[nz] = startCol + 2*i + 1;
		nz++;
	}
}
/// voltage magnitude Jacobian number of non-zeros
int  PowerSolver::VoltageMagnitudeJac_nnz() {
	return 2*nBus_;
}

/// calculate the voltage magnitude Hessian
void PowerSolver::VoltageMagnitudeHess ( const std::complex<Number> *V, const Number *multipliers, Number *HessValues )
{
	int i, nz=0;
	
	for (i=0;i<nBus_;i++) {
		HessValues[nz] += 2*multipliers[i];
		nz++;
		HessValues[nz] += 2*multipliers[i];
		nz++;
	}
}
/// calculate the voltage magnitude Hessian sparsity pattern
void PowerSolver::VoltageMagnitudeHessPattern ( int *rows, int *cols )
{
	int i, nz=0;
	int ix = x_index(VOLTAGE);
	for (i=0;i<nBus_;i++) {
		// real portion
		rows[nz] = ix + i*2;
		cols[nz] = ix + i*2;
		nz++;
		// imag portion
		rows[nz] = ix + i*2 + 1;
		cols[nz] = ix + i*2 + 1;
		nz++;
	}
}
/// voltage magnitude Hessian number of non-zeros
int  PowerSolver::VoltageMagnitudeHess_nnz() {
	return nBus_*2;
}

/// generator output constraint result
void PowerSolver::GenLimits ( const cx *G, const Number *GenStatus, cx *Output )
{
	for (int i=0; i<nGen_; i++ ) {
		Output[i] = G[i] - Gmin_[i]*GenStatus[i] + Gmax_[i]*(1-GenStatus[i]);
	}
}
/// generator output constraint Jacobian (linear, so no inputs)
void PowerSolver::GenLimitsJac ( Number *JacValues ) {
	int i, nz=0;
	
	for (i=0; i<nGen_; i++) {
		// dReal_dReal
		JacValues[ nz ] = 1; 
		nz++;
		// dImag_dImag
		JacValues[ nz ] = 1;
		nz++;
		// dReal_dStatus
		JacValues[ nz ] = -Gmax_[i].real()-Gmin_[i].real();
		nz++;
		// dImag_dStatus
		JacValues[ nz ] = -Gmax_[i].imag()-Gmin_[i].imag();
		nz++;
	}
}
/// generator output constraint Jacobian pattern
void PowerSolver::GenLimitsJacPattern ( int RowStartIndex, int *rows, int *cols ) {
	int nz=0, i;
	int startColG, startColStatus;
	
	startColG = x_index(GENERATION);
	startColStatus = x_index(GEN_STATUS);
	
	for (i=0; i<nGen_; i++) {
		// dReal_dReal
		rows[nz] = RowStartIndex + 2*i;
		cols[nz] = startColG     + 2*i;
		nz++;
		// dImag_dImag
		rows[nz] = RowStartIndex + 2*i + 1;
		cols[nz] = startColG     + 2*i + 1;
		nz++;
		// dReal_dStatus
		rows[nz] = RowStartIndex  + 2*i;
		cols[nz] = startColStatus + i;
		nz++;
		// dImag_dStatus
		rows[nz] = RowStartIndex  + 2*i + 1;
		cols[nz] = startColStatus + i;
		nz++;
	}
}
/// gen limits Jac number of non-zeros
int  PowerSolver::GenLimitsJac_nnz() {
	return 4*nGen_;
}

/// calculate the demand limits
void PowerSolver::DemandLimits (const Number *DemandFactor, const Number *DemandStatus, Number *Output )
{
	for (int d=0; d<nDemand_; d++) {
		Output[d] = DemandFactor[d] - DemandStatus[d];
	}
}
/// calculate the demand limits Jacobian (linear)
void PowerSolver::DemandLimitsJac ( Number *JacValues )
{
	int nz=0;
	for (int d=0; d<nDemand_; d++) {
		JacValues[nz] = 1; nz++;  // dLimit/dFactor
		JacValues[nz] = -1; nz++; // dLimit/dStatus
	}
}
/// calculate the demand limits Jacobian pattern
void PowerSolver::DemandLimitsJacPattern ( int RowStartIndex, int *rows, int *cols ) {
	int nz=0, d, startColDF, startColDS;
	
	startColDF = x_index(DEMAND);
	startColDS = x_index(DEMAND_STATUS);
	for (d=0; d<nDemand_; d++) {
		rows[nz] = RowStartIndex + d;
		cols[nz] = startColDF + d;
		nz++;
		rows[nz] = RowStartIndex + d;
		cols[nz] = startColDS + d;
		nz++;
	}
}
/// demand limits Jac number of non-zeros
int PowerSolver::DemandLimitsJac_nnz() {
	return nDemand_*2;
}

/// Branch current equality constraints (non-linear)
void PowerSolver::BranchCurrentEquality ( const cx *V, const Number *BranchStatus, const cx *I_FT, const cx *I_TF, cx *Error_F, cx *Error_T )
{
	cx Vf, Vt;
	Number status;
	for (int br=0; br<nBranch_; br++) {
		Vf = V[ F_[br] ];
		Vt = V[ T_[br] ];
		status = BranchStatus[br];
		Error_F[br] = status * ( Vf*y_FF_[br] + Vt*y_FT_[br] ) * baseMVA_ - I_FT[br];
		Error_T[br] = status * ( Vt*y_TT_[br] + Vf*y_TF_[br] ) * baseMVA_ - I_TF[br];
	}
}
/// Branch current equality constraints Jacobian (not linear wrt V)
void PowerSolver::BranchCurrentEqualityJac (const cx *V, const Number *BranchStatus, Number *JacValues ) {
	int nz=0, br;
	cx I_f, I_t;
	cx Vf, Vt, y_ff, y_tt, y_ft, y_tf;
	Number status;
	
	for (br=0; br<nBranch_; br++) {
		// extract some data for this branch:
		Vf = V[ F_[br] ];
		Vt = V[ T_[br] ];
		y_ff = y_FF_[br];
		y_tt = y_TT_[br];
		y_ft = y_FT_[br];
		y_tf = y_TF_[br];
		I_f = (Vf*y_ff + Vt*y_ft)*baseMVA_; // the branch current if the branch status is 1
		I_t = (Vt*y_tt + Vf*y_tf)*baseMVA_; // the branch current if the branch status is 1
		status = BranchStatus[br];
		// thus the equations are:
		// Error_F[br] = status - I_FT[br] * I_f, where I_f = Vf*y_ff + Vt*y_ft
		// Error_T[br] = status - I_TF[br] * I_t, where I_t = Vt*y_tt + Vf*y_tf
		// The derivative elements follow:
		// dError / dI
		JacValues[nz] = -1; nz++; // d Re(Error_F) / d Re(I_FT[br])
		JacValues[nz] = -1; nz++; // d Im(Error_F) / d Im(I_FT[br])
		JacValues[nz] = -1; nz++; // d Re(Error_T) / d Re(I_TF[br])
		JacValues[nz] = -1; nz++; // d Im(Error_T) / d Im(I_TF[br])
		// dError / d status
		JacValues[nz] = I_f.real(); nz++; // d Re(Error_F) / d status
		JacValues[nz] = I_f.imag(); nz++; // d Im(Error_F) / d status
		JacValues[nz] = I_t.real(); nz++; // d Re(Error_T) / d status
		JacValues[nz] = I_t.imag(); nz++; // d Im(Error_T) / d status
		// dError_F / dVf = ...
		JacValues[nz] =  status * y_ff.real() * baseMVA_; nz++; // d Re(Error_F) / d Re(Vf)
		JacValues[nz] = -status * y_ff.imag() * baseMVA_; nz++; // d Re(Error_F) / d Im(Vf)
		JacValues[nz] =  status * y_ff.imag() * baseMVA_; nz++; // d Im(Error_F) / d Re(Vf)
		JacValues[nz] =  status * y_ff.real() * baseMVA_; nz++; // d Im(Error_F) / d Im(Vf)
		// dError_F / dVt = ...
		JacValues[nz] =  status * y_ft.real() * baseMVA_; nz++; // d Re(Error_F) / d Re(Vt)
		JacValues[nz] = -status * y_ft.imag() * baseMVA_; nz++; // d Re(Error_F) / d Im(Vt)
		JacValues[nz] =  status * y_ft.imag() * baseMVA_; nz++; // d Im(Error_F) / d Re(Vt)
		JacValues[nz] =  status * y_ft.real() * baseMVA_; nz++; // d Im(Error_F) / d Im(Vt)
		// dError_T / dVt = ...
		JacValues[nz] =  status * y_tt.real() * baseMVA_; nz++; // d Re(Error_T) / d Re(Vt)
		JacValues[nz] = -status * y_tt.imag() * baseMVA_; nz++; // d Re(Error_T) / d Im(Vt)
		JacValues[nz] =  status * y_tt.imag() * baseMVA_; nz++; // d Im(Error_T) / d Re(Vt)
		JacValues[nz] =  status * y_tt.real() * baseMVA_; nz++; // d Im(Error_T) / d Im(Vt)
		// dError_T / dVf = ...
		JacValues[nz] =  status * y_tf.real() * baseMVA_; nz++; // d Re(Error_T) / d Re(Vf)
		JacValues[nz] = -status * y_tf.imag() * baseMVA_; nz++; // d Re(Error_T) / d Im(Vf)
		JacValues[nz] =  status * y_tf.imag() * baseMVA_; nz++; // d Im(Error_T) / d Re(Vf)
		JacValues[nz] =  status * y_tf.real() * baseMVA_; nz++; // d Im(Error_T) / d Im(Vf)
		
	}
}
/// Branch current equality constraints Jacobian pattern
void PowerSolver::BranchCurrentEqualityJacPattern ( int RowStartIndex, int *rows, int *cols )
{
	int nz=0, br, col;
	int row_F_Re, row_F_Im, row_T_Re, row_T_Im;
	
	for (br=0; br<nBranch_; br++) {
		// collect the row indeces... Assumes the full set of Error_F constraints come before the full set of Error_T constraints
		row_F_Re = RowStartIndex + br*2;
		row_F_Im = RowStartIndex + br*2 + 1;
		row_T_Re = RowStartIndex + br*2     + nBranch_*2;
		row_T_Im = RowStartIndex + br*2 + 1 + nBranch_*2;
		// The derivative elements follow in the same order as BranchCurrentEqualityJac() (don't change the order !!!)
		// dError / dI
		// d Re(Error_F) / d Re(I_FT[br])
		col = x_index(CURRENT_F,br,REAL);
		rows[nz] = row_F_Re;
		cols[nz] = col;
		nz++;
		// d Im(Error_F) / d Im(I_FT[br])
		rows[nz] = row_F_Im;
		cols[nz] = col+1;
		nz++;
		// d Re(Error_T) / d Re(I_TF[br])
		col = x_index(CURRENT_T,br,REAL);
		rows[nz] = row_T_Re;
		cols[nz] = col;
		nz++;
		// d Im(Error_T) / d Im(I_TF[br])
		rows[nz] = row_T_Im;
		cols[nz] = col+1;
		nz++;
		
		// dError / d status
		col = x_index(BRANCH_STATUS,br);
		// d Re(Error_F) / d status
		rows[nz] = row_F_Re;
		cols[nz] = col;
		nz++;
		// d Im(Error_F) / d status
		rows[nz] = row_F_Im;
		cols[nz] = col;
		nz++;
		// d Re(Error_T) / d status 
		rows[nz] = row_T_Re;
		cols[nz] = col;
		nz++;
		// d Im(Error_T) / d status
		rows[nz] = row_T_Im;
		cols[nz] = col;
		nz++;
		
		// dError_F / dVf = ...
		col = x_index(VOLTAGE,F_[br],REAL);
		// d Re(Error_F) / d Re(Vf)
		rows[nz] = row_F_Re;
		cols[nz] = col;
		nz++;
		// d Re(Error_F) / d Im(Vf)
		rows[nz] = row_F_Re;
		cols[nz] = col+1;
		nz++;
		// d Im(Error_F) / d Re(Vf)
		rows[nz] = row_F_Im;
		cols[nz] = col;
		nz++;
		// d Im(Error_F) / d Im(Vf)
		rows[nz] = row_F_Im;
		cols[nz] = col+1;
		nz++;
		
		// dError_F / dVt = ...
		col = x_index(VOLTAGE,T_[br],REAL);
		// d Re(Error_F) / d Re(Vt)
		rows[nz] = row_F_Re;
		cols[nz] = col;
		nz++;
		// d Re(Error_F) / d Im(Vt)
		rows[nz] = row_F_Re;
		cols[nz] = col+1;
		nz++;
		// d Im(Error_F) / d Re(Vt)
		rows[nz] = row_F_Im;
		cols[nz] = col;
		nz++;
		// d Im(Error_F) / d Im(Vt)
		rows[nz] = row_F_Im;
		cols[nz] = col+1;
		nz++;
		
		// dError_T / dVt = ...
		//  same as above: col = x_index(VOLTAGE,T[br],REAL);
		// d Re(Error_T) / d Re(Vt)
		rows[nz] = row_T_Re;
		cols[nz] = col;
		nz++;
		// d Re(Error_T) / d Im(Vt)
		rows[nz] = row_T_Re;
		cols[nz] = col+1;
		nz++;
		// d Im(Error_T) / d Re(Vt)
		rows[nz] = row_T_Im;
		cols[nz] = col;
		nz++;
		// d Im(Error_T) / d Im(Vt)
		rows[nz] = row_T_Im;
		cols[nz] = col+1;
		nz++;
		
		// dError_T / dVf = ...
		col = x_index(VOLTAGE,F_[br],REAL);
		// d Re(Error_T) / d Re(Vf)
		rows[nz] = row_T_Re;
		cols[nz] = col;
		nz++;
		// d Re(Error_T) / d Im(Vf)
		rows[nz] = row_T_Re;
		cols[nz] = col+1;
		nz++;
		// d Im(Error_T) / d Re(Vf)
		rows[nz] = row_T_Im;
		cols[nz] = col;
		nz++;
		// d Im(Error_T) / d Im(Vf)
		rows[nz] = row_T_Im;
		cols[nz] = col+1;
		nz++;
	}
}
/// Branch current equality constraints Jacobian number of non-zeros
int  PowerSolver::BranchCurrentEqualityJac_nnz() {
	return 24*nBranch_;
}
/// Branch current equality constraints Hessian
void PowerSolver::BranchCurrentEqualityHess (const std::complex<Number> *V, 
		const Number *BranchStatus, const Number *multipliers, Number *HessValues )
{
	int start, br;
	cx y_ff, y_tt, y_ft, y_tf;
	Number multiplier;
	
	// there are 4 hess elements per branch with 4 constraints effecting each element
	for (br=0; br<nBranch_; br++) {
		// extract some data for this branch:
		y_ff = y_FF_[br];
		y_tt = y_TT_[br];
		y_ft = y_FT_[br];
		y_tf = y_TF_[br];
		// find the starting point in the hess element vector
		start = br*4;
		
		// the equations are:
		// Error_F[br] = status * I_f - I_FT[br], where I_f = Vf*y_ff + Vt*y_ft
		// Error_T[br] = status * I_t - I_TF[br], where I_t = Vt*y_tt + Vf*y_tf
		// The second derivative elements follow:
		
		// elements related to d Re(Error_F)
		multiplier = multipliers[2*br];
		//d^2(d Re(Error_F)) / dVf.real() / status
		HessValues[start+0] += +y_ff.real() * baseMVA_ * multiplier;
		//d^2(d Re(Error_F)) / dVf.imag() / status
		HessValues[start+1] += -y_ff.imag() * baseMVA_ * multiplier;
		//d^2(d Re(Error_F)) / dVt.real() / status
		HessValues[start+2] += +y_ft.real() * baseMVA_ * multiplier;
		//d^2(d Re(Error_F)) / dVt.imag() / status
		HessValues[start+3] += -y_ft.imag() * baseMVA_ * multiplier;
		
		// elements related to d Im(Error_F)
		multiplier = multipliers[2*br+1];
		//d^2(d Im(Error_F)) / dVf.real() / status
		HessValues[start+0] += y_ff.imag() * baseMVA_ * multiplier;
		//d^2(d Im(Error_F)) / dVf.imag() / status
		HessValues[start+1] += y_ff.real() * baseMVA_ * multiplier;
		//d^2(d Im(Error_F)) / dVt.real() / status
		HessValues[start+2] += y_ft.imag() * baseMVA_ * multiplier;
		//d^2(d Im(Error_F)) / dVt.imag() / status
		HessValues[start+3] += y_ft.real() * baseMVA_ * multiplier;
		
		// elements related to d Re(Error_T)
		multiplier = multipliers[2*nBranch_ + 2*br];
		//d^2(d Re(Error_T)) / dVf.real() / status
		HessValues[start+0] += +y_tf.real() * baseMVA_ * multiplier;
		//d^2(d Re(Error_T)) / dVf.imag() / status
		HessValues[start+1] += -y_tf.imag() * baseMVA_ * multiplier;
		//d^2(d Re(Error_T)) / dVt.real() / status
		HessValues[start+2] += +y_tt.real() * baseMVA_ * multiplier;
		//d^2(d Re(Error_T)) / dVt.imag() / status
		HessValues[start+3] += -y_tt.imag() * baseMVA_ * multiplier;
		
		// elements related to d Im(Error_T)
		multiplier = multipliers[2*nBranch_ + 2*br+1];
		//d^2(d Im(Error_T)) / dVf.real() / status
		HessValues[start+0] += y_tf.imag() * baseMVA_ * multiplier;
		//d^2(d Im(Error_T)) / dVf.imag() / status
		HessValues[start+1] += y_tf.real() * baseMVA_ * multiplier;
		//d^2(d Im(Error_T)) / dVt.real() / status
		HessValues[start+2] += y_tt.imag() * baseMVA_ * multiplier;
		//d^2(d Im(Error_T)) / dVt.imag() / status
		HessValues[start+3] += y_tt.real() * baseMVA_ * multiplier;
	}
}
/// Branch current equality constraints Hessian pattern
void PowerSolver::BranchCurrentEqualityHessPattern ( int *rows, int *cols )
{
	int br, nz=0;
	int ix_status_all = x_index(BRANCH_STATUS);
	int ix_V_all      = x_index(VOLTAGE);
	int ix_status, ix_Vf, ix_Vt;
	
	// branch status go in rows, and voltages go in cols in order to only fill in lower triangle
	for ( br=0; br<nBranch_; br++ ) {
		ix_status = ix_status_all + br;
		ix_Vf = ix_V_all + F_[br]*2;
		ix_Vt = ix_V_all + T_[br]*2;
		// just 4 elements so:
		rows[nz]=ix_status; cols[nz]=ix_Vf; nz++;
		rows[nz]=ix_status; cols[nz]=ix_Vf+1; nz++;
		rows[nz]=ix_status; cols[nz]=ix_Vt; nz++;
		rows[nz]=ix_status; cols[nz]=ix_Vt+1; nz++;
	}
	
}
/// Branch current equality constraints Hessian number of non-zeros
int  PowerSolver::BranchCurrentEqualityHess_nnz()
{
	return 4*nBranch_; // this is the number for the lower triagle of the matrix, there are actualy 2* this many
}

/// Branch current inequality (magnitude) constraints
void PowerSolver::BranchCurrentMagnitude ( const cx *I_F, const cx* I_T, Number *ImagSquared )
{
	cx If, It;
	// gives the average, squared branch current over the branch
	for (int br=0; br<nBranch_; br++)
	{
		If = I_F[br];
		It = I_T[br];
		
		ImagSquared[br] = (pow(If.real(),2) + pow(If.imag(),2) + pow(It.real(),2) + pow(It.imag(),2))/2;
	}
}
/// Branch current inequality constraints Jacobian (not linear)
void PowerSolver::BranchCurrentMagnitudeJac ( const cx *I_F, const cx *I_T, Number *JacValues )
{
	int br, nz=0;
	for (br=0; br<nBranch_; br++)
	{
		// dImag_F / d Re(I_F)
		JacValues[nz] = I_F[br].real();
		nz++;
		// dImag_F / d Im(I_F)
		JacValues[nz] = I_F[br].imag();
		nz++;
		// dImag_T / d Re(I_T)
		JacValues[nz] = I_T[br].real();
		nz++;
		// dImag_T / d Im(I_T)
		JacValues[nz] = I_T[br].imag();
		nz++;
	}
}
/// Branch current inequality constraints Jacobian pattern
///  This function assumes that the constraints are ordered as above
///  with |I_FT| first and |I_TF| following.
void PowerSolver::BranchCurrentMagnitudeJacPattern ( int RowStartIndex, int *rows, int *cols )
{
	int nz=0, row, br;
	
	for (br=0; br<nBranch_; br++) 
	{
		row = RowStartIndex + br;
		// dImag_F / d Re(I_F)
		rows[nz] = row;
		cols[nz] = x_index(CURRENT_F,br,REAL);
		nz++;
		// dImag_F / d Im(I_F)
		rows[nz] = row;
		cols[nz] = x_index(CURRENT_F,br,IMAG);
		nz++;
		// dImag_T / d Re(I_T)
		rows[nz] = row;
		cols[nz] = x_index(CURRENT_T,br,REAL);
		nz++;
		// dImag_T / d Im(I_T)
		rows[nz] = row;
		cols[nz] = x_index(CURRENT_T,br,IMAG);
		nz++;
	}
}
/// Branch current inequality constraints Jacobian number of non-zeros
int  PowerSolver::BranchCurrentMagnitudeJac_nnz() {
	return 4*nBranch_;
}

///  Branch current inequality constraints Hessian (not linear)
void PowerSolver::BranchCurrentMagnitudeHess ( const cx *I_F, const cx *I_T, const Number* multipliers, Number *HessValues )
{
	int nz=0, br;
	Number multiplier;
	
	for (br=0; br<nBranch_; br++)
	{
		multiplier = multipliers[br];
		// work on from side
		// dImag / d Re(If) / d Re(If)
		HessValues[nz] += 1 * multiplier;
		nz++;
		// dImag / d Im(If) / d Im(If)
		HessValues[nz] += 1 * multiplier;
		nz++;
		
		// work on To side
		// dImag / d Re(It) / d Re(It)
		HessValues[nz] += 1 * multiplier;
		nz++;
		// dImag / d Im(It) / d Im(It)
		HessValues[nz] += 1 * multiplier;
		nz++;
	}
}
///  Branch current inequality constraints Hessian pattern
void PowerSolver::BranchCurrentMagnitudeHessPattern ( int *rows, int *cols )
{
	int br, nz=0;
	int start_F = x_index(CURRENT_F);
	int start_T = x_index(CURRENT_T);
	
	for (br=0; br<nBranch_; br++) {
		// dImag / d Re(If) / d Re(If)
		rows[nz] = start_F + br*2;
		cols[nz] = start_F + br*2;
		nz++;
		// dImag / d Im(If) / d Im(If)
		rows[nz] = start_F + br*2 + 1;
		cols[nz] = start_F + br*2 + 1;
		nz++;
		// dImag / d Re(It) / d Re(It)
		rows[nz] = start_T + br*2;
		cols[nz] = start_T + br*2;
		nz++;
		// dImag / d Im(It) / d Im(It)
		rows[nz] = start_T + br*2 + 1;
		cols[nz] = start_T + br*2 + 1;
		nz++;
	}
}
///  Branch current inequality constraints Hessian number of non-zeros
int  PowerSolver::BranchCurrentMagnitudeHess_nnz()
{
	return 4*nBranch_;
}

/// calculate the simple mismatch (the result is a single complex variable--Mismatch)
void PowerSolver::DemandGenEquality ( const cx *G, const Number *DemandFactor, cx *Mismatch )
{
	int i;
	
	Mismatch[0]=0;
	
	// contribution from generators
	for(i=0;i<nGen_;i++) {
		Mismatch[0] += G[i];
	}
	// contribution from demands
	for(i=0;i<nDemand_;i++) {
		Mismatch[0] -= DemandFactor[i]*D_[i];
	}
}

///  calculate the Jac
void PowerSolver::DemandGenEqualityJac ( const std::complex<Number> *G, const Number *DemandFactor, Number *JacValues ) 
{
	int i, nz=0;
	
	// contribution from generators
	for(i=0;i<nGen_;i++) {
		JacValues[nz]=1; nz++; // real portion
		JacValues[nz]=1; nz++; // imag portion
	}
	// contribution from demands
	for(i=0;i<nDemand_;i++) {
		JacValues[nz] = -D_[i].real(); nz++; // real portion
		JacValues[nz] = -D_[i].imag(); nz++; // imag portion
	}
}
///  calculate the Jac pattern
void PowerSolver::DemandGenEqualityJacPattern ( int RowStartIndex, int *rows, int *cols )
{
	int i, nz=0, col_ix;
	
	// contribution from generators
	col_ix = x_index(GENERATION);
	for(i=0;i<nGen_;i++) {
		// real portion
		rows[nz] = RowStartIndex;
		cols[nz] = col_ix + i*2;
		nz++;
		// imag portion
		rows[nz] = RowStartIndex+1;
		cols[nz] = col_ix + i*2 + 1;
		nz++; 
	}
	// contribution from demands
	col_ix = x_index(DEMAND);
	for(i=0;i<nDemand_;i++) {
		// real portion
		rows[nz] = RowStartIndex;
		cols[nz] = col_ix + i;
		nz++;
		// imag portion
		rows[nz] = RowStartIndex+1;
		cols[nz] = col_ix + i;
		nz++;
	}
}
///  calculate the Jac number of non-zeros
int  PowerSolver::DemandGenEqualityJac_nnz() {
	return 2*nGen_ + 2*nDemand_;
}

/// Bonmin/Ipopt functions
//@{
/// get the variable types. Valid types are: (INTEGER, BINARY, CONTINUOUS)
bool PowerSolver::get_var_types(Index n, VariableType* var_types)
{
	int i, ix_gen_status = x_index(GEN_STATUS);
	
	for (i=0; i<ix_gen_status; i++) {
		var_types[i] = CONTINUOUS;
	}
	for (i=ix_gen_status; i<n; i++) {
		var_types[i] = BINARY;
	}
	return true;
}
/// get the constraint types. Valid types are: (LINEAR, NON_LINEAR)
bool PowerSolver::get_constraints_linearity(Index m, TNLP::LinearityType* con_types)
{
	int i, start;
	
	if (UseGenLimits_) {
		start = g_index(GEN_LIMITS);
		for (i=start; i<start+nGen_*2; i++) {
			con_types[i]=Ipopt::TNLP::LINEAR;
		}
	}
	
	if (UseDemandLimits_) {
		start = g_index(DEMAND_LIMITS);
		for (i=start; i<start+nDemand_*2; i++) {
			con_types[i]=Ipopt::TNLP::LINEAR;
		}
	}
	
	if (UseDemandGenEquality_) {
		i = g_index(DEMAND_GEN_EQUALITY);
		con_types[i] = con_types[i+1] = Ipopt::TNLP::LINEAR;
	}
	
	// for now just assert on the stuff that is not ready yet
	if (UseBusInjectionMismatch_) {
		start = g_index(BUS_INJECTION_MISMATCH);
		for (i=start; i<start+nBus_*2; i++) {
			con_types[i] = Ipopt::TNLP::NON_LINEAR;
		}
	}
	if (UseVoltageMagnitude_) {
		start = g_index(VOLTAGE_MAGNITUDE);
		for (i=start;i<start+nBus_*2;i++) {
			con_types[i] = Ipopt::TNLP::NON_LINEAR;
		}
	}
	if (UseBranchCurrentEquality_) {
		start = g_index(BRANCH_CURRENT_EQ);
		for (i=start; i<start+nBranch_*4; i++) {
			con_types[i] = Ipopt::TNLP::NON_LINEAR;
		}
	}
	if (UseBranchCurrentMagnitude_) {
		start = g_index(BRANCH_CURRENT_MAG);
		for (i=start; i<start+nBranch_; i++) {
			con_types[i] = Ipopt::TNLP::NON_LINEAR;
		}
	}
	return true;
}
/// get the problem dimensions
bool PowerSolver::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style) 
{
	n = nX_;
	m = g_index(N_CON);
	nnz_jac_g = jac_index(NNZ_JAC);
	nnz_h_lag = hess_index(NNZ_HESS);
	index_style = TNLP::C_STYLE;
	assert(m>0);
	assert(n>0);
	return true;
}
/// get the bounds information
bool PowerSolver::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)
{
	int i, start;
	
	assert(n==nX_);
	assert(m==g_index(N_CON));
	
	// xmin / xmax
	for(i=0;i<n;i++) {
		x_l[i] = x_min_[i];
		x_u[i] = x_max_[i];
	}
	if (m>0)
	{
		// constraint min/max values
		// gen limits go from 0 to Gmax
		if (UseGenLimits_) {
			start=g_index(GEN_LIMITS);
			for(i=0;i<nGen_;i++) {
				g_l[start+2*i]   = 0;
				g_l[start+2*i+1] = Gmin_[i].imag();
				g_u[start+2*i]   = Gmax_[i].real();
				g_u[start+2*i+1] = Gmax_[i].imag();
			}
		}
		// demand limits
		if (UseDemandLimits_) {
			start = g_index(DEMAND_LIMITS);
			for(i=0;i<nGen_;i++) {
				g_l[start+i] = -POWER_SOLVER_INF;
				g_u[start+i] = 0;
			}
		}
		// G/D equality constraint
		if (UseDemandGenEquality_) {
			start = g_index(DEMAND_GEN_EQUALITY);
			g_l[start] = g_l[start+1] = 0;
			g_u[start] = g_u[start+1] = 0;
		}
		/* for now just assert on the others because they are not done yet */
		if (UseBusInjectionMismatch_) {
			start = g_index(BUS_INJECTION_MISMATCH);
			for (i=0;i<nBus_*2;i++) {
				g_l[start+i] = g_u[start+i] = 0;
			}
		}
		//
		if (UseBranchCurrentEquality_) {
			start = g_index(BRANCH_CURRENT_EQ);
			for ( i=0; i<nBranch_*4; i++ ) {
				g_l[start+i] = g_u[start+i] = 0;
			}
		}
		if (UseBranchCurrentMagnitude_) {
			start = g_index(BRANCH_CURRENT_MAG);
			for ( i=0; i<nBranch_; i++ ) {
				g_l[start+i] = -POWER_SOLVER_INF;
				g_u[start+i] = pow(BranchLimits_[i],2);
			}
		}
		// voltage magnitude bounds
		if (UseVoltageMagnitude_) {
			start = g_index(VOLTAGE_MAGNITUDE);
			for ( i=0; i<nBus_; i++ ) {
				g_l[start+i] = pow(Vmin_[i],2);
				g_u[start+i] = pow(Vmax_[i],2);
			}
		}
	}
	return true;
}
/// get the start point
bool PowerSolver::get_starting_point(Index n, bool init_x, Number* x, 
		                                          bool init_z, Number* z_L, Number* z_U, Index m, 
		                                          bool init_lambda, Number* lambda)
{
	// initialize x
	if (init_x) {
		for(int i=0;i<n;i++) {
			x[i] = x_[i];
		}
	}
	init_lambda=false;
	init_z=false;
	return true;
}
/// get the start point
bool PowerSolver::get_starting_point(Index n, bool init_x, Number* x, Index m, bool init_lambda, Number* lambda)
{
	// initialize x
	if (init_x) {
		for(int i=0;i<n;i++) {
			x[i] = x_[i];
		}
	}
	init_lambda=false;
	return true;
}
/// calculate the objective function
bool PowerSolver::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
	const cx *G = (cx*) &x[x_index(GENERATION)];
	const Number *GenStatus = &x[x_index(GEN_STATUS)];
	const Number *DemandFactor = &x[x_index(DEMAND)];
	const Number *DemandStatus = &x[x_index(DEMAND_STATUS)];
	
	assert(n==nX_);

	obj_value = GenDispatchCost(G, GenStatus) + DemandReductionCost(DemandFactor, DemandStatus);
	//obj_value = 0;
	
	return true;
}
/// evaluate the gradient of the objective function
bool PowerSolver::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
	int i;
	const cx     *G             = (cx*) &x[x_index(GENERATION)];
	const Number *GenStatus     = &x[x_index(GEN_STATUS)];
	const Number *DemandFactor  = &x[x_index(DEMAND)];
	const Number *DemandStatus  = &x[x_index(DEMAND_STATUS)];
	cx     *dCost_dG            = (cx*) &grad_f[x_index(GENERATION)];
	Number *dCost_dGenStatus    = &grad_f[x_index(GEN_STATUS)];
	Number *dCost_dDemandFactor = &grad_f[x_index(DEMAND)];
	Number *dCost_dDemandStatus = &grad_f[x_index(DEMAND_STATUS)];
	
	// set the grad to zero
	for (i=0; i<n; i++) {
		grad_f[i]=0;
	}
	// calculate the gen portion of the gradient
	GenDispatchGrad(G, GenStatus, dCost_dG, dCost_dGenStatus);
	// calculate the demand portion of the gradient
	DemandReductionGrad(DemandFactor, DemandStatus, dCost_dDemandFactor, dCost_dDemandStatus);
	
	return true;
}
/// calculate the output of the constraint functions
bool PowerSolver::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	const cx     *G            = (cx*) &x[x_index(GENERATION)];
	const cx     *V            = (cx*) &x[x_index(VOLTAGE)];
	const cx     *I_F          = (cx*) &x[x_index(CURRENT_F)];
	const cx     *I_T          = (cx*) &x[x_index(CURRENT_T)];
	const Number *GenStatus    = &x[x_index(GEN_STATUS)];
	const Number *BranchStatus = &x[x_index(BRANCH_STATUS)];
	const Number *DemandFactor = &x[x_index(DEMAND)];
	const Number *DemandStatus = &x[x_index(DEMAND_STATUS)];
	cx *Error_F, *Error_T;
	
	if (UseGenLimits_) {
		GenLimits ( G, GenStatus, (cx*) &g[g_index(GEN_LIMITS)] );
	}
	
	if (UseDemandLimits_) {
		DemandLimits ( DemandFactor, DemandStatus, (Number*) &g[g_index(DEMAND_LIMITS)] );
	}
	
	if (UseDemandGenEquality_) {
		DemandGenEquality( G, DemandFactor, (cx*) &g[g_index(DEMAND_GEN_EQUALITY)] );
	}
	
	// for now just assert on the stuff that is not ready yet
	if (UseBusInjectionMismatch_) {
		BusInjectionMismatch ( G, DemandFactor, I_F, I_T, V, (cx*) &g[g_index(BUS_INJECTION_MISMATCH)] );
	}
	if (UseBranchCurrentEquality_) {	
		Error_F = (cx*) &g[g_index(BRANCH_CURRENT_EQ_F)];
		Error_T = (cx*) &g[g_index(BRANCH_CURRENT_EQ_T)];
		BranchCurrentEquality( V, BranchStatus, I_F, I_T, Error_F, Error_T );
	}
	if (UseBranchCurrentMagnitude_) {
		BranchCurrentMagnitude( I_F, I_T, &g[g_index(BRANCH_CURRENT_MAG)]);
	}
	if (UseVoltageMagnitude_) {
		VoltageMagnitude ( V, &g[g_index(VOLTAGE_MAGNITUDE)]);
	}
	return true;
}
/// calculate the Jacobian of the constraint functions
bool PowerSolver::eval_jac_g(Index n, const Number* x, bool new_x,
												Index m, Index nele_jac, Index* iRow, Index *jCol,
												Number* values)
{
	int jac_ix;
	const cx     *G            = (cx*) &x[x_index(GENERATION)];
	const cx     *V            = (cx*) &x[x_index(VOLTAGE)];
	const cx     *I_F          = (cx*) &x[x_index(CURRENT_F)];
	const cx     *I_T          = (cx*) &x[x_index(CURRENT_T)];
	const Number *BranchStatus = &x[x_index(BRANCH_STATUS)];
	const Number *DemandFactor = &x[x_index(DEMAND)];
	//const Number *GenStatus    = &x[x_index(GEN_STATUS)];
	//const Number *DemandStatus = &x[x_index(DEMAND_STATUS)];
		
	if (values==NULL) {
		if (UseGenLimits_) {
			jac_ix = jac_index(GEN_LIMITS);
			GenLimitsJacPattern ( g_index(GEN_LIMITS), iRow+jac_ix , jCol+jac_ix );
		}
		
		if (UseDemandLimits_) {
			jac_ix = jac_index(DEMAND_LIMITS);
			DemandLimitsJacPattern ( g_index(DEMAND_LIMITS), iRow+jac_ix , jCol+jac_ix );
		}
		
		if (UseDemandGenEquality_) {
			jac_ix = jac_index(DEMAND_GEN_EQUALITY);
			DemandGenEqualityJacPattern ( g_index(DEMAND_GEN_EQUALITY), iRow+jac_ix , jCol+jac_ix );
		}
		
		if (UseBusInjectionMismatch_) {
			jac_ix = jac_index(BUS_INJECTION_MISMATCH);
			BusInjectionJacPattern( g_index(BUS_INJECTION_MISMATCH), iRow+jac_ix , jCol+jac_ix );
		}
		if (UseBranchCurrentEquality_) {
			jac_ix = jac_index(BRANCH_CURRENT_EQ);
			BranchCurrentEqualityJacPattern( g_index(BRANCH_CURRENT_EQ), iRow+jac_ix , jCol+jac_ix );
		}
		// branch current magnitude pattern
		if (UseBranchCurrentMagnitude_) {
			jac_ix = jac_index(BRANCH_CURRENT_MAG);
			BranchCurrentMagnitudeJacPattern( g_index(BRANCH_CURRENT_MAG), iRow+jac_ix , jCol+jac_ix );
		}
		// for now just assert on the stuff that is not ready yet
		if (UseVoltageMagnitude_) {
			jac_ix = jac_index(VOLTAGE_MAGNITUDE);
			VoltageMagnitudeJacPattern ( g_index(VOLTAGE_MAGNITUDE), iRow+jac_ix , jCol+jac_ix );
		}
	} else {
		if (UseGenLimits_) {
			GenLimitsJac ( values + jac_index(GEN_LIMITS) );
		}
		
		if (UseDemandLimits_) {
			DemandLimitsJac ( values + jac_index(DEMAND_LIMITS) );
		}
		
		if (UseDemandGenEquality_) {
			DemandGenEqualityJac ( G, DemandFactor, values + jac_index(DEMAND_GEN_EQUALITY) );
		}	
		if (UseBusInjectionMismatch_) {
			BusInjectionJac ( G, DemandFactor, I_F, I_T, V, &values[jac_index(BUS_INJECTION_MISMATCH)] ) ;
		}
		if (UseBranchCurrentEquality_) {
			BranchCurrentEqualityJac( V, BranchStatus, &values[jac_index(BRANCH_CURRENT_EQ)] );
		}
		if (UseBranchCurrentMagnitude_) {
			BranchCurrentMagnitudeJac ( I_F, I_T, &values[jac_index(BRANCH_CURRENT_MAG)] );
		}
		// for now just assert on the stuff that is not ready yet
		if (UseVoltageMagnitude_) {
			VoltageMagnitudeJac ( V, &values[jac_index(VOLTAGE_MAGNITUDE)] );
		}
	}
	return true;
}
/// calculate the Hessian of the LaGrangian
 bool PowerSolver::eval_h(Index n, const Number* x, bool new_x,
										Number obj_factor, Index m, const Number* lambda,
										bool new_lambda, Index nele_hess, Index* iRow,
										Index* jCol, Number* values)
{
	const cx *G   = (cx*) &x[x_index(GENERATION)];
	const cx *V   = (cx*) &x[x_index(VOLTAGE)];
	const cx *I_F = (cx*) &x[x_index(CURRENT_F)];
	const cx *I_T = (cx*) &x[x_index(CURRENT_T)];
	const Number *BranchStatus = &x[x_index(BRANCH_STATUS)];
	int ix, i;
	
	// the hess for the objective function:
	if (values==NULL)
	{
		// objective function
		GenDispatchHessPattern( iRow, jCol );
		// bus injection mismatch
		if (UseBusInjectionMismatch_) {
			ix = hess_index(BUS_INJECTION_MISMATCH);
			BusInjectionHessPattern( &iRow[ix], &jCol[ix] );
		}
		// branch current eq
		if (UseBranchCurrentEquality_) {
			ix = hess_index(BRANCH_CURRENT_EQ);
			BranchCurrentEqualityHessPattern ( &iRow[ix], &jCol[ix] );
		}
		// branch current mag
		if (UseBranchCurrentMagnitude_) {
			ix = hess_index(BRANCH_CURRENT_MAG);
			BranchCurrentMagnitudeHessPattern ( &iRow[ix], &jCol[ix] );
		}
		// bus voltage mag
		if (UseVoltageMagnitude_) {
			ix = hess_index ( VOLTAGE_MAGNITUDE );
			VoltageMagnitudeHessPattern ( &iRow[ix], &jCol[ix] );
		}
		
	} else {
		// zero out all of the elements
		for (i=0; i<nele_hess; i++) {
			values[i]=0;
		}
		// hess for the objective
		GenDispatchHess( G, values, obj_factor );
		// hess for the bus injection mismatch
		if (UseBusInjectionMismatch_) {
			ix = hess_index(BUS_INJECTION_MISMATCH);
			BusInjectionHess ( &lambda[g_index(BUS_INJECTION_MISMATCH)], &values[ix] );
		}
		// hess for the branch current eq
		if (UseBranchCurrentEquality_) {
			ix = hess_index(BRANCH_CURRENT_EQ);
			BranchCurrentEqualityHess( V, BranchStatus, &lambda[g_index(BRANCH_CURRENT_EQ)], &values[ix] );
		}
		// hess for branch current magnitude
		if (UseBranchCurrentMagnitude_) {
			ix = hess_index(BRANCH_CURRENT_MAG);
			BranchCurrentMagnitudeHess( I_F, I_T, &lambda[g_index(BRANCH_CURRENT_MAG)], &values[ix] );
		}
		// bus voltage mag
		if (UseVoltageMagnitude_) {
			ix = hess_index(VOLTAGE_MAGNITUDE);
			VoltageMagnitudeHess( V, &lambda[g_index(VOLTAGE_MAGNITUDE)], &values[ix] );
		}
	}
	// the hess for the branch current eq cons
	
	return true;
}
/// do some clean up after the optimization has finished
void PowerSolver::finalize_solution(SolverReturn status,
																Index n, const Number* x, const Number* z_L, const Number* z_U,
																Index m, const Number* g, const Number* lambda,
																Number obj_value)
{
	int i;
	const cx     *G            = (cx*) &x[x_index(GENERATION)];
	const cx     *V            = (cx*) &x[x_index(VOLTAGE)];
	const cx     *I_F          = (cx*) &x[x_index(CURRENT_F)];
	const cx     *I_T          = (cx*) &x[x_index(CURRENT_T)];
	const Number *BranchStatus = &x[x_index(BRANCH_STATUS)];
	const Number *DemandFactor = &x[x_index(DEMAND)];
	const Number *GenStatus    = &x[x_index(GEN_STATUS)];
	const Number *DemandStatus = &x[x_index(DEMAND_STATUS)];
	
	if (status==SUCCESS) {
		//copy the results into the power system structure
		printf("Solution found. Writing result to the input structure");
		// write the voltage
		for (i=0;i<nBus_;i++) {
			PS_->bus[i].set_V(V[i]);
		}
		// write the gen data
		for (i=0;i<nGen_;i++) {
			PS_->gen[i].set_G(G[i]);
			PS_->gen[i].status = (status_e) GenStatus[i];
		}
		// write the load data
		for (i=0;i<nDemand_;i++) {
			PS_->load[i].Pd *= DemandFactor[i];
			PS_->load[i].Qd *= DemandFactor[i];
			PS_->load[i].status = (status_e) DemandStatus[i];
		}
		// write the branch data
		for (i=0;i<nBranch_;i++) {
			PS_->branch[i].set_voltages( abs( V[F_[i]] ), abs( V[T_[i]] ), arg( V[F_[i]] ) - arg( V[T_[i]] ) );
			PS_->branch[i].set_status( (status_e) BranchStatus[i] );
		}		
	} else {
		printf("Warning: Optimization problem did not converge\n");
	}
}
//@}
