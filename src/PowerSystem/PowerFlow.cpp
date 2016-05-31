#include "PowerFlow.hpp"
#include "../matrix/matrix.hpp"
#include <set>
#include <cassert>

using namespace std;

/// default options for the power flow solver
PowerFlowOptions::PowerFlowOptions() {
	convergence_eps = 1e-6;
	max_iterations  = 20;
	print_level     = 0;
	flat_start      = false;
	max_line_search_iterations = 10;
}

class PowerFlowSolver {
	public:
		/// solve
		bool solve( Sparse_cx &Ybus, bus_type_e *types, DenseVector_cx &Sbus, DenseVector_cx &V, DenseVector_cx & mis_cx,
					PowerFlowOptions & options_in )
		{
			// variables
			double step_size=0.0, total_mismatch, old_mismatch=1e6, cur_mismatch;
			int i, iteration, nX;
			DenseVector x;
			DenseVector new_x;
			DenseVector mis;
			DenseVector direction;
			Sparse Jac;
			
			options = options_in;
			
			// code
			clear(); // clear out all of the local data members
			nBus = Ybus.rows();
			if( not record_bus_types( types ) ) return false;
			find_neighbors( Ybus );
			nX = nPQ() + nBus - 1;
			Jac.resize( nX, nX );
			x  .resize( nX, 0  );
			new_x.resize( nX, 0 );
			mis.resize( nX, 0  );
			mis_cx.resize( nBus );
			
			prepare_voltages( V, options.flat_start );
			total_mismatch = calc_mismatch( Ybus, V, Sbus, mis_cx, mis );
			// set to flat start if we are starting from a strange position
			if( total_mismatch>1e5 ) {
				set_flat_start( V );
				total_mismatch = calc_mismatch( Ybus, V, Sbus, mis_cx, mis );
			}
			// begin the inner power-flow loop
			for( iteration=1; iteration<=options.max_iterations; iteration++ ) {
				// print something
				if ( options.print_level>0 ) {
					printf("Power flow iteration %2d of %2d, Mismatch = %12g\n",
						   iteration, options.max_iterations, total_mismatch );
				}
				// check for convergence			
				if ( total_mismatch < options.convergence_eps ) {
					update_Sbus( mis_cx, Sbus );
					return true;
				}
				// get the Jac
				calc_jacobian( Ybus, V, Jac );
				// solve the linear system
				//Jac.print("Jac");
				//x.print("x");
				//exit(0);
				if ( not direction.solve( Jac, mis ) ) {
					printf("Could not solve linear system in power flow.\n");
					return false;
				}
				// get the current x
				V_to_x( V, x );
				// choose a step size
				step_size = 1.0;
				int step_it=0;
				do {
					// recalculate the decision vector
					for( i=0; i<nX; i++ ) {
						new_x[i] = x[i] - step_size*direction[i];
					}
					// change x to V
					x_to_V( new_x, V );
					// check the mismatch
					cur_mismatch = calc_mismatch( Ybus, V, Sbus, mis_cx, mis );
					if ( options.print_level>1 ) {
						printf("  Search Iteration %2d of %2d: Step-size = %12g, Mismatch = %12g;\n", 
							   step_it, options.max_line_search_iterations, step_size, cur_mismatch );
					}
					if( cur_mismatch < options.convergence_eps ) break;
					if ( step_it == options.max_line_search_iterations ) break;
					step_size *= 0.5;
					step_it++;
				} while ( cur_mismatch > total_mismatch );
				old_mismatch = total_mismatch;
				total_mismatch = cur_mismatch;
				
				// update x
				x.copy( new_x );
			}		
			return false;
		}
	private:
		/// return the phase angle of the complex value
		inline double angle( complex<double> & value ) { return arg(value); }
		/// change a complex voltage vector to an x vector
		///  x = [theta_pv; theta_pq; Vmag_pq];
		void V_to_x( DenseVector_cx &V, DenseVector &x ) {
			set<int>::iterator i;
			int i_x = 0;
			
			// theta_pv
			for( i=pv_list.begin(); i!=pv_list.end(); i++ ) {
				x[i_x] = angle( V[*i] );
				i_x++;
			}
			// theta_pq
			for( i=pq_list.begin(); i!=pq_list.end(); i++ ) {
				x[i_x] = angle( V[*i] );
				i_x++;
			}
			// Vmag_pq
			for( i=pq_list.begin(); i!=pq_list.end(); i++ ) {
				x[i_x] = abs( V[*i] );
				i_x++;
			}
		}
		/// change an x vector to complex voltage
		void x_to_V( DenseVector &x, DenseVector_cx &V ) {
			set<int>::iterator i;
			int i_x = 0;
			int n_pq = nPQ();
			double Vmag, theta;
			
			// go through the pv buses
			for( i=pv_list.begin(); i!=pv_list.end(); i++ ) {
				Vmag  = abs( V[ *i ] );
				theta = x[ i_x ];
				V[*i] = polar( Vmag, theta );
				i_x++;
			}
			// go through the pq buses
			for( i=pq_list.begin(); i!=pq_list.end(); i++ ) {
				Vmag  = x[ i_x + n_pq ];
				theta = x[ i_x ];
				V[*i] = polar( Vmag, theta );
				i_x++;
			}
		}
		/// calculate the mismatch
		double calc_mismatch( Sparse_cx & Ybus, DenseVector_cx & V, DenseVector_cx & Sbus, DenseVector_cx & mis_cx, DenseVector & mis ) {
			DenseVector_cx Ibus(nBus);
			set<int>::iterator iter;
			int i=0, i_x=0, n_pq=nPQ();
			
			// calculate Ibus = Ybus*V
			Ibus.mult( Ybus, V );
			
			// calculate mismatch = V .* conj( Ibus ) - Sbus
			for( i=0; i<nBus; i++ ) {
				mis_cx[i] = V[i] * conj( Ibus[i] ) - Sbus[i];
			}			
			// go through pv_list
			for( iter=pv_list.begin(); iter!=pv_list.end(); iter++ ) {
				mis[i_x] = mis_cx[*iter].real();
				i_x++;
			}
			// and pq_list
			for( iter=pq_list.begin(); iter!=pq_list.end(); iter++ ) {
				mis[i_x]      = mis_cx[*iter].real();
				mis[i_x+n_pq] = mis_cx[*iter].imag();
				i_x++;
			}
			return mis.max_abs();
		}
		/// calculate the jacobian
		bool calc_jacobian( Sparse_cx & Ybus, DenseVector_cx & V, Sparse & Jac ) {
			set<int>::iterator iter;
			int    i, j, n=nBus;
			int    dP_row, dQ_row, dVmag_col, dTheta_col;
			double dP_dTheta_diag, dP_dVmag_diag, dQ_dTheta_diag, dQ_dVmag_diag;
			double Vmag_i, Vmag_j, theta_i, theta_j;
			double theta_ij, sin_theta_ij, cos_theta_ij;
			double b_ii, g_ii, b_ij, g_ij;
			cx	   y_ij, y_ii;
			
			// go through each bus and calculate the Jac elements
			for( i=0; i<n; i++ ) {
				// calculate the row indeces
				dP_row = Jac_index[i];
				dQ_row = Jac_index[i+n];
				// extract data that will be useful
				y_ii = Ybus(i,i);
				g_ii = y_ii.real();
				b_ii = y_ii.imag();
				theta_i = angle( V[i] );
				Vmag_i  =   abs( V[i] );
				// initialize the diagonal elements
				dP_dVmag_diag  =  2*Vmag_i*g_ii;
				dQ_dVmag_diag  = -2*Vmag_i*b_ii;
				dP_dTheta_diag = 0; // not sure about this term...???
				dQ_dTheta_diag = 0; // not sure about this term...???
		
				// for each connected bus
				for( iter=neighbors[i].begin(); iter!=neighbors[i].end(); iter++ )
				// calculate each of the following elements
				{
					// figure the index variables
					j = *iter;
					dTheta_col = Jac_index[j];
					dVmag_col  = Jac_index[j+n];
					// extract useful data
					y_ij = Ybus.get(i,j);
					g_ij = y_ij.real();
					b_ij = y_ij.imag();
					theta_j = angle( V[j] );
					Vmag_j  =   abs( V[j] );
					theta_ij = theta_i - theta_j;
					cos_theta_ij = cos( theta_ij );
					sin_theta_ij = sin( theta_ij );
			
					// off-diag elements:
					if ( dP_row != POWER_EMPTY )
					{
						// dP_dVmag  for off-diag, see p.174 of Bergen, "Power Systems Analysis"
						if ( dVmag_col != POWER_EMPTY )
							Jac.set( dP_row, dVmag_col, Vmag_i*( g_ij*cos_theta_ij + b_ij*sin_theta_ij ) ); //ok
						// dP_dTheta for off-diag, see p.174 of Bergen (1986), "Power Systems Analysis"
						if ( dTheta_col != POWER_EMPTY )
							Jac.set( dP_row, dTheta_col, Vmag_i*Vmag_j*(g_ij*sin_theta_ij - b_ij*cos_theta_ij) );//ok
					}
			
					if (dQ_row != POWER_EMPTY)
					{
						// dQ_dVmag  for off-diag, see p.174 of Bergen (1986), "Power Systems Analysis"
						if ( dVmag_col != POWER_EMPTY )
							Jac.set( dQ_row, dVmag_col, Vmag_i*( g_ij*sin_theta_ij - b_ij*cos_theta_ij ) ); //ok
						// dQ_dTheta for off-diag, see p.175 of Bergen (1986), "Power Systems Analysis"
						if ( dTheta_col != POWER_EMPTY )
							Jac.set(dQ_row, dTheta_col, -Vmag_i*Vmag_j*( g_ij*cos_theta_ij + b_ij*sin_theta_ij ) ); //ok
					}
					// diagonal elements: again see p. 174, 175 of Bergen (1986), "Power Systems Analysis"
					// dP_dTheta for diagonal
					dP_dTheta_diag += Vmag_i*Vmag_j*( -g_ij*sin_theta_ij + b_ij*cos_theta_ij ); //ok
					// dQ_dTheta for diagonal
					dQ_dTheta_diag += Vmag_i*Vmag_j*(  g_ij*cos_theta_ij + b_ij*sin_theta_ij ); //ok
					// dP_dVmag  for diagonal
					dP_dVmag_diag  += Vmag_j*( g_ij*cos_theta_ij + b_ij*sin_theta_ij ); //ok
					// dQ_dVmag  for diagonal
					dQ_dVmag_diag  += Vmag_j*( g_ij*sin_theta_ij - b_ij*cos_theta_ij ); //ok
				}
				// put the diagonal elements into the matrix
				dVmag_col  = Jac_index[i+n];
				dTheta_col = Jac_index[i];
				if (dP_row != POWER_EMPTY)
				{
					if (dTheta_col != POWER_EMPTY)
						Jac.set( dP_row, dTheta_col, dP_dTheta_diag );
					if (dVmag_col != POWER_EMPTY)
						Jac.set( dP_row, dVmag_col, dP_dVmag_diag );
				}
				if (dQ_row != POWER_EMPTY)
				{
					if (dTheta_col != POWER_EMPTY)
						Jac.set( dQ_row, dTheta_col, dQ_dTheta_diag );
					if (dVmag_col != POWER_EMPTY)
						Jac.set( dQ_row, dVmag_col, dQ_dVmag_diag );
				}
			}
			//Jac.print("Jac1");
			//exit(0);
			return true;
		}
		/// update the Sbus given
		void update_Sbus( DenseVector_cx & mis_cx, DenseVector_cx & Sbus ) {
			int i;
			
			for( i=0; i<nBus; i++ ) {
				Sbus[i] += mis_cx[i];
			}
		}
		/// record the bus types
		bool record_bus_types( bus_type_e *types ) {
			int i, i_x=0, nRef=0;
			set<int>::iterator iter;
			
			pv_list.clear();
			pq_list.clear();
			
			for( i=0; i<nBus; i++ ) {
				switch( types[i] ) {
					case PV:  pv_list.insert(i); break;
					case PQ:  pq_list.insert(i); break;
					case REF: ref=i; nRef++; break;
					default:  throw "Unsupported bus type.";
				}
			}
			if ( nRef == 0 ) {
				if ( nPV()>0 ) {
					printf("No reference bus found, changing to the first PV bus.\n");
					ref = *pv_list.begin();
					pv_list.erase(ref);
					types[ref] = REF;
				} else {
					printf("Could not find a reference bus.\n");
					return false;
				}
			}
			if ( nRef > 1 ) {
				printf("More than one reference (swing/slack) bus is not allowed.\n");
				return false;
			}
			// update the Jacobian index
			Jac_index.clear(); Jac_index.resize( nBus*2, POWER_EMPTY );
			// go through pv_list
			for( iter=pv_list.begin(); iter!=pv_list.end(); iter++ ) {
				Jac_index[*iter] = i_x;
				i_x++;
			}
			// and pq_list
			for( iter=pq_list.begin(); iter!=pq_list.end(); iter++ ) {
				Jac_index[ *iter ] = i_x;
				Jac_index[ *iter+nBus ] = i_x + nPQ();
				i_x++;
			}
			return true;
		}
		/// check for strange voltage values
		void prepare_voltages( DenseVector_cx & V, bool set_flat=false ) {
			int i, n=V.length();
			double Vmag, theta;
			
			if (set_flat) {
				set_flat_start(V);
				return;
			}
			// check for strange voltage values
			for( i=0; i<n; i++ ) {
				Vmag  = std::abs(V[i]);
				theta = std::arg(V[i]);
				if( isnan(Vmag) or isinf(Vmag) or isnan(theta) or isinf(theta) or Vmag>2.0 or Vmag<0.5 ) {
					set_flat_start(V);
					return;
				}
			}
		}
		/// set to flat start mode
		void set_flat_start( DenseVector_cx & V ) {
			set<int>::iterator iter;
			
			if( options.print_level>0 ) printf("Setting voltages to flat-start.\n");
			// PV buses
			for( iter=pv_list.begin(); iter!=pv_list.end(); iter++ ) {
				V[*iter] = std::abs(V[*iter]);
			}
			// PQ buses
			for( iter=pq_list.begin(); iter!=pq_list.end(); iter++ ) {
				V[*iter] = 1.0;
			}
			V[ref] = std::abs(V[ref]);
		}
		/// figure out which nodes are neighboring
		void find_neighbors( Sparse_cx & Ybus ) {
			unsigned i=0, j=0;
			complex<double> value;
			
			neighbors.resize(nBus);
			for( i=0; i<(unsigned)nBus; i++ ) {
				Ybus[i].reset_next();
				while( Ybus[i].get_next( j, value ) ) {
					if ((int)i>=nBus or (int)j>=nBus) {
						Ybus.print("Ybus");
						throw "Error in Ybus";
					}
					if (i!=j) neighbors[i].insert( (int)j );
				}
			}
		}
		void clear() {
			pv_list.clear();
			pq_list.clear();
			Jac_index.clear();
			neighbors.clear();
			ref  = -1;
			nBus = -1;
		}
		set<int> pv_list;
		set<int> pq_list;
		vector<int>  Jac_index; ///< an index used to find locations within the Jacobian
		vector< set<int> > neighbors;
		int ref;
		int nBus;
		int nPQ() { return pq_list.size(); }
		int nPV() { return pv_list.size(); }
		PowerFlowOptions options;
} solver;

bool SolvePowerFlow( Sparse_cx        &Ybus,     //the Ybus (network admittance) matrix
					 bus_type_e       *BusTypes, // a vector of bux types
					 DenseVector_cx   &Sbus,     // a vector with the bus power injections
					 DenseVector_cx   &V,        // a vector with the bus voltages
					 DenseVector_cx   &Mismatch, // the output mismatch vector
					 PowerFlowOptions options )
{
	return solver.solve( Ybus, BusTypes, Sbus, V, Mismatch, options );
}


