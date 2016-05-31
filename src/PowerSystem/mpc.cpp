/** mpc.cpp
  * This file applies the MPC algorithm in the PowerSystem class to a power network. 
  */ 

#include "PowerSystem.hpp"
#include "../regex/regex.hpp"
#include <string>
#include <iostream>
#include <iterator>
#include <stdio.h>

using namespace std;

int main( int argc, char* argv[] )
{
	PowerSystem sys;      // An object that holds the actual power system data
	char default_filename[] = "case300_001_003.xml";
	char filename[1000];  // The location of the input data file= "case6_mod.xml";
	Sparse dPg, dPd, dVg; // The sparse matrices that hold the control actions to be taken
	SparseVector dVmag, dImag;
	double dTheta_slack;
	unsigned number, k;    // temporary variables to hold the row and column of matrix entries
	double value;         // temporary variable to hold the value associated with number and k above
	bool do_control=true;
	int i, nViols;
	match_t match;
	
	// get the input file argument
	if (argc>1) {
		strcpy(filename, argv[1]);
	} else {
		printf( "No input data given. Using default: %s.\n", default_filename );
		strcpy( filename, default_filename );
	}
	// process the remaining inputs
	if (argc>2) {
		for( i=2; i<argc; i++ ) {
			if( regex( argv[i], "--no-control" ) )   do_control=false;
			if( regex( argv[i], "--options=(.*)" ) ) strcpy( filename, match[1].c_str() );
		}
	}
		
	// set some options
	sys.options.print_level=MINIMAL;
	
	// read the file
	printf("Trying to read data file: %s\n", filename);
	if ( sys.read_file( filename, POWER_XML ) ) {
		// begin
		if ( not sys.runPowerFlow() ) {
			printf("Could not solve initial power flow.\n");
			exit(1);
		}
		if (sys.nBus()<100) {
			printf("Completed reading data file.  Here is the data after an initial power flow:\n");
			sys.print();
		}
		// initialize the simulation data
		printf("Begining the simulation.\n");
		sys.sim_init();
		// start looping through the simulation
		while ( sys.sim_step() ) {
			// print the time
			printf(" t = %f\n", sys.sim_t() );
			// print out any violations
			nViols = sys.print_violations();
			
			if ( do_control and nViols>0 ) {
				// calculate the mpc actions for this time period
				if ( sys.SolveMPC( dPd, dPg, dVg, dVmag, dImag, dTheta_slack ) ) {
					// if the set of generator control actions is non-empty print some info
					if (dPg.nnz()>0) {
						printf("%3d gen shedding actions calculated:\n", dPg.nnz());
						dPg.reset_next();
						while ( dPg.get_next( number, k, value ) ) {
							printf("  Change gen %3d power at t_%d by %f;\n", sys.gen(number).number, k, value);
						}
					}
					// if the set of generator control actions is non-empty print some info
					if (dVg.nnz()>0) {
						printf("%3d gen voltage change actions calculated:\n", dVg.nnz());
						dVg.reset_next();
						while ( dVg.get_next( number, k, value ) ) {
							printf("  Change gen %3d (bus %3d) voltage at t_%d by %f;\n", sys.gen(number).number, sys.gen(number).busNo, k, value);
						}
					}
					// if the set of demand control actions is non-empty print some info
					if (dPd.nnz()>0) {
						printf("%3d demand reduction actions calculated:\n", dPd.nnz());
						dPd.reset_next();
						while ( dPd.get_next( number, k, value ) ) {
							printf("  Change load %3d (bus %3d) at t_%d by %f;\n", sys.load(number).number, sys.load(number).busNo, k, value);
						}
					}
					// implement the calculated control actions
					sys.control( dPd, dPg, dVg );
				} else {
					printf("Could not find a solution to the mpc problem.\n");
				}
			}
		}
		nViols = sys.print_violations();
		printf("Simulation completed with %3d violations outstanding.\n", nViols);
	}
	printf("\n");
	// exit
	return 0;
}
