#include <string>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include "PowerSystem.hpp"

using namespace std;

int main( int argc, char* argv[] )
{
	PowerSystem ps;
	char filename[100]  = "../../data/case6_mod.xml";
	ps.options.print_level = NO_PRINT;
	int i;
	
	// get the input file argument
	if (argc>1) strcpy(filename, argv[1]);
	
	// read the file
	if ( ps.read_file(filename,POWER_XML) ) {
		// Run the base case power flow
		printf("Trying to run the base case power flow\n");
		if ( !ps.runPowerFlow() ) {
			printf("Could not solve power flow\n");
			exit(0);
		} else {
			ps.print();
		}
		// Remove a line and re-run
		printf("Trying to run a power flow with a branch outage\n");
		ps.removeBranch(1);
		if ( !ps.runPowerFlow() ) {
			printf("Could not solve power flow\n");
			exit(0);
		} else {
			ps.print();
		}
	} else {
		printf("Could not open file %s\n",filename);
	}	
	printf("\n");
	// try the acsimsep solver
	Sim_Solver acsimsep;
	acsimsep.run(&ps);
	// exit
	return 0;
}

/*
	ps.options.print_level=VERBOSE;
	ps.runPowerFlow();
	ps.removeBranch(4);
	ps.removeBranch(5);
	
	// simulate the control actions
	for ( i=0; i<10; i++ )
	{
		if ( ps.runPowerFlow() )
		{
			ps.solveMPC ( delta_Pg, delta_Pd );
			delta_Pg.print("delta_Pg");
			delta_Pd.print("delta_Pd");
			ps.control ( delta_Pg, delta_Pd );
		}
		else
		{
			
			printf("\n Could not solve power flow \n");
			break;
		}
	}

*/

	/*
	//int i, N=10;
	//double time_sum=0;
	Sparse delta_Pg, delta_Pd;
	
	// set the options
	ps.options.print_level=NO_PRINT;
	ps.options.MPC_K = 2;
	
	// read the data
	if ( ps.read(filename,POWER_XML) ) printf("\nData read OK\n");
	else
	{
		printf("\nCould not read data\n");
		return 0;
	}
	// run power flow
	ps.runPowerFlow();
	ps.print();

	
	// inflict the disturbance
	ps.removeBranch(4);
	ps.removeBranch(5);
	
	// simulate the control actions
	for ( i=0; i<10; i++ )
	{
		if ( ps.runPowerFlow() )
		{
			ps.solveMPC ( delta_Pg, delta_Pd );
			delta_Pg.print("delta_Pg");
			delta_Pd.print("delta_Pd");
			ps.control ( delta_Pg, delta_Pd );
			
			if (ps.options.MPC_K>1) ps.options.MPC_K--;
		}
		else
		{
			
			printf("\n Could not solve power flow \n");
			break;
		}
	} */
	/*
	for (i=0; i<N; i++)
	{
		// first power flow
		if ( ps.read(filename) ) printf("\nOK");
		else printf("\nnot OK");
		tic();
		if (ps.runPowerFlow())
			printf("\nPower flow converged");
		else
			printf("\nPower flow did not converge");
		time_sum += toc();
	}
	printf("\nAverage power flow completion time = %f.\n", time_sum/N);
	*/	
	/*
	// test the neighbor set routine
	{
		unsigned i;
		std::vector< std::set<int> > neighbors;
		ps.neighbor_sets( 65, 1, 2, neighbors );
		for (i=0; i<neighbors.size(); i++ )
		{
			cout << "\nneighbors " << i << endl;
			copy( neighbors[i].begin(), neighbors[i].end(), ostream_iterator<const int>(cout, " ") );
			cout << endl;
		}
	}
	
	{
		std::string str;
		
		ps.bus[1].print();
		ps.bus[1].write(str);
		cout << str << endl;
		ps.bus[1].read(str);
		ps.bus[1].print();
		ps.bus[1].write(str);
		cout << str << endl;
	}
*/
