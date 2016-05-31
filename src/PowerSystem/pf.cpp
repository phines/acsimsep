/** mpc.cpp
  * This file applies the MPC algorithm in the PowerSystem class to a power network. 
  */ 
 
#include "PowerSystem.hpp"
#include <string>
#include <iostream>
#include <iterator>
#include <stdio.h>

using namespace std;

int main( int argc, char* argv[] )
{
	PowerSystem sys;      // An object that holds the actual power system data
	char filename[1000];  // The location of the input data file= "case6_mod.xml";
	
	// get the input file argument
	if (argc>1) {
		strcpy(filename, argv[1]);
	} else {
		printf("No input data given. Using default.\n");
		strcpy(filename, "case300_mod.xml");
	}
	// get the output file
	if (argc>2) strcpy(filename, argv[2]);
		
	// set some options
	sys.options.print_level=VERBOSE;
	
	// read the file
	if ( sys.read_file( filename, POWER_XML ) )
	{
		printf("Finished reading the input file: %s\n", filename);
		// initialize the simulation data
		if ( sys.runPowerFlow( ) ) {
			sys.print();
		} else {
			printf("Could not solve power flow.\n");
		}
	} else {
		printf("Could not read data file.\n");
	}
	return 0;
}
