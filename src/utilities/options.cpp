#include "options.hpp"
#include "../regex/regex.h"
#include <iostream>
#include <fstream>

using namespace std;

/// read the options found in @param filename, and put them in the structure
bool options_t::read_file( const std::string & filename, bool showErrorMessages ) {
	string line, option, value;
	ifstream file( filename.c_str() );
	
	if ( file.is_open() ) {
		while ( !file.eof() ) {
			match_t match;
			getline( file, line );
			if ( regex( line, "(\\w+)\\s*=\\s*([-a-z_A-Z0-9.+]+)", match ) ) {
				opts_[ match[1] ] = match[2];
			}
		}
		file.close();
	} else {
		if( showErrorMessages ) {
			printf("Could not open options file: %s\n", filename.c_str() );
		}
		return false;
	}
	return true;
}

/// get the option with name @param opt_name
std::string options_t::get( const std::string & opt_name ) {
	string result;
	
	if ( opts_.find( opt_name ) != opts_.end() ) {
		result = opts_[opt_name];
	}
	return result;
}

/// get a numeric option
double options_t::get_numeric( const std::string & opt_name ) {
	return atof( get( opt_name ).c_str() );
}

/// set a string value
void options_t::set( const std::string & opt_name, const std::string & value ) {
	opts_[ opt_name ] = value;
}

/// set a numeric value
void options_t::set( const std::string & opt_name, double value ) {
	char str[100];
	
	sprintf( str, "%g", value );
	opts_[ opt_name ] = str;
}
