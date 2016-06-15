#include "PowerElement.hpp"
#include <cstdlib>

int power_element_t::nextNo=1;

std::complex<double> read_complex ( const std::string &data )
{
	std::complex<double> value(0);
	double re=0, im=0;
	match_t match;
	
	if ( regex ( data, "([.0-9\\-]+).*?([.0-9\\-]+)", match ) )
	{
		re = atof( match[1].c_str() );
		im = atof( match[2].c_str() );
		value = std::complex<double>( re, im );
	}
	return value;
}


bool is_unknown( double value ) {
	return std::abs( value - POWER_UNKNOWN ) < 2.0;
}

bool is_unknown( std::complex<double> value ) {
	return std::abs( value.real() - POWER_UNKNOWN ) < 2.0;
}

