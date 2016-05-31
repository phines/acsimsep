
#include <assert.h>
#include <stdlib.h>
#include "PowerGlobals.hpp"
#include "../regex/regex.hpp"

int read_line( const std::string& line, std::vector<double>& data )
{
	int n = 0;
	matchVec_t matches;
	double num;
	
	data.clear();

	while ( regex(line, "([0-9eE+\\-.]+)\\s*", matches) )
	{
		num = atof(matches[0].c_str());
		data.push_back(num);
		n++;
	}
	return n;
}

int read_line(const char* line, std::vector<double>& data)
{
	std::string str;
	
	str.assign(line);
	
	return read_line (str, data);
}
