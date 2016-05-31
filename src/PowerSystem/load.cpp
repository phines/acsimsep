////////////////////////////////////////////////////////////////////////////////
// Implementation of load_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#include "load.hpp"
#include "../regex/regex.hpp"
#include <sstream>
#include <cstdio>

#define DEFAULT_LOAD_VALUE 1000

using namespace std;

// constructor
load_t::load_t() {
  number = POWER_EMPTY;
  busNo  = POWER_EMPTY;
  bi     = POWER_EMPTY;
  Pd     = 0;
  Qd     = 0;
  Pmax   = 0;
  Qmax   = 0;
  Gs     = 0;
  Bs     = 0;
  Ire    = 0;
  Iim    = 0;
  value  = DEFAULT_LOAD_VALUE;
  status = ON;
  strcpy(name, "");
  T_measured=0;
}
// destructor
load_t::~load_t() {
  // nothing here to kill
}
// print
void load_t::print( string &output ) const {
	char str[1000];
	// if this is the first one, print the column headers:
	if ( ix()==0 ) {
		output.append("  Num   Bus      P        Q         Gs        Bs       value  status\n");
	}
	// otherwise just print the data:
	sprintf( str, "%6d %6d %8.2f %8.2f %9.4f %9.4f %8.1f  %d\n", number, busNo, Pd, Qd, Gs, Bs, value, (int)status );
	output.append(str);
}

/// read
bool load_t::read( const string &data, data_format_e format )
{
	switch (format)
	{
		case MATPOWER:
			return read_matpower(data);
			break;
		case POWER_XML:
			return read_xml(data);
			break;
		default:
			return read_xml(data);
			break;
	}
	return false;
}

bool load_t::read( MyXmlNode & load_node ) {
	MyXmlNode node;
	string tag, content;
	complex<double> cxNumber;
	double time;
	int num;
	
	// some error checking
	err_if( load_node.name()!="load", "It doesn't look like this is a load node." );
	num = (int) load_node.get_numeric_attribute( "number" );
	err_if( num!=number, "Invalid load number in load_t::read" );
	
	// first get the time for the measurements if it is included
	time = load_node.get_child_numeric_content( "time" );
	if ( (int)time!=MYXML_EMPTY and time < T_measured ) return true;
	if ( time>=T_measured ) T_measured=time;
	// potential load tags:
	// matpower, S, Smax, P, Q, Pmax, Qmax, I, Ire, Iim, Ys, Gs, Bs, status, name
	for( node=load_node.child(); not node.is_null(); node = node.next() ) {
		tag = node.name();
		content = node.content();
		// deal with each potential tag
		if      (tag=="matpower") read_matpower(content);
		else if (tag=="S") {
			cxNumber = read_complex(content);
			Pd = cxNumber.real();
			Qd = cxNumber.imag();
		}
		else if (tag=="Smax") {
			cxNumber = read_complex(content);
			Pmax = cxNumber.real();
			Qmax = cxNumber.imag();
		}
		else if (tag=="Ys")	{
			cxNumber = read_complex(content);
			Gs = cxNumber.real();
			Bs = cxNumber.imag();
		}
		else if (tag=="I") {
			cxNumber = read_complex(content);
			Ire = cxNumber.real();
			Iim = cxNumber.imag();
		}
		else if (tag=="P")      Pd   = atof(content.c_str());
		else if (tag=="Q")      Qd   = atof(content.c_str());
		else if (tag=="Gs")     Gs   = atof(content.c_str());
		else if (tag=="Bs")     Bs   = atof(content.c_str());
		else if (tag=="Ire")    Ire  = atof(content.c_str());
		else if (tag=="Iim")    Iim  = atof(content.c_str());
		else if (tag=="Pmax")   Pmax = atof(content.c_str());
		else if (tag=="Qmax")   Pmax = atof(content.c_str());
		else if (tag=="busNo")  busNo = atoi(content.c_str());
		else if (tag=="value")  value = atof(content.c_str());
		else if (tag=="name")   strcpy( name, content.c_str() );
		else if (tag=="status") status = (status_e) atoi( content.c_str() );
	}
	return true;
	
}


/// read_matpower
bool load_t::read_matpower ( const string& data )
{
	istringstream ins(data);
	double num;
	
	// extract
	ins >> busNo;
	ins >> value;
	ins >> Pd;
	if ( !(ins >> Qd) ) return false;	
	ins >> Gs;
	ins >> Bs;
	ins >> Ire;
	ins >> Iim;
	if (ins >> num) status = (status_e) num;
	ins >> Pmax;
	ins >> Qmax;
	if (number==POWER_EMPTY)
	{
		number=nextNo;
		nextNo++;
	}
	return true;
}

/// read_xml
bool load_t::read_xml ( const string& data )
{
	match_t match;
	int data_index=0;
	string tag, content;
	complex<double> number;
	
	// first get the time for the measurements if it is included
	if ( regex ( data, "<time>(.*?)</time>", match ) ) T_measured = atof ( match[1].c_str() );
	// potential load tags:
	// matpower, S, Smax, P, Q, Pmax, Qmax, I, Ire, Iim, Ys, Gs, Bs, status, name
	while ( get_next_xml ( data, tag, content, data_index ) )
	{
		// deal with each potential tag
		if      (tag=="matpower") read_matpower(content);
		else if (tag=="S")
		{
			number = read_complex(content);
			Pd = number.real();
			Qd = number.imag();
		}
		else if (tag=="Smax")
		{
			number = read_complex(content);
			Pmax = number.real();
			Qmax = number.imag();
		}
		else if (tag=="Ys")
		{
			number = read_complex(content);
			Gs = number.real();
			Bs = number.imag();
		}
		else if (tag=="I")
		{
			number = read_complex(content);
			Ire = number.real();
			Iim = number.imag();
		}
		else if (tag=="P")      Pd = atof(content.c_str());
		else if (tag=="Q")      Qd = atof(content.c_str());
		else if (tag=="Gs")     Gs = atof(content.c_str());
		else if (tag=="Bs")     Bs = atof(content.c_str());
		else if (tag=="Ire")    Ire = atof(content.c_str());
		else if (tag=="Iim")    Iim = atof(content.c_str());
		else if (tag=="Pmax")   Pmax = atof(content.c_str());
		else if (tag=="Qmax")   Pmax = atof(content.c_str());
		else if (tag=="busNo")  busNo = atoi(content.c_str());
		else if (tag=="value")  value = atof(content.c_str());
		else if (tag=="name")   strcpy( name, content.c_str() );
		else if (tag=="status") status = (status_e) atoi( content.c_str() );
	}
	return true;
}

///  write formated output
bool load_t::write ( string &output, data_format_e format ) const
{
	switch (format)
	{
		case POWER_XML:
			return write_xml(output);
			break;
		case MATPOWER:
			return write_matpower(output);
			break;
		default:
			return write_xml(output);
	}
	return false;
}

/// write_xml
bool load_t::write_xml ( string &output ) const
{
	char str[2000];
	
	output.clear();
	sprintf(str, "<load number=\"%d\">",number);
	output.append(str);
	// busNo
	sprintf( str, "<busNo>%d</busNo>", busNo );
	output.append(str);
	// S, I, Ys
	sprintf( str, "<S>%g, %g</S> <I>%g, %g</I> <Ys>%.6g, %.6g</Ys> <Smax>%g, %g </Smax>",
	                 Pd, Qd,        Ire, Iim,       Gs, Bs,          Pmax, Qmax  );
	output.append(str);
	// status, value
	sprintf(str, "<status>%d</status><value>%g</value>", (unsigned)status, value);
	output.append(str);
	output.append("</load>\n");
	
	return true;
}

/// write_matpower
bool load_t::write_matpower(string& output) const
{
	char str[2000];
              //bn va P  Q  G  B  Ir Ii st Px Qx

	sprintf(str, "%d %g %g %g %g %g %g %g %d %g %g",
		busNo, value, Pd, Qd, Gs, Bs, Ire, Iim, status, Pmax, Qmax);
	output.assign(str);
	
	return true;
}

/// operators
bool load_t::operator==(const load_t& other) const
{
	if (other.number == number)
		return true;
	else
		return false;
}
bool load_t::operator< (const load_t& other) const
{
	if ( number < other.number )
		return true;
	else
		return false;
}
bool load_t::operator> (const load_t& other) const
{
	if ( number > other.number )
		return true;
	else
		return false;
}

void load_t::change(double delta)
{
	if ( Pd+delta > 0)
	{
		double ratio = Qd/Pd;
		Pd += delta;
		Qd += (delta*ratio);
	}
	else if (Pd>0)
	{
		Pd=0;
		Qd=0;
		status=OFF;
	}
}
