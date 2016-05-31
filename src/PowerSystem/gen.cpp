////////////////////////////////////////////////////////////////////////////////
// Definition of gen_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#include "gen.hpp"
#include "bus.hpp"
#include "../regex/regex.hpp"
#include <cstdio>
#include <cstring>
#include <sstream>

using namespace std;

// constructor
gen_t::gen_t() {
  number = POWER_EMPTY;
  busNo  = POWER_EMPTY;
  bi     = POWER_EMPTY;
  Pg     = 0;
  Pmax   = 0;
  Pmin   = 0;
  Qg     = 0;
  Qmax   = 0;
  Qmin   = 0;
  Pref   = 0;
  Vref   = 1.0;
  mu_Pmax = 0;
  mu_Pmin = 0;
  mu_Qmax = 0;
  mu_Qmin = 0;
  T_measured=0;
  
  strcpy(name," ");
  status  = ON;
}

// destructor
gen_t::~gen_t() {
  // nothing here...
}

// print
void gen_t::print( string &output ) const {
	char str[1000];
	// if this is the first one print a header:
	if ( ix()==0 ) {
		output.append("  Num  Bus   P         Q        |V| status \n");
	}
	// otherwise just print the data
	sprintf( str, "%6d %6d %8.2f %8.2f %8.4f  %d\n", number, busNo, Pg, Qg, Vref, (int)status );
	output.append(str);
}

/// read
bool gen_t::read( const string &data, data_format_e format ) {
	switch (format) {
		case MATPOWER:   return read_matpower(data);
		case POWER_XML:  return read_xml(data);
		default:         return read_xml(data);
	}
	return false;
}

/// read_matpower: read a line of data in matpower sequence
bool gen_t::read_matpower(  const string& data )
{
	istringstream ins(data);
	double num;
	
	if ( data.size() < 10 ) return false;
	// extract
	ins >> busNo;
	ins >> Pg;
	ins >> Qg;
	ins >> Qmax;
	ins >> Qmin;
	ins >> Vref;
	ins >> mBase;
	if (ins >> num) status = (status_e) num;
	ins >> Pmax;
	if ( !(ins >> Pmin) ) { return false; };
	ins >> mu_Pmax;
	ins >> mu_Pmin;
	ins >> mu_Qmax;
	ins >> mu_Qmin;
	// if the number is empty fill it in
	if (number==POWER_EMPTY)
	{
		number = nextNo;
		nextNo++;
	}
	
	return true;
}

/// change the generator voltage
void gen_t::change_Vg( double delta ) {
	Vref += delta;
}

/// read xml data
bool gen_t::read( MyXmlNode & gen_node ) {
	MyXmlNode node;
	string tag, content;
	complex<double> value;
	double time;
	int num;
	
	// error checking
	err_if( gen_node.name()!="gen", "This is not a gen data node." );
	num = (int)gen_node.get_numeric_attribute("number");
	err_if( num != number, "Wrong gen number.");

	// process the time
	time = gen_node.get_child_numeric_content( "time" );
	if ( (int)time!=MYXML_EMPTY and time < T_measured ) return true;
	if ( time>=T_measured ) T_measured=time;
	
	// read the data
	for( node=gen_node.child(); not node.is_null(); node = node.next() ) {
		tag = node.name();
		content = node.content();
		// deal with each potential tag
		if      (tag=="matpower") read_matpower(content);
		else if (tag=="S") {
			value = read_complex(content);
			Pg = value.real();
			Qg = value.imag();
		}
		else if (tag=="Smin") {
			value = read_complex(content);
			Pmin = value.real();
			Qmin = value.imag();
		}
		else if (tag=="Smax") {
			value = read_complex(content);
			Pmax = value.real();
			Qmax = value.imag();
		}
		else if (tag=="busNo") busNo = atoi(content.c_str());
		else if (tag=="P")     Pg    = atof(content.c_str());
		else if (tag=="Q")     Qg    = atof(content.c_str());
		else if (tag=="Pmax")  Pmax  = atof(content.c_str());
		else if (tag=="Pmin")  Pmin  = atof(content.c_str());
		else if (tag=="Qmin")  Qmin  = atof(content.c_str());
		else if (tag=="Qmax")  Qmax  = atof(content.c_str());
		else if (tag=="Vref")  Vref  = atof(content.c_str());
		else if (tag=="status") status = (status_e) atoi( content.c_str() );
		else if (tag=="name") strcpy( name, content.c_str() );
		else if (tag=="cost") {
			// get the first token of the string
			char *tok = strtok ( const_cast<char*>(content.c_str()), " ," );
			// resize the cost polynomial
			cost.polynomial.resize(0);
			// get all of the subsequent string tokens
			while (tok != NULL) {
				cost.polynomial.push_back(atof(tok));
				tok = strtok(NULL," ,");
			}
			// if the polynomial is not at least 4 in size, pad it with zeros
			if (cost.polynomial.size() < 4) cost.polynomial.resize(4,0);
		}
	}
	return true;
}


/// read_xml: read some data formated as xml
bool gen_t::read_xml ( const string& data )
{
	match_t match;
	string tag, content;
	int data_index=0;
	complex<double> value;
	
	// first get the time for the measurements if it is included
	if ( regex ( data, "<time>(.*?)</time>", match ) ) T_measured = atof( match[1].c_str() );
	// potential gen tags:
	// matpower, S, Smin, Smax, P, Q, Pmax, Pmin, Qmin, Qmax, Vref, status, name
	while ( get_next_xml ( data, tag, content, data_index ) ) {
		// deal with each potential tag
		if      (tag=="matpower") read_matpower(content);
		else if (tag=="S")
		{
			value = read_complex(content);
			Pg = value.real();
			Qg = value.imag();
		}
		else if (tag=="Smin") {
			value = read_complex(content);
			Pmin = value.real();
			Qmin = value.imag();
		}
		else if (tag=="Smax") {
			value = read_complex(content);
			Pmax = value.real();
			Qmax = value.imag();
		}
		else if (tag=="busNo") busNo=atoi(content.c_str());
		else if (tag=="P")    Pg=atof(content.c_str());
		else if (tag=="Q")    Qg=atof(content.c_str());
		else if (tag=="Pmax") Pmax=atof(content.c_str());
		else if (tag=="Pmin") Pmin=atof(content.c_str());
		else if (tag=="Qmin") Qmin=atof(content.c_str());
		else if (tag=="Qmax") Qmax=atof(content.c_str());
		else if (tag=="Vref") Vref=atof(content.c_str());
		else if (tag=="status") status = (status_e) atoi( content.c_str() );
		else if (tag=="name") strcpy( name, content.c_str() );
		else if (tag=="cost") {
			// get the first token of the string
			char *tok = strtok ( const_cast<char*>(content.c_str()), " ," );
			// resize the cost polynomial
			cost.polynomial.resize(0);
			// get all of the subsequent string tokens
			while (tok != NULL) {
				cost.polynomial.push_back(atof(tok));
				tok = strtok(NULL," ,");
			}
			// if the polynomial is not at least 4 in size, pad it with zeros
			if (cost.polynomial.size() < 4) cost.polynomial.resize(4,0);
		}
	}
	return true;
}

/// write to a specified format
bool gen_t::write ( string &output, data_format_e format ) const
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
bool gen_t::write_xml ( string &output ) const
{
	char str[2000];
	
	output.clear();
	sprintf(str, "<gen number=\"%d\">",number);
	output.append(str);
	// busNp
	sprintf( str, "<busNo>%d</busNo>", busNo );
	output.append(str);
	// S, Smax, Smin
	sprintf(str, "<S>%g, %g</S> <Smin>%g, %g</Smin> <Smax>%g, %g</Smax>",Pg, Qg, Pmin, Qmin, Pmax, Qmax);
	output.append(str);
	// control reference points
	sprintf(str, "<Pref>%g</Pref> <Vref>%g</Vref>", Pref, Vref);
	output.append(str);
	// status, machine base
	sprintf(str, "<status>%d</status><mBase>%g</mBase>", (unsigned)status, mBase);
	output.append(str);
	output.append("</gen>\n");
	
	return true;
}

/// write_matpower
bool gen_t::write_matpower(string& output) const {
	char str[2000];
	//bn P  Q  Qx Qn V  vb st Px Pn mu mu mu mu
	sprintf(str, "%d %g %g %g %g %g %g %d %g %g %g %g %g %g;",
	        busNo, Pg, Qg, Qmax, Qmin, Vref, mBase, (int) status,
	        Pmax, Pmin, mu_Pmax, mu_Pmin, mu_Qmax, mu_Qmin);
	
	output.assign(str);
	return true;
}

// comparison functions
bool gen_t::operator==(const gen_t& other) const
{
	if (other.number == number)
		return true;
	else
		return false;
}
bool gen_t::operator< (const gen_t& other) const
{
	if ( number < other.number )
		return true;
	else
		return false;
}
bool gen_t::operator> (const gen_t& other) const
{
	if ( number > other.number )
		return true;
	else
		return false;
}

/// change the gen power by the specified amount
void gen_t::change(double delta) {
	double newPg = Pg+delta;
	// 
	if ( (Pmin <= newPg) and (newPg <= Pmax) ) {
		Pg   = newPg;
		Pref = newPg;
	} else if ( Pmin > newPg ) {
		Pg=0;
		Qg=0;
		Pref=0;
		status=OFF;
	} else if ( newPg > Pmax ) {
		Pg=Pmax;
	} else {
		printf("Strange command to gen_t::change.\n");
	}
}

