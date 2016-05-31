
////////////////////////////////////////////////////////////////////////////////
// Implementation of bus_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <sstream>
#include <iostream>
#include <complex>

#include "bus.hpp"
#include "PowerGlobals.hpp"
#include "../myxml/myxml.hpp"
#include "../regex/regex.hpp"

using namespace std;

// member functions
bus_t::bus_t() {
  Vmag   = 1;
  Vang   = POWER_EMPTY;
  Vmax   = 1.1;
  Vmin   = 0.9;
  baseKV = 500;
  area   = 0;
  zone   = 0;
  lamP   = 0;
  lamQ   = 0;
  type   = PQ;
  locX   = 0;
  locY   = 0;
  status = ON;
  strcpy(name," ");
  freq = 60;
  T_measured=0;
  Pd=0;
  Qd=0;
  Gs=0;
  Bs=0;
  converged=false;
}

void bus_t::set_voltage( std::complex<double> Vin ) {
	Vmag=std::abs(Vin);
	if( Vmag<1e-3 or Vmag>2 ) { Vmag=0; Vang=0; status=OFF; Vin=0.0; }
	else { Vang=std::arg(Vin)*180/PI; }
}

bool bus_t::operator==(const bus_t& other) const {
  return (number==other.number);
}

bool bus_t::operator< (const bus_t& other) const {
  return (number<other.number);
}

bool bus_t::operator> (const bus_t& other) const {
  return (number>other.number);
}

double bus_t::theta() {
	if ( is_unknown(Vang) ) {
		if (type==REF) { Vang=0.0; return 0.0; }
		else return POWER_UNKNOWN;
	}	
	return (Vang*PI/180);
}

/// read
bool bus_t::read( const string& data, data_format_e format) {
	switch (format) {
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
/// read_matpower: read a line of data in matpower sequence
bool bus_t::read_matpower(  const string& data )
{
	istringstream ins(data);
	double num;
	
	// if the size is improbably small, return false;
	if ( data.size() < 10 ) return false;
	// read the data in matpower order
	ins >> number;
	ins >> num; type = (bus_type_e) num;
	ins >> Pd;
	ins >> Qd;
	ins >> Gs;
	ins >> Bs;
	ins >> area;
	ins >> Vmag;
	ins >> Vang;
	ins >> baseKV;
	ins >> zone;
	ins >> Vmax;
	if ( !(ins >> Vmin) ) return false;
	ins >> lamP;
	ins >> lamQ;
	ins >> mu_Vmax;
	ins >> mu_Vmin;
	ins >> locX;
	ins >> locY;
	
	return true;
}

bool bus_t::read( MyXmlNode & bus_node ) {
	MyXmlNode node;
	string tag, content;
	int num;
	double time;
	
	// error checking
	err_if( bus_node.name()!="bus", "This is not a bus data node." );
	num = (int)bus_node.get_numeric_attribute("number");
	err_if( num != number, "Wrong bus number.");
	// get the time and check to see if we want to read this data
	time = bus_node.get_child_numeric_content( "time" );
	if ( (int)time!=MYXML_EMPTY and time < T_measured ) return true;
	if ( time>=T_measured ) T_measured=time;
	// read the data in the children nodes
	for( node=bus_node.child(); not node.is_null(); node = node.next() ) {
		tag = node.name();
		content = node.content();
		if( tag=="matpower") read_matpower( node.content() );
		else if( tag=="V" )  set_voltage( read_complex(content) );
		else if (tag=="Vmag") Vmag = atof( content.c_str() );
		else if (tag=="Vang") Vang = atof( content.c_str() );
		else if (tag=="Vmin") Vmin = atof( content.c_str() );
		else if (tag=="Vmax") Vmax = atof( content.c_str() );
		else if (tag=="type") {
			if      ( content=="PV"  ) type=PV;
			else if ( content=="PQ"  ) type=PQ;
			else if ( content=="REF" ) type=REF;
			else if ( content=="AGC" ) type=AGC;
			else type = (bus_type_e) atoi( content.c_str() );
		}
		else if (tag=="status") status = (status_e) atoi( content.c_str() );
		else if (tag=="freq") freq = atof( content.c_str() );
		else if (tag=="zone") zone = atoi( content.c_str() );
		else if (tag=="area") area = atoi( content.c_str() );
		else if (tag=="locX") locX = atof( content.c_str() );
		else if (tag=="locY") locY = atof( content.c_str() );
		else if (tag=="name") strcpy( name, content.c_str() );
		
	}
	return true;
}

/// read_xml: read some data formated as xml
bool bus_t::read_xml ( const string& data )
{
	match_t match;
	string tag, content;
	int data_index=0;
	
	// first get the time for the measurements if it is included
	if ( regex ( data, "<time>(.*?)</time>", match ) ) T_measured = atof( match[1].c_str() );
	// potential bus tags:
	// matpower, V, Vmag, Vang, Vmax, Vmin, type, baseKV, area, zone, locX, locY, status, name, freq
	while ( get_next_xml ( data, tag, content, data_index ) )
	{
		// deal with each potential tag
		if      (tag=="matpower") {
			read_matpower(content);
		}
		else if (tag=="V") {
			complex<double> Vcx = read_complex(content);
			Vmag = abs(Vcx);
			Vang = arg(Vcx)*180/PI;
		}
		else if (tag=="Vmag") Vmag = atof( content.c_str() );
		else if (tag=="Vang") Vang = atof( content.c_str() );
		else if (tag=="Vmin") Vmin = atof( content.c_str() );
		else if (tag=="Vmax") Vmax = atof( content.c_str() );
		else if (tag=="type") {
			if      ( regex(content,"PV")  ) type=PV;
			else if ( regex(content,"PQ")  ) type=PQ;
			else if ( regex(content,"REF") ) type=REF;
			else if ( regex(content,"AGC") ) type=AGC;
			else type = (bus_type_e) atoi( content.c_str() );
		}
		else if (tag=="status") status = (status_e) atoi( content.c_str() );
		else if (tag=="freq") freq = atof( content.c_str() );
		else if (tag=="zone") zone = atoi( content.c_str() );
		else if (tag=="area") area = atoi( content.c_str() );
		else if (tag=="locX") locX = atof( content.c_str() );
		else if (tag=="locY") locY = atof( content.c_str() );
		else if (tag=="name") strcpy( name, content.c_str() );
	}
	return true;
}

/// write
bool bus_t::write( string& output, data_format_e format ) const
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
bool bus_t::write_xml(string& output) const
{
	char str[200];
	
	output.clear();
	// write the opening tag/bus number
	sprintf(str, "<bus number=\"%d\">", number);
	output.append(str);
	// write the type
	if      (type==PQ)  output.append("<type>PQ</type>");
	else if (type==PV)  output.append("<type>PV</type>");
	else if (type==REF) output.append("<type>REF</type>");
	else if (type==AGC) output.append("<type>AGC</type>");
	// write the voltage
	sprintf( str,"<Vmag>%g</Vmag><Vang>%g</Vang>", Vmag, Vang );
	output.append(str);
	// write the V limts
	sprintf( str, "<Vmax>%g</Vmax><Vmin>%g</Vmin>", Vmax, Vmin );
	output.append(str);
	// write area and zone
	sprintf( str, "<area>%d</area><zone>%d</zone>", area, zone );
	output.append(str);
	// write the location
	sprintf( str, "<locX>%g</locX><locY>%g</locY>", locX, locY );
	output.append(str);
	// write the frequency
	sprintf( str, "<freq>%g</freq>", freq );
	output.append(str);
	output.append("</bus>\n");
	
	return true;
}

/// write_matpower
bool bus_t::write_matpower(string& output) const
{
	char str[2000];
	
	sprintf(str, "%d %d %g %g %g %g %d %g %g %g %d %g %g;",
	        number, type, Pd, Qd, Gs, Bs, area, Vmag, Vang, baseKV, zone, Vmax, Vmin);
	output.assign(str);
	return true;
}

/// write state
///   Writes the state of the bus (including branch, load, gen vars) to a string
bool bus_t::write_state( std::string& state, double t ) {
	char str[1000];
	unsigned i;
	bool newTime=false;
	
	// record the measurement time if given
	if (t>T_measured) { T_measured=t; newTime=true; }
	// write the bus data
	sprintf(str, "<bus number=\"%d\"><Vmag>%g</Vmag><Vang>%g</Vang><time>%g</time></bus>\n", 
	        number, Vmag, Vang, T_measured );
	state.append(str);
	// write the load data
	for ( i=0; i<nLoad(); i++ ) {
		if ( newTime ) load(i).T_measured=t;
		sprintf( str, "<load number=\"%d\"><S>%g %g</S><status>%d</status><time>%g</time></load>\n", 
		         load(i).number, load(i).Pd, load(i).Qd, load(i).status, load(i).T_measured );
		state.append(str);
	}
	// write the gen data
	for ( i=0; i<nGen(); i++ ) {
		if ( newTime ) gen(i).T_measured=t;
		sprintf( str, "<gen number=\"%d\"><S>%g %g</S><Vref>%g</Vref><status>%d</status><time>%g</time></gen>\n", 
		         gen(i).number, gen(i).Pg, gen(i).Qg, gen(i).Vref, gen(i).status, gen(i).T_measured );
		state.append(str);
	}
	// write the branch data
	for ( i=0; i<nBranch(); i++ ) {
		if ( number==branch(i).fromBusNo ) {
			if ( newTime ) branch(i).Tf_measured=t;
			sprintf( str, "<branch number=\"%d\"><Imag_f>%g</Imag_f><Vmag_f>%g</Vmag_f><theta_ft>%g</theta_ft><status_f>%d</status_f><time_f>%g</time_f></branch>\n", 
					 branch(i).number, branch(i).Imag_f, branch(i).Vmag_f(), branch(i).theta_ft(), branch(i).status_f, branch(i).Tf_measured );
		} else {
			if ( newTime ) branch(i).Tt_measured=t;
			sprintf( str, "<branch number=\"%d\"><Imag_t>%g</Imag_t><Vmag_t>%g</Vmag_t><theta_ft>%g</theta_ft><status_t>%d</status_t><time_t>%g</time_t></branch>\n", 
					 branch(i).number, branch(i).Imag_t, branch(i).Vmag_t(), branch(i).theta_ft(), branch(i).status_t, branch(i).Tt_measured );
		}
		state.append(str);
	}
	return true;
}

/// print
void bus_t::print( string &output ) const {
	char str[1000];
	// if this is the first one print the heading:
	if ( ix()==0 ) {
		sprintf(str," Number  Type   |V|         /_V      min|V|  max|V| \n");
		output.append(str);
	}
	// print the number and type
	if      (type==PV)  sprintf(str, " %6d PV  ", number );
	else if (type==PQ)  sprintf(str, " %6d PQ  ", number );
	else if (type==REF) sprintf(str, " %6d REF ", number );
	else error("Unknown bus type in bus_t::print.");
	output.append(str);
	// print the remaining data
	if ( is_unknown(Vang) ) {
		sprintf(str,"  %10.7f   ??????   %7.4f %7.4f \n", Vmag, Vmin, Vmax );
	} else {
		sprintf(str,"  %10.7f %10.5f %7.4f %7.4f \n", Vmag, Vang, Vmin, Vmax );
	}
	output.append(str);
}

/// eraseIx
void bus_t::eraseIx() {
  gen_set.clear();
  load_set.clear();
  branch_set.clear();
  neighbor_set.clear();
}

