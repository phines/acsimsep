////////////////////////////////////////////////////////////////////////////////
// Definition of bus_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#include "branch.hpp"
#include "bus.hpp"
#include "../regex/regex.hpp"
#include <cstdio>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <complex>

#define EPS 1E-12
#define BRANCH_LARGE 1000000

using namespace std;

typedef std::complex<double> cx;

// define the static member
double branch_t::baseMVA_ = 100;

// constructor
branch_t::branch_t() {
	number    = POWER_EMPTY;
	fromBusNo = POWER_EMPTY;
	toBusNo   = POWER_EMPTY;
	fbi       = POWER_EMPTY;
	tbi       = POWER_EMPTY;
	R         = 0.01;
	X         = 0.1;
	B         = 0;
	G         = 0;
	rateA     = BRANCH_LARGE;
	rateB     = BRANCH_LARGE;
	rateC     = BRANCH_LARGE;
	tap       = 1.0;
	shift     = 0.0;
	status_t  = ON;
	status_f  = ON;
	strcpy( name, " " );
	If = 0;
	It = 0;
	Imag_f = 0;
	Imag_t = 0;
	mu_f = 0;
	mu_t = 0;
	Tf_measured=0; ///< the time that the from end data was measured
	Tt_measured=0; ///< the time that the to end data was measured
	overload = 0;
	overload_limit = POWER_EMPTY;
	overload_time_constant = 5.0;
	Vmag_f_    = 1.0;
	Vmag_t_    = 1.0;
	theta_ft_  = 0.0;
	Imag_prev_ = 0.0;
}

double branch_t::baseMVA(100);

// destructor
branch_t::~branch_t() {
  // nothing here
}

/// add or subtract 2*pi to/from @param ang until you get ang in [-pi, pi]
static double unwrap( double ang ) {
	while (ang>PI) {
		ang -= 2*PI;
	}
	while (ang<-PI) {
		ang += 2*PI;
	}
	return ang;
}

/// take the absolute value of a complex number
static double cxabs(const cx& a) {
	return sqrt( pow(a.real(),2) + pow(a.imag(),2) );
}

std::complex<double> branch_t::Sf() const {
	return Vf() * conj(If) * baseMVA;
}

std::complex<double> branch_t::St() const {
	return Vt() * conj(It) * baseMVA;
}

void branch_t::set_voltages( double Vmag_f, double Vmag_t, double theta_ft, bool recalc_currents ) {
	Vmag_f_   = Vmag_f;
	Vmag_t_   = Vmag_t;
	theta_ft_ = (double)status() * theta_ft;
	if (recalc_currents) calc_currents();
}

void branch_t::calc_currents() {
	cx y_ft, y_tf, y_tt, y_ff;
	
	calc_admittances( y_ft, y_tf, y_ff, y_tt );
	
	If = (double)status_f * ( y_ff*Vf() + y_ft*Vt() );
	It = (double)status_t * ( y_tt*Vt() + y_tf*Vf() );
	
	Imag_f = cxabs( If );
	Imag_t = cxabs( It );
}

/// return max(args)
static double maximum(double a, double b) {
	return a>b ? a : b;
}

/*
/// return min(args)
static double minimum(double a, double b) {
	return a<b ? a : b;
}
*/

/// return cos(C) given side lengths a, b, and c
///  cos(C) = ( a^2 + b^2 - c^2 ) / (2*a*b)
static double law_of_cosines( double a, double b, double c ) {
	double cosC = (pow(a,2) + pow(b,2) - pow(c,2)) / (2*a*b) ;
	return cosC;
}

/// return the min/max delta, given the specified limit
///  this isn't used currently
void branch_t::delta_min_max(double &delta_min, double &delta_max, BranchRateNo_e rate)
{
	// vars
	double cosC;
	double Imax = rateB/baseMVA_;
	std::vector<double>  d(4,0);
	cx y_ft, y_tf, y_tt, y_ff;
	// use the law of Cosines to get the min/max angle
	switch (rate) {
		case RATE_A:
			Imax = rateA/baseMVA_;
			break;
		case RATE_B:
			Imax = rateB/baseMVA_;
			break;
		case RATE_C:
			Imax = rateC/baseMVA_;
			break;
		default:
			error("Unknown rate type.");
	}
	// calculate the admittances
	calc_admittances( y_ft, y_tf, y_ff, y_tt );
	// apply law of Cosines for the from-to direction
	cosC = law_of_cosines ( cxabs(y_ft)*cxabs(Vt()), cxabs(y_ff)*cxabs(Vf()), Imax );
	if ( abs(cosC)<1 ) {
		d[0] = unwrap(  PI - acos(cosC) - arg(y_ff) + arg(y_ft) );
		d[1] = unwrap( -PI + acos(cosC) - arg(y_ff) + arg(y_ft) );
	} else {
		d[0] = -delta(); // NOTE: not sure what should go here!
		d[1] = +delta(); // NOTE: not sure what should go here!
	}
	// apply law of Cosines for the to-from direction
	cosC = law_of_cosines ( cxabs(y_tf)*cxabs(Vf()), cxabs(y_tt)*cxabs(Vt()), Imax );
	if ( abs(cosC)<1 ) {
		d[2] = unwrap(  PI - acos(cosC) - arg(y_tt) + arg(y_tf) );
		d[3] = unwrap( -PI + acos(cosC) - arg(y_tt) + arg(y_tf) );
	} else {
		d[2] = -delta(); // NOTE: not sure what should go here!
		d[3] = +delta(); // NOTE: not sure what should go here!
	}
	
	sort( d.begin(), d.end() );
	
	delta_min = d[1];
	delta_max = d[2];
}

// derivatives
void branch_t::dImag_dx( double & dImag_dVf, double & dImag_dVt, double & dImag_dDelta ) {
	double a, b, c, C, Imag, dImag_da, dImag_db, dImag_dC;
	cx y_ft, y_tf, y_tt, y_ff;
	double Vmag_f = Vmag_f_;
	double Vmag_t = Vmag_t_;
	
	calc_admittances( y_ft, y_tf, y_ff, y_tt );
	
	if ( Imag_f > Imag_t ) {
		Imag = Imag_f;
		a = cxabs(y_ft)*Vmag_t;
		b = cxabs(y_ff)*Vmag_f;
		c = Imag;
		C = PI - ( arg(y_ff) - arg(y_ft) + delta() );
		dImag_da  = ( a - b*cos(C) ) / c;
		dImag_db  = ( b - a*cos(C) ) / c;
		dImag_dC  = a*b*sin(C) / c;
		dImag_dVt = dImag_da * cxabs(y_ft);
		dImag_dVf = dImag_db * cxabs(y_ff);
		dImag_dDelta = -dImag_dC;
	} else {
		Imag = Imag_t;
		a = cxabs(y_tf)*Vmag_f;
		b = cxabs(y_tt)*Vmag_t;
		c = Imag;
		C = PI - ( arg(y_tt) - arg(y_tf) - delta() );
		dImag_da  = ( a - b*cos(C) ) / c;
		dImag_db  = ( b - a*cos(C) ) / c;
		dImag_dC  = a*b*sin(C) / c;
		dImag_dVf = dImag_da * cxabs(y_tf);
		dImag_dVt = dImag_db * cxabs(y_tt);
		dImag_dDelta = dImag_dC;
	}
}

// operator ==
bool branch_t::operator==(const branch_t& other) const
{
  return (fromBusNo == other.fromBusNo) and 
         (toBusNo == other.toBusNo)     and
         (X == other.X)                 and
         (R == other.R)                 and
         (B == other.B)                 and
         (G == other.G);
}

// operator <
bool branch_t::operator<(const branch_t& other) const
{
  return (fromBusNo < other.fromBusNo) || 
         (fromBusNo==other.fromBusNo and toBusNo < other.toBusNo);
}
// operator >
bool branch_t::operator>(const branch_t& other) const
{
  return (fromBusNo > other.fromBusNo) || 
         (fromBusNo==other.fromBusNo and toBusNo > other.toBusNo);
}
// print
void branch_t::print( string &output ) const {
	char str[1000];
	// if this is the first branch in the set print the header info
	if( ix()==0 ) {
		sprintf(str," Num   From     To     R         X         B         P (from->to) Q        P (to->from) Q      Imag   |I|/max    delta  status\n");
		output.append(str);
	}	
	// print the data for this branch
	sprintf(str,"%4d %6d %6d %9.4f %9.4f %9.4f %10.4f %10.4f %10.4f %10.4f %8.4f %8.6f %8.5f  %d\n",
    number, fromBusNo, toBusNo, R, X, B, Sf().real(), Sf().imag(), St().real(), St().imag(), Imag()*baseMVA, Inorm(), delta(), status() );
	output.append(str);
}

/// calc admittances:
bool branch_t::calc_admittances( cx &y_ft, cx &y_tf, cx &y_ff, cx &y_tt, 
                                 cx &y_s, cx &tap_shift, bool ignore_status )
{
	tap_shift = cx(1,0);
	y_ft = 0;
	y_tf = 0;
	y_ff = 0;
	y_tt = 0;
	y_s = 0;
	
	if ( status()==ON || ignore_status ) {
		y_s = cx(1,0)/cx(R,X);
		
		// prevent data errors:
		if (abs(tap) < EPS) tap=1.0;
		if (abs(tap) < 0.1) printf("Warning: small tap value in data.\n");
		// calculate the tap-shift term:
		tap_shift = cx(tap,0) * polar( 1.0, shift*PI/180 );
		
		y_ft = -y_s / conj(tap_shift);
		y_tf = -y_s / tap_shift;
		y_tt =  y_s + cx(G/2, B/2);
		y_ff =  y_tt / ( tap_shift * conj(tap_shift) );
	}
	return true;
}

bool branch_t::calc_admittances( cx &y_ft, cx &y_tf, cx &y_ff, cx &y_tt, bool ignore_status )
{
	cx tap_shift(0), y_s(0);
	
	return calc_admittances ( y_ft, y_tf, y_ff, y_tt, y_s, tap_shift, ignore_status );
}

inline bool branch_t::is_shift()
{
	if ( abs(shift)>EPS )
		return true;
	else	
		return false;
}
inline bool branch_t::is_tap()
{
	if ( abs(tap)<EPS ) { // if nearly zero
		return false;
	} else if ( abs(tap-1)>EPS ) { // if not nearly zero and nearly 1
		return false;
	}
	// otherwise probably an off nominal tap
	return true;
}

//// read/write routines
///  write formated output
bool branch_t::write( string &output, data_format_e format ) const
{
	switch (format)
	{
		case POWER_XML:
			return write_xml(output);
			break;
		case REDUCED:
			return write_xml(output,true);
			break;
		case MATPOWER:
			return write_matpower(output);
			break;
		default:
			return write_xml(output);
	}
	return false;
}
/// read
bool branch_t::read( const string &data, data_format_e format )
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

/// read xml data
bool branch_t::read( MyXmlNode & branch_node ) {
	MyXmlNode node;
	string tag, content;
	double t, time_fr=BRANCH_LARGE, time_to=BRANCH_LARGE;
	double max_time = (Tf_measured>Tt_measured) ? Tf_measured : Tt_measured;
	status_e s;
	int num;
	cx value;
	
	// error checking
	err_if( branch_node.name()!="branch", "Not a branch node.");
	num = (int) branch_node.get_numeric_attribute( "number" );
	err_if( number != num, "Wrong branch number." );
	
	// process the time vars
	t = branch_node.get_child_numeric_content( "time" );
	if ( (int)t!=MYXML_EMPTY ) {
		time_fr = time_to = t;
	}
	t = branch_node.get_child_numeric_content( "time_t" );
	if ( (int)t!=MYXML_EMPTY ) {
		time_to = t;
	}
	t = branch_node.get_child_numeric_content( "time_f" );
	if ( (int)t!=MYXML_EMPTY ) {
		time_fr = t;
	}
	
	for( node=branch_node.child(); not node.is_null(); node = node.next() ) {
		tag = node.name();
		content = node.content();
		
		if      (tag=="matpower") read_matpower(content);
			// measurement variables must deal with the time
		else if (tag=="status"   and time_fr>=max_time)    { s=(status_e)atoi(content.c_str()); status_f=s; status_t=s; }
		else if (tag=="status_f" and time_fr>=Tf_measured) { status_f=(status_e)atoi(content.c_str()); }
		else if (tag=="status_t" and time_to>=Tt_measured) { status_t=(status_e)atoi(content.c_str()); }
		else if (tag=="Imag_f"   and time_fr>=Tf_measured) { Imag_f  = atof(content.c_str()); if(time_fr<BRANCH_LARGE) Tf_measured=time_fr; }
		else if (tag=="Imag_t"   and time_to>=Tt_measured) { Imag_t  = atof(content.c_str()); if(time_to<BRANCH_LARGE) Tt_measured=time_to; }
		else if (tag=="Vmag_f"   and time_fr>=Tf_measured) { Vmag_f_ = atof(content.c_str()); }
		else if (tag=="Vmag_t"   and time_to>=Tt_measured) { Vmag_t_ = atof(content.c_str()); }
		else if (tag=="theta_ft" and (time_to>=Tt_measured or time_fr>=Tf_measured)) { theta_ft_ = atof(content.c_str()); }
		else if (tag=="If"       and time_fr>=Tf_measured) { If=read_complex(content); if(time_fr<BRANCH_LARGE) Tf_measured=time_fr; }
		else if (tag=="It"       and time_to>=Tt_measured) { It=read_complex(content); if(time_to<BRANCH_LARGE) Tt_measured=time_to; }
		else if (tag=="Sf"       and time_fr>=Tf_measured) { /* do nothing */ }
		else if (tag=="St"       and time_to>=Tt_measured) { /* do nothing */ }
		else if (tag=="If_re"    and time_fr>=Tf_measured) {
		  If = cx( atof(content.c_str()), If.imag() );
		  if(time_fr<BRANCH_LARGE) Tf_measured=time_fr;
		}
		else if (tag=="It_re"    and time_to>=Tt_measured) {
		  It = cx( atof(content.c_str()), It.imag() );
		  if(time_to<BRANCH_LARGE) Tt_measured=time_to;
		}
		else if (tag=="If_im"    and time_fr>=Tf_measured) {
		  If = cx( If.real(), atof(content.c_str()) );
		  if(time_fr<BRANCH_LARGE) Tf_measured=time_fr;
		}
		else if (tag=="It_im"    and time_to>=Tt_measured){
		  It = cx( It.real(), atof(content.c_str()) );
		  if(time_to<BRANCH_LARGE) Tt_measured=time_to;
		}
		else if (tag=="Pf"       and time_fr>=Tf_measured) { /* do nothing */ }
		else if (tag=="Pt"       and time_to>=Tt_measured) { /* do nothing */ }
		else if (tag=="Qf"       and time_fr>=Tf_measured) { /* do nothing */ }
		else if (tag=="Qt"       and time_to>=Tt_measured) { /* do nothing */ }
			// the remaining variables are time invariant for the most part
		else if (tag=="from")  fromBusNo=atoi(content.c_str());
		else if (tag=="to")    toBusNo=atoi(content.c_str());
		else if (tag=="R")     R=atof(content.c_str());
		else if (tag=="X")     X=atof(content.c_str());
		else if (tag=="B")     B=atof(content.c_str());
		else if (tag=="rateA") rateA=atof(content.c_str());
		else if (tag=="rateB") rateB=atof(content.c_str());
		else if (tag=="rateC") rateC=atof(content.c_str());
		else if (tag=="tap")   tap=atof(content.c_str());
		else if (tag=="shift") shift=atof(content.c_str());
		else if (tag=="name")  strcpy( name, content.c_str() );
		else if (tag=="Z") {
			value = read_complex(content);
			R = value.real();
			X = value.imag();
		}
	}
	return true;
}


/// read_xml
///  Valid XML tags for branch_t class include:
///   If, It, Sf, St, from, to, rateA, rateB, tap, shift, status, R, X, B, Z, name
///   If_re, If_im, It_re, It_im, Pf, Pt, Qf, Qt.
bool branch_t::read_xml ( const string &data )
{
	match_t match;
	string tag, content;
	int data_index=0;
	complex<double> value;
	double time_fr=BRANCH_LARGE, time_to=BRANCH_LARGE;
	double max_time = (Tf_measured>Tt_measured) ? Tf_measured : Tt_measured;
	status_e s;
	
	// first get the time or times for the measurements if it is included
	if ( regex ( data, "<time>(.*?)</time>", match ) ) {
		time_fr = time_to = atof( match[1].c_str() );
	}
	if ( regex ( data, "<time_t>(.*?)</time_t>", match ) ) {
		time_to = atof( match[1].c_str() );
	}
	if ( regex ( data, "<time_f>(.*?)</time_f>", match ) ) {
		time_fr = atof( match[1].c_str() );
	}	
	
	// valid branch tags are shown above.
	while ( get_next_xml ( data, tag, content, data_index ) )
	{
		if      (tag=="matpower") read_matpower(content);
		// measurement variables must deal with the time
		else if (tag=="status"   and time_fr>=max_time)    { s=(status_e)atoi(content.c_str()); status_f=s; status_t=s; }
		else if (tag=="status_f" and time_fr>=Tf_measured) { status_f=(status_e)atoi(content.c_str()); }
		else if (tag=="status_t" and time_to>=Tt_measured) { status_t=(status_e)atoi(content.c_str()); }
		else if (tag=="Imag_f"   and time_fr>=Tf_measured) { Imag_f = atof(content.c_str()); if(time_fr<BRANCH_LARGE) Tf_measured=time_fr; }
		else if (tag=="Imag_t"   and time_to>=Tt_measured) { Imag_t = atof(content.c_str()); if(time_to<BRANCH_LARGE) Tt_measured=time_to; }
		else if (tag=="Vmag_f"   and time_fr>=Tf_measured) { Vmag_f_ = atof(content.c_str()); if(time_fr<BRANCH_LARGE) Tf_measured=time_fr; }
		else if (tag=="Vmag_t"   and time_to>=Tt_measured) { Vmag_t_ = atof(content.c_str()); if(time_to<BRANCH_LARGE) Tt_measured=time_to; }
		else if (tag=="theta_ft" and (time_to>=Tt_measured or time_fr>=Tf_measured)) { theta_ft_ = atof(content.c_str()); }
		else if (tag=="If"       and time_fr>=Tf_measured) { If=read_complex(content); if(time_fr<BRANCH_LARGE) Tf_measured=time_fr; }
		else if (tag=="It"       and time_to>=Tt_measured) { It=read_complex(content); if(time_to<BRANCH_LARGE) Tt_measured=time_to; }
		else if (tag=="Sf"       and time_fr>=Tf_measured) { /* do nothing */ }
		else if (tag=="St"       and time_to>=Tt_measured) { /* do nothing */ }
		else if (tag=="If_re"    and time_fr>=Tf_measured) {
		  If = cx( atof(content.c_str()), If.imag() );
		  if(time_fr<BRANCH_LARGE) Tf_measured=time_fr;
		}
		else if (tag=="It_re"    and time_to>=Tt_measured) {
		  It = cx( atof(content.c_str()), It.imag() );
		  if(time_to<BRANCH_LARGE) Tt_measured=time_to;
		}
		else if (tag=="If_im"    and time_fr>=Tf_measured) {
		  If = cx( If.real(), atof(content.c_str()) );
		  if(time_fr<BRANCH_LARGE) Tf_measured=time_fr;
		}
		else if (tag=="It_im"    and time_to>=Tt_measured){
		  It = cx( It.real(), atof(content.c_str()) );
		  if(time_to<BRANCH_LARGE) Tt_measured=time_to;
		} 
		else if (tag=="Pf"       and time_fr>=Tf_measured) { /* do nothing */ }
		else if (tag=="Pt"       and time_to>=Tt_measured) { /* do nothing */ }
		else if (tag=="Qf"       and time_fr>=Tf_measured) { /* do nothing */ }
		else if (tag=="Qt"       and time_to>=Tt_measured) { /* do nothing */ }
		// the remaining variables are time invariant for the most part
		else if (tag=="from")  fromBusNo=atoi(content.c_str());
		else if (tag=="to")    toBusNo=atoi(content.c_str());
		else if (tag=="R")     R=atof(content.c_str());
		else if (tag=="X")     X=atof(content.c_str());
		else if (tag=="B")     B=atof(content.c_str());
		else if (tag=="rateA") rateA=atof(content.c_str());
		else if (tag=="rateB") rateB=atof(content.c_str());
		else if (tag=="rateC") rateC=atof(content.c_str());
		else if (tag=="tap")   tap=atof(content.c_str());
		else if (tag=="shift") shift=atof(content.c_str());
		else if (tag=="name")  strcpy( name, content.c_str() );
		else if (tag=="Z") {
			value = read_complex(content);
			R = value.real();
			X = value.imag();
		}
	}
	// change a zero tap value to a 1 tap value
	if (abs(tap)<EPS)   tap=1;
	if (abs(shift)<EPS) shift=0;
	
	// calc overload limit for relays (arbitrary)
	return true;
}

/// write_xml
bool branch_t::write_xml ( string &output, bool reduced ) const
{
	char str[200];
	
	output.clear();
	
	sprintf(str, "<branch number=\"%d\">", number );
	output.append(str);
	
	// from/to
	sprintf(str, "<from>%d</from><to>%d</to>", fromBusNo, toBusNo);
	output.append(str);
	// R, X, B, 
	sprintf( str, "<R>%g</R><X>%g</X><B>%g</B>", R, X, B );
	output.append(str);
	// rates
	sprintf( str, "<rateA>%g</rateA><rateB>%g</rateB><rateC>%g</rateC>", rateA, rateB, rateC );
	output.append(str);
	// tap, shift, status
	sprintf( str, "<tap>%g</tap><shift>%g</shift>", tap, shift );
	output.append(str);
	if (!reduced) {
		// name
		sprintf( str, "<name>%s</name>", name );
		output.append(str);
		// branch flow vars and time
		sprintf( str, "<Imag_f>%g</Imag_f><Imag_t>%g</Imag_t><Sf>%g,%g</Sf><St>%g,%g</St><time_t>%g</time_t><time_f>%g</time_f>", 
			Imag_f, Imag_t, Sf().real(), Sf().imag(), St().real(), St().imag(), Tt_measured, Tf_measured );
		output.append(str);
		// add the time that the measurements were taken
	}
	
	output.append("</branch>\n");
	return true;
}

/// read_matpower
bool branch_t::read_matpower ( const string &data )
{
	istringstream ins(data);
	int num;
	double re, im;
	
	// extract
	ins >> fromBusNo;
	ins >> toBusNo;
	ins >> R;
	ins >> X;
	ins >> B;
	ins >> rateA;
	ins >> rateB;
	ins >> rateC;
	ins >> tap;
	if ( !(ins >> shift) ) return false;
	if ( ins >> num ) { status_f=(status_e)num; status_t=(status_e)num; };
	ins >> re;
	ins >> im;
	//Sf = cx(re, im); discarding Sf -- prefer to use If--more reliable
	ins >> re;
	ins >> im;
	//St = cx(re, im); discarding St -- prefer to use If--more reliable
	ins >> mu_f;
	ins >> mu_t;
	// the following extend from matpower
	ins >> re;
	ins >> im;
	If = cx(re, im);
	ins >> re;
	ins >> im;
	It = cx(re, im);

	// assign the number
	if (number==POWER_EMPTY) {
		number = nextNo;
		nextNo++;
	}
	return true;
}
/// write_matpower
bool branch_t::write_matpower ( string &data ) const
{
	char str[2000];
	
	sprintf(str, "%d %d %g %g %g %g %g %g %g %g %d %g %g %g %g %g %g %g %g %g %g;",
		fromBusNo, toBusNo, R, X, B, rateA, rateB, rateC, tap, shift, status(), 
		Sf().real(), Sf().imag(), St().real(), St().imag(), mu_f, mu_t, If.real(), If.imag(), It.real(), It.imag());
	
	data.assign(str);
	return true;
}

/// update the branch relays
bool branch_t::update_relays( double dt ) {
	double overload_limit = (rateC - rateB) * overload_time_constant;
	double excess = (Imag_prev_ + Imag())*baseMVA/2 - rateB;
	Imag_prev_ = Imag();
	// update the overload var
	if (excess<0) overload=0;
	else overload = maximum( 0 , overload + excess*dt );
	
	if (overload > overload_limit) {
		printf("Branch %d switched off due to overload.\n", number);
		status_t=OFF;
		status_f=OFF;
		overload=0;
		If=0;
		It=0;
		Imag_f=0;
		Imag_t=0;
		return true;
	}
	return false;
}


