/// PowerSystem definition file
#include <string>
#include <iostream>
#include <fstream>
#include <complex>
#include <cmath>
//#include "IpIpoptApplication.hpp"
#include "../regex/regex.hpp"
#include "../utilities/graph.hpp"
#include "PowerGlobals.hpp"
#include "PowerFlow.hpp"
//#include "PowerSolver.hpp"
//#include "PowerMPC.hpp"
#include "PowerSystem.hpp"

#define EPS           1e-9
#define MAX_PF_ITERS  20

bool debug = false;

using namespace std;

// define the static members
DenseVector PowerSystem::EmptyDenseVector_=DenseVector();
std::vector<unsigned> PowerSystem::EmptyIntVector_=std::vector<unsigned>();

/// constructor
PowerSystem::PowerSystem() {
	baseMVA = 100;
	refNo_ = POWER_EMPTY;
	baseFrequency = options.baseFrequency;
	Ybus_.set_strategy( AUTO );
	ufls_factor_ = 1.0;
	dataFileName = "none";
	is_blackout_ = false;
	nIslands_ = 0;
}

/// destructor
PowerSystem::~PowerSystem() {
}

/// print
void PowerSystem::print() const {	
	string output;
	print(output);
	printf( "%s\n", output.c_str() );
}

static double maximum( double a, double b ) {
	return a>b ? a : b;
}

/// print
void PowerSystem::print( string &output ) const {	
	unsigned i;
	char str[1000];
	
	output.clear();
	// bus
	output.append("Bus data:\n");
	for (i=0; i<nBus(); i++)
		bus[i].print(output);
	// branch
	output.append("Branch data:\n");
	for (i=0; i<nBranch(); i++)
		branch[i].print(output);
	// gen
	output.append("Generator data:\n");
	for (i=0; i<nGen(); i++)
		gen[i].print(output);
	// load
	output.append("Load data:\n");
	for (i=0; i<nLoad(); i++)
		load[i].print(output);
	sprintf(str,"Base MVA = %f\n", baseMVA);
	output.append(str);
}


int PowerSystem::print_violations() {
	int nViols;
	string str;
	
	nViols = print_violations( str );
	printf("%s\n", str.c_str() );
	return nViols;
}

int PowerSystem::print_violations( std::string & output, BranchRateNo_e rate ) {
	int nViols = 0;
	unsigned i;
	char str[1000];
	double Imag, Imax, Vmag, Vmin, Vmax;
	
	// branches
	for (i=0;i<nBranch();i++) {
		Imag = branch[i].Imag()*(double)baseMVA;
		Imax = branch[i].rateB;
		if (Imag>Imax) {
			sprintf(str, "Branch %d: |I|=%g > %g. ", branch[i].number, Imag, Imax );
			output.append(str);
			nViols++;
		}
	}
	// buses
	for (i=0; i<nBus(); i++) {
		Vmag = bus[i].Vmag;
		Vmin = bus[i].Vmin;
		Vmax = bus[i].Vmax;
		// undervoltage
		if ( Vmag < Vmin and Vmag > 0.001 ) {
			sprintf( str, "Bus %d: |V|=%g < %g. ", bus[i].number, Vmag, Vmin );
			output.append(str);
			nViols++;
		}
		/*
		// overvoltage
		if ( Vmag > Vmax ) {
			sprintf( str, "Bus %d: |V|=%g > %g. ", bus[i].number, Vmag, Vmax );
			output.append(str);
			nViols++;
		}
		*/
	}
	if( nViols==0 ) output.append("No violations. ");
	return nViols;
}

#include <fstream>

bool fileExists(const std::string& fileName) {
	std::fstream fin;
	bool exists;
	fin.open( fileName.c_str() );
	exists = fin.is_open();
	fin.close();
	return exists;
}

/// read data from a text file
bool PowerSystem::read_file( const std::string & filename0, data_format_e format ) {
	string fileText;
	char filename1[1000];
	char filename2[1000];
	const char *filename[3]; //3 character string pointers
	filename[0] = filename0.c_str();
	filename[1] = filename1;
	filename[2] = filename2;
	match_t match;
	
	// record the file name
	dataFileName = filename0;
	// include a few filename variants
	sprintf(filename1, "../data/%s", filename[0]);
	sprintf(filename2, "./data/%s",  filename[0]);
	
	// read the contents
	for (int f=0; f<3; f++) {
		if ( fileExists(filename[f]) ) {
			// clear out the existing power network data
			clear();
			// read the data
			switch (format) {
				case POWER_XML: return read_xml_file( filename[f] );
				case MATPOWER:  return read_matpower_file( filename[f] );
				default: error("Other formats not supported yet.");
			}
		}
	}
	printf( "Could not open data file: %s. \n", filename[0] );
	return false;
}

/// write data to a text file
bool PowerSystem::write_file( const char* FileName, data_format_e format ) {
	string output;
	FILE* fp = fopen(FileName,"w");
	if (fp==NULL) {
		printf("Could not open file: %s\n", FileName);
		return false;
	}
	// write the data to a string
	if (!write(output, format) ) return false;
	// print the data to a file
	fprintf( fp, "%s", output.c_str() );
	fclose(fp);
	return true;
}
/// read data from a string
bool PowerSystem::read( const std::string& data, data_format_e format ) {
	switch (format) {
		case MATPOWER:
			return read_matpower(data);
			break;
		case POWER_XML:
			return read_xml(data);
			break;
		default:
			return read_xml(data);
	}
	return false;
}

/// write formated data to a string
bool PowerSystem::write( std::string& data, data_format_e format )
{
	switch (format)
	{
		case MATPOWER:
			return write_matpower(data);
			break;
		case POWER_XML:
			return write_xml(data);
			break;
		default:
			return write_xml(data);
	}
	return false;
}

/// write_matpower
bool PowerSystem::write_matpower ( std::string &output ) {
	output.clear();
	// FIXME:  neet to write a routine for MATPOWER output.
	return true;
}

bool PowerSystem::read_matpower_file( const string & filename ) {
	// FIXME this isn't working yet
	return false;
}

///  write_xml
bool PowerSystem::write_xml ( std::string &output )
{
	unsigned i;
	char cstr[1000];
	string str;
	output.clear();
	output.append("<?xml version=\"1.0\"?>\n");
	output.append("<!DOCTYPE PowerSystem SYSTEM \"PowerSystem.dtd\">\n");
	output.append("<?xml-stylesheet type=\"text/xsl\" href=\"PowerSystem.xsl\"?>\n");
	output.append("\n\n<PowerSystem>\n");
	
	sprintf(cstr,"<description>%s</description>\n",description.c_str());
	output.append(cstr);
	
	sprintf(cstr,"<baseMVA>%g</baseMVA>\n",baseMVA);
	output.append(cstr);

	sprintf(cstr,"<frequency>%g</frequency>\n",baseFrequency);
	output.append(cstr);
	// write the bus data
	for (i=0; i<nBus(); i++)
	{
		bus[i].write(str,POWER_XML);
		output.append("   ");
		output.append(str);
	}
	// write the gen data
	for (i=0; i<nGen(); i++)
	{
		gen[i].write(str,POWER_XML);
		output.append("   ");
		output.append(str);
	}
	// write the load data
	for (i=0; i<nLoad(); i++)
	{
		load[i].write(str,POWER_XML);
		output.append("   ");
		output.append(str);
	}
	// write the branch data
	for (i=0; i<nBranch(); i++)
	{
		branch[i].write(str,POWER_XML);
		output.append("   ");
		output.append(str);
	}
	output.append("\n</PowerSystem>\n");
	return true;
}

/// read data in xml format
bool PowerSystem::read_xml( MyXmlDocument & doc ) {
	MyXmlNode node;
	int index, number;
	
	for( node=doc.root_node().child(); not node.is_null(); node = node.next() ) {
		// process bus data
		if (node.name()=="bus") {
			number = (int) node.get_numeric_attribute("number");
			err_if( number<0, "Invalid bus number" );
			index = bus.index( number );
			if (index<0) {
				bus_t a_bus;
				a_bus.number=number;
				a_bus.read(node);
				insert(a_bus);
			}
			else bus[index].read( node );
		}
		// process branch data
		if (node.name()=="branch") {
			number = (int) node.get_numeric_attribute("number");
			err_if( number<0, "Invalid branch number" );
			index = branch.index( number );
			if (index<0) {
				branch_t a_branch;
				a_branch.number=number;
				a_branch.read(node);
				insert(a_branch);
			}
			else branch[index].read( node );
		}
		// process load data
		if (node.name()=="load") {
			number = (int) node.get_numeric_attribute("number");
			err_if( number<0, "Invalid load number" );
			index = load.index( number );
			if (index<0) {
				load_t a_load;
				a_load.number=number;
				a_load.read(node);
				insert(a_load);
			}
			else load[index].read( node );
		}
		// process gen data
		if (node.name()=="gen") {
			number = (int) node.get_numeric_attribute("number");
			err_if( number<0, "Invalid gen number" );
			index = gen.index( number );
			if (index<0) {
				gen_t a_gen;
				a_gen.number=number;
				a_gen.read(node);
				insert(a_gen);
			}
			else gen[index].read( node );
		}
		// event
		if (node.name()=="event") {
			number = (int) node.get_numeric_attribute("number");
			if (number==MYXML_EMPTY) {
				number = event_t::nextNo;
				event_t::nextNo++;
			}
			index = event.index( number );
			if (index<0) {
				event_t a_event;
				a_event.number=number;
				a_event.read(node);
				insert(a_event);
			}
			else event[index].read( node );
		}
		// description
		if (node.name()=="description") {
			description = node.content();
		}
		// baseMVA
		if (node.name()=="baseMVA") {
			baseMVA = node.numeric_content();
		}
		// frequency
		if (node.name()=="frequency") {
			baseFrequency = node.numeric_content();
		}
	}
	return true;
}


bool PowerSystem::read_xml_file( const string & filename ) {
	MyXmlDocument doc;
	
	if ( not doc.read_file( filename ) ) {
		printf( "Could not read xml file.\n" );
		return false;
	}
	
	if ( doc.root_node().name() != "PowerSystem" ) {
		printf( "The file does not appear to be a valid PowerSystem file.\n" );
		return false;
	}
	
	return read_xml( doc );
}

/// read data in xml format
bool PowerSystem::read_xml( const string& data )
{
	MyXmlDocument doc;
	
	doc.read_string( data );
	
	return read_xml( doc );
	/*
	string tag;
	string content;
	string attribute;
	string local_data;
	int start_point=0;
	int index, num;
	
	// the following section subsets the data, extracting only the portion with <PowerSystem> </PowerSystem>,
	//  though if this tag is missing, the routine continues anyway
	if ( get_next_xml( data, tag, content ) && tag=="PowerSystem" ) local_data = content;
	else local_data = data;
	
	while ( get_next_xml ( local_data, tag, content, attribute, start_point ) )
	{
		if (tag=="bus")
		{
			num = get_number(attribute);
			err_if( num==POWER_EMPTY, "Could not extract the bus number" );
			index = bus.index( num );
			if (index<0)
			{
				bus_t a_bus;
				a_bus.number=num;
				a_bus.read(content,POWER_XML);
				insert(a_bus);
			}
			else bus[index].read( content, POWER_XML );
		}
		else if (tag=="gen")
		{
			num = get_number(attribute);
			err_if( num==POWER_EMPTY, "Could not extract the gen number" );
			index = gen.index( num );
			if (index<0)
			{
				gen_t a_gen;
				a_gen.number=num;
				a_gen.read(content,POWER_XML);
				insert(a_gen);
			}
			else gen[index].read( content, POWER_XML );
		}
		else if (tag=="load")
		{
			num = get_number(attribute);
			err_if( num==POWER_EMPTY, "Could not extract the load number" );
			index = load.index( num );
			if (index<0)
			{
				load_t a_load;
				a_load.number=num;
				a_load.read(content,POWER_XML);
				insert(a_load);
			}
			else load[index].read( content, POWER_XML );
		}
		else if (tag=="branch")
		{
			num = get_number(attribute);
			err_if( num==POWER_EMPTY, "Could not extract the branch number" );
			index = branch.index( num );
			if (index<0)
			{
				branch_t a_branch;
				a_branch.number=num;
				a_branch.read(content,POWER_XML);
				insert(a_branch);
			}
			else branch[index].read( content, POWER_XML );
			// invalidate the system matrices
			Ybus_valid_=false;
			PTDF_valid_=false;
		}
		else if (tag=="event")
		{
			num = get_number(attribute);
			if (num>=POWER_EMPTY)
			{
				num=event_t::nextNo;
				event_t::nextNo++;
			}
			//err_if( num==POWER_EMPTY, "Could not extract the event number" );
			error("broken");
		}
		else if (tag=="baseMVA")     baseMVA = atof(content.c_str());
		else if (tag=="frequency")   baseFrequency = atof(content.c_str());
		else if (tag=="description") description = content;
		
	}
	// set the static baseMVA var in the branch data
	branch[0].baseMVA = baseMVA;
	return true;
	*/
}

/// read_matpower - Reads a string of data in MATPOWER format.
///  Warning: this function will only read the data that is in
///   matrix blocks.  It won't acutally run any m-code.  Code
///   that edits the data after it is initially read will be
///   ignored.
bool PowerSystem::read_matpower( const std::string& data_str )
{
	match_t        match;
	string         line;
	istringstream  ins;
	bool           success=true;
	unsigned       count=0;
	
	// read bus data
	match.reset();
	if (regex(data_str, "(?s)bus\\s*=.*?\\[(.*?)\\]", match) )
	{
		istringstream ins( match[1] );
		while ( getline( ins, line ) )
		{
			bus_t a_bus;
			if ( a_bus.read( line, MATPOWER ) )
			{
				insert( a_bus ); // incorporates the data into the system model
				count++;
			}
		}
	}
	else
	{
		printf("Warning in PowerSystem::read_matpower: No bus data found\n");
		success = false;
	}
	
	// read gen data
	match.reset();
	if ( regex(data_str, "(?s)gen\\s*=.*?\\[(.*?)\\]", match) )
	{
		istringstream ins( match[1] );
		while ( getline( ins, line ) )
		{
			gen_t a_gen;
			if ( a_gen.read( line, MATPOWER ) )
			{
				insert( a_gen ); // incorporates the data unsignedo the system model
				count++;
			}
		}
	}
	else
	{
		printf("Warning in PowerSystem::read_matpower: No gen data found\n");
		success=false;
	}
	
	// read load data
	match.reset();
	if ( regex(data_str, "(?s)load\\s*=.*?\\[(.*?)\\]", match) )
	{
		if ( options.print_level>NO_PRINT ) 
			printf("Load data found, ignoring Pd, Qd, Gs, Bs in bus data.\n");
		// sort through and read the load data
		istringstream ins( match[1] );
		while ( getline( ins, line ) )
		{
			load_t a_load;
			if ( a_load.read( line, MATPOWER ) )
			{
				insert( a_load ); // incorporates the data unsignedo the system model
				count++;
			}
		}
	}
	else
	{
		if ( options.print_level>NO_PRINT )
			printf("No load data found, building from bus data.\n");
		for ( unsigned bi=0; bi<nBus(); bi++ )
		{
			if ( abs(bus[bi].Pd)>EPS || abs(bus[bi].Qd)>EPS ||
			     abs(bus[bi].Bs)>EPS || abs(bus[bi].Gs)>EPS )
			{
				load_t a_load;
				
				a_load.number = load_t::nextNo; load_t::nextNo++;
				a_load.busNo  = bus[bi].number;
				a_load.Pd = bus[bi].Pd;
				a_load.Qd = bus[bi].Qd;
				a_load.Bs = bus[bi].Bs;
				a_load.Gs = bus[bi].Gs;
				if (bus[bi].number)
					printf("debug");
				a_load.status = ON;
				insert( a_load );
				// in order to prevent confusion, zero out the load data in the bus data structure.
				bus[bi].Pd=0;
				bus[bi].Qd=0;
				bus[bi].Bs=0;
				bus[bi].Gs=0;
			}
		}
	}

	// read branch data
	match.reset();
	if ( regex(data_str, "(?s)branch\\s*=.*?\\[(.*?)\\]", match) )
	{
		istringstream ins( match[1] );
		while ( getline( ins, line ) )
		{
			branch_t       a_branch;
			if ( a_branch.read( line ) )
			{
				insert( a_branch ); // incorporates the data into the system model
				count++;
			}
		}
	}
	else
	{
		printf("Warning in PowerSystem::read_matpower: No branch data found\n");
		success=false;
	}
	update();
	// set the static baseMVA var in the branch data
	branch[0].baseMVA = baseMVA;
	return success;
}

/// update the bus types
void PowerSystem::updateBusTypes() {
	unsigned bi, ge;
	unsigned nRef = 0;
	double largestPg=0;
	
	// go through all the buses
	refNo_ = POWER_EMPTY;
	nRef=0;
	for( bi=0; bi<nBus(); bi++ ) {
		int ng = bus[bi].nGen();
		bus_type_e type = bus[bi].type;
		// check to make sure that the buses know their location in the index correctly
		err_if( bus[bi].ix() != bi, "Problem with the bus data index.");
		// record the reference bus
		if ( type==REF ) {
			refNo_ = bus[bi].number;
			nRef++;
		} else if ( ng>0 and type==PQ ) {	
			// if there are generators, then this must be a PV bus (at least in the current implementation)
			bus[bi].type=PV;
		} else if ( ng==0 and type!=PQ ) {
			// if there are no generators then this is a PQ bus
			bus[bi].type=PQ;
		}
	}
	
	// if we have too many reference buses
	if ( nRef>1 ) {
		// make the first ref that we come to a PV bus
		for ( bi=0; bi<nBus(); bi++ ) {
			if ( bus[bi].type==REF and nRef>1 ) {
				// if we already have a reference bus make this one a pv bus
				bus[bi].type=PV;
				nRef--;
			}
		}
	}
	
	// if we didn't find a reference bus, choose one
	if ( nRef==0 and nGen()>0 ) {
		largestPg = 10.0;
		for (ge=0;ge<nGen();ge++) {
			if ( gen[ge].Pg>largestPg and gen[ge].status==ON ) {
				largestPg=gen[ge].Pg;
				refNo_=gen[ge].busNo;
				nRef=1;
			}
		}
		// if we found a new reference bus fix the index vars
		if ( nRef>0 ) {
			bus(refNo_).type=REF;
		}
	}
	
	if ( nRef!=1 or nGen()==0 ) {
		// something is broken...
		set_blackout();
	}
}

/// update the index variables
void PowerSystem::updateIndexVars() {
	unsigned i;
	//// update the index vars
	// bus index
	for( i=0; i<nBus(); i++ ) bus[i].set_ix(i);
	// make sure the gen index vars are correct
	for( i=0; i<nGen(); i++ ) {
		gen[i].set_ix(i);
		gen[i].bi = bus( gen[i].busNo ).ix();
	}
	// make sure the load index vars are correct
	for( i=0; i<nLoad(); i++ ) {
		load[i].set_ix(i);
		load[i].bi = bus(load[i].busNo).ix();
	}
	// update the branch data
	for ( i=0; i<nBranch(); i++ ) {
		branch[i].set_ix(i);
		// make sure that fbi/tbi are correct
		branch[i].fbi = bus(branch[i].fromBusNo).ix();
		branch[i].tbi = bus(branch[i].toBusNo).ix();
	}
}

/// update the PowerGraph
void PowerSystem::updatePowerGraph() {
	vector<int> from, to;
	set<int> bus_set;
	unsigned i;
	
	// build the lists
	for(i=0;i<nBranch();i++) {
		if (branch[i].status()==ON) {
			from.push_back(branch[i].fromBusNo);
			to.push_back(branch[i].toBusNo);
		}
	}
	for(i=0;i<nBus();i++) {
		bus_set.insert( bus[i].number );
	}
	// build the graph
	PowerGraph_.build( from, to, bus_set );
}

/// update the network data
bool PowerSystem::update() {
	unsigned i, bi;
	
	// make sure baseMVA is recorded in the branch data
	if (nBranch()>0) branch[0].set_baseMVA(baseMVA);
	
	// first update the index vars
	updateIndexVars();
	
	// rebuild the PowerGraph
	updatePowerGraph();
	
	// update the bus types
	updateBusTypes();
	
	// update the Ybus matrix
	calcYbus();
	
	// pull the voltage magnitudes from the gen reference voltages
	for( i=0; i<nGen(); i++ ) {
		bi = gen[i].bi;
		if ( bus[bi].Vmag>0.001 and bus[bi].status==ON and gen[i].status==ON ) {
			bus[gen[i].bi].Vmag = gen[i].Vref;
		} else {
			gen[i].status = OFF;
			gen[i].Pg = 0.0;
			gen[i].Qg = 0.0;
			bus[bi].type=PQ;
		}
	}
	
	return true;
}

/// calcYbus()
bool PowerSystem::calcYbus( bool force_update )
{
	if ( !Ybus_valid_ or force_update ) {
		unsigned br, t, f, i;
		cx  y_ft, y_tf, y_ff, y_tt, y_s, tap;
		DenseVector_cx diag( nBus() );
		
		// clear out all of the matrices
		Ybus_.clear();
		Yf_  .clear();
		Yt_  .clear();
		Ybr_ .clear();
		Ybus_.resize ( nBus()   , nBus() );
		Yf_  .resize ( nBranch(), nBus() );
		Yt_  .resize ( nBranch(), nBus() );
		Ybr_ .resize ( nBranch(), nBus() );
		
		// go through each branch and add in the elements:
		for ( br=0; br<nBranch(); br++ )
		{
			t = branch[br].tbi;
			f = branch[br].fbi;
			branch[br].calc_admittances(y_ft, y_tf, y_ff, y_tt, y_s, tap);
			
			diag[t]    += y_tt;
			diag[f]    += y_ff;
			Ybus_(f,t) += y_ft;
			Ybus_(t,f) += y_tf;
			// complete branch flow matrices:
			Yf_(br,f) = y_ff;
			Yf_(br,t) = y_ft;
			Yt_(br,f) = y_tf;
			Yt_(br,t) = y_tt;
			Ybr_(br,f) =  y_s / tap;
			Ybr_(br,t) = -y_s;
		}
		// add in the shunt elements from the load data
		for ( unsigned lo=0; lo<nLoad(); lo++) {
			diag[load[lo].bi] += cx( load[lo].Gs, load[lo].Bs ) / baseMVA;
		}
		// build diag unsigned ybus
		for (i=0; i<diag.size(); i++)  Ybus_.set( i, i, diag[i] );
	}
	Ybus_valid_=true;
	return true;
}

/// calcSbus()
bool PowerSystem::calcSbus() {
	unsigned bi, ge, lo;
	cx S;
	
	Sbus_.resize( nBus() );
	
	for (bi=0; bi<nBus(); bi++) {
		S = 0;
		for (ge=0;ge<bus[bi].nGen(); ge++) {
			S += bus[bi].gen(ge).Sg()/baseMVA;
		}
		for ( lo=0; lo<bus[bi].nLoad(); lo++ ) {
			S -= bus[bi].load(lo).Sd()/baseMVA;
		}
		Sbus_[bi] = S;
	}	
	return true;
}

/// calcSbus()
bool PowerSystem::calcSbus( DenseVector_cx & Sbus ) {
	unsigned bi, ge, lo;
	cx S;
	
	Sbus.resize( nBus() );
	
	for (bi=0; bi<nBus(); bi++) {
		S = 0;
		for (ge=0;ge<bus[bi].nGen(); ge++) {
			S += bus[bi].gen(ge).Sg()/baseMVA;
		}
		for ( lo=0; lo<bus[bi].nLoad(); lo++ ) {
			S -= bus[bi].load(lo).Sd()/baseMVA;
		}
		Sbus[bi] = S;
	}	
	return true;
}

/// MakePowerFlowJac
///  @param Jac is the output Jacobian
///  @param x ix a vector with voltage magnitudes and angles
///  @param row_index is an index that indicates which entries to calculate
///  @param col_index is an index that indicates which entries to calculate
bool PowerSystem::MakePowerFlowJac( Sparse &Jac, DenseVector &x,
                                    std::vector<unsigned> &row_index, std::vector<unsigned> &col_index )
{
	unsigned i, j;
	unsigned n = nBus();
	unsigned dP_row, dQ_row, dVmag_col, dTheta_col;
	double dP_dTheta_diag, dP_dVmag_diag, dQ_dTheta_diag, dQ_dVmag_diag;
	double Vmag_i, Vmag_j, theta_i, theta_j;
	double theta_ij, sin_theta_ij, cos_theta_ij;
	double	b_ii, g_ii, b_ij, g_ij;
	bool use_x = (x.size()==n*2);
	cx	y_ij, y_ii;
	set<int>::iterator iter;
	int busNo;
	
	// check to make sure that the Jac is sized right
	if (Jac.rows()==0 or Jac.cols()==0) {
		Jac.resize(n*2,n*2);
		row_index.clear();
		col_index.clear();
	}
	// rebuild row/col indeces if needed
	if (row_index.size()!=n*2) {
		row_index.resize(n*2);
		col_index.resize(n*2);
		for(i=0;i<n*2;i++) {
			row_index[i]=i;
			col_index[i]=i;
		}
	}
	// go through each bus and calculate the Jac elements
	for (i=0; i<n; i++)
	{
		// calculate the row indeces
		dP_row = row_index[i];
		dQ_row = row_index[i+n];
		// extract data that will be useful
		y_ii = Ybus_(i,i);
		g_ii = y_ii.real();
		b_ii = y_ii.imag();
		if (use_x) {
			theta_i = x[i];
			Vmag_i  = x[i+n];
		} else {
			theta_i = bus[i].theta();
			Vmag_i  = bus[i].Vmag;
		}
		// initialize the diagonal elements
		dP_dVmag_diag  =  2*Vmag_i*g_ii;
		dQ_dVmag_diag  = -2*Vmag_i*b_ii;
		dP_dTheta_diag = 0; // not sure about this term...???
		dQ_dTheta_diag = 0; // not sure about this term...???
		
		// for each connected bus
		for( iter=bus[i].neighbor_set.begin(); iter!=bus[i].neighbor_set.end(); iter++ )
		// calculate each of the following elements
		{
			// figure the index variables
			busNo = *iter;
			j = bus.index(busNo);
			dTheta_col = col_index[j];
			dVmag_col  = col_index[j+n];
			// extract useful data
			y_ij = Ybus_.get(i,j);
			g_ij = y_ij.real();
			b_ij = y_ij.imag();
			if (use_x) {
				theta_j = x[j];
				Vmag_j  = x[j+n];
			} else {
				theta_j = bus[j].theta();
				Vmag_j  = bus[j].Vmag;
			}
			theta_ij = is_unknown(theta_i) || is_unknown(theta_j) ? 0.0 : theta_i - theta_j;
			cos_theta_ij = cos( theta_ij );
			sin_theta_ij = sin( theta_ij );
			
			// off-diag elements:
			if ( dP_row != POWER_EMPTY )
			{
				// dP_dVmag  for off-diag, see p.174 of Bergen, "Power Systems Analysis"
				if ( dVmag_col != POWER_EMPTY )
					Jac.set( dP_row, dVmag_col, Vmag_i*( g_ij*cos_theta_ij + b_ij*sin_theta_ij ) ); //ok
				// dP_dTheta for off-diag, see p.174 of Bergen (1986), "Power Systems Analysis"
				if ( dTheta_col != POWER_EMPTY )
					Jac.set( dP_row, dTheta_col, Vmag_i*Vmag_j*(g_ij*sin_theta_ij - b_ij*cos_theta_ij) );//ok
			}
			
			if (dQ_row != POWER_EMPTY)
			{
				// dQ_dVmag  for off-diag, see p.174 of Bergen (1986), "Power Systems Analysis"
				if ( dVmag_col != POWER_EMPTY )
					Jac.set( dQ_row, dVmag_col, Vmag_i*( g_ij*sin_theta_ij - b_ij*cos_theta_ij ) ); //ok
				// dQ_dTheta for off-diag, see p.175 of Bergen (1986), "Power Systems Analysis"
				if ( dTheta_col != POWER_EMPTY )
					Jac.set(dQ_row, dTheta_col, -Vmag_i*Vmag_j*( g_ij*cos_theta_ij + b_ij*sin_theta_ij ) ); //ok
			}
			
			// diagonal elements: again see p. 174, 175 of Bergen (1986), "Power Systems Analysis"
			// dP_dTheta for diagonal
			dP_dTheta_diag += Vmag_i*Vmag_j*( -g_ij*sin_theta_ij + b_ij*cos_theta_ij ); //ok
			// dQ_dTheta for diagonal
			dQ_dTheta_diag += Vmag_i*Vmag_j*(  g_ij*cos_theta_ij + b_ij*sin_theta_ij ); //ok
			// dP_dVmag  for diagonal
			dP_dVmag_diag  += Vmag_j*( g_ij*cos_theta_ij + b_ij*sin_theta_ij ); //ok
			// dQ_dVmag  for diagonal
			dQ_dVmag_diag  += Vmag_j*( g_ij*sin_theta_ij - b_ij*cos_theta_ij ); //ok
		}
		// put the diagonal elements into the matrix
		dVmag_col = col_index[i+n];
		dTheta_col = row_index[i];
		if (dP_row != POWER_EMPTY)
		{
			if (dTheta_col != POWER_EMPTY)
				Jac.set( dP_row, dTheta_col, dP_dTheta_diag );
			if (dVmag_col != POWER_EMPTY)
				Jac.set( dP_row, dVmag_col, dP_dVmag_diag );
		}
		if (dQ_row != POWER_EMPTY)
		{
			if (dTheta_col != POWER_EMPTY)
				Jac.set( dQ_row, dTheta_col, dQ_dTheta_diag );
			if (dVmag_col != POWER_EMPTY)
				Jac.set( dQ_row, dVmag_col, dQ_dVmag_diag );
		}
	}
	// debug
	//Jac.print("Jac");
	//exit(0);
	return true;
}

/// calcMismatch
///  DenseVector& x -- the the vector of voltage magnitudes and angles
///  DenseVector& mismatch -- the output mismatch vector
///  vector<unsigned>& index -- the 
double	PowerSystem::calcMismatch( DenseVector& x, DenseVector& mismatch, const vector<unsigned>& index )
{
	unsigned bi;
	unsigned n = nBus();
	double  mismatch_sum=0;
	DenseVector_cx Ibus( n );
	DenseVector_cx V( n );
	DenseVector_cx mismatch_cx( n );
	
	assert(index.size()==2*n || index.size()==0);
	assert(mismatch.size()==2*n);
	for (bi=0; bi<n; bi++)
		V[bi] = polar(x[bi+n],x[bi]);
		
	Ibus.mult( Ybus_, V ); // Ibus = Ybus * V
	
	for (bi=0; bi<n; bi++) {
		mismatch_cx[bi] = V[bi] * conj( Ibus[bi] ) - Sbus_[bi];
		mismatch[bi] = mismatch_cx[bi].real();
		mismatch[bi+n] = mismatch_cx[bi].imag();
		if ( index.size()==0 || index[bi] != POWER_EMPTY )
			mismatch_sum += abs(mismatch[bi]);
		if ( index.size()==0 || index[bi+n] != POWER_EMPTY )
			mismatch_sum += abs(mismatch[bi+n]);
	}
	
	if (debug) {
		mismatch_cx.print("mismatch");
		throw "debug";
	}
	return mismatch_sum;
}
/// calcV
bool PowerSystem::calcV( DenseVector_cx& V )
{
	unsigned bi;
	
	assert( V.size() == nBus() );
	for (bi=0; bi<nBus(); bi++)
	{
		V[bi] = bus[bi].V();
	}
	return true;
}

/// calcFlows
bool PowerSystem::calcFlows() {
	unsigned i;
	DenseVector_cx If( nBranch() );
	DenseVector_cx It( nBranch() );
	DenseVector_cx V ( nBus() );
	
	calcV( V );
	
	// calculate branch currents
	If.mult ( Yf_, V );
	It.mult ( Yt_, V );
	 
	for (i=0;i<nBranch(); i++) {
		cx Vf( V[ branch[i].fbi ] );
		cx Vt( V[ branch[i].tbi ] );
		
		branch[i].set_voltages( abs( Vf ), abs( Vt ), arg(Vf)-arg(Vt), true );
	}
	
	return true;
}

/// insert a bus
void PowerSystem::insert( bus_t a_bus ) // inserts a bus into the model
{
	// clear stuff in the bus object that will cause us problems later
	a_bus.eraseIx();
	// add references to the gen/load
	a_bus.p_gen = &gen;
	a_bus.p_load = &load;
	a_bus.p_branch = &branch;
	bus.insert(a_bus);
	PTDF_valid_=false; // adding a bus changes the PTDF matrix
	Ybus_valid_=false; // adding a bus changes the Ybus matrix
}

/// insert an event
void PowerSystem::insert( event_t an_event ) // inserts an event into the model
{
	//an_event.ix = event.next();
	event.insert(an_event);
}

/// insert a gen
void PowerSystem::insert ( gen_t a_gen ) // inserts a gen into the model
{
	// if the gen does not exist yet add appropriate references
	if ( !gen.exist(a_gen.number) )
	{
		// if the bus does not exist yet make a new one
		if ( !bus.exist(a_gen.busNo) )
		{
			bus_t a_bus;
			a_bus.number = a_gen.busNo;
			bus.insert(a_bus);
			Ybus_valid_=false; // adding a bus changes the Ybus matrix
		}
		a_gen.bi = bus.index(a_gen.busNo);
		bus[a_gen.bi].add_ref(a_gen);
		PTDF_valid_=false; // adding a gen changes the PTDF matrix
	}
	// insert the gen into the system model
	gen.insert(a_gen);
}

/// insert a load
void PowerSystem::insert( load_t a_load ) // inserts a load into the model
{
	// if the load does not exist yet add appropriate references
	if ( !load.exist(a_load.number) )
	{
		// if the referenced bus does not exist, create it
		if ( !bus.exist(a_load.busNo) )
		{
			bus_t a_bus;
			a_bus.number = a_load.busNo;
			insert(a_bus);
			Ybus_valid_=false; // adding a bus changes the Ybus matrix
		}
		a_load.bi = bus.index(a_load.busNo);
		bus[a_load.bi].add_ref(a_load);
		PTDF_valid_=false; // adding a load changes the PTDF matrix

	}
	// insert the load into the system model
	load.insert(a_load);
	// also, if the load has non-zero B or S elements, invalidate Ybus
	if ( abs(a_load.Bs) > EPS || abs(a_load.Gs) > EPS ) {
		Ybus_valid_ = false;
	}
}

/// insert a branch
void PowerSystem::insert( branch_t a_branch ) // inserts a branch into the model
{
	if ( !branch.exist( a_branch.number ) )
	{
		if ( !bus.exist( a_branch.fromBusNo ) )
		{
			bus_t a_bus;
			a_bus.number = a_branch.fromBusNo;
			insert(a_bus);
		}
		if ( !bus.exist( a_branch.toBusNo ) )
		{
			bus_t a_bus;
			a_bus.number = a_branch.toBusNo;
			insert(a_bus);
		}
		a_branch.fbi = bus.index( a_branch.fromBusNo );
		a_branch.tbi = bus.index( a_branch.toBusNo );
		
		// update the bus references
		bus[a_branch.fbi].add_ref( a_branch );
		bus[a_branch.tbi].add_ref( a_branch );
	}
	branch.insert( a_branch );
	// this changes Ybus and PTDF
	Ybus_valid_ = false;
	PTDF_valid_ = false;
}
/// check the data
bool PowerSystem::check() {
	unsigned i;
	for (i=0;i<nBus();i++)
		if (bus[i].number==POWER_EMPTY) return false;
	for (i=0;i<nGen();i++)
		if (gen[i].number==POWER_EMPTY) return false;
	for (i=0;i<nLoad();i++)
		if (load[i].number==POWER_EMPTY) return false;
	for (i=0;i<nBranch();i++)
		if (branch[i].number==POWER_EMPTY) return false;
	
	return true;
}

void PowerSystem::neighbor_sets( int bus_no , unsigned r1, unsigned r2, vector< set<int> > &neighbors )
{
	unsigned r_max=100;
	unsigned r=0;
	unsigned set_no=0;
	int current_bus = bus_no;
	set<int> curr_layer;
	set<int> prev_layer;
	set<int> local_neighbors;
	set<int>::iterator iter, neigh_iter;
	vector< set<int> > all;
	unsigned old_size, new_size;
	
	// some error checking
	err_if( r2<=r1, "Invalid input into PowerSystem::neighbor_sets" );
	// begin the main code
	all.resize(4);
	all[0].insert(bus_no);
	// fill the previous layer with neighbors[0]
	prev_layer = all[0];
	for ( r=0; r<r_max; r++ ) {
		if ( r==0 || r==r1 || r==r2 ) {
			set_no++;
			all[set_no] = all[set_no-1];
		}
		// for each bus in prev_layer
		for ( iter=prev_layer.begin(); iter!=prev_layer.end(); iter++ ) {
			current_bus = *iter;
			curr_layer = set_union( curr_layer, bus(current_bus).neighbor_set );
		}
		// if curr_layer includes everything that is also included in all[set_no] then quit:		
		//  also update all[set_no] with the elements in curr_layer
		old_size = all[set_no].size();
		all[set_no] = set_union( all[set_no], curr_layer );
		new_size = all[set_no].size();
		if (old_size == new_size) break;
		prev_layer=curr_layer;
		curr_layer.clear();
	}
	// remove the elements in each layer that are already in the previous layer,
	//  and put the result in neighbors
	neighbors.resize(4);
	neighbors[0]=all[0];
	for ( set_no=1; set_no<neighbors.size(); set_no++ )
	{
		set<int> result;
		set_difference( all[  set_no  ].begin(), all[  set_no  ].end(),
		                all[ set_no-1 ].begin(), all[ set_no-1 ].end(),
		                inserter( result, result.begin() ) );
		neighbors[set_no] = result;
	}
}

bool PowerSystem::calcPTDF()
{
	if (!PTDF_valid_)
	{
		Sparse B ( nBus(), nBus() );
		Sparse H ( nBranch(), nBus() );
		DenseVector column( nBranch() );
		unsigned lo, ge;
		
		PTDF_.resize( nBranch(), nBus() );
		PTDF_loads_.resize( nBranch(), nLoad() );
		PTDF_gens_.resize ( nBranch(), nGen()  );
		
		B.copy_imag ( Ybus_ );
		H.copy_imag ( Ybr_ );
		
		PTDF_.divide ( H, B );
		
		// build the PTDF matrix for loads
		for ( lo=0; lo<nLoad(); lo++ )
		{
			column.copy_col( PTDF_, load[lo].bi );
			column.scale(-1);
			PTDF_loads_.copy_to_col( column , lo );
		}
		// build an injection matrix for gens
		for ( ge=0; ge<nGen(); ge++ )
		{
			column.copy_col( PTDF_, gen[ge].bi );
			PTDF_gens_.copy_to_col( column, ge );
		}
	}
	PTDF_valid_=true;
	//PTDF_.print("PTDF");
	//exit(0);
	return true;
}

void PowerSystem::clear()
{
	bus.clear();
	gen.clear();
	load.clear();
	branch.clear();
	Ybus_valid_ = false;
	PTDF_valid_ = false;
}

bool PowerSystem::SolveOPF() {
	// variables:
	bool success=false;
	printf("OPF is not working yet.\n");
	/*
This is not working yet
	ApplicationReturnStatus status;
	SmartPtr<IpoptApplication> app = new IpoptApplication();
	SmartPtr<TNLP> pIpoptProblem = new PowerSolver(this, OPF, RATE_A);
	//SmartPtr<TMINLP> pBonminProblem = (TMINLP*) new PowerSolver(sys);
	
	// initialize Ipopt
	app->Initialize();
	// solve the problem
	status = app->OptimizeTNLP(pIpoptProblem);
	if (status == Solve_Succeeded)
		success = true;
	*/
	return success;
}

bool PowerSystem::is_stressed() {
	unsigned i;
	double Imag, Imax, Vmin, Vmag;
	// branches
	for (i=0;i<nBranch();i++) {
		Imag = branch[i].Imag()*(double)baseMVA;
		Imax = branch[i].rateB;
		if (Imag>Imax) return true;
	}
	// buses
	for (i=0; i<nBus(); i++) {
		Vmag = bus[i].Vmag;
		Vmin = bus[i].Vmin * double(Vmag>0.001);
		// undervoltage
		if ( Vmag < Vmin ) return true;
	}
	return false;
}


bool PowerSystem::SolveMPC( Sparse &dPg, Sparse &dPd ) {
	printf("SolveMPC not working presently.\n");
	return false;
}

bool PowerSystem::SolveMPC( Sparse &dPd, Sparse &dPg, Sparse &dVg ) {
	printf("SolveMPC not working presently.\n");
	return false;
}

bool PowerSystem::SolveMPC( Sparse &dPd, Sparse &dPg, Sparse &dVg, SparseVector &dVmag, SparseVector &dImag, double &dTheta_slack ) {
	printf("SolveMPC not working presently.\n");
	return false;
}

/// remove a branch from service
void PowerSystem::removeBranch( int branch_number ) {
	unsigned br  = branch.index(branch_number);
	unsigned t, f;
	cx y_ft, y_tf, y_ff, y_tt, y_s, tap;
	
	// check to make sure the input is valid
	err_if ( br >= nBranch(), "Cannot remove branch. Invalid branch number." );
	// if the branch is already off, don't do anything
	if (branch[br].status()==OFF) {
		// check to make sure that it is not half off
		if ( branch[br].status_f==ON || branch[br].status_t==ON ) {
			error("Branch half on, half off.");
		}
		return;
	}
	// calculate the branch admittances with the branch on
	branch[br].calc_admittances( y_ft, y_tf, y_ff, y_tt, y_s, tap ) ;
	// update the branch status
	branch[br].set_status(OFF);
	// if the Ybus is already invlid, recalculate
	if (!Ybus_valid_) {
		calcYbus();
	} else {
		// update the Ybus elements with the change
		t = branch[br].tbi;
		f = branch[br].fbi;
		Ybus_(t,t) -= y_tt;
		Ybus_(f,f) -= y_ff;
		Ybus_(f,t) -= y_ft;
		Ybus_(t,f) -= y_tf;
		// update the branch flow matrices with the change
		Yf_(br,f)  = 0;
		Yf_(br,t)  = 0;
		Yt_(br,f)  = 0;
		Yt_(br,t)  = 0;
		Ybr_(br,f) = 0;
		Ybr_(br,t) = 0;
	}
	// invalidates PTDF
	PTDF_valid_=false;
	// remove the branch from the PowerGraph_
	PowerGraph_.remove_edge(branch[br].fromBusNo, branch[br].toBusNo);
}

/// restore a branch to service
void PowerSystem::restoreBranch( int branch_number )
{
	unsigned br  = branch.index(branch_number);
	unsigned t, f;
	cx y_ft, y_tf, y_ff, y_tt, y_s, tap;
	
	// check to make sure the input is valid
	err_if ( br >= nBranch(), "Invalid branch number" );
	// if the branch is already on, don't do anything
	if (branch[br].status()==ON) return;
	// otherwise update the data as follows:
	// update the branch status
	branch[br].set_status(ON);
	
	branch[br].calc_admittances( y_ft, y_tf, y_ff, y_tt, y_s, tap ) ;
	
	if (!Ybus_valid_) calcYbus();
	// update the Ybus elements with the change
	t = branch[br].tbi;
	f = branch[br].fbi;
	Ybus_(t,t) += y_tt;
	Ybus_(f,f) += y_ff;
	Ybus_(f,t) += y_ft;
	Ybus_(t,f) += y_tf;
	// update the branch flow matrices with the change
	Yf_(br,f) = y_ff;
	Yf_(br,t) = y_ft;
	Yt_(br,f) = y_tf;
	Yt_(br,t) = y_tt;
	Ybr_(br,f) =  y_s / tap;
	Ybr_(br,t) = -y_s;
	// invalidates PTDF
	PTDF_valid_=false;
}

void PowerSystem::set_flat_start() {
	unsigned i;
	
	for( i=0; i<nBus(); i++ ) {
		bus[i].Vmag = 1;
		bus[i].Vang = 0;
		bus[i].converged = true;
		bus[i].status = ON;
	}
}

/// runPowerFlow
bool PowerSystem::runPowerFlow( bool resolve_if_failure )
{
	return newPowerFlow( resolve_if_failure );
	/*
	// before we do anything update the system
	update();
	// variables	
	unsigned bi, i, k;
	unsigned n = nBus();
	unsigned nX = nPQ()+n-1;
	double mismatch_sum = 1;
	double step_size = -1;
	double direction_norm = 100.0;
	vector<unsigned> x_index;   // index that maps between the full vector and the subset that we are unsignederested in
	vector<unsigned> x_rev_index( n*2, POWER_EMPTY ); // index that maps between the the subset and the full vector
	vector<unsigned> mismatch_index; // index that maps between the full mismatch vector and the subset
	vector<unsigned> mismatch_rev_index( n*2, POWER_EMPTY ); // index that maps between the subset and the full mis vector
	DenseVector_cx V(n); // the bus voltage
	DenseVector x_full(n*2); // contains the full vector of voltage magnitudes and angles
	DenseVector mismatch_full(n*2); // the actual mismatch vector
	DenseVector mismatch_sub(nX);  // a subset of the mismatch vector
	DenseVector direction(nX); // gives the newton step direction
	vector<PowerSystem> islands; // used to store sub-islands in the system
	set<unsigned>::iterator iter;
	
	// check for conditions that would make this system trivial
	if ( nBus()<2 or nPV()==0 or nPQ()==0 or nGenOn()==0 or nLoadOn()==0 ) {
		set_blackout();
		return true; // the solution is valid...it just isn't a nice one
	}
	// check for and deal with islanded systems
	//  If an island is found, run a power flow on each island
	if ( nIslands_ > 0 ) {
		bool complete_success=true;
		bool success=true;
		
		findIslands(islands);
		for (i=0;i<islands.size();i++) {
			if (options.print_level>=MINIMAL) {
				printf("Running power flow on island %d of %d\n", i+1, (int)islands.size());
			}
			success = islands[i].runPowerFlow( resolve_if_failure );
			if ( not success ) {
				islands[i].set_blackout();
			}
			// read the islands data into the local system model
			read( islands[i] );
			complete_success &= success;
		}
		return complete_success;
	}
	// fix strange initial voltages and set things optimistically
	for ( bi=0; bi<n; bi++ ) {
		if ( bus[bi].Vmag < 0.5 or bus[bi].Vmag > 1.5 ) {
			bus[bi].Vmag = 1;
			bus[bi].Vang = 0;
		}
		bus[bi].status = ON;
		bus[bi].converged = true;
	}
	
	//// Beginning of standard power-flow algorithm
	// assemble the full x vector:
	// x is [theta;Vmag];
	for ( bi=0; bi<n; ++bi ) {
		x_full[bi]   = bus[bi].Vang*PI/180;
		x_full[bi+n] = bus[bi].Vmag;
	}
	// assemble the x vector and the index vectors
	for ( iter=pv_list_.begin(); iter!=pv_list_.end(); iter++ ) {
		bi = *iter;
		// add the theta to x_index
		x_index.push_back( bi );
		x_rev_index[bi] = x_index.size()-1;
		// mismatch -- P to mismatch_index
		mismatch_index.push_back(bi);
		mismatch_rev_index[bi] = mismatch_index.size()-1;	
	}
	for ( iter=pq_list_.begin(); iter!=pq_list_.end(); iter++ ) {
		bi = *iter;
		// add the theta to x_index
		x_index.push_back(bi);
		x_rev_index[bi] = x_index.size()-1;
		// mismatch -- P to mismatch_index
		mismatch_index.push_back(bi);
		mismatch_rev_index[bi] = mismatch_index.size()-1;	
	}
	for ( iter=pq_list_.begin(); iter!=pq_list_.end(); iter++) {
		bi = *iter;
		// add the Vmag to x_index
		x_index.push_back(bi+n);
		x_rev_index[bi+n] = x_index.size()-1;
		// mismatch -- Q to mismatch_index
		mismatch_index.push_back(bi+n);
		mismatch_rev_index[bi+n] = mismatch_index.size()-1;	
	}
	assert( mismatch_index.size()==nX );
	assert( x_index.size()==nX );
	// initialize the jacobian
	Jac_.zeros();
	Jac_.resize( nX, nX ) ;
	// calculate complex voltage
	for (bi=0; bi<n; bi++) {
		V[bi] = polar(bus[bi].Vmag, bus[bi].Vang*PI/180);
	}
	// begin the main power flow loop
	for ( k=0; k<options.max_pf_iterations; k++ ) {
		mismatch_sum = calcMismatch(x_full, mismatch_full, mismatch_rev_index);
		if ( options.print_level==VERBOSE) {
			printf("Power flow iteration %2d. Mismatch = %15.10f, Direction norm=%15.10f\n",
				k, mismatch_sum, direction_norm);
		}
		if ( mismatch_sum < options.pf_eps ) {
			power_flow_finish( x_full, mismatch_full, mismatch_rev_index);
			if ( options.print_level==VERBOSE ) printf("Power flow converged.\n");
			for(i=0;i<nBus();i++) bus[i].converged=true;
			return true;
		}
		// assemble the Jacobian
		MakePowerFlowJac( Jac_, x_full, x_rev_index, mismatch_rev_index );
		// update the mismatch sub-vector
		for (i=0;i<nX;i++) {
			mismatch_sub[i] = mismatch_full[ mismatch_index[i] ];
		}
		// solve for the newton step direciton
		if ( not direction.solve( Jac_, mismatch_sub ) ) {
			if ( options.print_level >= MINIMAL ) {
				printf("Could not solve linear system in power flow.\n");
			}
			for(i=0;i<nBus();i++) bus[i].converged=false;
			break;
		}
		direction_norm = direction.norm()/direction.length();

		// choose a step size
		step_size = 1;
		// update the decision vector:
		for (i=0; i<x_index.size(); i++) {
			x_full[x_index[i]] -= step_size * direction[i];
		}
	}
	
	if ( options.print_level >= MINIMAL ) printf("Power flow did not converge.\n");
	
	if ( resolve_if_failure ) {
		double factor;
		// first try to resolve with a flat start
		printf("Resolving powerflow from flat start.\n");
		set_flat_start();
		if ( runPowerFlow( false ) ) return true;
		// if that doesn't work try to shed some load
		while ( ufls_factor_ > 0.0 ) {
			factor = maximum( ufls_factor_ - 0.25, 0.0 );
			printf( "Reducing load/generation to %.2f %%.\n", factor*100 );
			scale( factor/ufls_factor_ ); // scale the system to reflect the new UFLS factor
			ufls_factor_ = factor;
			// force it to update the Ybus and start from a flat system
			invalidate();
			set_flat_start();
			// try to run the power flow
			if ( runPowerFlow( false ) ) return true;
		}
	}
	return false;
	*/
}

void PowerSystem::set_blackout() {
	unsigned i;
	
	if ( not is_blackout_ ) {
		is_blackout_ = true;
		printf("Blackout occured.\n");
		// buses
		for( i=0; i<nBus(); i++ ) {
			bus[i].Vmag = 0;
			bus[i].Vang = 0;
			bus[i].status = OFF;
		}
		// loads
		for( i=0; i<nLoad(); i++ ) {
			load[i].Pd = 0.0;
			load[i].Qd = 0.0;
			load[i].status = OFF;
		}
		// gens
		for( i=0; i<nGen(); i++ ) {
			gen[i].Pg = 0.0;
			gen[i].Qg = 0.0;
			gen[i].status = OFF;
		}
		// branches
		for( i=0; i<nBranch(); i++ ) {
			branch[i].set_voltages( 0, 0, 0 );
			branch[i].set_currents( 0, 0 );
		}
	}
}

void PowerSystem::read( PowerSystem & ps ) {
	unsigned i;
	
	// read buses
	for( i=0; i<ps.nBus(); i++ ) {
		insert( ps.bus[i] );
	}
	// read branches
	for( i=0; i<ps.nBranch(); i++ ) {
		insert( ps.branch[i] );
	}
	// read loads
	for( i=0; i<ps.nLoad(); i++ ) {
		insert( ps.load[i] );
	}
	// read gens
	for( i=0; i<ps.nGen(); i++ ) {
		insert( ps.gen[i] );
	}
}

void PowerSystem::scale( double scale_factor ) {
	unsigned i;
	
	// scale loads
	for( i=0; i<nLoad(); i++ ) {
		load[i].Pd *= scale_factor;
		load[i].Qd *= scale_factor;
		load[i].Bs *= scale_factor;
		load[i].Gs *= scale_factor;
	}

	// scale gens
	for( i=0; i<nGen(); i++ ) {
		gen[i].Pg *= scale_factor;
		gen[i].Qg *= scale_factor;
	}
}



int PowerSystem::findIslands( std::vector<PowerSystem> &islands )
{
	vector< set<int> > islandBusNos;
	unsigned nComponents;
	unsigned i;
	
	// figure out where the islands are
	islands.clear();
	nComponents = PowerGraph_.find_subgraphs(islandBusNos);
	if (nComponents>1) {
		islands.resize(nComponents);
		for (i=0;i<nComponents;i++) {
			islands[i].subset(*this,islandBusNos[i]);
		}
	}
	
	return nComponents-1;
}

/// return true if i is in A
static bool ismember( int i, set<int> A ) {
	return ( A.find(i) != A.end() );
}

void PowerSystem::subset( PowerSystem &InputSys, std::set<int> &busNos ) {
	// vars
	set<int>::iterator it;
	unsigned i;
	
	// clear out the internal data
	clear();
	// copy the buses
	for ( it=busNos.begin(); it!=busNos.end(); it++ ) {
		insert(InputSys.bus(*it));
	}
	// copy the branches that are completely contained within the system
	for ( i=0; i<InputSys.nBranch(); i++ ) {
		if ( ismember( InputSys.branch[i].fromBusNo, busNos ) && ismember( InputSys.branch[i].toBusNo, busNos ) ) {
			insert(InputSys.branch[i]);
		}
	}
	// copy the loads
	for ( i=0; i<InputSys.nLoad(); i++ ) {
		if ( ismember( InputSys.load[i].busNo, busNos ) ) {
			insert(InputSys.load[i]);
		}
	}
	// copy the gens
	for ( i=0; i<InputSys.nGen(); i++ ) {
		if ( ismember( InputSys.gen[i].busNo, busNos ) ) {
			insert(InputSys.gen[i]);
		}
	}
	// copy the options
	options = InputSys.options;
	// check to make sure that the local model is valid
	update();
}


bool PowerSystem::control( Sparse &dPd, Sparse &dPg, Sparse &dVg ) {
	unsigned number, k;
	double delta;
	int busNo;
	
	// dPd
	dPd.reset_next();
	while( dPd.get_next(number,k,delta) ) {
		if ( k==0 ) {
			load(number).change(delta);
		} else break;
	}
	// dPg
	dPg.reset_next();
	while( dPg.get_next(number,k,delta) ) {
		if ( k==0 ) {
			gen(number).change_Pg(delta);
		} else break;
	}
	// dVg
	dVg.reset_next();
	while( dVg.get_next(number,k,delta) ) {
		if ( k==0 ) {
			busNo = gen(number).busNo;
			gen(number).change_Vg(delta);
			bus(busNo).Vmag = gen(number).Vref;
			if( bus(busNo).Vmag<bus(busNo).Vmin ) {
				printf("Attempt to reduce gen voltage too far.\n");
			}
		} else break;
	}
	return true;
}
void PowerSystem::change_ref_bus( int busNo ) {
	bus(refNo_).type  = PV;
	bus(busNo).type = REF;
	refNo_ = busNo;
	update();
}

/// adjust the state variables after control
void PowerSystem::adjust_state_vars( SparseVector &dVmag, SparseVector &dImag ) {
	unsigned no;
	double change;
	
	// NOTE: this only works right when dVg's are not in dVmag
	dVmag.reset_next();
	while( dVmag.get_next(no, change) ) {
		bus(no).Vmag += change;
	}
	// 
	dImag.reset_next();
	while( dImag.get_next(no, change) ) {
		branch(no).Imag_f += change;
		branch(no).Imag_t += change;
	}	
}

/// write some recent data
void PowerSystem::write_recent_data( std::string &data, double threshold_time ) {
	unsigned i;
	// write the model data for each bus with a time stamp greater than threshold_time
	for ( i=0; i<nBus(); i++ ) {
		if ( bus[i].T_measured>threshold_time ) {
			bus[i].write_state( data );
		}
	}
}

/// sim step
bool PowerSystem::sim_step() {
	std::string msg;
	bool output = sim_step( NO_STEP_SIZE, msg );
	
	if (msg.size()>2) {
		printf("%s\n", msg.c_str() );
	}
	return output;
}

/// test the new power flow method
bool PowerSystem::newPowerFlow( bool use_load_shedding_power_flow ) {
	int i, f, t, ge, bi, n=nBus(), sub;
	vector<bus_type_e> types( n );
	DenseVector_cx  Mismatch( n );
	DenseVector_cx      Sbus( n );
	DenseVector_cx         V( n );
	DenseVector_cx  Mismatch_sub;
	DenseVector_cx      Sbus_sub;
	DenseVector_cx         V_sub;
	Sparse_cx           Ybus_sub;
	vector<bus_type_e> types_sub;
	bool success=false, success_sub;
	int nSubgraphs;
	vector< set<int> > subgraphs;
	set<int>::iterator iter;
	double old_scale_factor, scale_factor=1.0;
	vector<double> scale_factors( n, 1.0 );
	PowerFlowOptions pfOptions;
	
	// prepare the data
	update();
	calcSbus( Sbus );
	calcV( V );
	for( i=0; i<n; i++ ) types[i] = bus[i].type;
	// run the power flow algorithm on each island
	nSubgraphs = PowerGraph_.find_subgraphs( subgraphs );
	for( sub=0; sub<nSubgraphs; sub++ ) {
		int i_sub=0, retry_no=0, nZeros=0;
		int nBus_sub = subgraphs[sub].size();
		set<int> bus_index_set;
		Mismatch_sub.resize(nBus_sub);
		Sbus_sub    .resize(nBus_sub);
		V_sub       .resize(nBus_sub);
		types_sub   .resize(nBus_sub);
		Ybus_sub    .resize(nBus_sub,nBus_sub);
		
		// print something
		if( options.print_level>0 ) printf("Solving Power Flow for sub-system %2d of %2d (nBus = %d).\n", sub+1, nSubgraphs, nBus_sub );
		// build the subsets
		for( iter=subgraphs[sub].begin(); iter!=subgraphs[sub].end(); iter++ ) {
			bi = bus(*iter).ix();
			bus_index_set.insert(bi);
			Sbus_sub [i_sub] = Sbus[bi];
			V_sub    [i_sub] = V[bi];
			types_sub[i_sub] = types[bi];
			scale_factors[bi] = 1.0;
			i_sub++;
			// check for zero voltage
			if( abs( V[bi] ) < 0.001 ) {
				nZeros++;
			}
		}
		// if all the buses are zero voltage, no need to solve
		if( nZeros == nBus_sub ) {
			success_sub=true;
			V_sub.zeros();
			Mismatch_sub.zeros();
		} else {
			Ybus_sub.subset( Ybus_, bus_index_set, bus_index_set );
			
			// find an initial solution
			pfOptions.flat_start=false;
			success_sub = SolvePowerFlow( Ybus_sub, &types_sub[0], Sbus_sub, V_sub, Mismatch_sub, pfOptions );
			if (success_sub) success = true;
			// do the load-shedding power flow if requested
			if( use_load_shedding_power_flow and not success_sub and nBus_sub>1 ) {
				// start from a flat start
				pfOptions.flat_start=true;
				// 
				while ( not success_sub and scale_factor>1e-3 ) {
					if( retry_no>0 ) {
						old_scale_factor = scale_factor;
						scale_factor = maximum( scale_factor-0.25, 0.0 );
						// scale Sbus
						Sbus_sub.scale( scale_factor/old_scale_factor );
					}
					// rerun the power flow
					if( scale_factor<1e-3 ) {
						printf("No power flow solution found.\n");
						break;
					} else {
						printf("Re-running Power Flow at %.0f %% load factor.\n", scale_factor*100 );
						success_sub = SolvePowerFlow( Ybus_sub, &types_sub[0], Sbus_sub, V_sub, Mismatch_sub, pfOptions );
						if (success_sub) success = true; // we'll call it success if any of the subgraphs give a valid solution
					}
					retry_no++;
				}
			}
		}
		// if we could not find a solution, set this system to the blackout state
		if (not success_sub or scale_factor<1e-3 ) {
			V_sub.zeros();
			Mismatch_sub.zeros();
		}
		// copy the result back to the main vector outputs
		i_sub=0;
		for( iter=bus_index_set.begin(); iter!=bus_index_set.end(); iter++ ) {
			bi = *iter;
			V[bi]             = V_sub[i_sub];
			types[bi]         = types_sub[i_sub];
			Mismatch[bi]      = Mismatch_sub[i_sub];
			scale_factors[bi] = scale_factor;
			i_sub++;
		}
	}
	
	// update bus voltage data
	for( i=0; i<n; i++ ) {
		bus[i].set_voltage( V[i] );
	}
	// update branch current data
	for( i=0; i<(int)nBranch(); i++ ) {
		f = branch[i].fbi;
		t = branch[i].tbi;
		branch[i].set_voltages( abs(V[f]), abs(V[t]), arg(V[f]) - arg(V[t]), true );
	}
	// update generator data
	for( ge=0; ge<(int)nGen(); ge++ ) {
		bi = gen[ge].bi;
		if ( std::abs( V[bi] ) < 1e-3 ) {
			gen[ge].shutdown();
		} else {
			gen[ge].Pg *= scale_factors[bi];
			gen[ge].Qg *= scale_factors[bi];
			if ( types[bi]==REF ) {
				gen[ge].Pg += Mismatch[bi].real()*baseMVA/bus[bi].nGen();
			}
			if ( types[bi]==PV or types[bi]==REF ) {
				gen[ge].Qg += Mismatch[bi].imag()*baseMVA/bus[bi].nGen();
			}
		}
	}
	// update the load data
	for( i=0; i<(int)nLoad(); i++ ) {
		bi = load[i].bi;
		if ( std::abs( V[bi] ) < 1e-3 ) {
			load[i].Pd = 0.0;
			load[i].Qd = 0.0;
			load[i].status = OFF;
		} else {
			load[i].Pd *= scale_factors[bi];
			load[i].Qd *= scale_factors[bi];
		}
	}
	// all done
	return success;
}
