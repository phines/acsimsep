#ifndef BUS_H
#define BUS_H
////////////////////////////////////////////////////////////////////////////////
// Definition of bus_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <set>
#include "../myxml/myxml.hpp"
#include "PowerGlobals.hpp"
#include "PowerElement.hpp"
#include "branch.hpp"
#include "load.hpp"
#include "gen.hpp"

class bus_t : public power_element_t
{
	public: 
		/// number, ix, status, name, nextNo, defined in power_element_t
		double      Vmag;
		double      Vang;
		double      Vmax;
		double      Vmin;
		double      baseKV;
		unsigned    area;
		unsigned    zone;
		double      lamP;
		double      lamQ;
		double      mu_Vmax;
		double      mu_Vmin;
		bus_type_e  type;
		double      locX;
		double      locY;
		double      Pd;
		double      Qd;
		double      Gs;
		double      Bs;
		double      freq; ///< frequency (defaults to 60)
		double      T_measured; ///< gives the time that the bus state was measured
		bool        converged;  ///< set to true when the most current power flow converged 
		// pointers to the other data members
		gen_list_t*     p_gen;
		load_list_t*    p_load;
		branch_list_t*  p_branch;
		/// the element numbers of the elements connected to this bus
		std::set<int> load_set;
		std::set<int> gen_set;
		std::set<int> branch_set;
		std::set<int> neighbor_set;
		// member functions
		bus_t();
		~bus_t() {};
		/// return the number of locally connected generators
		inline unsigned nGen()      const { return gen_set.size();  }
		/// return the number of locally connected loads
		inline unsigned nLoad()     const { return load_set.size(); }
		/// return the number of locally connected branches
		inline unsigned nBranch()   const { return branch_set.size(); }
		/// return the number of neighbor buses
		inline unsigned nNeighbor() const { return neighbor_set.size(); }
		/// set the local voltage
		inline void set_V(std::complex<double> V) { Vmag = std::abs(V); Vang = std::arg(V)*180/PI; }
		bool operator==(const bus_t&) const;
		bool operator> (const bus_t&) const;
		bool operator< (const bus_t&) const;
		// type related
		bool is_pq()  { return type==PQ; }
		bool is_pv()  { return type==PV; }
		bool is_ref() { return type==REF; }
		// functions for reading and writing data:
		bool read          ( const std::string&, data_format_e format=POWER_XML );
		bool read          ( MyXmlNode & node );
		bool read_matpower ( const std::string& );
		bool read_xml      ( const std::string& );
		bool write         ( std::string&, data_format_e format=POWER_XML ) const;
		bool write_xml     ( std::string& ) const;
		bool write_matpower( std::string& ) const;
		bool write_state   ( std::string &data, double t=-1.0 );
		void print( std::string &output ) const; ///< print the data to a string
		/// print the data to standard out
		void print() { std::string str; print(str); printf("%s", str.c_str()); }
		void eraseIx(); /// erase the index variables
		void clear() { eraseIx(); bus_t(); }; /// clear all of the data
		/// return the complex bus voltage
		std::complex<double> V() {
			if ( is_unknown(Vang) ) return POWER_UNKNOWN;
			else return std::polar( Vmag, Vang*PI/180 );
		}
		/// set the bus voltage
		void set_voltage( std::complex<double> Vin );
		/// return a normalized value that tells us the severity of any votlage violations
		///  if Vnorm<1, not a violation, if Vnorm>1, violation
		double Vnorm() {
			double span = Vmax - Vmin;
			double center = Vmin + (span/2);
			double result = fabs(Vmag-center) / (span/2);
			return result;
		}
		/// return the phase angle in radians:
		double theta();
		/// add a reference to the gen in a_gen
		inline void add_ref(const gen_t & a_gen) { gen_set.insert( a_gen.number ); }
		/// add a reference to the load
		inline void add_ref(const load_t& a_load) { load_set.insert( a_load.number ); }
		/// add a reference to the branch
		inline void add_ref(const branch_t& a_branch) {
			branch_set.insert( a_branch.number );
			if( a_branch.fromBusNo != number ) neighbor_set.insert( a_branch.fromBusNo );
			if( a_branch.toBusNo != number )   neighbor_set.insert( a_branch.toBusNo );
		}
		/// return the i_th connected gen
		inline gen_t& gen(unsigned i) const {
			assert( i>=0 && i<gen_set.size() && p_gen!=NULL );
			std::set<int>::iterator iter=gen_set.begin();
			for( unsigned j=0; j<i; j++ ) iter++;
			return p_gen->operator()( *iter );
		}
		/// return the i_th connected load
		inline load_t& load( unsigned i ) const {
			assert( i>=0 && i<load_set.size() && p_load!=NULL );
			std::set<int>::iterator iter=load_set.begin();
			for( unsigned j=0; j<i; j++ ) iter++;
			return p_load->operator()( *iter );
		}
		/// return the i_th connected branch
		inline branch_t& branch( unsigned int i ) const {
			assert( i>=0 && i<branch_set.size() && p_branch!=NULL );
			std::set<int>::iterator iter=branch_set.begin();
			for( unsigned j=0; j<i; j++ ) iter++;
			return p_branch->operator()( *iter );
		}
		/// return the bus type in char form
		inline const char * type_char() {
			if (type==PV) return "PV";
			else if (type==PQ) return "PQ";
			else if (type==REF) return "REF";
			else error("unsupported bus type");
			return "unsupported bus type";
		}
};

typedef element_list_t<bus_t> bus_list_t;

#endif


