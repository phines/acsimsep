////////////////////////////////////////////////////////////////////////////////
// Declaration of load_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#ifndef LOAD_H
#define LOAD_H

#include "../myxml/myxml.hpp"
#include "PowerElement.hpp"
typedef class bus_t;
typedef element_list_t<bus_t> bus_list_t;

class load_t : public power_element_t
{
	public: 
		/// number, ix, status, name, defined in power_element_t
		int       busNo;
		int       bi;
		double    Pd;
		double    Qd;
		double    Pmax; // this is the maximum demand under normal conditions
		double    Qmax; // reactive portion of above
		double    Gs;
		double    Bs;
		double    Ire;
		double    Iim;
		double    value;
		double    T_measured;
	
		// member functions
		load_t();
		~load_t();
		bool operator==(const load_t&) const;
		bool operator< (const load_t&) const;
		bool operator> (const load_t&) const;
		/// read data from a string
		bool read( const std::string&, data_format_e format=POWER_XML );
		/// write data to a string
		bool write( std::string&, data_format_e format=POWER_XML ) const;
		/// read data from an XML node
		bool read( MyXmlNode & node );
		/// print the data to a string
		void print( std::string &output ) const;
		/// print the data to stdout
		void print() { std::string str; print(str); printf("%s",str.c_str()); }
		/// return the complex demand
		inline std::complex<double> Sd() { return (std::complex<double>(Pd, Qd)*(double)status); };
		/// return the ratio Qd/Pd
		inline double QP_fraction() { return std::abs(Pd)>0 ? Qd/Pd : 0 ; };
		/// change the load (Q and P) by the specified delta (Q gets changed proportionally)
		void change(double delta);
	// private members
	private:
		bool read_xml ( const std::string& );
		bool read_matpower ( const std::string& );
		bool write_xml ( std::string& ) const;
		bool write_matpower ( std::string& ) const;
};

typedef element_list_t<load_t> load_list_t;

#endif

