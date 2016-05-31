////////////////////////////////////////////////////////////////////////////////
// Declaration of gen_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#ifndef GEN_H
#define GEN_H

#include "PowerElement.hpp"
#include "../myxml/myxml.hpp"

typedef class bus_t;
typedef element_list_t<bus_t> bus_list_t;

/// machine_t
///  A struc to hold machine dynamic data for a generator
class machine_t
{
public:
	// data members
	double dP_dT_max;
	double dP_dT_min;
	double baseMVA;
	double H; ///< the machine's per unit inertia constant (see Bergen, 1986, p. 263)
	double D; ///< the machine's per unit damping constant (see Bergen, 1986, p. 263)
	
	/// default values:
	machine_t() { dP_dT_max=10; dP_dT_min=-10; baseMVA=100; H=0; D=0; };
};

/// gen_cost_t
///  A struct to hold generator cost data
struct gen_cost_t {
public:
	// data members
	double ramp_up;
	double ramp_down;
	std::vector<double> polynomial;
	double& a() { return polynomial[0]; } ///< a is the fixed cost when the generator is on
	double& b() { return polynomial[1]; } ///< b is the linear cost ($/MW)
	double& c() { return polynomial[2]; } ///< c is the quadratic costs ($/MW^2)
	double& d() { return polynomial[3]; } ///< d is the cubic costs
	/// default values:
	gen_cost_t() { ramp_up=100; ramp_down=30; polynomial.resize(4,0); b()=30; };
};

class gen_t : public power_element_t {
	public:
		/// number, ix, status, name, defined in power_element_t
		int    busNo;
		unsigned  bi;
		double    Pg; ///< the amount of real electrical power injected at busNo
		double    Qg; ///< the amount of reactive electrical power injected at busNo
		double    Pmax; ///< maximum real power output at busNo
		double    Pmin; ///< minimum real power output at busNo
		double    Qmax; ///< maximum reactive power output at busNo
		double    Qmin; ///< minimum reactive power output at busNo
		double    Vref; ///< the reference voltage for the Q controller
		double    Pref; ///< the reference power output for the P controller
		double    mBase;
		double    mu_Pmax;
		double    mu_Pmin;
		double    mu_Qmax;
		double    mu_Qmin;
		double    dP_dT_max; // ramp rate in MW / sec
		double    dP_dT_min; // ramp rate in MW / sec
		double    T_measured; ///< time that the state was measured
		machine_t  mac;
		gen_cost_t cost;
	
		// constructor/destructor
		gen_t();  ///< build a clean generator
		~gen_t(); ///< destroy the generator structure
	
		// read/write routines
		/// read data in the specified format
		bool read( const std::string& input,  data_format_e format=MATPOWER );
		/// write data to the specified format
		bool write( std::string& output, data_format_e format=POWER_XML ) const;
		/// print formated information about the generator
		void print( std::string &output ) const;
		/// print to stdout
		void print() { std::string str; print(str); printf("%s", str.c_str()); }
		/// read xml data
		bool read( MyXmlNode & gen_node );
	
		// return formated data
		inline std::complex<double> Sg()   { return std::complex<double>(Pg, Qg)*(double)status; }
		inline std::complex<double> Smax() { return std::complex<double>(Pmax,Qmax); }
		inline std::complex<double> Smin() { return std::complex<double>(Pmin,Qmin); }
		inline double& RDC() { return cost.ramp_down; }
		inline double& RUC() { return cost.ramp_up;   }
		inline double& RRU() { return mac.dP_dT_max;  }
		inline double& RRD() { return mac.dP_dT_min;  }
		// general functions
		/// change the generation (Pg) by the specified amount (Pg+=delta)
		void change(double delta);
		void change_Pg(double delta) { change(delta); }
		/// change the generator voltage
		void change_Vg(double delta);
		/// set the real/reactive generation
		void set_G(std::complex<double> G) { Pg=G.real(); Qg=G.imag(); }
		/// shut down the generator
		void shutdown() { Pg=0; Qg=0; status=OFF; }
	
		// comparison
		bool operator==(const gen_t&) const;
		bool operator< (const gen_t&) const;
		bool operator> (const gen_t&) const;
	private:
		//  other read/write
		bool read_xml( const std::string& input );
		bool read_matpower( const std::string& input );
		bool write_xml( std::string& output ) const;
		bool write_matpower( std::string& output ) const;
};

typedef element_list_t<gen_t> gen_list_t;

#endif
