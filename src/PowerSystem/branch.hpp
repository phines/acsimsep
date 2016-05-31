#ifndef BRANCH_H
#define BRANCH_H
////////////////////////////////////////////////////////////////////////////////
// Definition of branch_t
//  by Paul Hines, June 2005
////////////////////////////////////////////////////////////////////////////////

#include "PowerElement.hpp"
#include "../myxml/myxml.hpp"

enum BranchRateNo_e { RATE_A=1, RATE_B=2, RATE_C=3 };

struct branch_t : public power_element_t {
	/// number, ix, status, name, defined in power_element_t
	int       fromBusNo;
	int       toBusNo;
	unsigned  fbi;      // from bus index
	unsigned  tbi;      // to bus index
	double    R;
	double    X;
	double    G;
	double    B;
	double    rateA;
	double    rateB;
	double    rateC;
	double    tap;
	double    shift;
	double    mu_f;
	double    mu_t;
	double    Tf_measured;
	double    Tt_measured;
	double    overload;       ///< a cumulative measure used by relays for switching operations
	double    overload_limit; ///< the limit for the above variable
	double    overload_time_constant; ///< a time constant used to calculate overload_limit
	static double baseMVA;
	status_e  status_f;
	status_e  status_t;
	double    Imag_f;  ///< the (measured) current magnitude on the from end in p.u.
	double    Imag_t;  ///< the (measured) current magnitude on the to end in p.u.
	std::complex<double> If;
	std::complex<double> It;
	// some functions that return manipulated forms of the data
	inline double Pf()    const { return Sf().real(); };
	inline double Pt()    const { return St().real(); };
	inline double Qf()    const { return Sf().imag(); };
	inline double Qt()    const { return St().imag(); };
	inline double If_re() const { return If.real(); };
	inline double If_im() const { return If.imag(); };
	inline double It_re() const { return It.real(); };
	inline double It_im() const { return It.imag(); };
	inline double Imag()  const { double a=Imag_f*status_f; double b=Imag_t*status_t; return (a>b)?a:b; };
	inline double SignedImag() const { double sign = (Pf()>0)? +1.0 : -1.0; return Imag()*sign; }
	inline double Inorm() const { return Imag()*baseMVA / rateB; }
	
	/// return the from end bus voltage
	std::complex<double> Vf() const { return std::polar( Vmag_f_, theta_ft_ ); }
	/// return the to end bus voltage
	std::complex<double> Vt() const { return Vmag_t_; }
	/// return the from end power transfer
	std::complex<double> Sf() const;
	/// return the to end power transfer
	std::complex<double> St() const;
	/// return the difference between the phase angles on the two ends in radian
	double delta()    const { return theta_ft(); }
	double theta_ft() const { return (double)status() * theta_ft_; }
	double Vmag_f()   const { return Vmag_f_; }
	double Vmag_t()   const { return Vmag_t_; }
	/// return the minimum delta, given the specified limit (rateB is assumed)
	/// @param delta_min (output) minimum
	/// @param delta_max (output) maximum
	void delta_min_max(double &delta_min, double& delta_max, BranchRateNo_e rate=RATE_B);
	/// get some derivatives of the current magnitude
	/// @param dImag_dVf (output) is the derivative wrt the from end voltage magnitude
	/// @param dImag_dVf (output) is the derivative wrt the to   end voltage magnitude
	/// @param dImag_dDelta (output) is the derivative wrt the diff. between the from and to ends
	void dImag_dx( double & dImag_dVf, double & dImag_dVt, double & dImag_dDelta );
	/// return the status of the branch
	inline status_e status() const { if ( status_f==OFF || status_t==OFF ) return OFF; return ON; };
	/// set the branch status
	inline void set_status( status_e stat_in ) { status_t=stat_in; status_f=stat_in; };
	/// set the voltage variables
	void set_voltages( double Vmag_f, double Vmag_t, double theta_ft, bool recalc_currents=false );
	/// set currents
	void set_currents( std::complex<double> I_from, std::complex<double> I_to ) {
		If = I_from; It = I_to;
		Imag_f = std::abs(If); Imag_t = std::abs(It);
	}
	/// calculate the currents given the branch voltages
	void calc_currents();
	/// set the local baseMVA value
	inline void set_baseMVA(double value) { baseMVA_=value; }
	inline double& rate(BranchRateNo_e rate) {
		switch (rate) {
			case RATE_A: return rateA;
			case RATE_B: return rateB;
			case RATE_C: return rateC;
		}
		return rateA;
	}
	/// update the simulated relay(s) on the branch
	/// @returns true when a discrete change occured
	bool update_relays( double dt );
	/// constructor
	branch_t();
	/// destructor
	~branch_t();
	/// print the data to a string
	void print( std::string &output ) const;
	/// print the data to stdout
	void print() { std::string str; print(str); printf("%s",str.c_str() ); }
	/// operators:
	bool operator==(const branch_t&) const;
	bool operator< (const branch_t&) const;
	bool operator> (const branch_t&) const;
	bool is_shift();
	bool is_tap();
	/// calculate the branch admittance values
	bool calc_admittances( std::complex<double>& y_ft, std::complex<double>& y_tf,
	                       std::complex<double>& y_ff, std::complex<double>& y_tt,
	                       std::complex<double>& y_s , std::complex<double>& tap,
	                       bool ignore_status=false );
	bool calc_admittances( std::complex<double>& y_ft, std::complex<double>& y_tf,
	                       std::complex<double>& y_ff, std::complex<double>& y_tt,
	                       bool ignore_status=false );
	bool read( const std::string &input,  data_format_e format=MATPOWER );
	/// read data from an xml node
	bool read( MyXmlNode & branch_node );
	/// write data to a string
	bool write( std::string &output, data_format_e format=POWER_XML ) const;
	private:
		bool read_xml     ( const std::string &input );
		bool read_matpower( const std::string &input );
		bool write_xml     ( std::string &output , bool reduced=false ) const;
		bool write_matpower( std::string &output ) const;
		static double baseMVA_;
		double Vmag_f_;
		double Vmag_t_;
		double theta_ft_;
		double Imag_prev_;
		
};

typedef element_list_t<branch_t> branch_list_t;

#endif


