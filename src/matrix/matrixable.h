
#ifndef MATRIXABLE_H
#define MATRIXABLE_H

extern "C"
{
	#include <stdlib.h>
	#include <stdio.h>
	#include <time.h>
}
#include "../utilities/utilities.h"
#include <complex>
#include <stdexcept>
#include <string>
#define MATRIX_EPS   1e-16


class MatrixException : public std::exception
{
	public:
		MatrixException(const char *msg) { msg_.append(msg); }
		virtual ~MatrixException() throw() {};
		virtual const char* what() { return msg_.c_str(); }
	private:
		std::string msg_;
};

/// class Matrixable
///  A class with a few simple routines, used as a base for other classes in the matrix project
class Matrixable
{
	public:
		/// constrctor
		Matrixable() { srandom( time(NULL) ); }
		/// error - prints an error message and exits the program
		/// @param msg - the message to be printed
		inline void error(const char * msg) const { throw std::runtime_error(msg); }
		/// err_if - prints an error message and exits the program if condition is true
		/// @param condition - the error condition (error is called if condition==true)
		/// @param msg - the message to print if contition is true 
		inline void err_if(bool condition, const char* msg="unspecified error") const { if(condition) error(msg); }
		/// swap - swaps the values of a and b
		inline void swap(unsigned &a, unsigned &b) const { unsigned temp_a=a; a=b; b=temp_a; }
		/// min - returns the minimum of a and b
		template <typename U>
		inline U min (U a, U b) { return ( a<b ? a : b ); }
		/// max - returns the maximum of a and b
		template <typename U>
		inline U max (U a, U b) { return ( a>b ? a : b ); }
		/// rand - returns a random number with the specified distribution
		inline double rand( distribution_e dist=UNIFORM ) const { return random(dist); }
		/// print_value - prints a double value to stdout
		void print_value( const double &value ) const { printf(" %15.10g ", value); }
		/// print_value - prints a complex double value to stdout
		void print_value( const std::complex<double> &value ) const { printf(" %15.10g+%15.10gj ", value.real(), value.imag() ); }
		/// absolute
		double absolute( const double &value ) const { return fabs(value); }
		double absolute( const std::complex<double> &value ) const { return abs(value); }
		inline bool nearly_zero( double value )               { return ( fabs(value) < MATRIX_EPS ); };
		inline bool nearly_zero( std::complex<double> value ) { return (  abs(value) < MATRIX_EPS ); };
	protected:
};

#endif
