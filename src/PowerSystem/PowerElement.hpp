#ifndef ELEMENT_H
#define ELEMENT_H

#include <vector>
#include <string>
#include <complex>
#include <cstring>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "PowerGlobals.hpp"
#include "../regex/regex.hpp"

#define NAME_SIZE           100
#define DEFAULT_INDEX_SIZE  1000
#define POWER_EMPTY         9999999
#define POWER_UNKNOWN       9999999
#define MAX_ELEMENT_NUMBER  POWER_EMPTY-1
#define POWER_ELEMENT_EPS   1e-3


/// check to see if the given value is equal to the unknown value
bool is_unknown( double value );
/// check for a complex value equal to unknown
bool is_unknown( std::complex<double> value );

/// data file format variable
enum data_format_e { MATPOWER=0, PSSE=1, IEEE_CDF=2, POWER_XML=3, REDUCED=4, PARTIAL=4 };

/// device status
enum status_e {ON=1, OFF=0};

class power_element_t {
	public:
		/// the index variable, common to all elements
		unsigned ix() const { return ix_; }
		void set_ix( unsigned i ) { ix_=i; }
		/// the number variable, also common to all elements
		int number;
		/// the next number 
		static int nextNo;
		/// the status variable
		status_e status;
		/// the element name
		char name[NAME_SIZE];
		/// a flag that indicates whether this element is in a local model
		bool is_local;
		/// default constructor
		power_element_t() {
			is_local=true;
			number=POWER_EMPTY;
			status=ON;
			strcpy(name,"");
		}
		/// copy constructor
		power_element_t(const power_element_t& other) {
			ix_      = other.ix();
			number   = other.number;
			status   = other.status;
			std::strncpy(name, other.name, NAME_SIZE);
			is_local = other.is_local;
		}
		/// copy constructor
		power_element_t& operator=(const power_element_t& other) {
			number = other.number;
			status = other.status;
			std::strncpy(name, other.name, NAME_SIZE);
			is_local = other.is_local;
			return *this;
		}
		/// err_if function
		void err_if(bool condition, const char *msg) const { if(condition) error(msg); }
		/// error function
		void error(const char *msg) const {
			printf( "PowerElement Error: %s", msg );
			throw msg;
		}
	private:
		unsigned ix_;
};

template < typename T >
class element_list_t
{
	public:
		/// constructor
		element_list_t() : index_(DEFAULT_INDEX_SIZE,-1), element_vec_(0), largest_number_(0) {};
		/// return an element reference by index:
		inline T& operator[]( unsigned int i ) {
			assert( i<POWER_EMPTY );
			assert( i<element_vec_.size() );
			return element_vec_[i];
		}
		/// return an element reference by index:
		inline const T& operator[]( unsigned int i ) const {
			assert( i<POWER_EMPTY );
			assert( i<element_vec_.size() );
			return element_vec_[i];
		}
		/// return an element reference by number:
		inline T& operator()( unsigned num ) { 
			assert( exist(num) );
			return element_vec_[ index_[ num ] ];
		}		
		/// return the index of a given element number
		inline int index( unsigned num ) const {
			if ( num>=index_.size() ) return -1;
			else return index_[num];
		}			
		/// insert an element into the list
		inline bool insert( const T& a_element ) {
			int number = a_element.number;
			if ( number>MAX_ELEMENT_NUMBER || number<0 )
				return false;
			if ( exist( number ) ) {
				element_vec_[ index_[number] ] = a_element;
			} else {
				element_vec_.push_back(a_element);
				if ( index_.size() <= (unsigned) number ) {
					index_.resize( (unsigned) number*2, -1 );
				}
				if ( index_.size() >= MAX_ELEMENT_NUMBER) {
					printf(" Number = %d\n", number );
					assert(0);
				}
				assert( (unsigned) number < index_.size() );
				assert( number >= 0 );
				index_[ number ] = last();
				// record the largest number
				if (number>largest_number_) largest_number_=number;
			}
			element_vec_[ index_[number] ].set_ix( index(a_element.number) );
			return true;
		}
		/// begin, returns a pointer to the begining of the list
		typename std::vector<T>::iterator begin() { return element_vec_.begin(); };
		/// end, returns a pointer to one past the end of the list
		typename std::vector<T>::iterator end()   { return element_vec_.end(); };
		/// push_back
		inline bool push_back ( const power_element_t& a_element ) { return insert(a_element); };		
		/// size
		inline int size() const { return element_vec_.size(); };
		/// next
		inline int next() const { return element_vec_.size(); };
		/// last
		inline int last() const { return element_vec_.size()-1; };
		/// exist: tells us if the element number exists or not
		inline bool exist( unsigned num ) const {
			if ( num<0 || num>=index_.size() ) return false;
			else return (index_[num]>=0);
		}
		/// clear:
		inline void clear() {
			index_.clear();
			index_.resize(DEFAULT_INDEX_SIZE, -1);
			element_vec_.clear();
			largest_number_=0;
		}
		inline int largest_number() { return largest_number_; }
	private:
		/// the index of references by number
		std::vector<int> index_;
		/// the vector of elements
		std::vector<T>   element_vec_;
		/// the largest number in the list
		int largest_number_;
};

std::complex<double> read_complex ( const std::string &data );
#endif


