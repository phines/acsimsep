#ifndef SPARSE_VECTOR_H
#define SPARSE_VECTOR_H

#include "vector_base.h"
#include <map>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <complex>
#include <set>

#define VALUE_TYPE       typename std::pair< const unsigned, T >
#define DATA_TYPE        typename T
#define MAP_ITERATOR     typename std::map< unsigned, T >::iterator
//#define MAP_ITERATOR     typename std::pair< const unsigned, T >*

template< typename T >
bool find( std::map<unsigned,T>& data, unsigned index, MAP_ITERATOR &iter )
{
	iter = data.find(index);
	return (iter!=data.end());
}

template< typename T >
class sparse_vector_t : public vector_base_t
{
	public:
		/// default constructor initializes things to zero size
		sparse_vector_t() { resize(0); reset_next(); };
		/// constructor which creates a sparse vector of size length
		sparse_vector_t( unsigned length ) { resize(length); reset_next(); };
		/// nnz - returns the number of non zeros in the vector
		inline unsigned nnz() const { return data_.size(); };
		/// clear - empties out the data, and sets the vector size to zero
		inline void clear() { resize(0); data_.clear(); reset_next(); };
		/// data - returns a reference to the actual data element
		inline const std::map<unsigned,T> &data() const { return data_; };
		/// resize - resizes the vector
		void resize( unsigned length );
		/// zeros, sets the vector to be all zeros
		void zeros() { data_.clear(); reset_next(); };
		/// get - returns a constant reference to the element at index
		/// @param index - the element number to set
		/// @returns a constant reference to the element at index or a local zero element if index is zero
		inline const T& get( unsigned index ) const {
			if ( find( data_, index, iter_ ) )  return iter_->second;
			else return ZERO;
		};
		/** set - assigns the element at index to value
		 * @param index - the element number to set
		 * @param value - the value of the element at index
		 * @returns true if a new element is created
		 */
		inline bool set( const unsigned &index, const T &value ) {
			// the simple way:
			if ( nearly_zero(value) ) return false;
			unsigned old_size = data_.size();
			data_[index] = value;
			return old_size < data_.size();
			
			/* The complicated way that doesn't work:
			std::pair<MAP_ITERATOR,bool> iter_bool;
			err_if ( index>=length_, "Out of bounds error in sparse_vector get" );
			if ( nearly_zero(value) ) return false;
			else {
				iter_bool = data_.insert( VALUE_TYPE(index, value) ); // try to insert value at index
				if (iter_bool.second==true) return true;
				else {
					iter_bool.first->second=value;
					return false;
				}
			}
			*/
		}
		/// current - return a reference to the value at location iter_
		inline T& current() { return iter_->second; };
		/// reset_next - resets a pointer that keeps track of the current value in the array
		inline void reset_next() const { iter_=data_.begin(); }
		/** get_next - get the next non-zero value in the array
		 * @param index is the index to the next element (output value)
		 * @param value is the value of the element at index
		 * @returns false if there are no non-zero values remaining in the vector
		 */
		inline bool get_next( unsigned& index, T& value ) const {
			if ( data_.size()>0 && iter_!=data_.end() ) {
				index = iter_->first;
				value = iter_->second;
				iter_++;
				return true;
			}
			return false;
		}
		
		/// fill_rand - fills the vector with random elements
		inline void fill_rand( double portion, distribution_e dist=NORMAL ) {
			unsigned i, index, n;
			T value;
			if ( portion<=0 ) return;
			if ( portion > 1) portion=1;
			n = unsigned(portion*length_);
			for( i=0; i<n; i++ )
			{
				index = random()%length_;
				value = T( rand(dist) );
				set( index, value );
			}
		}
		/// subtract two sparse vectors (*this = a - b)
		inline void sub( sparse_vector_t<T> &a, sparse_vector_t<T> &b ) {
			T value;
			unsigned i;
			err_if(a.length()!=b.length(), "Cannot subtract sparse vectors with different sizes");
			copy(a);
			b.reset_next();
			while ( b.get_next(i,value) ) {
				data_[i] -= value;
			}
		}
		/// add two sparse vectors (*this = a + b)
		inline void add( sparse_vector_t<T> &a, sparse_vector_t<T> &b ) {
			T value;
			unsigned i;
			err_if(a.length()!=b.length(), "Cannot add sparse vectors with different sizes");
			copy(a);
			b.reset_next();
			while ( b.get_next(i,value) ) {
				data_[i] += value;
			}
		}
		/// max_abs returns the maximum of the unsigned elements
		inline double max_abs() {
			unsigned i;
			T value=0;
			double result=0;
			reset_next();
			while (get_next(i,value)) {
				result = max ( result, absolute(value) );
			}
			return result;
		}
		/// operator( index ) - returns a constant reference to an element with the given index
		const T& operator()( unsigned index ) const { err_if(index>=length_,"sparse_vector, out of bounds"); return get(index); };
		/// operator( index ) - returns a reference to an element with the given index
		T& operator()( unsigned index ) { err_if(index>=length_,"sparse_vector, out of bounds"); return data_[index]; };
		/// copy - copies the vector in other to the current one
		void copy( sparse_vector_t<T>& other ) {
			// clear the existing data and copy the other members
			clear();
			length_     = other.length();
			transposed_ = other.transposed();
			data_       = other.data();
		}
		/// clear_data - clears out the data vector, but not the size
		inline void clear_data() { data_.clear(); };
		/// print
		void print( const char* name ) const;
		/// make this vector into a subset of the larger input vector
		void subset( sparse_vector_t<T> & larger, std::set<int> & index_set );
	protected:
		//// data members:
		mutable  std::map< unsigned, T > data_; ///< the data map for the vector
		mutable  MAP_ITERATOR iter_ ; ///< an iterator for this vector. Used to find stuff within the vector
		static const T ZERO; ///< a zero that we can return by reference if needed
};

template < typename T >
const T sparse_vector_t<T>::ZERO = T(0);

template class sparse_vector_t<double>;
template class sparse_vector_t< std::complex<double> >;

#endif
