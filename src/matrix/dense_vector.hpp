#ifndef DENSE_VECTOR_H
#define DENSE_VECTOR_H

#include <complex>
#include <vector>
#include "vector_base.hpp"
//#include "dense_matrix.h"
//#include "sparse_matrix.h"

template < typename T >
class dense_matrix_t;
template < typename T >
class sparse_matrix_t;

// I don't think we need to have more than 1e8 elements
//  for such tasks one would be better off using a different tool
#define MAX_DENSE_ELEMENTS 1e8

/** 
 * class dense_vector_t
 *  A simple class for manipulating dense vectors of double or complex double values
 */
template < typename T >
class dense_vector_t : public vector_base_t
{
	public:
		/// default constructor
		dense_vector_t() { ; }
		/// standard constructor
		///  initializes the matrix to length with the transposed sense trans
		dense_vector_t( unsigned length, T value=T(0.0) ) { resize(length,value); }
		/// destructor
		~dense_vector_t() { length_=0; transposed_=false; }
		/// clear - resizes the vector to zero length
		inline void clear() { length_=0; transposed_=false; }
		/// resize -- resize the vector and reallocate the memory if needed
		///  if the new vector is larger, the new data is filled with new_value
		void resize( unsigned length, T new_value=T(0.0) );
		/// set - set the entire vector to value
		inline void set( T value ) { for(unsigned i=0; i<length_; i++) data_[i]=value; };
		/// zeros - set the entire vector to zero
		inline void zeros() { set(0.0); }
		/// fill_rand - fills the entire vector with random values
		inline void fill_rand(distribution_e dist=UNIFORM) {
			for( unsigned i=0; i<length_; i++ ) data_[i] = T( rand( dist ) );
		}
		/// norm - returns the L2 norm of the vector
		double norm(); // see vector.cc
		/// solve - solves the linear system given by A and b
		///  and puts the result in *this ( A*(*this) = b, or *this = A\b )
		bool solve( sparse_matrix_t<T> &A, dense_vector_t<T> &b );
		/// operator[i] - returns a reference to the i-th element of the vector, without error checking
		inline T& operator[](unsigned i) { return data_[i]; }
		/// operator[i] - returns a constant reference to the i-th element of the vector, without error checking
		inline const T& operator[](unsigned i) const { return data_[i]; }
		/// operator(i) - returns a reference to the i-th element of the vector, with error checking
		inline T& operator()(unsigned i) { err_if( i>=length_,"Dense Vector out of bounds error" ); return data_[i]; }
		/// operator(i) - returns a constant reference to the i-th element of the vector, with error checking
		inline const T& operator()(unsigned i) const {
			err_if( i>=length_,"Dense vector out of bounds error" );
			return data_[i];
		}
		/// print - prints the vector in a matlab readable format
		/// @param name - the name that will be used when printing the matrix
		void print(const char* name) const;
		/// print - prints the vector in a matlab readable format
		inline void print() const { print("Vector"); }
		/// begin - returns a pointer to the actual data
		inline T* begin() { return &data_[0]; }
		/// data - returns a pointer to the actual data
		inline T* data() { return &data_[0]; }
		/// data - returns a constant pointer to the actual data
		inline const T* data() const { return &data_[0]; }
		// math functions -> return false if bad dimensions are given for the inputs
		/// mult_elements - multiplies the elements of a and b, puts the result in *this (*this = a*b)
		///  @return true if success, false otherwise (bad dimentions in a and b for example)
		bool mult_elements( dense_vector_t<T> &a,  dense_vector_t<T> &b );
		/// add - adds the elements of a and b, puts the result in *this (*this = a+b)
		///  @return true if success, false otherwise (bad dimentions in a and b for example)
		bool add( dense_vector_t<T> &a,  dense_vector_t<T> &b );
		/// sub - subtracts the elements of a and b, puts the result in *this (*this = a-b)
		///  @return true if success, false otherwise (bad dimentions in a and b for example)
		bool sub( dense_vector_t<T> &a,  dense_vector_t<T> &b );
		/// mult - multiplies the sparse matrix A by the dense vector b, and puts the result in *this (*this = A*b)
		///  @return true if success, false otherwise (bad dimentions in a and b for example)
		bool mult( sparse_matrix_t<T> &A, dense_vector_t<T> &b );
		/// scale - scales the vector by the input scalar value
		void scale( T scalar ) { for( unsigned i=0; i<length_; i++ ) data_[i]*=scalar; };
		//// a menu of copy routines:
		/// copy - makes *this equal to other
		void copy( const dense_vector_t<T>& other ) {
			resize( other.length(), other.transposed() );
			memcpy( &data_[0], other.data(), length_*sizeof(T) );
		}
		void copy_col( sparse_matrix_t<T> &other, unsigned col ); ///< copy a column of an input matrix
		void copy_col(  dense_matrix_t<T> &other, unsigned col ); ///< copy a column of an input matrix
		void copy_row( sparse_matrix_t<T> &other, unsigned row ); ///< copy a row of an input matrix
		void copy_row(  dense_matrix_t<T> &other, unsigned row ); ///< copy a row of an input matrix
		double max_abs();
	private:
		std::vector<T> data_;  ///< storage for the actual data
};

template class dense_vector_t<        double        >;
template class dense_vector_t< std::complex<double> >;
//template class dense_vector_t<       unsigned       >;
#endif

