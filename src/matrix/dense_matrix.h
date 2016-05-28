#ifndef DENSE_MATRIX_H
#define DENSE_MATRIX_H

#include <complex>
#include <vector>
#include "matrix_base.h"
#include "dense_vector.h"

template < typename T >
class sparse_matrix_t;

template < typename T >
class dense_matrix_t : public matrix_base_t
{
	public:
		/// constructor
		dense_matrix_t() { ; }
		/// constructor - build a matrix of size rows x cols
		dense_matrix_t( unsigned rows, unsigned cols, T value=T(0.0) ) { resize(rows, cols, value); }
		/// destructor
		~dense_matrix_t() { clear(); }
		/// resize - resize the matrix
		void resize( unsigned rows, unsigned cols, T value=T(0.0) );
		/// clear - resize the matrix to zero size
		void clear() { rows_=0; cols_=0; transposed_=false; }
		/// print - print the matrix with name
		void print(const char* name) const;
		/// print - print the matrix with name = Dense
		void print() const { print("Dense"); }
		/// get - returns an element of the matrix
		inline T get(unsigned row, unsigned col) const { return data_[ index(row,col) ]; }
		/// set - assign the element at (row, col) to value
		inline void set(unsigned row, unsigned col, T value) { data_[ index(row,col) ]=value; }
		/// set(row,col,value) - set the entire matrix to a scalar value
		inline void set(T value=0) { for (unsigned i=0;i<size();i++) data_[i]=value; }
		/// set(row,value) - set an entire row to value
		inline void set_row(unsigned row, T value) { for (unsigned col=0;col<cols_;col++) set(row,col,value); }
		/// set_row(row, vec) - set an entire row to be equal to the input vector
		inline void set_row(unsigned row, const dense_vector_t<T>& vec) {
			for (unsigned col=0;col<cols_;col++) set( row,col,vec[col] );
		}
		/// set_col(col, value) - set an entire column to be equal to a scalar value
		inline void set_col(unsigned col, T value) {
			for (unsigned row=0;row<rows_;row++) set(row,col,value);
		}
		/// set_col(col, vector) - set an entire column equal to the input vector
		inline void set_col(unsigned col, const dense_vector_t<T>& vec) {
			for (unsigned row=0;row<rows_;row++) set( row,col,vec[row] );
		}
		/// fill_rand - fill the vector with random values
		void fill_rand(distribution_e dist=NORMAL) { for (unsigned i=0;i<size();i++) data_[i]=rand(dist); };
		/// data - returns a pointer to the data
		T* data()  { return &data_[0]; }
		/// begin - returns a pointer to the data
		T* begin() { return &data_[0]; }
		/// solve - solves the set of linear systems given by A and B ( in Matlab *this = A\B )
		bool solve ( sparse_matrix_t<T>& A, sparse_matrix_t<T>& B );
		/// divide - solves a set of linear systems to obtain the matrix division of A and B ( *this = A/B )
		bool divide ( sparse_matrix_t<T>& A, sparse_matrix_t<T>& B );
		/// mult(A,B) - multiplies A and B
		bool mult  ( dense_matrix_t<T>& A,  sparse_matrix_t<T>& B );
		/// invert - calculates the inverse of A and puts the result in *this
		bool invert( sparse_matrix_t<T> & A );
		/// operator(row,col) - returns a constant reference to the element at (row, col)
		inline const T& operator()(unsigned row, unsigned col) const { return data_[ index(row,col) ]; }
		/// operator(row,col) - returns a reference to the element at (row, col)
		inline       T& operator()(unsigned row, unsigned col)       { return data_[ index(row,col) ]; }
		/// copy - copies other to *this
		inline void copy( dense_matrix_t<T> &other ) {
			resize( other.rows(), other.cols() );
			transposed_=other.transposed();
			memcpy( data_, other.begin(), size()*sizeof(double) );
		}
		/// copy_to_row - copies a vector to a row of the matrix
		void copy_to_row ( dense_vector_t<T> &vec, unsigned row );
		/// copy_to_col - copies a vector to a column of the matrix
		void copy_to_col ( dense_vector_t<T> &vec, unsigned col );
	private:
		//// function members
		/// return the index into data_ given by (row, col)
		inline unsigned index(unsigned row, unsigned col) const {
			if ( row>=rows_ || col>=cols_ ) {
				printf("Attempt to access row %d and col %d in a dense matrix with %d rows and %d cols.\n", row, col, rows_, cols_);
				error("Dense matrix index out of bounds.");
			}
			return ( transposed_ ? (col+row*cols_) : (row+col*rows_) );
		}
		//// data members
		std::vector<T> data_; ///< the actual block of data
};

#endif

