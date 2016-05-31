#ifndef SPARSE_MATRIX_H
#define SPARSE_MATRIX_H

#include "matrix_base.hpp"
#include "sparse_vector.hpp"
#include "dense_vector.hpp"
#include <set>
#include <vector>
#include <complex>
#include <iostream>

/// enum solve_strategy_e is used to help the solver choose a matrix solution strategy
enum solve_strategy_e { AUTO, UNSYMMETRIC, SYMMETRIC, PERMUTATION };

template< typename T >
class sparse_matrix_t : public matrix_base_t
{
	public:
		/// constructor 0
		sparse_matrix_t() : nnz_(0), strategy_(AUTO), data_size_(0), Numeric_(NULL), Symbolic_(NULL)
			{ resize(0,0); iter_=data_.begin(); };
		/// constructor 1
		sparse_matrix_t( unsigned n_rows, unsigned n_cols ) : nnz_(0), strategy_(AUTO), data_size_(0)
			{ resize( n_rows, n_cols); };
		/// destructor
		~sparse_matrix_t() {
			// free the UMFPACK data
			//if (Numeric!=NULL)  umfpack_di_free_numeric(&Numeric);
			//if (Symbolic!=NULL) umfpack_di_free_symbolic(&Symbolic);
		}
		/// solve
		bool solve(dense_vector_t < T > &b, dense_vector_t < T > &x );
		/// nnz - returns the number of non-zeros
		inline unsigned nnz()              const { return nnz_; };
		/// strategy - returns the recommended solver strategy for this matrix
		inline solve_strategy_e strategy() const { return strategy_; };
		/// set_strategy - assigns the internal solver strategy to the input
		inline void set_strategy( solve_strategy_e strategy ) { strategy_=strategy; };
		/// delete all the data in the matrix
		inline void clear() { nnz_=0; resize(0,0); for(unsigned i=0;i<data_size_;i++) data_.clear(); };
		/// resize the matrix
		void resize(unsigned rows, unsigned cols);
		/// reset the next pointer
		inline void reset_next() { iter_=data_.begin(); if (data_.size()>0) iter_->reset_next(); };
		/// get the next element in the matrix
		bool get_next(unsigned& row, unsigned& col, T& value);
		/// set an element to a value
		inline void set( unsigned row, unsigned col, const T& value ) {
			err_if( row>=rows_||col>=cols_, "index out of bounds in sparse set" );
			if (transposed_) swap(row,col);
			if ( data_[col].set(row, value) ) nnz_++;
		};
		/// get a reference to the element at row, col
		inline const T& get( unsigned row, unsigned col ) const {
			err_if( row>=rows_ || col>=cols_ , "index out of bounds in sparse get" );
			if (transposed_) swap(row,col);
			return data_[col].get( row );
		};
		/// operator[i] - return a reference to a row/col of the matrix (depending on transposed_)
		inline sparse_vector_t<T>& operator[](unsigned i) { return data_[i]; }
		/// return a reference to the specified column number
		inline sparse_vector_t<T>& get_col( unsigned i ) {
			err_if( i>=cols() || transposed_, "invalid call to get_col"); return data_[i];
		}
		/// operator[i] - return a const reference to a row/col of the matrix (depending on transposed_)
		inline const sparse_vector_t<T>& operator[](unsigned i) const { return data_[i]; };
		/// operator(row,col) - return a reference to the element at row, col.
		///  NOTE: use only for write access as it can introduce zero elements into the sparse vector.
		inline T& operator() ( unsigned row, unsigned col ) {
			// the way I am doing this is not very efficient
			err_if ( row>=rows() || col>=cols() , "Out of bounds in sparse_matrix<T>::operator()" );
			if (transposed_) swap(row,col);
			unsigned old_nnz = data_[col].nnz();
			T& result = data_[col](row);
			if( data_[col].nnz() > old_nnz ) nnz_++;
			return result;
		}
		/// n_vectors - returns the number of vectors stored in the data_ structure
		inline unsigned n_vectors() { return (transposed_? rows_: cols_ ); }
		/// zeros - fils the matrix with zeros (empties out all of the internal vectors
		inline void zeros() { if (nnz_>0) { for (unsigned i=0; i<data_size_; i++ ) data_[i].zeros(); nnz_=0; } }
		/// fill_rand - fill the matrix with random elements
		inline void fill_rand( double density ) {
			unsigned row, col, n_nz, i;
			T value;
			zeros();
			n_nz = unsigned( density*size() );
			for( i=0; i<n_nz; i++ ) {
				value = T( rand( UNIFORM ) );
				row = r_.randi(0,rows_);
				col = r_.randi(0,cols_);
				set(row,col,value);
			}
		}
		/// print - prints the matrix in Matlab format, with name
		void print( const char* name ); // implementation in sparse_matrix.cc
		/// print - prints the matrix with a default name
		void print() { print("Matrix"); }
		/// make_csc - return a copy of the matrix in compressed sparse column storage format
		void make_csc ( int*& index1, int*& index2, T*& values, bool &trans );
		/// free_csc - free the memory allocated by make_csc
		void free_csc ( int*& index1, int*& index2, T*& values );
		/// make_csr - return a copy of the matrix in compressed sparse row storage format
		void make_csr ( int*& index1, int*& index2, T*& values, bool &trans );
		/// free_csr - free the memory allocated by make_csr
		void free_csr ( int*& index1, int*& index2, T*& values );
		/// make_triplet - return a copy of the matrix in triplet format
		void make_triplet ( int*& row_index, int*& col_index, T*& values );
		/// free_triplet - free the memory allocated by make_triplet
		void free_triplet ( int*& row_index, int*& col_index, T*& values );
		/// copy - copy another matrix
		void copy( const sparse_matrix_t<T> & other ) {
			unsigned i;
			clear();
			transposed_ = other.transposed();
			resize( other.rows(), other.cols() );
			nnz_  = other.nnz();
			for( i=0; i<data_.size(); i++ ) data_[i]=other[i];
		};
		/// copy_real - copy just the real part of another matrix
		void copy_real( sparse_matrix_t< std::complex<double> > &other ) {
			unsigned row, col;
			std::complex<double> value;
			clear();
			transposed_ = other.transposed();
			resize( other.rows(), other.cols() );
			other.reset_next();
			while ( other.get_next ( row, col, value ) ) {
				set( row, col, value.real() );
			}
		};
		/// copy_imag - copy just the imaginary part of another matrix
		void copy_imag( sparse_matrix_t< std::complex<double> > & other ) {
			unsigned row, col;
			std::complex<double> value;
			clear();
			transposed_ = other.transposed();
			resize( other.rows(), other.cols() );
			other.reset_next();
			while ( other.get_next ( row, col, value ) )
			{
				set( row, col, value.imag() );
			}
		};
		/// clear data
		inline void clear_data() { for( unsigned i=0; i<data_size_; i++) data_[i].clear_data(); nnz_=0; };
		/// eye - makes the matrix into a scaled identity matrix with value on the diagonal
		inline void eye(double value) {
			clear_data();
			for (unsigned i=0; i<min(rows_,cols_); i++) set(i,i,value);
		}
		/// eye - makes the matrix into an identity matrix
		inline void eye() { eye(1.0); }
		/// subset makes this matrix into a subset of the @param larger matrix given the row and col sets
		void subset( sparse_matrix_t<T> & larger, std::set<int> & row_set, std::set<int> & col_set );
		/// Set the Numeric solver data
		void set_numeric(void * Num_in) { Numeric_=Num_in; }
		/// Set the Symbolic solver data
		void set_symbolic(void * Sym_in) {  Symbolic_=Sym_in; }
		/// Get the symbolic/numeric data
		void * get_symbolic() { return  Symbolic_; }
		void * get_numeric() { return  Numeric_; }
	private:
		//// member data:
		unsigned nnz_;  ///< the number of non-zero elements
		solve_strategy_e strategy_; ///< a hint to the solver indicating which strategy to use when solving linear systems
		std::vector< sparse_vector_t<T> > data_;  ///< a vector of sparse vectors in compressed column format
		unsigned data_size_; ///< the size of the data vector
		mutable typename std::vector< sparse_vector_t<T> >::iterator iter_; ///< a local value that indicates which element of data_ is next in line
		std::vector<int> col_starts_; ///< used for csc/csr form of the matrix
		std::vector<int> row_index_; ///< used for triplet form of the matrix
		std::vector<int> col_index_; ///< used for triplet and csc/csv form of the matrix
		std::vector<T> values_;    ///< used for triplet and csc/csr form of the matrix
		void * Symbolic_; ///< used to store the symbolic solve data used by the solver.
		void * Numeric_; ///< used to store the numeric solve data
};

template class sparse_matrix_t< double >;
template class sparse_matrix_t< std::complex<double> >;

#endif
