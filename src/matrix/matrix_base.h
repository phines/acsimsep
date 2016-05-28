
#ifndef MATRIX_BASE_H
#define MATRIX_BASE_H

extern "C"
{
	#include <stdlib.h>
	#include <stdio.h>
	#include <time.h>
}
#include "matrixable.h"

/// class matrix_base_t
///  A base class for all matrix routines.
class matrix_base_t : public Matrixable
{
	public:
		/// constructor
		matrix_base_t() : rows_(0), cols_(0), transposed_(false) { };
		/// rows returns the number of rows in the matrix
		inline unsigned rows()   const { return rows_; };
		/// cols returns the number of columns in the matrix
		inline unsigned cols()   const { return cols_; };
		/// size returns the size of the matrix ( rows()*cols() )
		inline unsigned size()   const { return rows_*cols_; };
		/// transposed returns true if the matrix is transposed in its storage
		inline bool transposed() const { return transposed_; };
		/// transpose the matrix
		inline void transpose()  { transposed_=!transposed_; swap(rows_,cols_); };
	protected:
		// data members
		unsigned rows_;   ///< the number of rows in the matrix
		unsigned cols_;   ///< the number of columns in the matrix
		bool transposed_; ///< true if the matrix in storage represents the transposd matrix
};

#endif
