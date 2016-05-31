#include "dense_matrix.hpp"
#include "sparse_matrix.hpp"
#include "matrix_solve.hpp"

typedef std::complex<double> cx;

/// resize the matrix
template <typename T>
void dense_matrix_t<T>::resize( unsigned rows, unsigned cols, T value )
{
	unsigned old_size = data_.size();
	unsigned new_size = max (rows*cols+10, old_size );
	
	rows_=rows;
	cols_=cols;
	if ( new_size > old_size ) {
		data_.resize(new_size,value);
	}
}
template void dense_matrix_t< int  >::resize( unsigned , unsigned , int  );
template void dense_matrix_t<double>::resize( unsigned , unsigned , double  );
template void dense_matrix_t<  cx  >::resize( unsigned , unsigned , std::complex<double>  );

/// print the matrix
template <typename T>
void dense_matrix_t<T>::print( const char* name ) const
{
	unsigned row, col;
	
	printf("\n%s = [ ... ", name );
	for (row=0; row<rows_; row++ )
	{
		printf("\n");
		for (col=0; col<cols_; col++ )
		{
			print_value( get(row, col) );
			printf(" ");
		}
		printf(";");
	}
	printf("\n];\n");
}
template void dense_matrix_t<double>::print(const char* name) const;
template void dense_matrix_t<  cx  >::print(const char* name) const;

/// solve a set of systems of matrix equations: A*X = B, where X = *this
template <typename T>
bool dense_matrix_t<T>::solve ( sparse_matrix_t<T>& A, sparse_matrix_t<T>& B )
{
	bool result=false;
	// if A is square
	if (A.rows()==A.cols())
	{
		result = sparse_solve(A, *this, B);
	}
	else error("In dense_matrix_t<T>::solve (A, B) , A must be square. ");
	
	return result;
}
template bool dense_matrix_t<double>::solve ( sparse_matrix_t<double>& A, sparse_matrix_t<double>& B );
template bool dense_matrix_t<  cx  >::solve ( sparse_matrix_t<  cx  >& A, sparse_matrix_t<  cx  >& B );

/// divide A by B (X = A/B), equivalent to X' = B'\A'
template <typename T>
bool dense_matrix_t<T>::divide ( sparse_matrix_t<T>& A, sparse_matrix_t<T>& B )
{
	bool result;
	
	err_if (B.rows() != B.cols(), "In dense_matrix_t<T>::divide(A,B), B must be square");
	err_if (A.cols() != B.rows(), "In dense_matrix_t<T>::divide(A,B), A must have the same number of columns as B has rows");
	
	//if (!transposed()) transpose();
	// transpose  A and B
	A.transpose();
	B.transpose();
	// solve for X = B'\A', which is equivalent to X = A/B
	result = this->solve( B, A );
	
	// return A and B to their original state
	A.transpose();
	B.transpose();
	this->transpose();
	return result;
}
template bool dense_matrix_t<double>::divide ( sparse_matrix_t<double>& A, sparse_matrix_t<double>& B );
template bool dense_matrix_t<  cx  >::divide ( sparse_matrix_t<  cx  >& A, sparse_matrix_t<  cx  >& B );

// calculate the inverse via *this = I / A
template <typename T>
bool dense_matrix_t<T>::invert( sparse_matrix_t<T> & A )
{
	err_if (A.rows()!=A.cols(), "Cannot invert a non-square matrix.");
	// vars
	sparse_matrix_t<T> I(A.rows(),A.rows());
	I.eye();
	
	// returen *this = I/A
	return divide(I,A);
}
template bool dense_matrix_t<double>::invert ( sparse_matrix_t<double> & A );
template bool dense_matrix_t<  cx  >::invert ( sparse_matrix_t<  cx  > & A );


/// multiply
template <typename T>
bool dense_matrix_t<T>::mult ( dense_matrix_t<T>& A, sparse_matrix_t<T>& B )
{
	return false; // not implemented yet
}
template bool dense_matrix_t<double>::mult ( dense_matrix_t<double>& A, sparse_matrix_t<double>& B );
template bool dense_matrix_t<  cx  >::mult ( dense_matrix_t<  cx  >& A, sparse_matrix_t<  cx  >& B );

/// copy a vector to a row of the matrix
template <typename T>
void dense_matrix_t<T>::copy_to_row ( dense_vector_t<T> &vec, unsigned row )
{
	unsigned col;
	err_if ( vec.length()!=cols(), "In copy_to_row, the input vector is an invalid size " );
	err_if ( row >= rows(), "In copy_to_row, the row input is invalid");
	for (col=0; col<cols_; col++) set(row, col, vec[col]);
}
template void dense_matrix_t<double>::copy_to_row ( dense_vector_t<double> &, unsigned  );
template void dense_matrix_t<  cx  >::copy_to_row ( dense_vector_t<  cx  > &, unsigned  );

/// copy a vector to a column of the matrix
template <typename T>
void dense_matrix_t<T>::copy_to_col ( dense_vector_t<T> &vec, unsigned col )
{
	unsigned row;
	err_if ( vec.length()!=rows(), "In copy_to_col, the input vector is an invalid size " );
	err_if ( col >= cols(), "In copy_to_col, the colunm input is invalid");
	for (row=0; row<rows_; row++) set(row, col, vec[row]);
}
template void dense_matrix_t<double>::copy_to_col ( dense_vector_t<double> &, unsigned  );
template void dense_matrix_t<  cx  >::copy_to_col ( dense_vector_t<  cx  > &, unsigned  );

