#include "dense_vector.h"
#include "matrix_solve.h"
#include "dot_product.h"
#include "dense_matrix.h"
#include "sparse_matrix.h"

using namespace std;

typedef std::complex<double> cx;

/// resize the 
template < typename T >
void dense_vector_t<T>::resize( unsigned len, T value ) {
	err_if( len>MAX_DENSE_ELEMENTS, "Cannot resize dense vector. Too many elements.");
	if ( len>=data_.size() ) {
		data_.resize( len+10, value );		
	}
	length_ = len;
}
template void dense_vector_t<double>::resize ( unsigned len, double value );
template void dense_vector_t<  cx  >::resize ( unsigned len, cx value );
//template void dense_vector_t<unsigned>::resize ( unsigned len, unsigned value );

template < typename T >
double dense_vector_t<T>::max_abs() {
	double value, mx=0;
	for( unsigned i=0; i<length_; i++ ) {
		value = std::abs(data_[i]);
		if( value>mx ) mx=value;
	}
	return mx;
}
template double dense_vector_t<double>::max_abs();
template double dense_vector_t<  cx  >::max_abs();

/// multiply a sparse matrix by a dense vector
template < typename T >
bool dense_vector_t<T>::mult ( sparse_matrix_t<T>& A, dense_vector_t<T>& b)
{
	unsigned row, col;
	T value;
	err_if( A.cols()!=b.rows(), "wrong dimensions in dense_vector_t<T>::mult" );
	resize( A.rows(), false );
	set(0);
	
	A.reset_next();
	while ( A.get_next( row, col, value ) )
	{
		data_[row] += ( value*b[col] ) ;
	}
	return true;
}
template bool dense_vector_t<double>::mult ( sparse_matrix_t<double>&, dense_vector_t<double>&);
template bool dense_vector_t<  cx  >::mult ( sparse_matrix_t<  cx  >&, dense_vector_t<  cx  >&);

/// multiply two vectors element by element
template < typename T >
bool dense_vector_t<T>::mult_elements ( dense_vector_t<T>& a, dense_vector_t<T>& b)
{
	err_if( a.length()!=b.length(), "wrong sizes in dense_vector_t::mult_elements");
	unsigned sz = a.length();
	resize( sz );
	
	for(unsigned i=0; i<sz; i++) data_[i] = a[i]*b[i];
	
	return true;
}
template bool dense_vector_t<double>::mult_elements ( dense_vector_t<double>&, dense_vector_t<double>&);
template bool dense_vector_t<  cx  >::mult_elements ( dense_vector_t<  cx  >&, dense_vector_t<  cx  >&);

/// add 
template < typename T >
bool dense_vector_t<T>::add ( dense_vector_t<T> &a, dense_vector_t<T> &b )
{
	unsigned i;
	err_if( a.rows()!=b.rows() || a.cols()!=b.cols(), "wrong sizes in dense_vector_t::add");
	resize( a.length() );
	
	for(i=0; i<length_; i++) data_[i] = a[i]+b[i];
	return true;
}
template bool dense_vector_t<double>::add( dense_vector_t<double> &, dense_vector_t<double> & );
template bool dense_vector_t<  cx  >::add( dense_vector_t<  cx  > &, dense_vector_t<  cx  > & );

/// subtract
template < typename T >
bool dense_vector_t<T>::sub( dense_vector_t<T> &a, dense_vector_t<T> &b )
{
	unsigned i;
	err_if( a.rows()!=b.rows() || a.cols()!=b.cols(), "wrong sizes in dense_vector_t::sub");
	resize( a.length() );
	
	for( i=0; i<length_; i++ ) data_[i] = a[i]-b[i];
	return true;
}
template bool dense_vector_t<double>::sub( dense_vector_t<double> &, dense_vector_t<double> & );
template bool dense_vector_t<  cx  >::sub( dense_vector_t<  cx  > &, dense_vector_t<  cx  > & );

/// vector norm for real vectors
template<>
double dense_vector_t<double>::norm()
{
	unsigned i;
	double total=0;
	
	for( i=0; i<length_; i++ )
		total += data_[i]*data_[i];
	
	return sqrt(total);
}

/// vector norm for complex vectors
template<>
double dense_vector_t< cx >::norm()
{
	unsigned i;
	double total=0;
	cx product;
	
	for( i=0; i<length_; i++ )
	{
		product = data_[i] * conj( data_[i] );
		total += product.real();
	}
	return sqrt(total);
}

/// generic print
template <typename T>
void dense_vector_t<T>::print(const char *name) const
{
	unsigned i;
	
	printf("\n%s = [", name);
	for ( i=0; i<length_; i++ )
	{
		print_value( data_[i] );
		printf(" ");
	};
	if (transposed_) printf("] ;\n");
	else             printf("].' ;\n");
}
template void dense_vector_t<double>::print(const char*) const;
template void dense_vector_t< cx   >::print(const char*) const;

/// solve a system of linear equations
template <typename T>
bool dense_vector_t<T>::solve ( sparse_matrix_t<T> &A, dense_vector_t<T> &b )
{
	return sparse_solve ( A, *this, b );
}
template bool dense_vector_t<double>::solve( sparse_matrix_t<double>&, dense_vector_t<double>& );
template bool dense_vector_t<  cx  >::solve( sparse_matrix_t<  cx  >&, dense_vector_t<  cx  >& );

// copy routines:
template <typename T>
void dense_vector_t<T>::copy_col( sparse_matrix_t<T> &other, unsigned col )
{
	unsigned row;
	resize( other.rows() );
	for ( row=0; row<length_; row++ ) data_[row] = other.get( row, col );
}
template void dense_vector_t<double>::copy_col( sparse_matrix_t<double>&, unsigned );
template void dense_vector_t<  cx  >::copy_col( sparse_matrix_t<  cx  >&, unsigned );

template <typename T>
void dense_vector_t<T>::copy_col( dense_matrix_t<T> &other, unsigned col )
{
	unsigned row;
	resize( other.rows() );
	for ( row=0; row<length_; row++ ) data_[row] = other.get( row, col );
}
template void dense_vector_t<double>::copy_col( dense_matrix_t<double>&, unsigned );
template void dense_vector_t<  cx  >::copy_col( dense_matrix_t<  cx  >&, unsigned );

template <typename T>
void dense_vector_t<T>::copy_row( sparse_matrix_t<T> &other, unsigned row )
{
	unsigned col;
	resize( other.cols() );
	for ( col=0; col<length_; col++ ) data_[col] = other.get( row, col );
}
template void dense_vector_t<double>::copy_row( sparse_matrix_t<double>&, unsigned );
template void dense_vector_t<  cx  >::copy_row( sparse_matrix_t<  cx  >&, unsigned );

template <typename T>
void dense_vector_t<T>::copy_row( dense_matrix_t<T> &other, unsigned row )
{
	unsigned col;
	resize( other.cols() );
	for ( col=0; col<length_; col++ ) data_[col] = other.get( row, col );
}
template void dense_vector_t<double>::copy_row( dense_matrix_t<double>&, unsigned );
template void dense_vector_t<  cx  >::copy_row( dense_matrix_t<  cx  >&, unsigned );

