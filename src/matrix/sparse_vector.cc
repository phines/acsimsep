#include "sparse_vector.h"

#include <algorithm>

typedef std::complex<double> cx;

template <typename T>
void sparse_vector_t<T>::resize(unsigned sz) {
	length_=sz;
	if (sz>0) {
		// erase all of the elements where the index is greater than or equal to sz
		data_.erase( data_.upper_bound( sz-1 ), data_.end() );
	}
	reset_next();
};
template void sparse_vector_t<double>::resize( unsigned sz );
template void sparse_vector_t<  cx  >::resize( unsigned sz );


template <typename T>
void sparse_vector_t<T>::print( const char* name ) const
{
	unsigned i;
	T value;
	
	if (transposed_)
		printf( "%s = sparse(1,%d); %%nnz=%d\n", name, length(), nnz() );
	else
		printf( "%s = sparse(%d,1); %%nnz=%d\n", name, length(), nnz() );
	// print the values
	reset_next();
	while (get_next(i,value)) {
		printf("%s(%d) = ", name, i);
		print_value(value);
		printf(";\n");
	}	
}
template void sparse_vector_t<double>::print( const char* name ) const;
template void sparse_vector_t<  cx  >::print( const char* name ) const;


template <typename T>
void sparse_vector_t<T>::subset( sparse_vector_t<T> & larger, std::set<int> & index_set )
{
	unsigned i, local_i;
	T value;
	std::set<int>::iterator iter;
	
	// go through the values in larger, and put in the local vector
	larger.reset_next();
	while ( larger.get_next( i, value ) ) {
		iter = index_set.find(i);
		if ( iter!=index_set.end() ) {
			local_i = std::distance( index_set.begin(), iter );
			err_if( local_i>=length() );
			set( local_i, value );
		}
	}
}
template void sparse_vector_t<double>::subset( sparse_vector_t<double> & larger, std::set<int> & index_set );
template void sparse_vector_t<  cx  >::subset( sparse_vector_t<  cx  > & larger, std::set<int> & index_set );
