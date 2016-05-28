#ifndef DOT_PRODUCT_H
#define DOT_PRODUCT_H

#include <complex>
#include "sparse_vector.h"
#include "dense_vector.h"

template< typename T >
T dot_product( dense_vector_t<T> &a, dense_vector_t<T> &b )
{
	T result=0;
	unsigned sz=a.length();
	if( a.rows()!=b.rows() || a.cols()!=b.cols() )
	{
		printf("wrong sizes in dot_product" );
		exit(0);
	}	
	for( int i=0; i<sz; i++ ) result += a[i]*b[i];
	
	return result;
}

template< typename T >
T dot_product( dense_vector_t<T> &a, sparse_vector_t<T> &b )
{
	unsigned index, sz=a.length();
	T result=0;
	T value;
	if( a.length()!=b.length() )
	{
		printf("wrong sizes in dot_product" );
		exit(0);
	}
	
	b.reset_next();
	while ( b.get_next(index, value) && index<sz )
	{
		result+=a[index]*value;
	}
	return result;
}

template< typename T >
T dot_product ( sparse_vector_t<T> &a, dense_vector_t<T> &b )
{
	return dot_product( b, a );
}

template <typename T>
T dot_product ( sparse_vector_t<T> &a, sparse_vector_t<T> &b )
{
	T result=0;
	unsigned sz=a.length();
	unsigned index_a, index_b;
	T        value_a, value_b;
	bool more=true;
	
	if( a.rows()!=b.rows() || a.cols()!=b.cols() )
	{
		printf("wrong sizes in dot_product" );
		exit(0);
	}	
	a.reset_next();
	b.reset_next();
	a.get_next(index_a, value_a);
	b.get_next(index_b, value_b);
	while ( more && index_a<sz && index_b<sz )
	{
		if ( index_a==index_b )
		{
			result+=value_a*value_b;
			more = ( a.get_next(index_a, value_a) && b.get_next(index_b, value_b) );
		}
		else if ( index_a > index_b )
		{
			more = b.get_next(index_b, value_b);
		}
		else // index_a < index_b
		{
			more = a.get_next(index_a, value_a);
		}
	}
	return result;
}

#endif
