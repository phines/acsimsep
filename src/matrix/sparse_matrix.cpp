#include "sparse_matrix.hpp"
#include <algorithm>
#define SPARSE_MATRIX_EMPTY 99999999

using namespace std;

typedef std::complex<double> cx;

template < typename T >
void sparse_matrix_t<T>::print(const char* name)
{
	unsigned row, col; T value;
	// go through all of the elements and print them
	printf( "\n%s = sparse(%d,%d); %%nnz=%d", name, rows_, cols_, nnz_ );
	reset_next();
	while ( get_next( row, col, value ) ) {
		printf("\n%s(%2d,%2d) = ", name, row+1, col+1 );
		print_value ( value );
		printf(";");
	}
	printf( "\n" );
}
template void sparse_matrix_t<double>::print(const char* name);
template void sparse_matrix_t<  cx  >::print(const char* name);

/// resize
template < typename T >
void sparse_matrix_t<T>::resize(unsigned rows, unsigned cols)
{
	unsigned i;
	unsigned data_len = (transposed_ ? rows : cols);
	unsigned vec_len  = (transposed_ ? cols : rows);
	rows_=rows;
	cols_=cols;
	data_.resize( data_len );
	for (i=0; i<data_len; i++) data_[i].resize(vec_len);
	data_size_ = data_.size();
	iter_=data_.begin();
};

template void sparse_matrix_t<double>::resize(unsigned rows, unsigned cols);
template void sparse_matrix_t<  cx  >::resize(unsigned rows, unsigned cols);

/// make_csc ( int* col_starts, int* row_indeces, T* data,  bool& trans)
///  col_starts -- Gives the index into the row_indeces vector for the start of the next column
///                 The size of col_start_locs must be data_.size() + 1
///  row_indeces -- Gives the row number for each entry in value.
///                 The size is nnz()
///  values -- The vector of values with size nnz()
///  trans -- whether the output is actually the transpose of the matrix or not
///  NOTE: When the matrix is transposed, this function reports the compressed sparse row
///        format as this represents the internal storage. trans tells the user whether the
///        output is transposed or not
template < typename T >
void sparse_matrix_t<T>::make_csc( int*& col_starts, int*& row_indeces, T*& values,  bool& trans)
{
	unsigned row=0;
	unsigned col=0;
	unsigned prev_col=SPARSE_MATRIX_EMPTY;
	unsigned nz_no=0;
	int i;
	int col_starts_size=(transposed_?rows_:cols_)+1; // the size of the col_starts vector
	T value;
	
	// set all of the index values to -1 so that we know if we missed one
	col_starts_.clear();
	col_starts_.resize( col_starts_size + 10, -1 );
	row_index_ .resize( nnz_ + 10, -1 );
	values_    .resize( nnz_ + 10, 0.0 );
	
	col_starts  = & col_starts_[0];
	row_indeces = & row_index_[0];
	values      = & values_[0];
	
	//go through all of the col start pointers and set to -1 so that we know if we missed one
	for (i=0;i<col_starts_size;i++) col_starts[i]=-1;
	
	reset_next();
	while ( get_next( row, col, value ) ) {
		if (transposed_) swap(row,col);
		if (col!=prev_col) col_starts[col]=nz_no;
		prev_col=col;
		row_indeces[nz_no] = row;
		values[nz_no] = value;
		nz_no++;
	}
	err_if(nz_no!=nnz(), "in sparse_matrix_t<T>::make_csc: nz_no should be equal to nnz");
	col_starts [ col_starts_size-1 ] = nnz();
	// fill any columns that got skipped with the appropriate value
	for (i=col_starts_size-2; i>=0; i--)
	{
		if (col_starts[i]==-1) col_starts[i]=col_starts[i+1];
	}
	trans = transposed_;
}
template void sparse_matrix_t<double>::make_csc(int*&, int*&, double*&, bool&);
template void sparse_matrix_t<  cx  >::make_csc(int*&, int*&, cx*&,     bool&);

/// make_csr ( int* row_start_locs, int* col_indeces, T* data,  bool& trans)
template < typename T >
void sparse_matrix_t<T>::make_csr( int*& row_start_locs, int*& col_indeces, T*& data,  bool& trans)
{
	// the following is a cheater way of doing things, but it works...
	make_csc( row_start_locs, col_indeces, data, trans );
	trans = !trans;
}
template void sparse_matrix_t<double>::make_csr( int*&, int*&, double*&, bool&);
template void sparse_matrix_t<  cx  >::make_csr( int*&, int*&,     cx*&, bool&);

/// make_triplet
template < typename T >
void sparse_matrix_t<T>::make_triplet( int*& row_indeces, int*& col_indeces, T*& data )
{
	unsigned row, col, next=0;
	T value;

	row_index_.resize( nnz_+10, -1 );
	col_index_.resize( nnz_+10, -1 );
	values_   .resize( nnz_+10, 0.0 );
	
	row_indeces = &row_index_[0];
	col_indeces = &col_index_[0];
	data        = &values_[0];
	
	reset_next();
	while ( get_next( row, col, value ) )
	{
		row_indeces[next] = row;
		col_indeces[next] = col;
		data[next] = value;
		next++;
	}
	err_if( next!=nnz_, "error in make_triplet" );
}
template void sparse_matrix_t<double>::make_triplet( int*&, int*&, double*&);
template void sparse_matrix_t<  cx  >::make_triplet( int*&, int*&,     cx*&);

/// functions for freeing stuff:

/// free_csc
template < typename T >
void sparse_matrix_t<T>::free_csc( int*& index1, int*& index2, T*& values )
{
	// do nothing;
}
template void sparse_matrix_t<double>::free_csc( int*&, int*&, double*&);
template void sparse_matrix_t<  cx  >::free_csc( int*&, int*&,     cx*&);
/// free_csr
template < typename T >
void sparse_matrix_t<T>::free_csr( int*& index1, int*& index2, T*& values )
{
	// do nothing
}
template void sparse_matrix_t<double>::free_csr( int*&, int*&, double*&);
template void sparse_matrix_t<  cx  >::free_csr( int*&, int*&,     cx*&);
/// free_triplet
template < typename T >
void sparse_matrix_t<T>::free_triplet( int*& index1, int*& index2, T*& values )
{
	// do nothing
}
template void sparse_matrix_t<double>::free_triplet( int*&, int*&, double*&);
template void sparse_matrix_t<  cx  >::free_triplet( int*&, int*&,     cx*&);

/// get_next
template <typename T>
bool sparse_matrix_t<T>::get_next(unsigned& row, unsigned& col, T& value)
{
	col = std::distance(data_.begin(),iter_);
	if (col>=n_vectors()) return false;
	while ( !iter_->get_next( row, value ) ) {
		// since we didn't find a nz in the last column, look in the next one
		iter_++; col++;
		// if we have run out of columns:
		if (col>=n_vectors()) return false;
		// reset the pointer for the current data vector
		else iter_->reset_next();
	}
	if (transposed_) swap(row,col);
	return true;
}
template bool sparse_matrix_t<double>::get_next(unsigned& row, unsigned& col, double& value);
template bool sparse_matrix_t<  cx  >::get_next(unsigned& row, unsigned& col, cx& value);

/// subset
template <typename T>
void sparse_matrix_t<T>::subset( sparse_matrix_t<T> & larger, std::set<int> & row_set, std::set<int> & col_set )
{
	unsigned col, local_col=0;
	int n = row_set.size();
	int m = col_set.size();
	std::set<int>::iterator col_iter;
	
	// check the inputs
	err_if( n==0 or m==0, "Empty set passed to sparse_matrix_t<T>::subset." );
	err_if( larger.transposed(), "This doesn't work for transposed matries." );
	
	// prepare this matrix
	zeros();
	resize(n,m);
	
	// go through the columns and take the subsets
	for( col_iter=col_set.begin(); col_iter!=col_set.end(); col_iter++ ) {
		col = *col_iter;
		data_[local_col].subset( larger[col], row_set );
		local_col++;
	}
}
template void sparse_matrix_t<double>::subset( sparse_matrix_t<double> & larger, std::set<int> & row_set, std::set<int> & col_set );
template void sparse_matrix_t<  cx  >::subset( sparse_matrix_t<  cx  > & larger, std::set<int> & row_set, std::set<int> & col_set );


template <>
bool sparse_matrix_t<double>::solve(dense_vector_t <double> &b, dense_vector_t <double> &x )
{
	bool success=false;

	// First make a csc
	printf("sparse_matrix_t<double>::solve is not working yet\n");

	return success;
}

template <typename cx>
bool sparse_matrix_t<cx>::solve(dense_vector_t <cx> &b, dense_vector_t <cx> &x )
{
	bool success=false;

	// First make a csc
	printf("sparse_matrix_t<cx>::solve is not working yet\n");

	return success;
}


