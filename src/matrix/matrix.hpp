#ifndef MATRIX_H
#define MATRIX_H

	#include "sparse_vector.h"
	#include "sparse_matrix.h"
	#include "dense_vector.h"
	#include "dense_matrix.h"
	#include "matrix_solve.h"
	#include "matrix_globals.h"
	#include <complex>
	#include <vector>
	
	/*! \mainpage 
	  * <h1> Overview of the "matrix" project </h1>
	  * <h2> Prerequisites </h2>
	  *  The matrix class depends on the following libraries:
	  *  <ul>
	  *    <li>umfpack - available from http://www.cise.ufl.edu/research/sparse </li>
	  *    <li>amd - available from http://www.cise.ufl.edu/research/sparse</li>
	  *    <li>blas libraries - the same ones that were compiled with umfpack, amd</li>
	  *  </ul>
	  *
	  * <h2> Installation </h2>
	  *  To install the matrix project:
	  *  <ol>
	  *    <li> edit the top level makefile </li>
	  *    <li> "make" to build the executable and library </li>
	  *    <li> "make install" to install the libraries and headers </li>
	  *  </ol>
	  *  This will generate an executable (./matrix) with some sort of test routine (though not a very good one),
	  *  and a static link library (libmatrix.a)
	  *
	  * <h2> Data Types </h2>
	  *  The following data types (classes) are defined by this project:
	  *  <ul>
	  *    <li>Dense     - a dense matrix of double values</li>
	  *    <li>Dense_cx  - a dense matrix of complex double values</li>
	  *    <li>DenseVector    - a dense vector of double values</li>
	  *    <li>DenseVector_cx - a dense vector of complex double values</li>
	  *    <li>Sparse    - a sparse matrix of double values</li>
	  *    <li>Sparse_cx - a sparse matrix of complex double values</li>
	  *    <li>SpVector  - a sparse vector of double values</li>
	  *    <li>SpVector_cx - a sparse vector of complex double values</li>
	  *  </ul>
	  */
	
	/// cx -
	typedef std::complex<double> cx;
	
	// define the types that are available for use
	
	// dense vectors:
	typedef class dense_vector_t<double>   DenseVector;
	typedef class dense_vector_t<  cx  >   DenseVector_cx;
	//typedef class std::vector<unsigned>    Index; // can be used to index elements of other vectors
	
	// sparse vectors:
	typedef class sparse_vector_t<double>  SparseVector;
	typedef class sparse_vector_t<  cx  >  SparseVector_cx;

	// sparse matrices:
	typedef class sparse_matrix_t<double>  Sparse;
	typedef class sparse_matrix_t<  cx  >  Sparse_cx;
	
	// dense matrices:
	typedef class dense_matrix_t<double>   Dense;
	typedef class dense_matrix_t<  cx  >   Dense_cx;
	
	// solvers:
	bool matrix_solve( Sparse    &A, DenseVector    &x, DenseVector    &b, solver_e solver=UMFPACK );
	bool matrix_solve( Sparse_cx &A, DenseVector_cx &x, DenseVector_cx &b, solver_e solver=UMFPACK );
	
	// a few utility functions
	double dtime();
	void   tic();
	double toc();
	
	template <typename T>
	void print_vec ( const std::vector<T> &vec, const char* name )
	{
		unsigned i;
		printf("\n%s=[", name);
		for (i=0; i<vec.size(); i++)
			printf("%g ", (double) vec[i] );
		printf("]';\n");
	}
	template <typename T>
	void print_vec ( const std::vector<T> &vec ) { print ( vec, "stl_vec" ); }
#endif
