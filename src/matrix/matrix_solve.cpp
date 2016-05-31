#include "matrix_solve.hpp"
#include <umfpack.h>
#include <assert.h>
#include <stdio.h>
//#include <slu_ddefs.h>
#include "sparse_matrix.hpp"
#include "dense_vector.hpp"
#include "dense_matrix.hpp"

typedef std::complex<double> cx;

/// return the UMFPACK strategy
int get_umfpack_strategy(solve_strategy_e strategy)
{
	switch (strategy)
	{
		case AUTO:
			return UMFPACK_STRATEGY_AUTO;
		case UNSYMMETRIC:
			return UMFPACK_STRATEGY_UNSYMMETRIC;
		case SYMMETRIC:
			return UMFPACK_STRATEGY_SYMMETRIC;
		case PERMUTATION:
			return UMFPACK_STRATEGY_AUTO;
//			return UMFPACK_STRATEGY_2BY2;
	}
	return UMFPACK_STRATEGY_AUTO;
}

/// solve_umfpack: solve a matrix in sparse column format using umfpack
/// Ai - row indeces
/// Ap - col indeces
/// Ax - array of entries
/// X - array for the output vector
/// B - array for the input vector
bool solve_umfpack( sparse_matrix_t< double > &A,
                    dense_vector_t < double > &x,
                    dense_vector_t < double > &b )
{
	int*    Ap=NULL;
	int*    Ai=NULL;
	double* Ax=NULL;
	bool transposed;
	int m = A.rows();
	int n = A.cols();
	bool success = false;
	int status/*, sys*/;
	double Control [UMFPACK_CONTROL], Info [UMFPACK_INFO];
	void *Symbolic=NULL, *Numeric=NULL ;
	
	A.make_csc( Ap, Ai, Ax, transposed );
	//tic();
	// begin solver
	umfpack_di_defaults (Control);
	Control[UMFPACK_STRATEGY] = get_umfpack_strategy(A.strategy());
	status = umfpack_di_symbolic (m, n, Ap, Ai, Ax, &Symbolic, Control, Info);
	if ( status == UMFPACK_OK ) {
		status = umfpack_di_numeric (Ap, Ai, Ax, Symbolic, &Numeric, Control, Info);
		if ( status == UMFPACK_OK ) {
			if (transposed)
				status = umfpack_di_solve (UMFPACK_At, Ap, Ai, Ax, x.begin(), b.begin(), Numeric, Control, Info) ;
			else
				status = umfpack_di_solve (UMFPACK_A,  Ap, Ai, Ax, x.begin(), b.begin(), Numeric, Control, Info) ;
		}
	}
	if ( status == UMFPACK_OK ) success = true;
	else {
		printf("Warning: UMFPACK could not solve linear system\n");
		//Control[UMFPACK_PRL] = 5; // set the print level to high
		//umfpack_di_report_matrix (m, n, Ap, Ai, Ax, 1, Control);
		//umfpack_di_report_info( Control, Info );
		success = false;
	}
	//toc();
	
	// cleanup:
	A.free_csc( Ai, Ap, Ax );
	if( Symbolic!=NULL ) umfpack_di_free_symbolic( &Symbolic );
	if( Numeric!=NULL  ) umfpack_di_free_numeric ( &Numeric  );
	
	// return the result
	return success;
}

bool solve_umfpack ( sparse_matrix_t< cx > &A,
                     dense_vector_t < cx > &x,
                     dense_vector_t < cx > &b )
{
	printf("\ncomplex solve does not work yet");
	return false;
}

bool solve_superlu ( sparse_matrix_t< double > &A,
                    dense_vector_t  < double > &x,
                    dense_vector_t  < double > &b )
{
	printf("\n solve_superlu(A, x, b) is not working yet");
	return false;
}

bool solve_superlu ( sparse_matrix_t<   cx   > &A,
                    dense_vector_t  <   cx   > &x,
                    dense_vector_t  <   cx   > &b )
{
	printf("\n solve_superlu(A, x, b) is not working yet");
	return false;
}

/// solve_umfpack for a set of linear systems
bool solve_umfpack ( sparse_matrix_t< double > &A,
                      dense_matrix_t< double > &X,
                     sparse_matrix_t< double > &B )
{
	int*    Ap=NULL;
	int*    Ai=NULL;
	double* Ax=NULL;
	bool transposed;
	int m = A.rows();
	int n = A.cols();
	int equation_sets = B.cols();
	bool success = false;
	int status/*, sys*/;
	double Control [UMFPACK_CONTROL], Info [UMFPACK_INFO];
	void *Symbolic=NULL, *Numeric=NULL ;
	dense_vector_t<double> x(n);
	dense_vector_t<double> b(m);
	int col;
	
	A.make_csc( Ap, Ai, Ax, transposed );
	// begin solver
	umfpack_di_defaults (Control);
	Control[UMFPACK_STRATEGY] = get_umfpack_strategy(A.strategy());
	status = umfpack_di_symbolic (m, n, Ap, Ai, Ax, &Symbolic, Control, Info);
	if ( status == UMFPACK_OK )
	{
		//A.set_symbolic(Symbolic);
		status = umfpack_di_numeric (Ap, Ai, Ax, Symbolic, &Numeric, Control, Info);
		if ( status == UMFPACK_OK )
		{
			//A.set_numeric(Numeric);
			// for each system of equations, solve for the result
			for (col=0; col<equation_sets; col++)
			{
				if (transposed)
				{
					b.copy_col(B, col);
					status = umfpack_di_solve (UMFPACK_At, Ap, Ai, Ax, x.begin(), b.begin(), Numeric, Control, Info) ;
					X.copy_to_col(x, col);
				}
				else
				{
					b.copy_col(B, col);
					status = umfpack_di_solve (UMFPACK_A,  Ap, Ai, Ax, x.begin(), b.begin(), Numeric, Control, Info) ;
					X.copy_to_col(x, col);
				}
				if ( status!=UMFPACK_OK ) break;
			}
		}
	}
	if ( status == UMFPACK_OK )
		success = true;
	else
	{
		// Control[UMFPACK_PRL] = 5; // set the print level to high
		// umfpack_di_report_matrix (m, n, Ap, Ai, Ax, 1, Control);
		// umfpack_di_report_info( Control, Info );
		success = false;
	}
	
	// free the matrix
	A.free_csc( Ai, Ap, Ax );
	// free the UMFPACK data
	if (Numeric!=NULL)  umfpack_di_free_numeric(&Numeric);
	if (Symbolic!=NULL) umfpack_di_free_symbolic(&Symbolic);
	
	return success;
}
bool solve_umfpack ( sparse_matrix_t< cx > &A,
                      dense_matrix_t< cx > &X,
                     sparse_matrix_t< cx > &B )
{
	printf("\ncomplex solve does not work yet");
	return false;
}
///  solve_superlu for a set of linear systems
bool solve_superlu ( sparse_matrix_t< double > &A,
                      dense_matrix_t< double > &X,
                     sparse_matrix_t< double > &B )
{
	printf("\n solve_superlu(A, X, B) is not working yet");
	return false;
}
bool solve_superlu ( sparse_matrix_t< cx > &A,
                      dense_matrix_t< cx > &X,
                     sparse_matrix_t< cx > &B )
{
	printf("\ncomplex solve does not work yet");
	return false;
}

///  sparse_solve
///   solve a system of linear equations:
///   A * x = b
///    A must be square
template <typename T>
bool sparse_solve ( sparse_matrix_t< T > &A,
                    dense_vector_t < T > &x,
                    dense_vector_t < T > &b,
                    solver_e solver )
{
	bool success;
	
	// check sizes
	if (A.rows()!=A.cols())
	{
		printf("\nIn sparse_solve(A, x, b), A must be a square matrix\n");
		return false;
	}
	if ( A.rows()!=b.length() )
	{
		printf("\nIn sparse_solve(A, x, b), the length of b must be equal to the number of rows in A.\n");
		return false;
	}

	// resize x
	x.resize( A.cols(), false );

	// send the info to the appropriate solver
	switch (solver)
	{
		case UMFPACK:
			success = solve_umfpack(A, x, b);
			break;
		case SUPERLU:
			success = solve_superlu(A, x, b);
			break;
		default:
			printf("\nWarning: unknown solver specified to sparse_solve(A, x, b).\n");
			success = solve_umfpack(A, x, b);
			break;
	};
	// debug:
	if (!success && A.nnz()<100) A.print("A");
	
	return success;
}
template bool sparse_solve ( sparse_matrix_t<double>&, dense_vector_t <double>&, dense_vector_t<double>&, solver_e );
template bool sparse_solve ( sparse_matrix_t<  cx  >&, dense_vector_t <  cx  >&, dense_vector_t<  cx  >&, solver_e );

///  sparse_solve
///   solve a set of linear equation systems:
///   A * x = b
///    A must be square: n x n
///    x is the output: n x 1
///    b is the right hand side: also n x 1
template <typename T>
bool sparse_solve ( sparse_matrix_t< T > &A,
                     dense_matrix_t< T > &X,
                    sparse_matrix_t< T > &B,
                    solver_e solver )
{
	bool success;
	
	// check sizes
	if (A.rows()!=A.cols())
	{
		printf("\nIn sparse_solve(A, X, B), A must be a square matrix\n");
		return false;
	}
	if ( A.rows()!=B.rows() )
	{
		printf("\nIn sparse_solve(A, X, B), the number of rows in B must be equal to the number of rows in A.\n");
		return false;
	}
	if ( A.cols()==0 || B.cols()==0 || A.rows()==0 || B.rows()==0 || A.nnz()==0 || B.nnz()==0 )
	{
		printf("\nCannot solve an empty system.");
		return false;
	}
	// resize X
	X.resize( A.cols(), B.cols() );

	// send the info to the appropriate solver
	switch (solver)
	{
		case UMFPACK:
			success = solve_umfpack(A, X, B);
			break;
		case SUPERLU:
			success = solve_superlu(A, X, B);
			break;
		default:
			printf("\nWarning: unknown solver specified to sparse_solve(A, x, b).\n");
			success = solve_umfpack(A, X, B);
			break;
	};
	
	return success;
}
template bool sparse_solve ( sparse_matrix_t<double>&, dense_matrix_t <double>&, sparse_matrix_t<double>&, solver_e );
template bool sparse_solve ( sparse_matrix_t<  cx  >&, dense_matrix_t <  cx  >&, sparse_matrix_t<  cx  >&, solver_e );


/*
/// solve_superlu: solve a matrix in sparse column format using SuperLU
// Ai - row indeces
// Ap - col indeces
// Ax - array of entries
// X - array for the output vector
// B - array for the input vector
bool solve_superlu(sparse_t& A_in, vector<double>& x, vector<double>& b)
{
	// extract data from A_in
	int m = A_in.rows();
	int n = A_in.cols();
	int nnz = A_in.nnz();
	double* rhs = doubleMalloc(m);
	double* a = doubleMalloc(nnz);
	int* xa = intMalloc(m+1);
	int* asub = intMalloc(nnz);
	int i;
	
	error("solve_superlu doesnt work yet");
	for (i=0;i<m;i++) rhs[i] = b[i];
	for (i=0;i<nnz;i++) a[i] = A_in.data(i);
	for (i=0;i<m+1;i++) xa[i] = A_in.row_index(i);
	for (i=0;i<nnz;i++) asub[i] = A_in.col_index(i);
	
	// only works for square systems
	assert(m==n);
	assert(m==x.size());
	assert(m==b.size());

	// run SuperLU
	SuperMatrix A, L, U, B;
	int      *perm_r; //row permutations from partial pivoting 
	int      *perm_c; // column permutation vector 
	int      nrhs, info,  permc_spec;
	superlu_options_t options;
	SuperLUStat_t stat;
	
	// Create matrix A in the format expected by SuperLU.
	dCreate_CompCol_Matrix(&A, m, n, nnz, a, asub, xa, SLU_NC, SLU_D, SLU_GE);
	
	// Create right-hand side matrix B. 
	nrhs = 1;
	dCreate_Dense_Matrix(&B, m, nrhs, rhs, m, SLU_DN, SLU_D, SLU_GE);

	if ( !(perm_r = intMalloc(m)) ) ABORT("Malloc fails for perm_r[].");
	if ( !(perm_c = intMalloc(n)) ) ABORT("Malloc fails for perm_c[].");

	// Set the default input options. 
	set_default_options(&options);
	options.ColPerm = NATURAL;

	// Initialize the statistics variables. 
	StatInit(&stat);
	
	dgssv(&options, &A, perm_c, perm_r, &L, &U, &B, &stat, &info);
	
	// copy B to x 
	for (i=0; i<m; i++)  x[i] = b[i];
	
	// De-allocate storage 
	//SUPERLU_FREE (rhs);
	SUPERLU_FREE (perm_r);
	SUPERLU_FREE (perm_c);
	//Destroy_CompCol_Matrix(&A);
	//Destroy_SuperMatrix_Store(&B);
	Destroy_SuperNode_Matrix(&L);
	Destroy_CompCol_Matrix(&U);
	StatFree(&stat);
	

	return true;
	
}
*/
