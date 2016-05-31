// nothing to do here...
#include "matrix.hpp"
#include <ctime>
#include <sys/time.h>
#include <cstdlib>
#include <iostream>

#define CLOCK_TICKS_PER_SECOND CLOCKS_PER_SEC

using namespace std;


/** 
 Some Global operations
**/


// set up some global values
clock_t start = 0;
struct timeval t1;
struct timezone zone;
struct timeval t2;

double dtime()
{
	double now = 0;
	return now;
}

void tic()
{
	gettimeofday(&t1, NULL);
}

double toc()
{
	//clock_t finish = clock();
	double diff;
	
	//diff = ((double) (finish - start)) / CLOCKS_PER_SEC;
	gettimeofday(&t2, NULL);
	
	diff = (t2.tv_sec - t1.tv_sec) + ((t2.tv_usec - t1.tv_usec)/10e6);
	
	printf("\nElapsed time = %f\n", diff);
	
	return diff;
}

bool matrix_solve( Sparse& A, DenseVector& x, DenseVector& b, solver_e solver  )
{
	bool check = false;
	bool result;
	
	result = sparse_solve ( A, x, b, solver );
	
	if (check)
	{
		DenseVector b_2( x.size() );
		double eps = 1e-6;
		double diff, max_diff = 0;
		unsigned i;
		
		b_2.mult( A, x );
		for ( i=0; i<b_2.size(); i++ )
		{
			diff = fabs(b_2[i]-b[i]);
			max_diff = max(diff, max_diff);
			if ( diff>eps )
			{
				printf("\nmatrix_solve: Error in element %d greater than %g.  Difference = %g", i, eps, diff);
			}
		}
		printf("\nmatrix_solve: Maximum error is %g\n", max_diff);
	}
	
	return result;
}

bool matrix_solve( Sparse_cx& A, DenseVector_cx& x, DenseVector_cx& b, solver_e solver  )
{
	return false;
}

