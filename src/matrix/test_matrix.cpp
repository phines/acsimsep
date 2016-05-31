#include "matrix.hpp"
#include <iostream>
#include <exception>
#include <set>

using namespace std;

int main(void)
{
	// how big?
	int n = 20;
	// build a sparse random matrix
	Sparse A(n,n);
	A.fill_rand(0.3);
	A.print("A");
	
	// build a right hand size
	dense_vector_t<double> b(n);
	b.fill_rand();
	b.print("b");
	// and the output vector
	dense_vector_t<double> x(n);
	
	// And solve
	sparse_solve(A,x,b);

	// and show the results
	x.print("x");

	/*
	Sparse A_sub;
	Dense  inv_A(N,N);
	set<int> subset;
	
	// checking the resize routine
	inv_A.resize(12,12);
	inv_A.resize(N,N);
		
	// fill A with random values
	A.fill_rand( 5.0/N );
	// calc the inverse
	inv_A.invert( A );
	
	// subset A
	subset.insert(0);
	subset.insert(1);
	subset.insert(2);
	A_sub.subset( A, subset, subset );
	
	// print stff
	A.print("A");
	inv_A.print("inv_A");
	A_sub.print("A_sub");

	//x.print("x");
	*/
	return 0;
}
