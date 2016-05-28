#include "matrix.h"
#include <iostream>
#include <exception>
#include <set>

using namespace std;

int main(void)
{
	int N = 10;
	Sparse A(N,N);
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
	return 0;
}
