#include "../matrix/matrix.hpp"

/**
	NewtonSolve
		
		ptrObjFunc - pointer to a function of the following form:
			ptrObjFunc ( const DenseVector &x, DenseVector& error )
		ptrJacFunc - pointer to a function that evaluates the Jacobian such as the following:
			ptrJacFunc ( const DenseVector &x, Sparse& Jac )
		x - gives both the starting point, and the output
		epsilon - the convergence tolerance
		NOTES:
			Currently only works for square systems
**/
bool NewtonSolve( DenseVector &x,
                  int (*ptrObjFunc) ( const DenseVector &x, DenseVector& error ),
                  int (*ptrJacFunc) ( const DenseVector &x, Sparse& Jac ),
                  double epsilon=1e-12 )
{
	unsigned nX = x.length();
	double f = epsilon + 1;
	double alpha;
	DenseVector error(nX);
	DenseVector direction(nX);
	Sparse Jac(nX,nX);
	unsigned it=0, i;
	
	for ( it=0; it<20; it++ )
	{
		// calculate the Jacobian
		(*ptrJacFunc) ( x, Jac );
		
		// calculate the current error
		(*ptrObjFunc) ( x, error );
		
		// solve for the newton direction
		direction.solve( Jac, error );
		
		// choose alpha
		alpha = 1;
		
		// update x
		for ( i=0; i<nX; i++ )
			x[i] -= alpha*direction[i];
		
		// check for convergence
		f = direction.norm();
		if (f<epsilon)
			break;
	}
	
	if ( error.norm() < epsilon*10 )
		return true;
	else
		return false;
}
