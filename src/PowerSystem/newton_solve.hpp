
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
bool NewtonSolve( int (*ptrObjFunc), int (*ptrJacFunc), double* x0, double epsilon );

