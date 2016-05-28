#include "RNG.hpp"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/times.h>
#include <cassert>

//// Function definitions for the RNG class

// Constructor

RNG::RNG() { 
	tms * t = new tms;
	r_ = gsl_rng_alloc(gsl_rng_taus); 
	gsl_rng_set(r_, times(t));
	delete t;
}
/*
RNG::RNG() {
	r_ = gsl_rng_alloc (gsl_rng_taus);
	gsl_rng_set (r_, times(NULL));	
}
// Destructor
~RNG() {
	gsl_rng_free (r_);
}
*/

// the function that returns the random number
double RNG::rand(distribution_e dist) {
	assert( r_!=NULL );// make sure that we initialized the pointer
	switch(dist) {
		case UNIFORM:
			return gsl_rng_uniform(r_);
		case NORMAL:
			return gsl_ran_ugaussian(r_);
		default:
			return rand(UNIFORM);
	}
}

// two parameter version
double RNG::rand(distribution_e dist, double p1, double p2) {
	assert( r_!=NULL );// make sure that we initialized the pointer
	switch(dist) {
		case UNIFORM:
			// returns a value in the range [p1, p2)
			return gsl_rng_uniform(r_)*(p2-p1) + p1;
		case NORMAL:
			// mu=p1, sigma=p2
			return gsl_ran_gaussian(r_, p2) + p1;
		default:
			return rand(UNIFORM);
	}
}
// simple gaussian
double RNG::randn() { 
	return rand(NORMAL,1.0,0.0); 
};
