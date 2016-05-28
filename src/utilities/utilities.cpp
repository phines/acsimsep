#include "utilities.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/times.h>
#include <cassert>

using namespace std;

/// a random number generator class
class RNG
{
	private:
		gsl_rng *r_;
	public:
		// constructor
		RNG() {
			r_ = gsl_rng_alloc (gsl_rng_taus);
			assert( r_!=NULL );// make sure that we initialized the generator
			gsl_rng_set (r_, times(NULL));	
		}
		// destructor
		~RNG() {
			gsl_rng_free (r_);
		}
		// the function that returns the random number
		double get_number(distribution_e dist) {
			assert( r_!=NULL );// make sure that we initialized the pointer
			switch(dist) {
				case UNIFORM:
					return gsl_rng_uniform(r_);
				case NORMAL:
					return gsl_ran_ugaussian(r_);
				default:
					return get_number(UNIFORM);
			}
		}
		// return a random number with two parameters
		double get_number(distribution_e dist, double p1, double p2) {
			switch(dist) {
				case UNIFORM:
					// returns a value in the range [p1, p2)
					return gsl_rng_uniform(r_)*(p2-p1) + p1;
				case NORMAL:
					// mu=p1, sigma=p2
					return gsl_ran_gaussian(r_, p2) + p1;
				default:
					return get_number(UNIFORM);
			}
		}
} rng;


double random(distribution_e dist) {
	return rng.get_number(dist);
}

double random(distribution_e dist, double p1, double p2) {
	return rng.get_number(dist, p1, p2);
}

double randn() { return random(GAUSS); };
