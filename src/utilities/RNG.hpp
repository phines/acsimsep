#ifndef RNG_HPP
#define RNG_HPP
#include <gsl/gsl_rng.h>

/// an enum for random number distributions
enum distribution_e {UNIFORM=0, GAUSSIAN=1, NORMAL=1, GAUSS=1 };

// A random number generator class
class RNG
{
	private:
		gsl_rng *r_;
	public:
		// constructor
		RNG();
		// destructor
		~RNG() { gsl_rng_free (r_); r_=NULL; }
		// return a random number from distribution d
		double rand(distribution_e d=UNIFORM);
		// return a random number from distribution d with two parameters
		// rand(UNIFORM, lower_limit, upper_limit)
		// or 
		// rand(GAUSS, mu, sigma)
		double rand(distribution_e d, double p1, double p2);
		// return a gaussian variable with mean 0 and standard deviation 1
		double randn();
		// two parameter random gaussian
		double randn(double mu, double sigma);
		// two parameter random uniform
		double randu(double min_val, double max_val);
		// two parameter random integer
		int randi(int min_val,int max_val);
};

#endif
