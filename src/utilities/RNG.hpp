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
		~RNG();
		// return a random number from distribution d
		double rand(distribution_e d);
		// return a random number from distribution d with two parameters
		// rand(UNIFORM, lower_limit, upper_limit)
		// or 
		// rand(GAUSS, mu, sigma)
		double rand(distribution_e d, double p1, double p2);
		// return a gaussian variable with mean 0 and standard deviation 1
		double randn();
};
