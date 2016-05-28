#ifndef UTILITIES_H
#define UTILITIES_H

/// \mainpage utilities
///  Utilities provides a simple set of tools for phines' projects
///  Dependencies:
///   GSL - GNU Scientific Library


/// an enum for random number distributions
enum distribution_e {UNIFORM=0, GAUSSIAN=1, NORMAL=1, GAUSS=1 };

/// return a random number of the specified distribution, include 2 parameters
double random(distribution_e dist, double param_1, double param_2);

/// return a random number of the specified distribution
double random(distribution_e dist);

/// return a gaussian
double randn();
#endif
