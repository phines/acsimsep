#ifndef SOLVE_H
#define SOLVE_H

#include "matrix_globals.h"
#include <complex>

// pre-declare the matrix types that we will use:
template <typename T>
class sparse_matrix_t;
template <typename T>
class dense_vector_t;
template <typename T>
class dense_matrix_t;


template <typename T>
/// solve a single linear system
bool sparse_solve ( sparse_matrix_t< T > &A,
                     dense_vector_t< T > &x,
                     dense_vector_t< T > &b,
                    solver_e solver=UMFPACK );

/// solve a set of linear systems
template <typename T>
bool sparse_solve ( sparse_matrix_t< T > &A,
                     dense_matrix_t< T > &X,
                    sparse_matrix_t< T > &B,
                    solver_e solver=UMFPACK );

#endif
