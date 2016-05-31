## Process this file with automake to produce Makefile.in
bin_PROGRAMS = test_matrix
test_matrix_SOURCES = test_matrix.cpp
LDADD = libmatrix.a -L../utilities -lutilities -L../../ThirdParty/SuiteSparse/lib/ -lumfpack
noinst_LIBRARIES = libmatrix.a
libmatrix_a_SOURCES = dense_matrix.cpp dense_vector.cpp matrix.cpp matrix_base.cpp matrix_solve.cpp matrixable.cpp sparse_matrix.cpp sparse_vector.cpp vector_base.cpp
