# Basic information about this program:
AC_INIT([acsimsep], [0.1], [paul.hines@uvm.edu])

# For some reason maybe I need this:
#AC_CONFIG_MACRO_DIR([m4])

# Set some flags
PWD=`pwd`
# The following code needs to be fixed because of crappy hardwiring...ugh.
# Pavan suggests that this might cause troubles:
LDFLAGS="$LDFLAGS \
	-L/usr/local/lib \
	-L$PWD/ThirdParty/Coin-Bonmin/lib \
	-L$PWD/ThirdParty/SuiteSparse/lib -lumfpack \
	`xml2-config --libs`"
CPPFLAGS="$CPPFLAGS \
	-I/usr/local/include \ 
	-I$PWD/ThirdParty/Coin-Bonmin/include \ 
	-I$PWD/ThirdParty/SuiteSparse/include \
	`xml2-config --cflags`"

# Initialize Automake
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

# Where should everything be installed by default?  Here, we want it
# to be installed directly in 'bin', 'lib', 'include' subdirectories
# of the directory where configure is run.  The default would be
# /usr/local.
AC_PREFIX_DEFAULT([`pwd`])

# Stuff to check for:
AC_PROG_CC
AC_PROG_CXX
# check for math libraries
AC_CHECK_LIB([m],[cos])
AC_CHECK_LIB([gslcblas],[cblas_dgemm])
AC_CHECK_LIB([gsl],[gsl_blas_dgemm])
AC_CHECK_LIB([pcre],[pcre_compile])
AC_CHECK_LIB([umfpack],[umfpack_di_solve])
#AC_CHECK_LIB([xml2],[])

# Standard configuration headers:
AC_CONFIG_HEADERS([config.h])

# Since we are building libraries we need this
AC_PROG_RANLIB
# and this
AM_PROG_AR

# What files should we make
AC_CONFIG_FILES([
	Makefile
	src/Makefile
	src/regex/Makefile
	src/utilities/Makefile
	src/matrix/Makefile
	src/myxml/Makefile
	src/PowerSystem/Makefile
])

# Configure the Bonmin stuff
## AC_CONFIG_SUBDIRS([ThirdParty/Coin-Bonmin])

# Now do it
AC_OUTPUT
