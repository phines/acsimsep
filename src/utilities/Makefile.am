## Process this file with automake to produce Makefile.in
# we want to build a library
noinst_LIBRARIES = libutilities.a
# from these sources
libutilities_a_SOURCES = RNG.cpp graph.cpp options.cpp utilities.cpp 
# which will need these libraries
LDADD = libutilities.a ../regex/libregex.a
# and this test program
bin_PROGRAMS = test_utilities
test_utilities_SOURCES = test_utilities.cpp
