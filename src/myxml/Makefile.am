## Process this file with automake to produce Makefile.in
# The goal is is to build a library from the sources:
noinst_LIBRARIES = libmyxml.a
libmyxml_a_SOURCES = myxml.cpp test_myxml.cpp
# We also want to build a test program
bin_PROGRAMS = test_myxml
test_myxml_SOURCES = test_myxml.cpp
# which will need to link in the library
test_myxml_LDADD = libmyxml.a -lxml2

