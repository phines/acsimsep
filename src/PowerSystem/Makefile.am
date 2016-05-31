## Process this file with automake to produce Makefile.in
# The goal is is to build a library from the sources:
noinst_LIBRARIES = libPowerSystem.a
libPowerSystem_a_SOURCES = PowerElement.cpp PowerFlow.cpp PowerGlobals.cpp PowerOptions.cpp PowerSystem.cpp Sim_Solver.cpp branch.cpp bus.cpp event.cpp gen.cpp load.cpp newton_solve.cpp pf.cpp
# and we want to build a test program
bin_PROGRAMS = test_ps
test_ps_SOURCES = test_ps.cpp
# which will need to link in the library and a bunch of other stuff
LDADD = libPowerSystem.a ../matrix/libmatrix.a ../myxml/libmyxml.a ../regex/libregex.a ../utilities/libutilities.a -lxml2
