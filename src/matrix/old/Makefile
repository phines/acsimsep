## makefile for project: matrix
## 

## include my global options
include ../Make.inc
## project specific options
PROJECT_NAME = matrix
OBJECTS      = matrix.o sparse_matrix.o matrix_solve.o dense_vector.o dense_matrix.o sparse_vector.o matrix_base.o vector_base.o matrixable.o
MAIN         = main.cc
EXE          = $(PROJECT_NAME)
LINK_OPTS    = $(MATRIX_LINK)
FLAGS        = $(CXXFLAGS)
LIB          = lib$(PROJECT_NAME).a
DOXYFILE     = Doxyfile

default: install

all: install

test: $(EXE)

lib: $(LIB)

run: $(EXE)
	./$(EXE)
	
doc:
	doxygen $(DOXYFILE)

install: $(LIB) $(EXE)
	cp -f $(EXE) $(PREFIX)/bin
	cp -f $(LIB) $(PREFIX)/lib
	cp -f *.h* $(PREFIX)/include

clean:
	rm -fr *~ *.o *.exe *.a html deps $(EXE)

## build the executable
$(EXE): $(MAIN) $(LIB)
	$(CCC) $(FLAGS) -o $(EXE) $(MAIN) $(LINK_OPTS)
	
## build the library
$(LIB): $(OBJECTS)
	ar crv $(LIB) $(OBJECTS)

## macro to compile objects
%.o: %.cc %.h
	$(CCC) $(FLAGS) -c $*.cc -o $*.o
