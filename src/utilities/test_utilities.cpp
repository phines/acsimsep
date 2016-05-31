#include <vector>
#include <cstdio>
#include "utilities.hpp"

using namespace std;

int main(void)
{
	unsigned i;
	int N = 8;
	int f[] = {1,2,3,4,5,6,7,8};
	int t[] = {2,3,4,1,6,7,8,5};
	vector<int> from(f,f+N);
	vector<int>   to(t,t+N);
	vector< set<int> > subgraphs;
	set<int> A;
	set<int> B;
	
	printf("------------------------------------------\n");
	printf("Testing the Graph class\n");
	printf("------------------------------------------\n");
	
	// initialize and print the graph
	Graph G(from,to);
	G.print("G");
	// test the find_neighbors function
	printf("Finding (distance 2) neighbors of a set A: \n");
	A.insert(1);
	print_set(A);
	G.find_neighbors( A, 2, B );
	printf("Neighbors: \n");
	print_set(B);

	// see if the system is fully connected
	if ( G.is_fully_connected() ) printf("Graph is fully connected.\n");
	else printf("Graphs is not fully connected.\n");
	
	// see how many subgraphs there are
	printf("The graph has %d subgraph(s) in it.\n", G.n_subgraphs() );
	
	// erase an edge, add a vertex
	G.remove_edge(2,3);
	G.add_vertex(10);
	
	printf("The graph now looks like: ");
	G.print("G");
	
	// test the find subgraphs function
	printf("This time we found %d subgraphs:\n", G.find_subgraphs( subgraphs ));
	for ( i=0; i<subgraphs.size(); i++ ) {
		printf("%d: ",i);
		print_set(subgraphs[i]);
	}
	/// RNG
	printf("------------------------------------------\n");
	printf("Testing the random number generator class \n");
	printf("------------------------------------------\n");
	// initialize the random number generator
	RNG r;
	
	// print some random numbers:
	printf("Some Random Numbers:\n");
	printf(" Uniform:  ");
	for(i=0;i<10;i++) printf("%10f ", r.rand(UNIFORM) );
	printf("\n");
	
	// print some gaussian random numbers:
	printf(" Gaussian: ");
	for(i=0;i<10;i++) printf( "%10f ", r.rand(GAUSS) );
	printf("\n");
	
	printf(" Integer:  ");
	for(i=0;i<10;i++) printf("%10d ", r.randi(0, 100));
	printf("\n");
	printf("\n");
	
	
	// check the options file stuff
	printf("------------------------------------------\n");
	printf("Testing the options file reader \n");
	printf("------------------------------------------\n");
	options_t options;
	options.read_file( "options.txt" );
	double fred = options.get_numeric("fred");
	
	printf("\nOption: \"fred\" = %g\n", fred );
	printf("Option: \"jill\" = %s\n", options.get("jill").c_str() );
	printf("Option: \"a_b\" = %g\n", options.get_numeric("a_b") );
	return 0;
}
