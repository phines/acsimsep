#include "graph.h"
#include <algorithm>
#include <cassert>

using namespace std;

/// Vertex
void Vertex::print() const {
	set<int>::iterator it;
	
	printf("%5d: ",id_);
	for ( it=neighbors.begin(); it!=neighbors.end(); it++ ) {
		printf("%d, ",*it);
	}
	printf("\n");
}

/// Edge
void Edge::print() const {
	printf(" %d -> %d\n", f_, t_);
}

/// build a graph
void Graph::build(vector<int> &from, vector<int> &to) {
	assert(from.size()==to.size());
	build(&from[0],&to[0],from.size());	
}
/// with pointers
void Graph::build(int *from, int *to, int n) {
	int i;
	clear();
	for (i=0; i<n; i++) {
		insert_edge(from[i],to[i]);
	}
	return;
}
void Graph::build(vector<int> &from, vector<int> &to, set<int> &v_set) {
	set<int>::iterator i;
	
	build(from,to);
	for(i=v_set.begin();i!=v_set.end();i++) {
		add_vertex(*i);
	}
}

/// print the graph
void Graph::print(const char *name) const {
	set<Vertex>::iterator it_V;
	set<Edge>::iterator   it_E;
	
	printf("\"%s\" has %d Vertices and %d Edges:\n", name, (int)V_.size(), (int)E_.size() );
	printf("   ID: Neighbors\n");
	for ( it_V=V_.begin(); it_V!=V_.end(); it_V++ ) {
		it_V->print();
	}
	printf(" F -> T\n");
	for ( it_E=E_.begin(); it_E!=E_.end(); it_E++ ) {
		it_E->print();
	}
}

/// find_component
bool Graph::extract_component( set<int> &component ) {
	unsigned old_size=0;
	set<Vertex>::iterator v;
	set<int>::iterator it;
	component.clear();
	set<int> new_set;
	set<int> old_set;
	set<int> all;
	// insert the first element into the new set and component
	new_set.insert(V_.begin()->id());
	component.insert(new_set.begin(), new_set.end());
	
	while (component.size()>old_size) {
		old_size = component.size();
		old_set=new_set;
		new_set.clear();
		// go through the old set and insert the 
		for ( it=old_set.begin(); it!=old_set.end(); it++ ) {
			v = V_.find(*it);
			new_set.insert( v->neighbors.begin(), v->neighbors.end() );
		}
		// insert the new stuff into component
		component.insert(new_set.begin(), new_set.end());
	}
	// erase all of the graph elements in component
	for ( it=component.begin(); it!=component.end(); it++ ) {
		remove_vertex(*it);
	}
	return ( size() > 0 );
}

void Graph::merge_neighbors( std::set<int> &vertex_set )
{
	set<int>::iterator iter;
	set<Vertex>::iterator pVertex;
	set<int> old_set(vertex_set);
	insert_iterator< set<int> > output(vertex_set,vertex_set.begin());
	int v;
	
	if (vertex_set.size()==0) {
		printf("Attempt to call merge_neighbors with empty input set.\n");
		return;
	}
	
	for( iter=old_set.begin(); iter!=old_set.end(); iter++ ) {
		v = *iter;
		pVertex = V_.find(*iter);
		// check to make sure that this node exists
		if ( pVertex==V_.end() ) break;
		// merge the node's neighbors into the output
		set_union( vertex_set.begin(), vertex_set.end(), pVertex->neighbors.begin(), pVertex->neighbors.end(), output );
	}
}

/// find some neighbors connected to a set
void Graph::find_neighbors( std::set<int> &start_set, int distance, std::set<int> &neighbor_set, bool remove_start_set)
{
	int d;
	unsigned old_size=0;
	set<int> all = start_set;
	set<int> new_set, old_set;
	insert_iterator< set<int> > output(all, all.begin());
	
	for (d=1;d<=distance;d++) {
		if ( all.size()==size() or all.size()==old_size ) break;
		old_size = all.size();
		merge_neighbors( all );
	}
	if (remove_start_set) {
		neighbor_set = set_difference( all, start_set );
	} else {
		neighbor_set = all;
	}
}

/// find some neighbors connected to a set
void Graph::find_neighbors( std::set<int> &start_set, std::set<int> &neighbor_set, bool remove_start_set)
{
	find_neighbors( start_set, 1, neighbor_set, remove_start_set );
}
/// find some neighbors connected to a single node
void Graph::find_neighbors( int &start_vertex, int distance, std::set<int> &neighbor_set, bool remove_start_set)
{
	set<int> start_set;
	start_set.insert(start_vertex);
	find_neighbors( start_set, distance, neighbor_set, remove_start_set );
}

/// 
bool Graph::is_fully_connected() {
	unsigned old_size=0;
	set<int> component;
	
	if (size()==0) {
		printf("Graph::Called is_fully_connected for an empty graph.\n");
		return true;
	} else {
		component.insert( V_.begin()->id() );
	}
	
	/// find all the nodes connected to the first node
	while ( component.size()>old_size ) {
		old_size = component.size();
		merge_neighbors( component );	
	}
	return component.size() == V_.size();
}

int Graph::find_subgraphs( vector< set<int> > & subgraphs ) {
	int nComponents = 0;
	Graph G(*this);
	set<int> subgraph;
	bool more;
	
	subgraphs.clear();
	do {
		more = G.extract_component(subgraph);
		subgraphs.push_back(subgraph);
		nComponents++;
	} while( more );
	return nComponents;	
}

int Graph::n_subgraphs() {
	int nComponents = 0;
	Graph G(*this);
	set<int> component;
	bool more;
	
	do {
		more = G.extract_component(component);
		nComponents++;
	} while (more);
	return nComponents;	
}

void print_set( std::set<int> &a ) {
	std::set<int>::iterator it;
	for ( it=a.begin(); it!=a.end(); it++ ) {
		printf("%d ", *it);
	}
	printf("\n");
}

int find_connected( std::vector<int> &from, std::vector<int> &to, std::vector< std::set<int> > &components  )
{
	Graph G(from,to);
	int nComponents = 0;
	set<int> component;
	bool more;
	
	do {
		more = G.extract_component(component);
		components.push_back(component);
		nComponents++;
	} while (more);
	
	return nComponents;
}

/// calculate the differences between two sets return (A - B)
std::set<int> set_difference( std::set<int> &A, std::set<int> &B )
{
	set<int> output_set;
	set_difference( A.begin(), A.end(), B.begin(), B.end(), inserter( output_set, output_set.begin() ) );
	return output_set;
}

void distance_matrix( std::vector<int> &from, std::vector<int> &to, std::vector< std::vector<int> > & D )
{
	printf("This doesn't work yet\n\n");
	exit(1);
	/*
	Graph G(from,to);
	int n=G.size();
	int i;
	// make sure D is sized correctly
	D.clear();
	D.resize(n);
	for (i=0;i<n;i++) {
		D[i].resize(n,MAXINT);
	}
	// start with a node, find its neighbors, and its neighbors,
	for (i=0;i<n;i++) {
		j=0;
	}
	*/
}

/// is a in set B
bool is_member( int a, std::set<int> B ) {
	return B.find(a)!=B.end();
}

/// set_union
std::set<int> set_union( std::set<int> &A, std::set<int> &B )
{
	set<int> output_set;
	set_union( A.begin(), A.end(), B.begin(), B.end(), inserter( output_set, output_set.begin() ) );
	return output_set;
}

/// set_union
std::set<int> set_union( std::set<int> &A, int b )
{
	set<int> B;	B.insert(b);
	return set_union( A, B );
}


