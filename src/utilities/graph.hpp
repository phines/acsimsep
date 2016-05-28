#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <set>

/// edges in an undirected & unweighted graph
class Edge {
	public:
		Edge() { f_=0; t_=0; }
		Edge(int from, int to) { f_=min(from,to); t_=max(from,to); }
		int f() const { return f_; };
		int t() const { return t_; };
		bool operator<(const Edge &other)  const { return ( f_<other.f() || ( f_==other.f() && t_<other.t() ) ); }
		bool operator==(const Edge &other) const { return ( f_==other.f() && t_==other.t() ); }
		void print() const;
	private:
		int f_, t_;
		int min(int a, int b) { return a<b ? a : b; }
		int max(int a, int b) { return a>b ? a : b; }
};

/// a vertex in an undirected graph
class Vertex {
	public:
		Vertex() { id_=0; }
		Vertex(int id) { id_=id; }
		/// the neighbors to this vertex
		mutable std::set<int> neighbors;
		/// the edges connected to this vertex
		mutable std::set<Edge> edges;
		/// add a neighbor
		void add_neighbor( int id ) const { if (id!=id_) neighbors.insert(id); }
		/// remove a neighbor
		void remove_neighbor( int id ) const { if( neighbors.find(id)!=neighbors.end() ) neighbors.erase(id); }
		/// add an edge reference
		void add_edge( const Edge e ) const {
			edges.insert(e);
			add_neighbor(e.f());
			add_neighbor(e.t());
		}
		/// remove an edge reference
		void remove_edge( const Edge e ) const {
			if( edges.find(e)!=edges.end() ) {
				edges.erase(e);
				remove_neighbor( e.f() );
				remove_neighbor( e.t() );
			}
		}
		/// return the id
		int id() const { return id_; }
		/// less than
		bool operator<(const Vertex &other) const  { return id_ < other.id(); }
		/// equal
		bool operator==(const Vertex &other) const { return id_ == other.id(); }
		/// print the vertex
		void print() const;
	private:
		int id_;
};

/// Graph is an class for manipulating undirected graphs
class Graph {
	public:
		/// standard constructor does nothing
		Graph() { }
		/// initialize a graph from a set of vertex numbers and edge from/to pairs
		Graph(std::vector<int> &from, std::vector<int> &to, std::set<int> &v_set) {
			build(from,to,v_set);
		}
		/// initialize a graph from a set of from and to vertex numbers
		Graph(std::vector<int> &from, std::vector<int> &to) { build(from,to); }
		/// or with standard pointers
		Graph(int *from, int *to, int n) { build(from,to,n); }
		/// build the graph
		void build(std::vector<int> &from, std::vector<int> &to);
		/// build the graph with pointers
		void build(int *from, int *to, int n);
		/// build the graph with a node list also
		void build(std::vector<int> &from, std::vector<int> &to, std::set<int> &v_set);
		/// return a reference to a specified vertex
		const Vertex &V(int i) { return *V_.find(Vertex(i)); }
		/// add vertex
		void add_vertex(int i) { if( V_.find(i)==V_.end() ) V_.insert(i); }
		/// extract a single connected component from the graph
		/// @returns true when there remain more components to extract
		/// @param G is the input graph
		/// @param component is the output component
		bool extract_component( std::set<int> &component );
		/// copy the vertex indeces in the graph to output @param vertices
		void copy_vertices( std::set<int> &vertices );
		/// get the neighbors connected to @param V
		void neighbors( int V, std::set<int> );
		/// find the neighbors connected to a set of nodes, going out the specified distance from the start set
		/// @param start_set (input) gives the set of nodes to start with
		/// @param distance  (input) specifies the distance to go
		/// @param neighbor_set (output) gives the neighbors connected to that set
		void find_neighbors( std::set<int> &start_set, int distance, std::set<int> &neighbor_set, bool remove_start_set=true );
		/// find the neighbors connected to a set of nodes
		void find_neighbors( std::set<int> &start_set, std::set<int> &neighbor_set, bool remove_start_set=true );
		/// find the neighbors connected to a single node
		void find_neighbors( int &start_vertex, int distance, std::set<int> &neighbor_set, bool remove_start_set=true );
		/// merge all of the neighbors in vertex_set into the set
		void merge_neighbors( std::set<int> &vertex_set );
		/// return true if the graph is is fully connected
		bool is_fully_connected();
		/// return the number of subgraphs in the graph
		int n_subgraphs();
		/// find all of the fully connected subgraphs found withtin the graph
		int find_subgraphs( std::vector< std::set<int> > & subgraphs );		
		/// print the graph
		void print(const char *name="none") const;
		/// return the number of vertices in the graph
		unsigned size() const { return V_.size(); }
		/// remove a vertex from the graph
		void remove_vertex( int id ) {
			std::set<Vertex>::iterator v = V_.find(id);
			if( v != V_.end() ) {
				std::set<Edge>::iterator e;
				std::set<Edge> edges = v->edges;
				// erase any memory of the associated edges
				for( e=edges.begin(); e!=edges.end(); e++ ) {
					remove_edge(e->f(),e->t()); // erase the edge
				}
				V_.erase(id); // erase the vertex
			}
		}
		/// remove an edge from the graph
		void remove_edge( int from, int to ) {
			Edge e(from,to);
			std::set<Edge>::iterator iter = E_.find(e);
			if( iter!=E_.end() ) {
				V(from).remove_edge(*iter);
				V(to)  .remove_edge(*iter);
				E_.erase(iter);
			}
		}
		/// return a reference to an edge
		const Edge & E(int f, int t) { return *E_.find(Edge(f,t)); }
		/// clear the graph
		void clear() { V_.clear(); E_.clear(); }
	private:
		/// data members
		std::set<Vertex> V_;
		std::set<Edge>   E_;
		/// insert an edge
		void insert_edge (int from, int to) {
			if ( from != to ) {
				std::set<Edge>::iterator e;
				e = E_.insert ( Edge(from,to) ).first;
				insert_vertex(from,e);
				insert_vertex(to,e);
			}
		}
		/// insert a vertex, return the iterator pointing to the new element
		std::set<Vertex>::iterator insert_vertex ( int id, const std::set<Edge>::iterator &e ) {
			std::set<Vertex>::iterator v = V_.insert ( Vertex(id) ).first;
			v->add_edge(*e);
			return v;
		}
};

/// find connected components in a graph
/// @param from indicates the orgin point for the edges
/// @param to   indicates the destination point for the edges
/// @param components gives the sets of vertices
/// @return value is the number of connected components found
int find_connected( std::vector<int> &from, std::vector<int> &to, std::vector< std::set<int> > &components  );

/// print the contents of a set
void print_set( std::set<int> &a );

/// calculate the differences between two sets return (A - B)
std::set<int> set_difference ( std::set<int> &A, std::set<int> &B );

/// a simple set union function
std::set<int> set_union ( std::set<int> &A, std::set<int> &B );
/// a simple set union function
std::set<int> set_union ( std::set<int> &A, int b );

/// make distance matrix
void distance_matrix ( std::vector<int> &from, std::vector<int> &to, std::vector< std::vector<int> > & DistanceMatrix );

/// is @param a in set B ?
bool is_member( int a, std::set<int> B );

#endif

