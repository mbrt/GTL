// =============================================================================
// 
//       Filename:  graph_test.cc
// 
//    Description:  Graph tests
//
//         Author:  Michele Bertasi 
//        Contact:  michele.bertasi $at$ gmail.com
//      Copyright:  Copyright (c) 2010, Michele Bertasi
//        Company:  University of Verona - ESD Group
//        License:  GNU Lesser General Public License (GNU LGPL)
//
//      Agreement:                
//       This file is part of 'GTL'.
//       'GTL' is free software: you can redistribute it and/or
//       modify it under the terms of the GNU Lesser General Public License 
//       as published by the Free Software Foundation, either version 3 of 
//       the License, or (at your option) any later version.
//
//       'GTL' is distributed in the hope that it will be useful,
//       but WITHOUT ANY WARRANTY; without even the implied warranty of
//       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//       GNU Lesser General Public License for more details.
//
//       You should have received a copy of the GNU Lesser General Public 
//       License along with 'GTL'. 
//       If not, see <http://www.gnu.org/licenses/>.
// 
// =============================================================================

#include "../src/graph.hh"

#include <cassert>
#include <boost/foreach.hpp>
#include <iostream>
#include <map>
#include <tr1/functional>
#include <tr1/unordered_set>
//#include <boost/graph/adjacency_list.hpp>


#define foreach BOOST_FOREACH
#define PASSED std::cout << "passed\n"

template <typename EdgeDescriptor, int num>
struct edge_predicate : std::unary_function<EdgeDescriptor, bool> {
  bool operator() (EdgeDescriptor e) {
    return *e == num;
  }
};

struct Foo {
  explicit Foo (int) {}
};

void multigraph_test()
{
#if 1
  typedef gtl::graph_t<int, int, true> G;
  typedef G::vertex_descriptor Vertex;
  typedef G::edge_descriptor Edge;
  
  G graph;
#endif

#if 1
  std::cout << "Inserting a vertex\n";
  Vertex v1 = graph.add_vertex(1);
  assert (*v1 == 1);
  PASSED;
  
  std::cout << "Iterating vertices\n";
  int count = 0;
  foreach (Vertex v, graph.vertices()) {
    assert (*v == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
 
  std::cout << "Inserting edges\n";
  Vertex v2 = graph.add_vertex(2);  
  std::pair<Edge, bool> inserted_pair = graph.add_edge (v1, v2, 1);
  assert (inserted_pair.second && *inserted_pair.first == 1);
  Edge e1 = inserted_pair.first;
  PASSED;
  
  std::cout << "Iterating edges\n";
  count = 0;
  foreach (Edge e, graph.edges()) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Iterating out edges\n";
  count = 0;
  foreach (Edge e, graph.out_edges(v1)) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Iterating in edges\n";
  count = 0;
  foreach (Edge e, graph.in_edges(v2)) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Inserting an already inserted edge\n";
  inserted_pair = graph.add_edge (v1, v2, 2);
  assert (inserted_pair.second && *inserted_pair.first == 2);
  PASSED;
  
  std::cout << "Iterating out edges\n";
  count = 0;
  foreach (Edge e, graph.out_edges(v1))
  {
    assert (*e == 1 || *e == 2);
    assert (graph.source(e) == v1 && graph.target(e) == v2);
    ++count;
  }
  assert (count == 2);
  PASSED;
  
  std::cout << "Iterating in edges\n";
  count = 0;
  foreach (Edge e, graph.in_edges(v2)) {
    assert (*e == 1 || *e == 2);
    ++count;
  }
  assert (count = 2);
  
  foreach (Edge e, graph.in_edges(v1)) {
    *e;
    assert (false && "NOT REACHABLE");
  }
  PASSED;
  
  std::cout << "In and out degrees\n";
  assert (graph.in_degree(v1) == 0 && graph.in_degree(v2) == 2);
  assert (graph.out_degree(v1) == 2 && graph.out_degree(v2) == 0);
  PASSED;
  
  std::cout << "Vertices and edges num\n";
  assert (graph.num_vertices() == 2 && graph.num_edges() == 2);
  PASSED;
  
  std::cout << "Edge between v1 and v2\n";
  inserted_pair = graph.edge (v1, v2);
  int val = *inserted_pair.first;
  assert (inserted_pair.second && (val == 1 || val == 2));
  PASSED;
  
  std::cout << "Edge range\n";
  count = 0;
  foreach (Edge e, graph.edge_range (v1, v2)) {
    assert (*e == 1 || *e == 2);
    ++count;
  }
  assert (count == 2);
  foreach (Edge e, graph.edge_range (v2, v1)) {
    assert (false && "Not reachable!");
    *e;
  }
  PASSED;
  
  std::cout << "Remove edge\n";
  graph.remove_edge (e1);
  assert (graph.num_edges() == 1);
  foreach (Edge e, graph.edge_range (v1, v2)) {
    assert (*e == 2);
  }
  PASSED;
  
  std::cout << "Remove edges between two vertices\n";
  graph.add_edge (v1, v2, 2);
  size_t removed = graph.remove_edge (v1, v2);
  assert (removed == 2);
  assert (graph.num_edges() == 0);
  foreach (Edge e, graph.edge_range (v1, v2)) {
    assert (false && "Not reachable!");
    *e;
  }
  PASSED;
  
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v2, 2);
  graph.add_edge (v1, v2, 3);
  graph.add_edge (v2, v1, 4);
    
  std::cout << "Clear out edges\n";
  graph.clear_out_edges (v1);
  foreach (Edge e, graph.out_edges (v1)) {
    assert (false && "Not reachable!");
    *e;
  }
  assert (graph.num_edges() == 1);
  PASSED;
  
  std::cout << "Clear in edges\n";
  graph.clear_in_edges (v1);
  assert (graph.num_edges() == 0);
  PASSED;
  
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v2, 2);
  graph.add_edge (v1, v2, 3);
  graph.add_edge (v2, v1, 4);
  
  std::cout << "Clear vertex\n";
  graph.clear_vertex (v1);
  foreach (Edge e, graph.edges ()) {
    assert (false && "Not reachable!");
    *e;
  }
  assert (graph.num_edges() == 0);
  PASSED;
  
  std::cout << "Remove vertex\n";
  graph.remove_vertex (v1);
  count = 0;
  foreach (Vertex e, graph.vertices()) {
    assert (*e == 2);
    ++count;
  }
  assert (count == 1 && graph.num_vertices() == 1);
  PASSED;
  
  std::cout << "Remove edge if\n";
  v1 = graph.add_vertex (1);
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v2, 2);
  graph.add_edge (v1, v2, 3);
  graph.add_edge (v2, v1, 4);
  graph.remove_edge_if (edge_predicate<Edge,1>());
  foreach (Edge e, graph.edges())
    assert (*e != 1);
  assert (graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Remove out edge if\n";
  graph.add_edge (v1, v2, 1);
  graph.remove_out_edge_if (v1, edge_predicate<Edge,2>());
  foreach (Edge e, graph.out_edges(v1))
    assert (*e != 2);
  assert (graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Remove in edge if\n";
  graph.add_edge (v1, v2, 2);
  graph.remove_in_edge_if (v2, edge_predicate<Edge,3>());
  foreach (Edge e, graph.in_edges(v2))
    assert (*e != 3);
  assert (graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Remove vertex if\n";
  graph.remove_vertex_if (edge_predicate<Vertex,1>());
  foreach (Vertex v, graph.vertices())
    assert (*v != 1);
  assert (graph.num_vertices() == 1);
  assert (graph.num_edges() == 0);
  PASSED;
  
  std::cout << "Changing the value stored in the vertices\n";
  assert (*v2 == 2);
  *v2 = 3;
  assert (*v2 == 3);
  PASSED;
  
  std::cout << "Iterating adjacent vertices\n";
  graph.clear();
  v1 = graph.add_vertex(1);
  v2 = graph.add_vertex(2);
  Vertex v3 = graph.add_vertex(3);
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v3, 2);
  graph.add_edge (v1, v2, 3);
  graph.add_edge (v1, v2, 4);
  count = 0;
  foreach (Vertex v, graph.adjacent_vertices (v1)) {
    ++count;
    assert (*v == 2 || *v == 3);
  }
  assert (count == 2);
  PASSED;
  
  std::pair<G::adjacency_iterator, G::adjacency_iterator> adj_pair
    = graph.adjacent_vertices(v1);
  G::adjacency_iterator adj1, adj2, adj3;
  adj1 = adj3 = adj_pair.first;
  adj2 = adj_pair.second;
  ++adj3;
  adj3--;
  assert (*adj1 == *adj3 && adj1 != adj3);

#endif
}



void graph_test ()
{
  #if 1
  typedef gtl::graph_t<int, int, false> G;
  typedef G::vertex_descriptor Vertex;
  typedef G::edge_descriptor Edge;
  
  G graph;
#endif

#if 1
  std::cout << "Inserting a vertex\n";
  Vertex v1 = graph.add_vertex(1);
  assert (*v1 == 1);
  PASSED;
  
  std::cout << "Iterating vertices\n";
  int count = 0;
  foreach (Vertex v, graph.vertices()) {
    assert (*v == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
 
  std::cout << "Inserting edges\n";
  Vertex v2 = graph.add_vertex(2);  
  std::pair<Edge, bool> inserted_pair = graph.add_edge (v1, v2, 1);
  assert (inserted_pair.second && *inserted_pair.first == 1);
  Edge e1 = inserted_pair.first;
  PASSED;
  
  std::cout << "Iterating edges\n";
  count = 0;
  foreach (Edge e, graph.edges()) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Iterating out edges\n";
  count = 0;
  foreach (Edge e, graph.out_edges(v1)) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Iterating in edges\n";
  count = 0;
  foreach (Edge e, graph.in_edges(v2)) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Inserting an already inserted edge\n";
  inserted_pair = graph.add_edge (v1, v2, 2);
  assert (!inserted_pair.second);
  assert (graph.num_edges() == 1);
  PASSED;
  
  std::cout << "Iterating out edges\n";
  Vertex v3 = graph.add_vertex (3);
  inserted_pair = graph.add_edge (v1, v3, 2);
  Edge e2 = inserted_pair.first;
  assert (inserted_pair.second && *e2 == 2);
  assert (*e1 == 1 && *e2 == 2);
  count = 0;
  foreach (Edge e, graph.out_edges(v1))
  {
    assert (*e == 1 || *e == 2);
    assert (graph.source(e) == v1);
    assert (graph.target(e) == v2 || graph.target(e) == v3);
    ++count;
  }
  assert (count == 2);
  PASSED;
  
  std::cout << "Iterating in edges\n";
  graph.add_edge (v3, v2, 3);
  count = 0;
  foreach (Edge e, graph.in_edges(v2)) {
    assert (*e == 1 || *e == 3);
    ++count;
  }
  assert (count == 2);
  
  foreach (Edge e, graph.in_edges(v1)) {
    *e;
    assert (false && "NOT REACHABLE");
  }
  PASSED;
  
  std::cout << "In and out degrees\n";
  assert (graph.in_degree(v1) == 0 && graph.in_degree(v2) == 2 &&
          graph.in_degree(v3) == 1);
  assert (graph.out_degree(v1) == 2 && graph.out_degree(v2) == 0 &&
          graph.out_degree(v3) == 1);
  PASSED;
  
  std::cout << "Vertices and edges num\n";
  assert (graph.num_vertices() == 3 && graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Edge between v1 and v2\n";
  inserted_pair = graph.edge (v1, v2);
  int val = *inserted_pair.first;
  assert (inserted_pair.second && val == 1);
  PASSED;
  
  std::cout << "Edge range\n";
  count = 0;
  foreach (Edge e, graph.edge_range (v1, v2)) {
    assert (*e == 1);
    ++count;
  }
  assert (count == 1);
  foreach (Edge e, graph.edge_range (v2, v1)) {
    assert (false && "Not reachable!");
    *e;
  }
  PASSED;
  
  std::cout << "Remove edge\n";
  assert (graph.source(e2) == v1 && graph.target(e2) == v3);
  assert (*e1 == 1 && *e2 == 2);
  graph.remove_edge (e1);
  assert (graph.num_edges() == 2 && graph.out_degree(v1) == 1);
  count = 0;
  foreach (Edge e, graph.out_edges (v1)) {
    assert (*e == 2);
    ++count;
  }
  assert (count == 1);
  PASSED;
  
  std::cout << "Remove edges between two vertices\n";
  graph.add_edge (v1, v2, 2);
  size_t removed = graph.remove_edge (v1, v2);
  assert (removed == 1);
  assert (graph.num_edges() == 2);
  foreach (Edge e, graph.edge_range (v1, v2)) {
    assert (false && "Not reachable!");
    *e;
  }
  PASSED;
  
  graph.add_edge (v1, v2, 1);
  std::cout << "Clear out edges\n";
  graph.clear_out_edges (v1);
  foreach (Edge e, graph.out_edges (v1)) {
    assert (false && "Not reachable!");
    *e;
  }
  assert (graph.num_edges() == 1);
  PASSED;
  
  std::cout << "Clear in edges\n";
  graph.add_edge (v2, v1, 4);
  graph.clear_in_edges (v1);
  assert (graph.num_edges() == 1);
  PASSED;
  
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v3, 2);
  graph.add_edge (v2, v1, 4);
  
  std::cout << "Clear vertex\n";
  graph.clear_vertex (v1);
  foreach (Edge e, graph.edges ()) {
    assert (*e == 3);
  }
  assert (graph.num_edges() == 1);
  PASSED;
  
  std::cout << "Remove vertex\n";
  graph.remove_vertex (v1);
  count = 0;
  foreach (Vertex v, graph.vertices()) {
    assert (*v == 2 || *v == 3);
    ++count;
  }
  assert (count == 2 && graph.num_vertices() == 2);
  PASSED;
  
  std::cout << "Remove edge if\n";
  v1 = graph.add_vertex (1);
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v3, 2);
  graph.add_edge (v3, v2, 3);
  graph.add_edge (v2, v1, 4);
  graph.remove_edge_if (edge_predicate<Edge,1>());
  foreach (Edge e, graph.edges())
    assert (*e != 1);
  assert (graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Remove out edge if\n";
  graph.add_edge (v1, v2, 1);
  graph.remove_out_edge_if (v1, edge_predicate<Edge,2>());
  foreach (Edge e, graph.out_edges(v1))
    assert (*e != 2);
  assert (graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Remove in edge if\n";
  graph.add_edge (v1, v3, 2);
  graph.remove_in_edge_if (v2, edge_predicate<Edge,3>());
  foreach (Edge e, graph.in_edges(v2))
    assert (*e != 3);
  assert (graph.num_edges() == 3);
  PASSED;
  
  std::cout << "Remove vertex if\n";
  graph.remove_vertex_if (edge_predicate<Vertex,1>());
  foreach (Vertex v, graph.vertices())
    assert (*v != 1);
  assert (graph.num_vertices() == 2);
  assert (graph.num_edges() == 0);
  PASSED;
  
  std::cout << "Changing the value stored in the vertices\n";
  assert (*v2 == 2);
  *v2 = 3;
  assert (*v2 == 3);
  PASSED;
  
  std::cout << "Iterating adjacent vertices\n";
  graph.clear();
  v1 = graph.add_vertex(1);
  v2 = graph.add_vertex(2);
  v3 = graph.add_vertex(3);
  Vertex v4 = graph.add_vertex(4);
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v1, v3, 2);
  graph.add_edge (v1, v4, 3);
  count = 0;
  foreach (Vertex v, graph.adjacent_vertices (v1)) {
    ++count;
    assert (*v == 2 || *v == 3 || *v == 4);
  }
  assert (count == 3);
  PASSED;
  
  std::cout << "Hash map test\n";
  std::tr1::unordered_set <Vertex, G::vertex_hash> vertex_hash_map;
  vertex_hash_map.insert (v1);
  assert (sizeof(int*) == sizeof(size_t));
  PASSED;
  
#endif
}


int main ()
{ 
  std::cout << "MULTIGRAPH\n";
  multigraph_test();
  std::cout << "GRAPH WITH NO PARALLEL EDGES\n";
  graph_test();
  
  // NO DATA GRAPH  
  typedef gtl::graph_t<gtl::no_data, gtl::no_data, false> NDG;
  NDG no_data_graph;
  NDG::vertex_descriptor ndv = no_data_graph.add_vertex();
  no_data_graph.add_edge (ndv, ndv);
  
  std::cout << "All the tests were passed\n";
  return 0;
  
//  // BOOST GRAPH
//  typedef boost::adjacency_list<boost::multisetS, boost::listS, boost::bidirectionalS> BG;
//  typedef boost::graph_traits<BG>::vertex_descriptor BVertex;
//  typedef boost::graph_traits<BG>::edge_descriptor BEdge;
//  typedef boost::graph_traits<BG>::adjacency_iterator BAdjIt;
//  BG bg;
//  BVertex bv1 = boost::add_vertex(bg);
//  BVertex bv2 = boost::add_vertex(bg);
//  BVertex bv3 = boost::add_vertex(bg);
//  std::pair<BEdge, bool> bins_pair = boost::add_edge(bv1, bv2, bg);
//  BEdge be1 = bins_pair.first;
//  bins_pair = boost::add_edge(bv1, bv3, bg);
//  BEdge be2 = bins_pair.first;
//  bins_pair = boost::add_edge(bv1, bv2, bg);
//  BEdge be3 = bins_pair.first;
//  std::pair<BAdjIt, BAdjIt> adj_pair = boost::adjacent_vertices (bv1, bg);
//  BAdjIt adit = adj_pair.first;
//  assert (*adit == bv2);
//  ++adit;
//  assert (*adit == bv2);
//  ++adit;
//  assert (*adit == bv3);
//  ++adit;
//  assert (adit == adj_pair.second);
//  
//  boost::adjacent_vertices (bv1, bg);
  
}
