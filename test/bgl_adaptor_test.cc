// =============================================================================
// 
//       Filename:  bgl_adaptor_test.cc
// 
//    Description:  Test for boost graph library adaptor
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
#include "../src/bgl_adaptor.hh"

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>
#include <iostream>
#include <map>
#include <cassert>
#include <vector>
#include <deque>

#define PASSED std::cout << "passed\n"


struct vertex_val {
  int color;
  int id;
  
  vertex_val () {}
  vertex_val (int id_) : id(id_) {}
};

void bgl_adaptor_test ()
{
  typedef gtl::graph_t<vertex_val, int, false> G;
  typedef G::vertex_descriptor Vertex;
  typedef G::edge_descriptor Edge;
  G graph;
  Vertex v = boost::add_vertex (vertex_val(1), graph);
  std::pair <Edge, bool> edge_pair = boost::add_edge (v, v, 1, graph);
  Edge e = edge_pair.first;
  
  assert (boost::source (e, graph) == v);
  assert (boost::target (e, graph) == v);
  boost::vertices (graph);
  boost::edges (graph);
  boost::out_edges (v, graph);
  boost::in_edges (v, graph);
  boost::num_vertices (graph);
  boost::num_edges (graph);
  boost::out_degree (v, graph);
  boost::in_degree (v, graph);
  
  std::map <Vertex, int> col_map;
  boost::vertices (graph);
  ::boost::breadth_first_search(graph, v, 
    boost::color_map(boost::std_container_adaptor<std::map<Vertex, int> >(col_map)));
  
  boost::remove_edge (e, graph);
  boost::remove_edge (v, v, graph);
  boost::clear_vertex (v, graph);
  boost::remove_vertex (v, graph);
  
  PASSED;
}


void bgl_topo_sort_test()
{
  typedef gtl::graph_t<const char*, gtl::no_data> G;
  typedef G::vertex_descriptor Vertex;
  G graph;
  
  std::vector<Vertex> vert(7);
  vert[0] = boost::add_vertex ((const char*)"pick up kids from school", graph);
  vert[1] = boost::add_vertex ((const char*)"buy groceries (and snacks)", graph);
  vert[2] = boost::add_vertex ((const char*)"get cash at ATM", graph);
  vert[3] = boost::add_vertex ((const char*)"drop off kids at soccer practice", graph);
  vert[4] = boost::add_vertex ((const char*)"cook dinner", graph);
  vert[5] = boost::add_vertex ((const char*)"pick up kids from soccer", graph);
  vert[6] = boost::add_vertex ((const char*)"eat dinner", graph);
  
  boost::add_edge (vert[0], vert[3], graph);
  boost::add_edge (vert[1], vert[3], graph);
  boost::add_edge (vert[1], vert[4], graph);
  boost::add_edge (vert[2], vert[1], graph);
  boost::add_edge (vert[3], vert[5], graph);
  boost::add_edge (vert[4], vert[6], graph);
  boost::add_edge (vert[5], vert[6], graph);
  
  std::map <Vertex, int> col_map;
  std::deque<Vertex> topo_order;
  
  boost::topological_sort (graph, std::front_inserter (topo_order),
    boost::color_map(boost::std_container_adaptor<std::map<Vertex, int> >(col_map)));
  
  int n = 1;
  for (std::deque<Vertex>::iterator it = topo_order.begin();
       it != topo_order.end(); ++it, ++n)
    std::cout << n << ": " << **it << std::endl;
}


int main()
{
  bgl_adaptor_test ();
  bgl_topo_sort_test ();
  std::cout << "All the tests are passed!\n";
  
}
