// =============================================================================
// 
//       Filename:  graph_algorithms.hh
// 
//    Description:  A template set of algorithms for graph classes
//
//         Author:  Michele Bertasi 
//        Contact:  michele.bertasi@gmail.com
//      Copyright:  Copyright (c) 2009, Giuseppe Di Guglielmo
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
#include <iostream>
#include <map>
#include <cassert>

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
  
  std::map <Vertex, int> color_map;
  boost::vertices (graph);
  boost::breadth_first_search(graph, v, 
    boost::color_map(boost::std_container_adaptor<std::map<Vertex, int> >(color_map)));
  
  boost::remove_edge (e, graph);
  boost::remove_edge (v, v, graph);
  boost::clear_vertex (v, graph);
  boost::remove_vertex (v, graph);
  
}


int main()
{
  bgl_adaptor_test ();
  
}
