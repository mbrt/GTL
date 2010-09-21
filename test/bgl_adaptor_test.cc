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
  G graph;
  Vertex v = graph.add_vertex ();
  
  std::map <Vertex, int> color_map;
  boost::vertices (graph);
  boost::breadth_first_search(graph, v, 
    boost::color_map(boost::std_container_adaptor<std::map<Vertex, int> >(color_map)));
}


int main()
{
  bgl_adaptor_test ();
  
}
