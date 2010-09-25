// =============================================================================
// 
//       Filename:  algorithm_test.cc
// 
//    Description:  Graph algorithm tests
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
#include "../src/graph_algorithm.hh"

#include <cassert>
#include <iostream>

#define PASSED std::cout << "passed\n"

struct vertex_val {
  gtl::default_color_t color;
  int id;
  
  vertex_val () {}
  vertex_val (int id_) : id(id_) {}
};

struct my_bfs_visitor : public gtl::bfs_visitor<gtl::graph_t<vertex_val, int, false> >
{
  void discover_vertex (Vertex u, Graph&) {
    int id = u->id;
    assert (id != 3);
  }
  
  void tree_edge (Edge e, Graph&) {
    assert (*e == 1);
  }
  
  void gray_target (Edge e, Graph& g) {
    assert (*e == 1 && g.target(e)->id == 1);
  }
  
  void finish_vertex (Vertex v, Graph&) {
    std::cout << "Color of: " << v->id << " = " << v->color << std::endl;
    assert (v->color == gtl::color_traits<gtl::default_color_t>::black());
  }
  
};

struct printer_bfs_visitor : public gtl::bfs_visitor<gtl::graph_t<vertex_val, int, false> >
{
  void discover_vertex (Vertex u, Graph&) {
    std::cout << "Discover vertex " << u->id << std::endl;
  }
};


void algorithms_test () 
{
  typedef gtl::graph_t<vertex_val, int, false> G;
  typedef G::vertex_descriptor Vertex;
  G graph;
  
  std::cout << "Color map\n";
  gtl::property_map_external_t<Vertex, gtl::default_color_t> col_map;
  typedef gtl::color_traits<gtl::default_color_t> Color;
  Vertex v1 = graph.add_vertex (1);
  col_map.put (v1, Color::black());
  assert (col_map.get (v1) == Color::black());
  PASSED;
  
  std::cout << "BFS visit\n";
  Vertex v2 = graph.add_vertex (2);
  Vertex v3 = graph.add_vertex (3);
  graph.add_edge (v1, v2, 1);
  graph.add_edge (v2, v1, 2);
  
  my_bfs_visitor my_v;
  printer_bfs_visitor printer_v;
  gtl::property_map_internal_t<Vertex, gtl::default_color_t> 
    color_map (&vertex_val::color);
  gtl::property_map_external_t<Vertex, Vertex> pred_map;
  gtl::property_map_external_t<Vertex, size_t> dist_map;
  gtl::property_map_external_t<Vertex, size_t> dtime_map, ftime_map, time_map;
  
  gtl::breadth_first_search (graph, v1, 
    gtl::make_bfs_visitor (my_v, printer_v, 
      gtl::record_bfs_predecessors(graph, pred_map),
      gtl::record_bfs_distances(graph, dist_map),
      gtl::stamp_bfs_times(graph, dtime_map, ftime_map)),
    color_map);
  
  assert (color_map.get(v1) == Color::black());
  assert (color_map.get(v2) == Color::black());
  assert (color_map.get(v3) == Color::white());
  assert (pred_map.get(v2) == v1);
  assert (dist_map.get(v1) == 0);
  assert (dist_map.get(v2) == 1);
  assert (dist_map.get(v3) == std::numeric_limits<size_t>::max());
  
  std::cout << "v1 time: [" << dtime_map.get(v1) << ", " << ftime_map.get(v1) << "]\n";
  std::cout << "v2 time: [" << dtime_map.get(v2) << ", " << ftime_map.get(v2) << "]\n";
  std::cout << "v3 time: [" << dtime_map.get(v3) << ", " << ftime_map.get(v3) << "]\n";
  PASSED;
  
  std::cout << "Color map internal\n";
  gtl::default_color_t vertex_val::*pointer;
  pointer = &vertex_val::color;
  (*v1).*pointer = Color::green();
  assert (v1->color == Color::green());
  PASSED;
  
  std::cout << "DFS visit\n";
  gtl::depth_first_search (graph, 
    gtl::make_dfs_visitor (gtl::record_dfs_predecessors(graph, pred_map), 
                           gtl::record_dfs_distances(graph, dist_map),
                           gtl::stamp_dfs_times (graph, dtime_map, ftime_map)),
    color_map);
  
  assert (pred_map.get(v1) == v1);
  assert (pred_map.get(v2) == v1);
  assert (color_map.get(v1) == Color::black());
  assert (color_map.get(v2) == Color::black());
  assert (color_map.get(v3) == Color::black());
  std::cout << "v1 time: [" << dtime_map.get(v1) << ", " << ftime_map.get(v1) << "]\n";
  std::cout << "v2 time: [" << dtime_map.get(v2) << ", " << ftime_map.get(v2) << "]\n";
  std::cout << "v3 time: [" << dtime_map.get(v3) << ", " << ftime_map.get(v3) << "]\n";
  PASSED;
}


int main ()
{
  std::cout << "ALGORITHMS TEST\n";
  algorithms_test();
  std::cout << "All the tests are passed\n";
  return 0;
}
