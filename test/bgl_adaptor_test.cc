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
#include "../src/property_map.hh"

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <map>
#include <cassert>
#include <vector>
#include <deque>

// #include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/filtered_graph.hpp>

#define PASSED std::cout << "passed\n"
#define foreach BOOST_FOREACH


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
  boost::breadth_first_search(graph, v, 
    boost::color_map(boost::associative_property_map<std::map<Vertex, int> >(col_map)));
  
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
  
  gtl::property_map_external_t<Vertex, gtl::default_color_t> col_map;
  std::deque<Vertex> topo_order;
  
  boost::topological_sort (graph, std::front_inserter (topo_order),
    boost::color_map(boost::make_gtl_map_adaptor(col_map)));
  
  int n = 1;
  for (std::deque<Vertex>::iterator it = topo_order.begin();
       it != topo_order.end(); ++it, ++n)
    std::cout << n << ": " << **it << std::endl;
  PASSED;
}

struct dijkstra_edge
{
  int id;
  int weight;
  
  dijkstra_edge (int id, int w) : id(id), weight(w) {}
};

struct dijkstra_vertex
{
  int id;
  int distance;
  gtl::default_color_t color;
  
  dijkstra_vertex (int id) : id(id) {}
};


void bgl_dijkstra_test ()
{
  typedef gtl::graph_t<dijkstra_vertex, dijkstra_edge> G;
  typedef G::vertex_descriptor Vertex;
  typedef G::edge_descriptor Edge;
  G graph;
  
  std::vector<Vertex> vert(4);
  vert[0] = graph.add_vertex (dijkstra_vertex(0));
  vert[1] = graph.add_vertex (dijkstra_vertex(1));
  vert[2] = graph.add_vertex (dijkstra_vertex(2));
  vert[3] = graph.add_vertex (dijkstra_vertex(3));
  
  graph.add_edge (vert[0], vert[1], dijkstra_edge(0, 3));
  graph.add_edge (vert[0], vert[2], dijkstra_edge(1, 2));
  graph.add_edge (vert[1], vert[3], dijkstra_edge(2, 2));
  graph.add_edge (vert[2], vert[3], dijkstra_edge(3, 1));
  graph.add_edge (vert[1], vert[2], dijkstra_edge(4, 5));
  
  gtl::property_map_internal_t<Edge, int> weight(&dijkstra_edge::weight);
  gtl::property_map_internal_t<Vertex, int> index(&dijkstra_vertex::id);
  gtl::property_map_external_t<Vertex, Vertex> predecessor;
  gtl::property_map_internal_t<Vertex, int> distance(&dijkstra_vertex::distance);
  gtl::property_map_internal_t<Vertex, gtl::default_color_t> 
    color(&dijkstra_vertex::color);

  boost::dijkstra_shortest_paths (graph, vert[0], 
    boost::weight_map(boost::make_gtl_map_adaptor(weight)).
    vertex_index_map(boost::make_gtl_map_adaptor(index)).
    predecessor_map(boost::make_gtl_map_adaptor(predecessor)).
    distance_map(boost::make_gtl_map_adaptor(distance)).
    color_map(boost::make_gtl_map_adaptor(color)));
  
  foreach (Vertex v, graph.vertices())
    std::cout << "Vertex " << v->id 
      << ") distance: " << v->distance
      << ", predecessor: " << predecessor[v]->id << std::endl;
  PASSED;
}

// struct flow_edge
// {
//   int id;
//   int flow;
//   int cost;
//   int capacity;
//   int reduced_cost;
// 
//   flow_edge() {};
//   flow_edge(int id, int cost, int capacity) 
//     : id(id), flow(0), cost(cost), capacity(capacity), reduced_cost(0) {};
// };
// 
// struct flow_vertex
// {
//   int id;
//   int need;
//   int potential;
//   int color;
// 
//   flow_vertex() {};
//   flow_vertex(int id, int need) : id(id), need(need), potential(0) {};
// };
// 
// template <typename Graph>
// struct flow_edge_filter
// {
//   const Graph* g;
// 
//   flow_edge_filter (const Graph& g) : g(&g) {};
//   flow_edge_filter () : g(NULL) {};
// 
//   template <typename Edge>
//   bool operator() (Edge e) const {
//     return (*g)[e].flow < (*g)[e].capacity && (*g)[e].reduced_cost == 0;
//   }
// };
// 
// template <typename Graph, typename PredecessorMap>
// class flow_visitor
//   : public boost::base_visitor<flow_visitor<Graph, PredecessorMap> >
// {
//   typedef typename Graph::vertex_descriptor Vertex;
//   typedef typename Graph::edge_descriptor Edge;
// 
// public:
//   flow_visitor (PredecessorMap& pred, Vertex& target, bool& reached) 
//     : pred(pred), target(target), reached(reached) {};
//   
//   void tree_edge (Edge e, Graph& g) {
//     Vertex t = boost::target(e, g);
//     boost::put (pred, t, e);
//     if (g[t].need > 0) {
//       reached = true;
//       target = t;
//     }
//   }
//  
//   PredecessorMap& pred;
//   Vertex& target;
//   bool& reached;
// };
// 
// void minimum_cost_maximum_flow_test ()
// {
//   typedef boost::adjacency_list<boost::vecS, boost::vecS, 
//                                 boost::bidirectionalS, flow_vertex, 
//                                 flow_edge> G;
//   typedef G::vertex_descriptor Vertex;
//   typedef G::edge_descriptor Edge;
// 
//   G graph(6);
//   boost::add_vertex (flow_vertex(0, -7), graph);
//   boost::add_vertex (flow_vertex(1, 0), graph);
//   boost::add_vertex (flow_vertex(2, -3), graph);
//   boost::add_vertex (flow_vertex(3, 6), graph);
//   boost::add_vertex (flow_vertex(4, 0), graph);
//   boost::add_vertex (flow_vertex(5, 4), graph);
// 
//   boost::add_edge (boost::vertex(0, graph), 
//                    boost::vertex(1, graph), 
//                    flow_edge(0, 5, 8), graph);
//   boost::add_edge (boost::vertex(0, graph), 
//                    boost::vertex(2, graph), 
//                    flow_edge(1, 3, 6), graph);
//   boost::add_edge (boost::vertex(1, graph), 
//                    boost::vertex(2, graph), 
//                    flow_edge(2, 6, 5), graph);
//   boost::add_edge (boost::vertex(2, graph),
//                    boost::vertex(3, graph), 
//                    flow_edge(3, 1, 6), graph);
//   boost::add_edge (boost::vertex(2, graph), 
//                    boost::vertex(4, graph), 
//                    flow_edge(4, 4, 7), graph);
//   boost::add_edge (boost::vertex(4, graph), 
//                    boost::vertex(1, graph), 
//                    flow_edge(5, 1, 4), graph);
//   boost::add_edge (boost::vertex(3, graph), 
//                    boost::vertex(5, graph),
//                    flow_edge(6, 2, 3), graph);
//   boost::add_edge (boost::vertex(4, graph), 
//                    boost::vertex(5, graph), 
//                    flow_edge(7, 12, 5), graph);
//   
//   foreach (Edge e, boost::edges(graph)) {
//     graph[e].reduced_cost = graph[e].cost;
//     graph[e].flow = 0;
//   }
// 
//   std::deque<Vertex> sources;
//   foreach (Vertex v, boost::vertices(graph)) {
//     if (graph[v].need < 0)
//       sources.push_back(v);
//   }
// 
//   boost::filtered_graph<G, flow_edge_filter<G> > 
//     filtered_g (graph, flow_edge_filter<G>(graph));
//   typedef std::vector<Edge> pmap_t;
//   pmap_t predecessor;
//   
//   bool finded = false;
//   Vertex target;
//   std::deque<Edge> cut_edges;
//   while (!sources.empty()) {
//     do {
//       finded = false;
//       boost::breadth_first_search (filtered_g, sources.front(), 
//         boost::color_map(get (&flow_vertex::color, graph)).
//         visitor(boost::make_bfs_visitor(
//           flow_visitor<G, pmap_t>(predecessor, target, finded))));
//       if (finded) {
//         std::cout << "Found\n";
//         int max_increase = -graph[target].need;
//         for (Edge e = predecessor[target]; 
//              source(e, graph) != sources.front();
//              e = predecessor[source(e, graph)])
//         {
//           int residual = graph[e].capacity - graph[e].flow;
//           max_increase = std::min (max_increase, residual);
//           cut_edges.push_back(e);
//         }
//         foreach (Edge e, cut_edges)
//           graph[e].flow += max_increase;
//         cut_edges.clear();
//         if (max_increase == sources.front())
//           sources.pop_front();
//       }
//       else {
//         std::cout << "Not found\n";
//         int max_increase = std::numeric_limits<int>::max();
//         foreach (Edge e, boost::edges(graph))
//           if (graph[boost::source(e, graph)].color !=
//               graph[boost::target(e, graph)].color) {
//             cut_edges.push_back (e);
//             if (graph[e].flow == 0)
//               max_increase = std::min (max_increase, graph[e].reduced_cost);
//           }
//         assert (max_increase < std::numeric_limits<int>::max());
//         foreach (Edge e, cut_edges)
//           graph[e].reduced_cost -= max_increase;
//         cut_edges.clear();
//         finded = false;
//       }
//     } while (finded);
//   }
//   
// }


int main()
{
  std::cout << "BGL adaptor test\n";
  bgl_adaptor_test ();
  std::cout << "Topological sort\n";
  bgl_topo_sort_test ();
  std::cout << "Dijkstra shortest paths\n";
  bgl_dijkstra_test ();
//  std::cout << "Minimum-cost maximum-flow\n";
//  minimum_cost_maximum_flow_test ();
  std::cout << "All the tests are passed!\n";
  
  return EXIT_SUCCESS;
}
