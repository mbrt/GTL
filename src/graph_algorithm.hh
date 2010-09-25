// =============================================================================
// 
//       Filename:  graph_algorithm.hh
// 
//    Description:  A template set of algorithms for graph classes
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

#ifndef GTL_GRAPH_ALGORITHM_HH
#define GTL_GRAPH_ALGORITHM_HH

#include <tr1/tuple>
#include <queue>
#include <limits>
#include <stack>

#include "property_map.hh"
#include "visitor.hh"


namespace gtl {


/// Breadth First Search Algorithm (Cormen, Leiserson, and Rivest p. 470)
///
/// @tparam Graph the graph type
/// @tparam BFSVisitor the visitor used uring the visit. This class must match
///  the bfs_visitor interface
/// @tparam ColorMap the map that associates a vertex to a color. As default
///  is used the external color map, that mantain an hash-table from vertex to
///  colors. If in the vertices data is present a field named color maybe you
///  want use an user-define map
/// @tparam Queue the queue used during the visit
/// 
/// @param g the graph
/// @param s the start vertex
/// @param vis the visitor used (a copy). This class must match the bfs_visitor 
///  interface. If you want to use a reference to a visitor use std::ref 
///  utility function.
/// @param color the color map, that associate a vertex to a color. This class
///  must match a property_map interface that associate a vertex descriptor with
///  a color (for example default_color_t).
/// @param q the queue used during the visit
template <typename Graph, 
          typename BFSVisitor,           
          typename ColorMap, 
          typename Queue> 
void 
breadth_first_search (Graph& g, 
                      typename Graph::vertex_descriptor s, 
                      BFSVisitor vis, 
                      ColorMap& color_map,
                      Queue q)
{
  typedef typename Graph::vertex_descriptor Vertex;
  typedef typename Graph::edge_descriptor Edge;
  typedef typename ColorMap::value_type ColorValue;
  typedef color_traits<ColorValue> Color;

  typename Graph::vertex_iterator it, end;
  typename Graph::out_edge_iterator ei, ei_end;
  
  std::tr1::tie(it, end) = g.vertices();
  for (; it != end; ++it) {
    vis.initialize_vertex (*it, g);   // visitor
    color_map.put (*it, Color::white());
  }
  vis.start_vertex (s, g);            // visitor
  color_map.put (s, Color::gray());
  vis.discover_vertex (s, g);         // visitor
  q.push (s);
  while (! q.empty()) {
    Vertex u = q.front();
    q.pop();
    vis.examine_vertex (u, g);        // visitor
    std::tr1::tie(ei, ei_end) = g.out_edges (u);
    for (; ei != ei_end; ++ei) {
      Edge e = *ei;
      Vertex v = g.target (e);
      vis.examine_edge (e, g);        // visitor
      ColorValue v_color = color_map.get(v);
      if (v_color == Color::white()) {
        vis.tree_edge (e, g);         // visitor
        color_map.put (v, Color::gray());
        vis.discover_vertex (v, g);   // visitor
        q.push (v);
      }
      else {
        vis.non_tree_edge (e, g);     // visitor
        if (v_color == Color::gray())
          vis.gray_target (e, g);     // visitor
        else
          vis.black_target (e, g);    // visitor
      }
    } // end for
    color_map.put (u, Color::black());
    vis.finish_vertex (u, g);         // visitor
  } // end while
}

/// Breadth First Search Algorithm (Cormen, Leiserson, and Rivest p. 470)
/// This version use as default the std::queue for the visit.
///
/// @tparam Graph the graph type
/// @tparam BFSVisitor the visitor used uring the visit. This class must match
///  the bfs_visitor interface
/// @tparam ColorMap the map that associates a vertex to a color. As default
///  is used the external color map, that mantain an hash-table from vertex to
///  colors. If in the vertices data is present a field named color maybe you
///  want use an user-define map
/// 
/// @param g the graph
/// @param s the start vertex
/// @param vis the visitor used (a copy). This class must match the bfs_visitor 
///  interface. If you want to use a reference to a visitor use std::ref 
///  utility function.
/// @param color the color map, that associate a vertex to a color. This class
///  must match a property_map interface that associate a vertex descriptor with
///  a color (for example default_color_t).
template <typename Graph, typename BFSVisitor, typename ColorMap>
inline void 
breadth_first_search (Graph& g, 
                      typename Graph::vertex_descriptor s, 
                      BFSVisitor vis, 
                      ColorMap& color)
{
  return breadth_first_search (g, s, vis, color, 
      std::queue<typename Graph::vertex_descriptor>());
}

/// Breadth First Search Algorithm (Cormen, Leiserson, and Rivest p. 470)
/// This version use as default the std::queue for the visit and the
/// proeperty_map_external_t for the vertex coloring.
///
/// @tparam Graph the graph type
/// @tparam BFSVisitor the visitor used uring the visit. This class must match
///  the bfs_visitor interface
/// 
/// @param g the graph
/// @param s the start vertex
/// @param vis the visitor used (a copy). This class must match the bfs_visitor 
///  interface. If you want to use a reference to a visitor use std::ref 
///  utility function.
template <typename Graph, typename BFSVisitor> 
inline void 
breadth_first_search (Graph& g, 
                      typename Graph::vertex_descriptor s, 
                      BFSVisitor vis)
{
  typedef typename Graph::vertex_descriptor Vertex;
  property_map_external_t<Vertex, default_color_t> cmap;
  return breadth_first_search (g, s, vis, cmap,
      std::queue<Vertex>());
}

/// Breadth First Search Algorithm (Cormen, Leiserson, and Rivest p. 470)
/// This version use as default the std::queue for the visit, the
/// property_map_external_t for the vertex coloring and the bfs_visitor that 
/// performs no operations during the visit.
///
/// @tparam Graph the graph type
/// 
/// @param g the graph
/// @param s the start vertex
template <typename Graph>
inline void 
breadth_first_search (Graph& g, typename Graph::vertex_descriptor s)
{
  typedef typename Graph::vertex_descriptor Vertex;
  property_map_external_t<Vertex, default_color_t> cmap;
  return breadth_first_search (g, s,
      bfs_visitor<Graph>(), 
      cmap,
      std::queue<Vertex>());
}

namespace impl {

template <typename Vertex, typename Iter>
struct context_info {
  typedef std::pair<Iter, Iter> Range;

  Vertex vertex;
  Range range;

  context_info () : vertex(), range() {}
  context_info (Vertex v, const Range& r) : vertex(v), range(r) {}
};


/// The dfs visit implementation is not recursive. Here a stack is used as
/// context switch when the algorithm back track after the finish of a 
/// vertex visit (and its successors).
template <typename Graph, typename DFSVisitor, typename ColorMap>
void dfs_visit (Graph& g, DFSVisitor& vis, ColorMap& color_map,
                typename Graph::vertex_descriptor u)
{
  typedef typename Graph::vertex_descriptor Vertex;
  typedef typename Graph::edge_descriptor Edge;
  typedef typename ColorMap::value_type ColorValue;
  typedef color_traits<ColorValue> Color;
  typedef context_info<Vertex, typename Graph::out_edge_iterator> Info;
  
  typename Graph::out_edge_iterator ei, ei_end;
  Edge e;
  std::stack<Info> stack;

  vis.start_vertex (u, g);
  color_map.put (u, Color::gray());
  vis.discover_vertex (u, g);
  stack.push (Info(u, g.out_edges(u)));

  while (! stack.empty()) {
    Info& context = stack.top();
    u = context.vertex;
    std::tr1::tie (ei, ei_end) = context.range;
    stack.pop();
    while (ei != ei_end) {
      e = *ei;
      Vertex v = g.target (e);
      vis.examine_edge (e, g);
      ColorValue v_color = color_map.get (v);
      if (v_color == Color::white()) {
        vis.tree_edge (e, g);
        stack.push (Info(u, std::make_pair(++ei, ei_end)));
        u = v;
        color_map.put (u, Color::gray());
        vis.discover_vertex (u, g);
        std::tr1::tie (ei, ei_end) = g.out_edges (u);
      }
      else if (v_color == Color::gray()) {
        vis.back_edge (e, g);
        ++ei;
      }
      else {
        vis.forward_or_cross_edge (e, g);
        ++ei;
      }
    } // end while
    color_map.put (u, Color::black());
    vis.finish_vertex (u, g);
  } // end while
}


} // namespace impl


/// Depth First Search Algorithm (Cormen, Leiserson, and Rivest)
///
/// @tparam Graph the graph type
/// @tparam DFSVisitor the visitor used uring the visit. This class must match
///  the dfs_visitor interface
/// @tparam ColorMap the map that associates a vertex to a color. As default
///  is used the external color map, that mantain an hash-table from vertex to
///  colors. If in the vertices data is present a field named color maybe you
///  want use an user-define map
/// 
/// @param g the graph
/// @param vis the visitor used (a copy). This class must match the dfs_visitor 
///  interface. If you want to use a reference to a visitor use std::ref 
///  utility function.
/// @param color the color map, that associate a vertex to a color. This class
///  must match a property_map interface that associate a vertex descriptor with
///  a color (for example default_color_t).
/// @param s the start vertex. After the vertex coloring the visit start with
///  this vertex.
template <typename Graph, typename DFSVisitor, typename ColorMap>
void depth_first_search (Graph& g, 
                         DFSVisitor vis, 
                         ColorMap& color_map, 
                         typename Graph::vertex_descriptor s)
{
  typedef typename Graph::vertex_descriptor Vertex;
  typedef typename ColorMap::value_type ColorValue;
  typedef color_traits<ColorValue> Color;
  
  typename Graph::vertex_iterator it, end;
  for (std::tr1::tie (it, end) = g.vertices(); it != end; ++it) {
    color_map.put (*it, Color::white());
    vis.initialize_vertex (*it, g);
  }

  if (s != *g.vertices().first)
    impl::dfs_visit (g, vis, color_map, s);
  
  for (std::tr1::tie (it, end) = g.vertices(); it != end; ++it) {
    s = *it;
    ColorValue color = color_map.get (s);
    if (color == Color::white())
      impl::dfs_visit (g, vis, color_map, s);
  }
}

/// Depth First Search Algorithm (Cormen, Leiserson, and Rivest).
/// This version use as start vertex the first in the vertices list.
///
/// @tparam Graph the graph type
/// @tparam DFSVisitor the visitor used uring the visit. This class must match
///  the dfs_visitor interface
/// @tparam ColorMap the map that associates a vertex to a color. As default
///  is used the external color map, that mantain an hash-table from vertex to
///  colors. If in the vertices data is present a field named color maybe you
///  want use an user-define map
/// 
/// @param g the graph
/// @param vis the visitor used (a copy). This class must match the dfs_visitor 
///  interface. If you want to use a reference to a visitor use std::ref 
///  utility function.
/// @param color the color map, that associate a vertex to a color. This class
///  must match a property_map interface that associate a vertex descriptor with
///  a color (for example default_color_t).
template <typename Graph, typename DFSVisitor, typename ColorMap>
inline void depth_first_search (Graph& g, DFSVisitor vis, ColorMap& color_map)
{
  return depth_first_search (g, vis, color_map, 
                             *g.vertices().first);
}

/// Depth First Search Algorithm (Cormen, Leiserson, and Rivest).
/// This version use as start vertex the first in the vertices list and the 
/// property_map_external_t for vertex coloring.
///
/// @tparam Graph the graph type
/// @tparam DFSVisitor the visitor used uring the visit. This class must match
///  the dfs_visitor interface
/// 
/// @param g the graph
/// @param vis the visitor used (a copy). This class must match the dfs_visitor 
///  interface. If you want to use a reference to a visitor use std::ref 
///  utility function.
template <typename Graph, typename DFSVisitor>
inline void depth_first_search (Graph& g, DFSVisitor vis)
{
  typedef typename Graph::vertex_descriptor Vertex;
  property_map_external_t<Vertex, default_color_t> cmap;
  return depth_first_search (g, vis, cmap, 
                             *g.vertices().first);
}

/// Depth First Search Algorithm (Cormen, Leiserson, and Rivest).
/// This version use as start vertex the first in the vertices list, the 
/// property_map_external_t for vertex coloring and the dfs_visitor that 
/// performs no operations during the visit.
///
/// @tparam Graph the graph type
/// @param g the graph
template <typename Graph>
inline void depth_first_search (Graph& g)
{
  typedef typename Graph::vertex_descriptor Vertex;
  property_map_external_t<Vertex, default_color_t> cmap;
  dfs_visitor<Graph> vis;
  return depth_first_search (g, vis, cmap, 
                             *g.vertices().first);
}

} // namespace gtl

#endif  // GTL_GRAPH_ALGORITHM_HH
