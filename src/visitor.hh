// =============================================================================
// 
//       Filename:  visitor.hh
// 
//    Description:  A template set of visitors for graphs, used in the visit
//                  algorithms
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

#ifndef GTL_VISITOR_HH
#define GTL_VISITOR_HH

#include "property_map.hh"

#ifdef NEED_GCC_NO_VARIADIC_WORKAOUND
#pragma GCC system_header
#endif


namespace gtl {

/// Default BFS visitor. You can inherit from this overriding some operations
/// called during the BFS visit.
/// @tparam _Graph the graph class
template <typename _Graph>
struct bfs_visitor
{  
  typedef _Graph Graph;
  typedef typename _Graph::vertex_descriptor Vertex;
  typedef typename _Graph::edge_descriptor Edge;
  
  /// This is invoked on the start vertex of the visit, before the start of 
  /// the graph search and after the initialization.
  void start_vertex (Vertex, Graph&) {};

  /// This is invoked on every vertex of the graph before the start of the 
  /// graph search. 
  void initialize_vertex (Vertex, Graph&) {};
  
  /// This is invoked when a vertex is encountered for the first time. 
  void discover_vertex (Vertex, Graph&) {};
  
  /// This is invoked on a vertex as it is popped from the queue. This happens 
  /// immediately before examine_edge() is invoked on each of the out-edges of 
  /// vertex u. 
  void examine_vertex (Vertex, Graph&) {};
  
  /// This is invoked on every out-edge of each vertex after it is discovered. 
  void examine_edge (Edge, Graph&) {};
  
  /// This is invoked on each edge as it becomes a member of the edges that 
  /// form the search tree. 
  void tree_edge (Edge, Graph&) {};
  
  /// This is invoked on back or cross edges for directed graphs and cross 
  /// edges for undirected graphs. 
  void non_tree_edge (Edge, Graph&) {};
  
  /// This is invoked on the subset of non-tree edges whose target vertex is 
  /// colored gray at the time of examination. The color gray indicates that 
  /// the vertex is currently in the queue. 
  void gray_target (Edge, Graph&) {};
  
  /// This is invoked on the subset of non-tree edges whose target vertex is 
  /// colored black at the time of examination. The color black indicates that 
  /// the vertex has been removed from the queue. 
  void black_target (Edge, Graph&) {};
  
  /// This invoked on a vertex after all of its out edges have been added to 
  /// the search tree and all of the adjacent vertices have been discovered 
  /// (but before the out-edges of the adjacent vertices have been examined). 
  void finish_vertex (Vertex, Graph&) {};
};

/// Default DFS visitor. You can inherit from this overriding some operations
/// called during the DFS visit.
/// @tparam _Graph the graph class
template <typename _Graph>
struct dfs_visitor 
{
  typedef _Graph Graph;
  typedef typename _Graph::vertex_descriptor Vertex;
  typedef typename _Graph::edge_descriptor Edge;
  
  /// This is invoked on every vertex of the graph before the start of the 
  /// graph search. 
  void initialize_vertex (Vertex u, Graph& g) {};
  
  /// This is invoked on the source vertex once before the start of the search.
  void start_vertex (Vertex u, Graph& g) {};

  /// This is invoked when a vertex is encountered for the first time.
  void discover_vertex (Vertex u, Graph& g) {};

  /// This is invoked on every out-edge of each vertex after it is discovered.
  void examine_edge (Edge e, Graph& g) {};

  /// This is invoked on each edge as it becomes a member of the edges that 
  /// form the search tree. 
  void tree_edge (Edge e, Graph& g) {};
  
  /// This is invoked on the back edges in the graph.
  void back_edge (Edge e, Graph& g) {};
  
  /// This is invoked on forward or cross edges in the graph. 
  void forward_or_cross_edge (Edge e, Graph& g) {};
  
  /// This is invoked on vertex u after finish_vertex has been called for all
  /// the vertices in the DFS-tree rooted at vertex u. If vertex u is a leaf in
  /// the DFS-tree, then the finish_vertex function is called on u after all 
  /// the out-edges of u have been examined.
  void finish_vertex (Vertex u, Graph& g) {};
};


// --------------------------------- impl --------------------------------------
namespace impl {

template <typename Graph, typename ... Other>
class bfs_visitor_list;

template <typename Graph>
struct bfs_visitor_list <Graph>;


template <typename Graph, typename V, typename ... Other>
class bfs_visitor_list <Graph, V, Other...> 
  : public bfs_visitor_list <Graph, Other...>
{
  typedef typename Graph::vertex_descriptor _Vertex;
  typedef typename Graph::edge_descriptor _Edge;
  typedef bfs_visitor_list <Graph, Other...> Base;

  V vis;

public:
  bfs_visitor_list (const V& vis_, Other... tail)
    : Base(tail...), vis(vis_) {}

  bfs_visitor_list ()
    : Base(), vis() {}

  void start_vertex (_Vertex v, Graph& g) {
    vis.start_vertex (v, g);
    Base::start_vertex (v, g);
  }

  void initialize_vertex (_Vertex v, Graph& g) {
    vis.initialize_vertex (v, g);
    Base::initialize_vertex (v, g);
  }

  void discover_vertex (_Vertex v, Graph& g) {
    vis.discover_vertex (v, g);
    Base::discover_vertex (v, g);
  }

  void examine_vertex (_Vertex v, Graph& g) {
    vis.examine_vertex (v, g);
    Base::examine_vertex (v, g); 
  }
  void examine_edge (_Edge e, Graph& g) {
    vis.examine_edge (e, g);
    Base::examine_edge (e, g);
  }

  void tree_edge (_Edge e, Graph& g) {
    vis.tree_edge (e, g);
    Base::tree_edge (e, g);
  }
  
  void non_tree_edge (_Edge e, Graph& g) {
    vis.non_tree_edge (e, g);
    Base::non_tree_edge (e, g);
  }
  
  void gray_target (_Edge e, Graph& g) {
    vis.gray_target (e, g);
    Base::gray_target (e, g);
  }

  void black_target (_Edge e, Graph& g) {
    vis.black_target (e, g);
    Base::black_target (e, g);
  }
  
  void finish_vertex (_Vertex v, Graph& g) {
    vis.finish_vertex (v, g);
    Base::finish_vertex (v, g);
  }
};

template <typename Graph>
struct bfs_visitor_list <Graph> 
  : public bfs_visitor<Graph>
{};


template <typename Graph, typename ... Other>
class dfs_visitor_list;

template <typename Graph>
struct dfs_visitor_list <Graph>;


template <typename Graph, typename V, typename ... Other>
class dfs_visitor_list <Graph, V, Other...> 
  : public dfs_visitor_list <Graph, Other...>
{
  typedef typename Graph::vertex_descriptor _Vertex;
  typedef typename Graph::edge_descriptor _Edge;
  typedef dfs_visitor_list <Graph, Other...> Base;

  V vis;

public:
  dfs_visitor_list (const V& vis_, Other... tail)
    : Base(tail...), vis(vis_) {}

  dfs_visitor_list ()
    : Base(), vis() {}
  
  void initialize_vertex (_Vertex v, Graph& g) {
    vis.initialize_vertex (v, g);
    Base::initialize_vertex (v, g);
  }

  void start_vertex (_Vertex v, Graph& g) {
    vis.start_vertex (v, g);
    Base::start_vertex (v, g);
  }
  
  void discover_vertex (_Vertex u, Graph& g) {
    vis.discover_vertex (u, g);
    Base::discover_vertex (u, g);
  };

  void examine_edge (_Edge e, Graph& g) {
    vis.examine_edge (e, g);
    Base::examine_edge (e, g);
  };

  void tree_edge (_Edge e, Graph& g) {
    vis.tree_edge (e, g);
    Base::tree_edge (e, g);
  };
  
  void back_edge (_Edge e, Graph& g) {
    vis.back_edge (e, g);
    Base::back_edge (e, g);
  };
  
  void forward_or_cross_edge (_Edge e, Graph& g) {
    vis.forward_or_cross_edge (e, g);
    Base::forward_or_cross_edge (e, g);
  };
  
  void finish_vertex (_Vertex u, Graph& g) {
    vis.finish_vertex (u, g);
    Base::finish_vertex (u, g);
  };

};

template <typename Graph>
struct dfs_visitor_list <Graph> 
  : public dfs_visitor<Graph>
{};

} // namespace impl


/// This function allows to combine visitor togheter. The visitor returned 
/// implements the same interface of dfs_visitor. When a member function is 
/// called (for example ::gray_target), the same function is called for 
/// every visitor of the list (V1::gray_target, V2::gray_target ..). Internally
/// this visitor stores by value each visitor given. 
template <typename V1, typename ... Other>
inline impl::bfs_visitor_list<typename V1::Graph, V1, Other...>
make_bfs_visitor (V1 v, Other... other)
{
  return impl::bfs_visitor_list<typename V1::Graph, V1, Other...> (v, other...);
}

/// This function allows to combine visitor togheter. The visitor returned 
/// implements the same interface of bfs_visitor. When a member function is 
/// called (for example ::tree_edge), the same function is called for 
/// every visitor of the list (V1::tree_edge, V2::tree_edge ..). Internally
/// this visitor stores by value each visitor given. 
template <typename V1, typename ... Other>
inline impl::dfs_visitor_list<typename V1::Graph, V1, Other...>
make_dfs_visitor (V1 v, Other... other)
{
  return impl::dfs_visitor_list<typename V1::Graph, V1, Other...> (v, other...);
}


template <typename BaseVisitor,
          typename PredecessorMap = 
            property_map_external_t<typename BaseVisitor::Vertex,
                                    typename BaseVisitor::Vertex> >
struct predecessor_recorder
  : public BaseVisitor
{
  /// Record in the map the predecessor of each vertex, following the visit tree
  void tree_edge (typename BaseVisitor::Edge e, 
                  typename BaseVisitor::Graph& g) {
    _map.put (g.target(e), g.source(e));
  }

  /// Initialize the predecessor of a vertex to itself. 
  void initialize_vertex (typename BaseVisitor::Vertex v,
                          typename BaseVisitor::Graph&) {
    _map.put (v, v);
  }

  predecessor_recorder (PredecessorMap& map) : _map(map) {}

  PredecessorMap& _map;
};


/// This is a visitor that records the predecessor (or parent) of a vertex 
/// in a predecessor property map. This is particularly useful in graph search
/// algorithms where recording the predecessors is an efficient way to encode 
/// the search tree that was traversed during the search.
/// @tparam Graph the graph
/// @tparam PredecessorMap the map that associate each vertex to its 
///  predecessor
template <typename Graph, 
          typename PredecessorMap = 
            property_map_external_t<typename Graph::vertex_descriptor,
                                    typename Graph::vertex_descriptor> >
struct bfs_predecessor_recorder 
  : public predecessor_recorder <bfs_visitor<Graph>, PredecessorMap>
{ 
  typedef predecessor_recorder <bfs_visitor<Graph>, PredecessorMap> Base;

  bfs_predecessor_recorder (PredecessorMap& pmap) : Base (pmap) {}
};

/// This is a visitor that records the predecessor (or parent) of a vertex 
/// in a predecessor property map. This is particularly useful in graph search
/// algorithms where recording the predecessors is an efficient way to encode 
/// the search tree that was traversed during the search.
/// @tparam Graph the graph
/// @tparam PredecessorMap the map that associate each vertex to its 
///  predecessor
template <typename Graph, 
          typename PredecessorMap = 
            property_map_external_t<typename Graph::vertex_descriptor,
                                    typename Graph::vertex_descriptor> >
struct dfs_predecessor_recorder 
  : public predecessor_recorder <dfs_visitor<Graph>, PredecessorMap>
{ 
  typedef predecessor_recorder <dfs_visitor<Graph>, PredecessorMap> Base;

  dfs_predecessor_recorder (PredecessorMap& pmap) : Base (pmap) {}
};

/// This function returns a visitor that records the predecessor (or parent) of 
/// a vertex in a predecessor property map. This is particularly useful in 
/// graph search algorithms where recording the predecessors is an efficient 
/// way to encode the search tree that was traversed during the search.
/// @param v the base visitor (use bfs_visitor or dfs_visitor)
/// @param pmap the map that associates each vertex to its predecessor (for
///  example you may use the default_property_map
template <typename BaseVisitor, typename PredecessorMap>
inline predecessor_recorder <BaseVisitor, PredecessorMap>
record_predecessors (const BaseVisitor& v, PredecessorMap& pmap) {
  return predecessor_recorder<BaseVisitor, PredecessorMap> (pmap);
}
          
/// This function returns a bfs visitor that records the predecessor (or 
/// parent) of a vertex in a predecessor property map. This is particularly 
/// useful in graph search algorithms where recording the predecessors is an 
/// efficient way to encode the search tree that was traversed during the 
/// search.
/// @param g the graph to visit. This parameter is only used to determine the
///  type of the graph. You can provide also an other graph of the same type.
/// @param pmap the map that associates each vertex to its predecessor (for
///  example you may use the default_property_map
template <typename Graph, typename PredecessorMap>
inline bfs_predecessor_recorder <Graph, PredecessorMap>
record_bfs_predecessors (const Graph&, PredecessorMap& pmap) {
  return bfs_predecessor_recorder<Graph, PredecessorMap> (pmap);
}

/// This function returns a dfs visitor that records the predecessor (or 
/// parent) of a vertex in a predecessor property map. This is particularly 
/// useful in graph search algorithms where recording the predecessors is an 
/// efficient way to encode the search tree that was traversed during the 
/// search.
/// @param g the graph to visit. This parameter is only used to determine the
///  type of the graph. You can provide also an other graph of the same type.
/// @param pmap the map that associates each vertex to its predecessor (for
///  example you may use the default_property_map
template <typename Graph, typename PredecessorMap>
inline dfs_predecessor_recorder <Graph, PredecessorMap>
record_dfs_predecessors (const Graph&, PredecessorMap& pmap) {
  return dfs_predecessor_recorder<Graph, PredecessorMap> (pmap);
}

template <typename BaseVisitor,
          typename DistanceMap = 
            property_map_external_t<typename BaseVisitor::Vertex, size_t> >
struct distance_recorder
  : public BaseVisitor
{
  /// Record in the map the predecessor of each vertex, following the visit tree
  void tree_edge (typename BaseVisitor::Edge e, 
                  typename BaseVisitor::Graph& g) {
    _map.put (g.target(e), _map.get(g.source(e)) + 1);
  }

  /// Initialize the vertex distance of the initial vertex
  void start_vertex (typename BaseVisitor::Vertex v,
                     typename BaseVisitor::Graph&) {
    _map.put (v, 0);
  }

  /// Initialize the other vertices to infinity distance (maximum of the distance
  /// type)
  void initialize_vertex (typename BaseVisitor::Vertex v,
                          typename BaseVisitor::Graph&) {
    _map.put (v, std::numeric_limits<typename DistanceMap::value_type>::max());
  }

  distance_recorder (DistanceMap& map) : _map(map) {}

  DistanceMap& _map;
};


/// This is an EventVisitor that records the distance of a vertex (using a 
/// property map) from some source vertex during a graph search. When applied 
/// to edge e = (u,v), the distance of v is recorded to be one more than the 
/// distance of u.
/// @tparam Graph the graph
/// @tparam DistanceMap the map that associate each vertex to its predecessor
///  distance
template <typename Graph, 
          typename DistanceMap = 
            property_map_external_t<typename Graph::vertex_descriptor, size_t> >
struct bfs_distance_recorder 
  : public distance_recorder <bfs_visitor<Graph>, DistanceMap>
{
  typedef distance_recorder <bfs_visitor<Graph>, DistanceMap> Base;

  bfs_distance_recorder (DistanceMap& dmap) : Base (dmap) {}
};

/// This is an EventVisitor that records the distance of a vertex (using a 
/// property map) from some source vertex during a graph search. When applied 
/// to edge e = (u,v), the distance of v is recorded to be one more than the 
/// distance of u.
/// @tparam Graph the graph
/// @tparam DistanceMap the map that associate each vertex to its predecessor
///  distance
template <typename Graph, 
          typename DistanceMap = 
            property_map_external_t<typename Graph::vertex_descriptor, size_t> >
struct dfs_distance_recorder 
  : public distance_recorder <dfs_visitor<Graph>, DistanceMap>
{
  typedef distance_recorder <dfs_visitor<Graph>, DistanceMap> Base;

  dfs_distance_recorder (DistanceMap& dmap) : Base (dmap) {}
};

/// This function returns a visitor that records the distance of a vertex (using
/// a property map) from some source vertex during a graph search. When applied 
/// to edge e = (u,v), the distance of v is recorded to be one more than the 
/// distance of u.
/// @param v the base visitor (use bfs_visitor or dfs_visitor). This parameter
///  is only used to determine the type of the base visitor. No copies nor 
///  references are taken from this parameter
/// @param dmap the map the map that associate each vertex to its predecessor
///  distance
template <typename BaseVisitor, typename DistanceMap>
inline distance_recorder <BaseVisitor, DistanceMap>
record_distances (const BaseVisitor& v, DistanceMap& dmap) {
  return distance_recorder<BaseVisitor, DistanceMap> (dmap);
}

/// This function returns a bfs  visitor that records the distance of a vertex 
/// (using a property map) from some source vertex during a graph search. When 
/// applied to edge e = (u,v), the distance of v is recorded to be one more 
/// than the distance of u.
/// All the vertices not reached by the visit has the distance equal to the 
/// maximum value of the distance type, and the start vertex is setted with
/// distance zero.
/// @param g the graph to visit. This parameter is only used to determine the
///  type of the graph. You can provide also an other graph of the same type, 
///  no copies nor references are taken from this parameter.
/// @param dmap the map the map that associate each vertex to its predecessor
///  distance
template <typename Graph, typename DistanceMap>
inline bfs_distance_recorder <Graph, DistanceMap>
record_bfs_distances (const Graph&, DistanceMap& dmap) {
  return bfs_distance_recorder<Graph, DistanceMap> (dmap);
}

/// This function returns a dfs  visitor that records the distance of a vertex 
/// (using a property map) from some source vertex during a graph search. When 
/// applied to edge e = (u,v), the distance of v is recorded to be one more 
/// than the distance of u.
/// All the vertices not reached by the visit has the distance equal to the 
/// maximum value of the distance type, and the start vertex is setted with
/// distance zero.
/// @param g the graph to visit. This parameter is only used to determine the
///  type of the graph. You can provide also an other graph of the same type, 
///  no copies nor references are taken from this parameter.
/// @param dmap the map the map that associate each vertex to its predecessor
///  distance
template <typename Graph, typename DistanceMap>
inline dfs_distance_recorder <Graph, DistanceMap>
record_dfs_distances (const Graph&, DistanceMap& dmap) {
  return dfs_distance_recorder<Graph, DistanceMap> (dmap);
}


template <typename BaseVisitor,
          typename DTimeMap = 
            property_map_external_t<typename BaseVisitor::Vertex, size_t>,
          typename FTimeMap = DTimeMap,
          typename Time = typename DTimeMap::value_type>
class time_stamper
  : public BaseVisitor
{

public:
  /// Increments the time counter and record the discover time of the vertex
  void discover_vertex (typename BaseVisitor::Vertex v,
                        typename BaseVisitor::Graph&) {
    _dtime_map.put (v, ++t);
  }

  /// Increments the time counter and record the finish time of the vertex
  void finish_vertex (typename BaseVisitor::Vertex v,
                      typename BaseVisitor::Graph&) {
    _ftime_map.put (v, ++t);
  }

  /// Constructor
  /// @param dtime_map the discover time map
  /// @param ftime_map the finish time map
  time_stamper (DTimeMap& dtime_map, FTimeMap& ftime_map) 
    : _dtime_map(dtime_map), _ftime_map(ftime_map), t(0) { }

protected:
  DTimeMap& _dtime_map;
  FTimeMap& _ftime_map;
  Time t;
};

/// This is an EventVisitor that records the discovering and the finish time 
/// stamp of each vertex (using two property map) during a dfs graph search. 
/// When applied to vertex v, the time counter is incremented, and stored in 
/// the correct time map (discover or finish).
/// @tparam Graph the graph
/// @tparam DTimeMap the map that associate each vertex to its discover time 
///  stamp
/// @tparam FTimeMap the map that associate each vertex to its finish time 
///  stamp
/// @tparam Time the time type
template <typename Graph,
          typename DTimeMap = 
            property_map_external_t<typename Graph::vertex_descriptor, size_t>,
          typename FTimeMap = DTimeMap,
          typename Time = typename DTimeMap::value_type>
struct dfs_time_stamper 
  : public time_stamper <dfs_visitor<Graph>, DTimeMap, FTimeMap, Time>
{
  typedef time_stamper <dfs_visitor<Graph>, DTimeMap, FTimeMap, Time> Base;

  dfs_time_stamper (DTimeMap& dtime_map, FTimeMap& ftime_map)
    : Base (dtime_map, ftime_map) { }
};

/// This is an EventVisitor that records the discovering and the finish time 
/// stamp of each vertex (using two property map) during a bfs graph search. 
/// When applied to vertex v, the time counter is incremented, and stored in 
/// the correct time map (discover or finish).
/// @tparam Graph the graph
/// @tparam DTimeMap the map that associate each vertex to its discover time 
///  stamp
/// @tparam FTimeMap the map that associate each vertex to its finish time 
///  stamp
/// @tparam Time the time type
template <typename Graph,
          typename DTimeMap = 
            property_map_external_t<typename Graph::vertex_descriptor, size_t>,
          typename FTimeMap = DTimeMap,
          typename Time = typename DTimeMap::value_type>
struct bfs_time_stamper 
  : public time_stamper <bfs_visitor<Graph>, DTimeMap, FTimeMap, Time>
{
  typedef time_stamper <bfs_visitor<Graph>, DTimeMap, FTimeMap, Time> Base;

  bfs_time_stamper (DTimeMap& dtime_map, FTimeMap& ftime_map)
    : Base (dtime_map, ftime_map) { }
};

/// This function returns a dfs visitor that records the discovering and the 
/// finish time stamp of each vertex (using two property map) during a graph 
/// search. When applied to vertex v, the time counter is incremented, and 
/// stored in the correct time map (discover or finish).
/// @param g the graph to visit. This parameter is only used to determine the
///  type of the graph. You can provide also an other graph of the same type, 
///  no copies nor references are taken from this parameter.
/// @param dtime_map the map that associate each vertex to its discover time 
///  stamp. You can provide the special map dummy_property_map if you don't 
///  care this value.
/// @param ftime_map the map that associate each vertex to its finish time 
///  stamp. You can provide the special map dummy_property_map if you don't 
///  care this value.
template <typename Graph, typename DTimeMap, typename FTimeMap>
inline dfs_time_stamper <Graph, DTimeMap, FTimeMap>
stamp_dfs_times (const Graph&, DTimeMap& dtime_map, FTimeMap& ftime_map) {
  return dfs_time_stamper<Graph, DTimeMap, FTimeMap> (dtime_map, ftime_map);
}

/// This function returns a dfs visitor that records the discovering and the 
/// finish time stamp of each vertex (using two property map) during a graph 
/// search. When applied to vertex v, the time counter is incremented, and 
/// stored in the correct time map (discover or finish).
/// @param g the graph to visit. This parameter is only used to determine the
///  type of the graph. You can provide also an other graph of the same type, 
///  no copies nor references are taken from this parameter.
/// @param dtime_map the map that associate each vertex to its discover time 
///  stamp. You can provide the special map dummy_property_map if you don't 
///  care this value.
/// @param ftime_map the map that associate each vertex to its finish time 
///  stamp. You can provide the special map dummy_property_map if you don't 
///  care this value.
template <typename Graph, typename DTimeMap, typename FTimeMap>
inline bfs_time_stamper <Graph, DTimeMap, FTimeMap>
stamp_bfs_times (const Graph&, DTimeMap& dtime_map, FTimeMap& ftime_map) {
  return bfs_time_stamper<Graph, DTimeMap, FTimeMap> (dtime_map, ftime_map);
}


} // namespace gtl

#endif  // GTL_VISITOR_HH
