// =============================================================================
// 
//       Filename:  graph.hh
// 
//    Description:  The graph.hh library for c++ provides an STL-like container 
//                  for graphs.
//
//         Author:  Michele Bertasi 
//        Contact:  michele.bertasi $at$ studenti.univr.it
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

#ifndef GTL_GRAPH_T_HH
#define GTL_GRAPH_T_HH

#include "graph_internals.hh"

#include <cassert>
#include <algorithm>
#include <deque>

namespace gtl
{

///
/// @brief Graph class with directed and parallel edges.
/// @author Michele Bertasi.
///
/// The graph.hh library for c++ provides an STL-like container for graphs, 
/// templated over the data stored at the vertices and the edges. Various types 
/// of iterators are provided. The edges are directed and there are the
/// possibility to permits parallel edges or not.
/// 
/// @tparam Vdata the data stored in the vertices. You can provide the special
///  value no_data. To get the data stored use the operator* or -> provided by
///  the vertex_descriptor.
/// @tparam Edata the data stored in the edges. You can provide the special
///  value no_data. To get the data stored use the operator* or -> provided by
///  the edge_descriptor.
/// @tparam parallel if is set to true the graph allows parallel edges. In this
///  case between two vertices can be more than one edge. If is set to false
///  between two vertices may be only one edge, but there are some performance
///  improvements in space (two pointers instead of four for each edge) and 
///  time (iterating adjacent vertices).
/// @tparam Allocator type of the allocator object used to define the storage
///  allocation model. By default, the std::allocator class template is used,
///  which defines the simplest memory allocation model and is value-independent.
///
template <typename Vdata = no_data, 
          typename Edata = no_data, 
          bool parallel = false,
          typename Allocator = std::allocator<void> >
class graph_t
{
  typedef impl::_Config<Vdata, Edata, parallel, Allocator> Config;  
  typedef typename Config::vertex_t vertex_t;
  typedef typename Config::edge_t edge_t;

public:
  /// It is a pointer to a vertex. It's dereferencing returns a reference to
  /// the data stored in the vertex (Vdata&).
  typedef typename Config::vertex_descriptor vertex_descriptor;
  
  /// It is a pointer to an edge. It's dereferencing returns a reference to
  /// the data stored in the edge (Edata&).
  typedef typename Config::edge_descriptor edge_descriptor;
  
  /// It is the iterator used iterating all the vertices. It's dereferencing
  /// returns the associated vertex_descriptor.
  typedef typename Config::vertex_iterator vertex_iterator;
  
  /// It is the iterator used iterating all the edges. It's dereferencing
  /// returns the associated edge_descriptor.
  typedef typename Config::edge_iterator edge_iterator;
  
  /// It is the iterator used iterating the out-edges of a vertex. It's 
  /// dereferencing returns the associated edge_descriptor.
  typedef typename Config::out_edge_iterator out_edge_iterator;
  
  /// It is the iterator used iterating the in-edges of a vertex. It's 
  /// dereferencing returns the associated edge_descriptor.
  typedef typename Config::in_edge_iterator in_edge_iterator;
  
  /// It is the iterator used iterating the adjacent vertices. It's 
  /// dereferencing returns the associated vertex_descriptor;
  /// In the case of graphs that allows parallel edges the increment and 
  /// decrement may be performed not in constant time (but the total time
  /// spended iterating is always linear to the adjacency list of the vertex).
  typedef typename Config::adjacency_iterator adjacency_iterator;
  
  /// The default hasher function for vertex descriptors
  typedef typename Config::vertex_hash vertex_hash;

  /// The default hasher function for edge descriptors
  typedef typename Config::edge_hash edge_hash;

  /// Default constructor. It creates an empty graph object with zero vertices
  /// and zero edges.
  explicit graph_t ();
  
  /// Destructor. This calls each of the contained element's destructors if
  /// are not pointers, and deallocates all the storage capacity allocated by 
  /// the graph_t container.
  ~graph_t ();
  
  /// Delete all vertices and edges. No memory is freed if the vertex or edge
  /// data are pointers.
  void clear (); 
  
  /// Returns an iterator range providing access to the vertex set.
  std::pair<vertex_iterator, vertex_iterator> vertices ();

  /// Returns an iterator range providing access to the edge set.
  std::pair<edge_iterator, edge_iterator> edges ();
  
  /// Returns an iterator range providing access to the out-edges of vertex v.
  std::pair<out_edge_iterator, out_edge_iterator> 
  out_edges (vertex_descriptor v);

  /// Returns an iterator range providing access to the in-edges of vertex v.
  std::pair<in_edge_iterator, in_edge_iterator> 
  in_edges (vertex_descriptor v);
  
  /// Returns the source vertex of the edge e.
  vertex_descriptor source (edge_descriptor e);

  /// Returns the target vertex of the edge e.
  vertex_descriptor target (edge_descriptor e);

  /// Returns the number of edges leaving vertex u.
  std::size_t out_degree (vertex_descriptor u) const;

  /// Returns the number of edges entering vertex u.
  std::size_t in_degree (vertex_descriptor u) const;

  /// Returns the number of vertices in the graph.
  std::size_t num_vertices () const;

  /// Returns the number of edges in the graph.
  std::size_t num_edges () const;

  /// Returns a pair, where pair::first is the edge connecting vertex u to 
  /// vertex v. If there are more than one edge between u and v, is one of 
  /// them. Member pair::second is set to true if the edge is found, false 
  /// otherwise. The complexity is O(log E), where E is the number of out 
  /// edges of u.
  /// @param u the source vertex
  /// @param v the target vertex
  std::pair<edge_descriptor, bool>
  edge (vertex_descriptor u, vertex_descriptor v);
  
  /// Returns a pair of out edge iterators that give the range for all the
  /// parallel edges from u to v. If there are no edges between the vertices,
  /// the range returned has a lenght of zero, with both iterators ponting
  /// to the same value. The complexity is O(log E), where E is the number of
  /// out edges of u.
  /// @param u the source vertex
  /// @param v the target vertex
  std::pair<out_edge_iterator, out_edge_iterator>
  edge_range (vertex_descriptor u, vertex_descriptor v);

  /// Returns an iterator range providing access to the vertices adjacent to
  /// the given vertex v. The iterating complexity is O(E), where E is the 
  /// number of out edges of the vertex v.
  /// @param v the vertex
  std::pair<adjacency_iterator, adjacency_iterator>
  adjacent_vertices (vertex_descriptor v);
  
  /// Adds a vertex in the graph. Operates in constant time. Builds the vertex
  /// data with default constructor.
  /// @return the descriptor of the new vertex added.
  vertex_descriptor add_vertex ();
  
  /// Adds a new vertex in the graph. Operates in constant time.
  /// @param vertex_data the value to store in the vertex (a copy).
  /// @return the descriptor of the new vertex added.
  vertex_descriptor add_vertex (const Vdata& vertex_data);

  /// Adds edge \(u, v\) to the graph and returns the edge descriptor for the 
  /// new edge. For graphs that not allow parallel edges, if the edge is 
  /// already in the graph, then a duplicate will not be added and the bool
  /// flag will be false. When the flag is false, the edge descriptor is 
  /// invalid, and any use of it is undefined. Builds the edge data with 
  /// default constructor. The complexity is O(log(out(u)) + log(in(v))), where
  /// out(u) is the number of out edges of u and in(v) is the number of in 
  /// edges of v.
  /// @param u the source of the edge to be added
  /// @param v the target of the edge to be added
  /// @return a pair, where pair::first is the descriptor of the edge added
  ///  and pair::second is true iff the edge is added.
  std::pair<edge_descriptor, bool> 
  add_edge (vertex_descriptor u, vertex_descriptor v);

  /// Adds edge \(u, v\) to the graph and returns the edge descriptor for the 
  /// new edge. For graphs that not allow parallel edges, if the edge is 
  /// already in the graph, then a duplicate will not be added and the bool
  /// flag will be false. When the flag is false, the edge descriptor is 
  /// invalid, and any use of it is undefined. Stores by value the data param
  /// in the edge. The complexity is O(log(out(u)) + log(in(v))), where  
  /// out(u) is the number of out edges of u and in(v) is the number of in 
  /// edges of v.
  /// @param u the source of the edge to be added
  /// @param v the target of the edge to be added
  /// @param data the data to be stored in the edge (a copy)
  /// @return a pair, where pair::first is the descriptor of the edge added
  ///  and pair::second is true iff the edge is added.
  std::pair<edge_descriptor, bool> 
  add_edge (vertex_descriptor u, vertex_descriptor v, const Edata& data);

  /// Removes the edge e from the graph. This differ from remove_edge \(u, v\)
  /// function in case of multigraph. This function remove a single edge.
  /// Invalidates any iterator and descriptor pointing to the edge. The 
  /// complexity is O(log(out(u)) + log(in(v))), where out(u) is the number of
  /// out edges of source(e) and in(v) is the number of in edges of target(e).
  /// @param e the edge to be removed
  void remove_edge (edge_descriptor e);

  /// Removes the edges between vertices u and v. Invalidates any iterator 
  /// pointing to the removed edges. The complixity is O(log(out(u)) + 
  /// log(in(v))), where out(u) is the number of out edges of source(e) and 
  /// in(v) is the number of in edges of target(e).
  /// @param u the source vertex
  /// @param v the target vertex
  /// @return the number of removed edges
  std::size_t remove_edge (vertex_descriptor u, vertex_descriptor v);
  
  /// Removes all edges from the graph that satisfy the predicate. That is
  /// if the predicate returns true when applied to an edge_descriptor, then
  /// the edge is removed. The complexity is O (E log(E/V)) where V is the 
  /// number of vertices and E is the number of edges in the graph.
  /// @param predicate the predicate
  template <typename Predicate>
  void remove_edge_if (Predicate predicate);
  
  /// Removes all out-edges of vertex v that satisfy the predicate. That is
  /// if the predicate returns true when applied to an edge_descriptor, then
  /// the edge is removed. The complexity is O (E log(E)) where V is the E 
  /// is the number of out edges of the vertex.
  /// @param v the vertex with the out-edges to be removed
  /// @param predicate the predicate
  template <typename Predicate>
  void remove_out_edge_if (vertex_descriptor v, Predicate predicate);
  
  /// Removes all in-edges of vertex v that satisfy the predicate. That is
  /// if the predicate returns true when applied to an edge_descriptor, then
  /// the edge is removed. The complexity is O (E log(E)) where V is the E 
  /// is the number of in edges of the vertex.
  /// @param v the vertex with the in-edges to be removed
  /// @param predicate the predicate
  template <typename Predicate>
  void remove_in_edge_if (vertex_descriptor v, Predicate predicate);

  /// Removes edges from vertex u. The incoming edges still appears in the
  /// edge set. The complexity is O (log E), where E is the number of edges.
  /// @param u the vertex to be cleared
  void clear_out_edges (vertex_descriptor u);
  
  /// Removes edges to vertex v. The outgoing edges still appears in the
  /// edge set. The complexity is O (log E), where E is the number of edges.
  /// @param v the vertex to be cleared
  void clear_in_edges (vertex_descriptor v);

  /// Removes all the edges to and from the vertex u. The vertex still appears
  /// in the vertices set. The complexity is O (log E), where E is the number
  /// of edges.
  /// @param u the vertex to be cleared
  void clear_vertex (vertex_descriptor u);
  
  /// Removes vertex u from the vertices set. Clear also the in and out edges
  /// of the vertex. The complexity is constant if the vertex has no incoming
  /// or outgoing edges, otherwise O (log E), where E is the number of edges.
  /// All the iterators and descriptors pointing to u or to edges of u will be
  /// invalidated.
  /// @param u the vertex to be removed
  void remove_vertex (vertex_descriptor u);
  
  /// Removes all vertices from the graph that satisfy the predicate. That is
  /// if the predicate returns true when applied to a vertex_descriptor, then
  /// the vertex is removed. The complexity is O (V log(E/V)) where V is the 
  /// number of vertices and E is the number of edges in the graph.
  /// @param predicate the predicate
  template <typename Predicate>
  void remove_vertex_if (Predicate predicate);

private:
  /// Disable copy construction
  graph_t (const graph_t&);

  /// The vertices list
  typename Config::vertex_list_type _vertices;

  /// The edges list
  typename Config::edge_list_type _edges;

  /// Allocator for vertices
  typename Config::vertex_alloc _vertex_alloc;

  /// Allocator for edges
  typename Config::edge_alloc _edge_alloc;

  /// A fake edge container for adjacency search
  typename Config::edge_list_type _fake_edge_container;

  /// The fake edge descriptor
  edge_descriptor _fake_edge_desc;
};

/// @brief Vertex class.
///
/// Stores by value a copy of the template value passed by the constructor.
/// @tparam Vdata the data stored in the vertex.
/// @tparam Config the configuration class.
template <typename Vdata, typename Config>
class graph_vertex_t_
{
  friend class Config::graph_t_;

  typedef typename Config::edge_descriptor edge_descriptor;
  typedef typename Config::out_edge_list_type out_edge_list_type;
  typedef typename Config::in_edge_list_type in_edge_list_type;
  typedef typename Config::out_edge_iterator out_edge_iterator;
  typedef typename Config::in_edge_iterator in_edge_iterator;
  typedef typename Config::OutEdgesAdaptor OutEdgesAdaptor;
  typedef typename Config::InEdgesAdaptor InEdgesAdaptor;
  typedef typename Config::edge_t edge_t;

public:
  /// The data stored in the vertex
  Vdata data;

private:
  /// Disable user construction. Use the add_vertex provided by graph_t class
  graph_vertex_t_ ();
  graph_vertex_t_ (const Vdata& data_);
  
  /// Insert the edge in the out edges set
  /// @return true if the edge is inserted, false otherwise
  bool push_out_edges (edge_descriptor edge);
  
  /// Insert the edge in the in edges set
  /// @return true if the edge is inserted, false otherwise
  bool push_in_edges (edge_descriptor edge);

  /// Remove the out edge pointing to e
  void remove_out_edge (edge_descriptor e);
  
  /// Remove the in edge pointing to e
  void remove_in_edge (edge_descriptor e);
  
  /// Remove the all the out edges with the same target of e
  std::size_t remove_all_out_edges (edge_descriptor e);
  
  /// Remove the all the in edges with the same source of e
  std::size_t remove_all_in_edges (edge_descriptor e);
  
  /// The out edge  set
  out_edge_list_type _out_edges;
  
  /// The in edge set
  in_edge_list_type _in_edges;

};

/// @brief Edge class.
///
/// Stores by value a copy of the template value passed by the constructor.
/// @tparam Edata the data stored in the edge.
/// @tparam Config the configuration class
template <typename Edata, typename Config>
class graph_edge_t_
  : public Config::EdgeBase
{
  friend class Config::graph_t_;
  friend class Config::vertex_t;
  friend class Config::compare_in;
  friend class Config::compare_out;
  friend class Config::adjacency_iterator_base_;

  typedef typename Config::vertex_descriptor vertex_descriptor;
  
public:
  /// The data stored in the edge
  Edata data;

private:
  /// Disable user construction. Use the add_edge provided by graph_t class
  graph_edge_t_ ();
  
  graph_edge_t_ (const vertex_descriptor& source, 
                 const vertex_descriptor& target);
  
  graph_edge_t_ (const vertex_descriptor& source, 
                 const vertex_descriptor& target,
                 const Edata& data_);
  
  /// The descriptor of the source vertex
  vertex_descriptor _source;
  
  /// The descriptor of the target vertex
  vertex_descriptor _target;

};


// ===================== template implementation ===============================

// ------------------------------- graph ---------------------------------------

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline graph_t<Vdata, Edata, parallel, Allocator>::graph_t () 
{ 
  // fake edge container setup (no constructor call to avoid errors for the
  // Edata without default constructor)
  _fake_edge_container.push_back (_edge_alloc.allocate(1));
  _fake_edge_desc = _fake_edge_container.begin();
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline graph_t<Vdata, Edata, parallel, Allocator>::~graph_t () 
{ 
  clear();
  // no destructor call for the fake edge
  _edge_alloc.deallocate (*_fake_edge_container.begin(), 1);
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
void graph_t<Vdata, Edata, parallel, Allocator>::clear () 
{
  impl::deleter<typename Config::vertex_alloc> vertex_del (_vertex_alloc);
  impl::deleter<typename Config::edge_alloc> edge_del(_edge_alloc);
  std::for_each (_vertices.begin(), _vertices.end(), vertex_del);
  std::for_each (_edges.begin(), _edges.end(), edge_del);
  _vertices.clear();
  _edges.clear();
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::vertex_iterator, 
          typename graph_t<Vdata, Edata, parallel, Allocator>::vertex_iterator> 
inline graph_t<Vdata, Edata, parallel, Allocator>::vertices ()
{
  return std::make_pair (vertex_iterator(_vertices.begin()),
                         vertex_iterator(_vertices.end()));
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::edge_iterator, 
          typename graph_t<Vdata, Edata, parallel, Allocator>::edge_iterator> 
inline graph_t<Vdata, Edata, parallel, Allocator>::edges ()
{
  return std::make_pair (edge_iterator(_edges.begin()),
                         edge_iterator(_edges.end()));
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline 
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::out_edge_iterator, 
          typename graph_t<Vdata, Edata, parallel, Allocator>::out_edge_iterator> 
graph_t<Vdata, Edata, parallel, Allocator>::
out_edges (vertex_descriptor v)
{
  vertex_t* vertex = v.internal_value();
  return std::make_pair (vertex->_out_edges.begin(), 
                         vertex->_out_edges.end());
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline 
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::in_edge_iterator, 
          typename graph_t<Vdata, Edata, parallel, Allocator>::in_edge_iterator> 
graph_t<Vdata, Edata, parallel, Allocator>::
in_edges (vertex_descriptor v)
{
  vertex_t* vertex = v.internal_value();
  return std::make_pair (vertex->_in_edges.begin(), vertex->_in_edges.end());
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline typename graph_t<Vdata, Edata, parallel, Allocator>::vertex_descriptor 
graph_t<Vdata, Edata, parallel, Allocator>::source (edge_descriptor e)
{
  edge_t* edge = e.internal_value();
  return edge->_source;
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline typename graph_t<Vdata, Edata, parallel, Allocator>::vertex_descriptor 
graph_t<Vdata, Edata, parallel, Allocator>::target (edge_descriptor e)
{
  edge_t* edge = e.internal_value();
  return edge->_target;
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline std::size_t
graph_t<Vdata, Edata, parallel, Allocator>::
out_degree (vertex_descriptor u) const
{
  vertex_t* vertex = u.internal_value();
  return vertex->_out_edges.size();
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline std::size_t
graph_t<Vdata, Edata, parallel, Allocator>::
in_degree (vertex_descriptor u) const
{
  vertex_t* vertex = u.internal_value();
  return vertex->_in_edges.size();
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline std::size_t
graph_t<Vdata, Edata, parallel, Allocator>::num_vertices () const
{
  return _vertices.size();
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline std::size_t
graph_t<Vdata, Edata, parallel, Allocator>::num_edges () const
{
  return _edges.size();
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::edge_descriptor,
          bool>
graph_t<Vdata, Edata, parallel, Allocator>::
edge (vertex_descriptor u, vertex_descriptor v)
{
  vertex_t* source = u.internal_value();
  _fake_edge_desc.internal_value()->_target = v;
  out_edge_iterator it = source->_out_edges.find (_fake_edge_desc);
  return std::make_pair (*it, it != source->_out_edges.end());
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::out_edge_iterator,
          typename graph_t<Vdata, Edata, parallel, Allocator>::out_edge_iterator>
graph_t<Vdata, Edata, parallel, Allocator>::
edge_range (vertex_descriptor u, vertex_descriptor v)
{
  vertex_t* source = u.internal_value();
  _fake_edge_desc.internal_value()->_target = v;
  return source->_out_edges.equal_range (_fake_edge_desc);
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::adjacency_iterator, 
          typename graph_t<Vdata, Edata, parallel, Allocator>::adjacency_iterator>
graph_t<Vdata, Edata, parallel, Allocator>::
adjacent_vertices (vertex_descriptor v)
{
  vertex_t* vertex = v.internal_value();
  return std::make_pair
    (adjacency_iterator(vertex->_out_edges.begin(), vertex->_out_edges),
     adjacency_iterator(vertex->_out_edges.end(), vertex->_out_edges));
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline typename graph_t<Vdata, Edata, parallel, Allocator>::vertex_descriptor
graph_t<Vdata, Edata, parallel, Allocator>::
add_vertex ()
{
  return add_vertex (Vdata());
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
typename graph_t<Vdata, Edata, parallel, Allocator>::vertex_descriptor 
graph_t<Vdata, Edata, parallel, Allocator>::
add_vertex (const Vdata& data)
{
  vertex_t* vertex = _vertex_alloc.allocate (1);
  _vertex_alloc.construct (vertex, vertex_t(data));
  _vertices.push_back (vertex);
  return vertex_descriptor(--_vertices.end());
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::edge_descriptor, 
          bool> 
graph_t<Vdata, Edata, parallel, Allocator>::
add_edge (vertex_descriptor u, vertex_descriptor v)
{
  return add_edge (u, v, Edata());
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
std::pair<typename graph_t<Vdata, Edata, parallel, Allocator>::edge_descriptor, 
          bool> 
graph_t<Vdata, Edata, parallel, Allocator>::
add_edge (vertex_descriptor u, vertex_descriptor v, const Edata& data)
{
  edge_t* edge = _edge_alloc.allocate (1);
  _edge_alloc.construct (edge, edge_t (u, v, data));
  // first add the new edge in the edge list
  _edges.push_back (edge);
  edge_descriptor edge_desc = edge_descriptor(--_edges.end());
  // add the edge to the out edge list of the source
  vertex_t* source = u.internal_value();
  if (!source->push_out_edges (edge_desc)) {
    assert (!parallel);
    _edge_alloc.destroy (edge);
    _edge_alloc.deallocate (edge, 1);
    _edges.pop_back();
    return std::make_pair (edge_desc, false);
  }
  // add the edge to the in edge list of the source
  vertex_t* target = v.internal_value();
  bool inserted = target->push_in_edges (edge_desc);
  assert (inserted);
  // all the work is done, return the pair
  return std::make_pair (edge_desc, true);
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
void 
graph_t<Vdata, Edata, parallel, Allocator>::
remove_edge (edge_descriptor e)
{
  edge_t* edge = e.internal_value();
  vertex_t* source = edge->_source.internal_value();
  vertex_t* target = edge->_target.internal_value();
  source->remove_out_edge(e);
  target->remove_in_edge(e);
  _edge_alloc.destroy (edge);
  _edge_alloc.deallocate (edge, 1);
  _edges.erase (e);
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
std::size_t 
graph_t<Vdata, Edata, parallel, Allocator>::
remove_edge (vertex_descriptor u, vertex_descriptor v)
{
  vertex_t* source = u.internal_value();
  vertex_t* target = v.internal_value();
  edge_t* reference = _fake_edge_desc.internal_value();
  reference->_source = u;
  reference->_target = v;
  std::pair<out_edge_iterator, out_edge_iterator> equal = 
    source->_out_edges.equal_range (_fake_edge_desc);
  // this is because the deletion of the edges must be done after the erasing
  // of the out and in edges lists, to avoid memory errors
  typedef typename Allocator::template rebind<edge_descriptor>::other QAlloc;
  typedef std::deque<edge_descriptor, QAlloc> deletion_que_t;
  deletion_que_t deletion_que (equal.first, equal.second);
  source->remove_all_out_edges (_fake_edge_desc);
  std::size_t ret = target->remove_all_in_edges (_fake_edge_desc);
  for (typename deletion_que_t::iterator it = deletion_que.begin();
       it != deletion_que.end(); ++it) {
    edge_t* e = it->internal_value();
    _edge_alloc.destroy (e);
    _edge_alloc.deallocate (e, 1);
    _edges.erase (*it);
  }
  return ret;
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
template <typename Predicate>
void graph_t<Vdata, Edata, parallel, Allocator>::
remove_edge_if (Predicate predicate)
{
  edge_iterator it, end, next;
  gtl::tie (it, end) = edges();
  while (it != end) {
    next = it;
    ++next;
    edge_descriptor edge = *it;
    if (predicate (edge))
      remove_edge (edge);
    it = next;
  }
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
template <typename Predicate>
void graph_t<Vdata, Edata, parallel, Allocator>::
remove_out_edge_if (vertex_descriptor v, Predicate predicate)
{
  out_edge_iterator it, end, next;
  gtl::tie (it, end) = out_edges (v);
  while (it != end) {
    next = it;
    ++next;
    edge_descriptor edge = *it;
    if (predicate (edge))
      remove_edge (edge);
    it = next;
  }
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
template <typename Predicate>
void graph_t<Vdata, Edata, parallel, Allocator>::
remove_in_edge_if (vertex_descriptor v, Predicate predicate)
{
  in_edge_iterator it, end, next;
  gtl::tie (it, end) = in_edges (v);
  while (it != end) {
    next = it;
    ++next;
    edge_descriptor edge = *it;
    if (predicate (edge))
      remove_edge (edge);
    it = next;
  }
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
void graph_t<Vdata, Edata, parallel, Allocator>::
clear_out_edges (vertex_descriptor u)
{
  std::pair<out_edge_iterator, out_edge_iterator> edge_range = out_edges (u); 
  while (edge_range.first != edge_range.second) {
    edge_descriptor edge_reached = *edge_range.first;
    vertex_descriptor v = edge_reached.internal_value()->_target;
    remove_edge (u, v);
    edge_range = out_edges (u);
  }
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
void graph_t<Vdata, Edata, parallel, Allocator>::
clear_in_edges (vertex_descriptor v)
{
  std::pair<out_edge_iterator, out_edge_iterator> edge_range = in_edges (v); 
  while (edge_range.first != edge_range.second) {
    edge_descriptor edge_reached = *edge_range.first;
    vertex_descriptor u = edge_reached.internal_value()->_source;
    remove_edge (u, v);
    edge_range = in_edges (v);
  }
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline void graph_t<Vdata, Edata, parallel, Allocator>::
clear_vertex (vertex_descriptor u)
{
  clear_in_edges (u);
  clear_out_edges (u);
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
inline void graph_t<Vdata, Edata, parallel, Allocator>::
remove_vertex (vertex_descriptor u)
{
  clear_vertex (u);
  vertex_t* vertex = u.internal_value();
  _vertex_alloc.destroy (vertex);
  _vertex_alloc.deallocate (vertex, 1);
  _vertices.erase (u);
}

template <typename Vdata, typename Edata, bool parallel, typename Allocator>
template <typename Predicate>
void graph_t<Vdata, Edata, parallel, Allocator>::
remove_vertex_if (Predicate predicate)
{
  vertex_iterator it, end, next;
  gtl::tie (it, end) = vertices();
  while (it != end) {
    next = it;
    ++next;
    vertex_descriptor vertex = *it;
    if (predicate (vertex))
      remove_vertex (vertex);
    it = next;
  }
}


// ------------------------------- vertex --------------------------------------

template <typename Vdata, typename Config>
inline graph_vertex_t_<Vdata, Config>::
graph_vertex_t_ () 
  : data () {}

template <typename Vdata, typename Config>
inline graph_vertex_t_<Vdata, Config>::
graph_vertex_t_ (const Vdata& data_) 
  : data (data_) {}

template <typename Vdata, typename Config>
inline bool graph_vertex_t_<Vdata, Config>::
push_out_edges (edge_descriptor edge)
{
  std::pair<out_edge_iterator, bool> inserted =
    OutEdgesAdaptor::insert (_out_edges, edge);
  edge_t* e = edge.internal_value();
  e->out_edge_position (inserted.first);
  return inserted.second;
}

template <typename Vdata, typename Config>
inline bool graph_vertex_t_<Vdata, Config>::
push_in_edges (edge_descriptor edge)
{
  std::pair<out_edge_iterator, bool> inserted = 
    InEdgesAdaptor::insert (_in_edges, edge);
  edge_t* e = edge.internal_value();
  e->in_edge_position (inserted.first);
  return inserted.second;
}

template <typename Vdata, typename Config>
inline void graph_vertex_t_<Vdata, Config>::
remove_out_edge (edge_descriptor edge)
{
  edge_t* e = edge.internal_value();
  e-> template remover_out<OutEdgesAdaptor>(_out_edges, edge);
}

template <typename Vdata, typename Config>
inline void graph_vertex_t_<Vdata, Config>::
remove_in_edge (edge_descriptor edge)
{
  edge_t* e = edge.internal_value();
  e-> template remover_in<InEdgesAdaptor>(_in_edges, edge);
}

template <typename Vdata, typename Config>
inline std::size_t graph_vertex_t_<Vdata, Config>::
remove_all_out_edges (edge_descriptor edge)
{
  return OutEdgesAdaptor::remove (_out_edges, edge);
}

template <typename Vdata, typename Config>
inline std::size_t graph_vertex_t_<Vdata, Config>::
remove_all_in_edges (edge_descriptor edge)
{
  return InEdgesAdaptor::remove (_in_edges, edge);
}


// -------------------------------- edge ---------------------------------------

template <typename Edata, typename Config>
inline graph_edge_t_<Edata, Config>::
graph_edge_t_ (const vertex_descriptor& source, 
               const vertex_descriptor& target)
  : data (), _source (source), _target (target) {}

template <typename Edata, typename Config>
inline graph_edge_t_<Edata, Config>::
graph_edge_t_ () {}

template <typename Edata, typename Config>
inline graph_edge_t_<Edata, Config>::
graph_edge_t_ (const vertex_descriptor& source, 
               const vertex_descriptor& target,
               const Edata& data_) 
  : data (data_), _source (source), _target (target) {}


} // namespace gtl

#endif  // GTL_GRAPH_T_HH
