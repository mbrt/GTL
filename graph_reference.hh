// =============================================================================
// 
//       Filename:  graph.hh
// 
//    Description:  A template set of classes representing a graph.
//
//         Author:  Michele Bertasi 
//                  Giuseppe Di Guglielmo
//        Contact:  giuseppe.diguglielmo@univr.it
//                  vr076644@studenti.univr.it
//      Copyright:  Copyright (c) 2009, Giuseppe Di Guglielmo
//        Company:  University of Verona - ESD Group
//        License:  GNU Lesser General Public License (GNU LGPL)
//
//      Agreement:                
//       This file is part of 'Phase 1'.
//       'Phase 1' is free software: you can redistribute it and/or
//       modify it under the terms of the GNU Lesser General Public License 
//       as published by the Free Software Foundation, either version 3 of 
//       the License, or (at your option) any later version.
//
//       'Phase 1' is distributed in the hope that it will be useful,
//       but WITHOUT ANY WARRANTY; without even the implied warranty of
//       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//       GNU Lesser General Public License for more details.
//
//       You should have received a copy of the GNU Lesser General Public 
//       License along with 'Phase 1'. 
//       If not, see <http://www.gnu.org/licenses/>.
// 
// =============================================================================

#ifndef GRAPH_T_HH
#define GRAPH_T_HH

#include <map>
#include <list>
#include <utility>
#include <cassert>
#include <exception>
#include <string>


namespace utils
{

template <typename Vdata, typename Edata, bool allow_parallel_edges>
class graph_t;
template <typename Vdata, typename Edata, bool allow_parallel_edges>
class graph_vertex_t;
template <typename Vdata, typename Edata, bool allow_parallel_edges>
class graph_edge_t;

/// Internal use only
template <typename Vdata, typename Edata, bool allow_parallel_edges>
class _PeHelper;
template <typename Vdata, typename Edata, bool allow_parallel_edges>
struct _MapType;

/// An empty struct that contains no data
struct NoData 
{
  template <typename Vdata, typename Edata, bool allow_parallel_edges>
  friend class graph_vertex_t;
  template <typename Vdata, typename Edata, bool allow_parallel_edges>
  friend class graph_edge_t;
  
private:
  /// Disable user construction
  NoData () {};
  NoData (const NoData& n);
  NoData& operator= (const NoData& n);
};

/// Helper struct to determine the map type used by adjacendy lists, do not 
/// use it directly
template <typename Vertex, typename EdgeIt>
struct _MapType <Vertex, EdgeIt, false>
{
  /// The map type with allow_parallel_edges = false
  typedef std::map <Vertex*, EdgeIt> map_type;
};

/// Helper struct to determine the map type used by adjacendy lists, do not 
/// use it directly
template <typename Vertex, typename EdgeIt>
struct _MapType <Vertex, EdgeIt, true>
{
  /// The map type with allow_parallel_edges = true
  typedef std::multimap <Vertex*, EdgeIt> map_type;
};

/// Exception class for graph
class graph_exception : public std::exception
{
  public:
  /// The exception constructor.
  /// @param msg The exception message.
  graph_exception (const std::string& msg) throw ()
  {
    _message = "\nGraph Exception: ";
    _message += msg;
    _message += "\n";
  };
  
  ~graph_exception () throw () 
  { };

  /// This function return the exception message.
  /// @return The exception message.
  const char* what () const throw()
  {
    return _message.c_str();
  };

private:
  /// The exception message.
  std::string _message;
};
    
///
/// @brief Graph class with directed and parallel edges.
/// @author Michele Bertasi, Giuseppe Di Guglielmo.
///
/// The graph.hh library for c++ provides an STL-like container for graphs, 
/// templated over the data stored at the vertices and the edges. Various types 
/// of iterators are provided. The edges are directed and there are the
/// possibility to permits parallel edges or not.
/// 
/// @tparam Vdata the data stored in the vertices. You can provide the special
///  value NoData.
/// @tparam Edata the data stored in the edges. You can provide the special
///  value NoData.
/// @tparam allow_parallel_edges if is set to true the graph allows parallel
///  edges.
///
template <typename Vdata, 
          typename Edata, 
          bool allow_parallel_edges = false>
class graph_t
{
  typedef graph_vertex_t <Vdata, Edata, allow_parallel_edges> vertex_t_;
  typedef graph_edge_t <Vdata, Edata, allow_parallel_edges> edge_t_;
  typedef typename std::list <vertex_t_*>::iterator _base_vertex_it;
  typedef typename std::list <edge_t_*>::iterator _base_edge_it;

  friend class graph_vertex_t <Vdata, Edata, allow_parallel_edges>;
  friend class graph_edge_t <Vdata, Edata, allow_parallel_edges>;
  friend class NoData;
  friend class _PeHelper <Vdata, Edata, allow_parallel_edges>;
  
public:
  /// Base class for iterators: do not use it directly.
  /// Has no operator * and ->.
  /// @tparam _Iterator the base iterator stored.
  template <typename _Iterator>
  class iterator_base
  {
  public:
    friend class graph_t;
    friend class _PeHelper <Vdata, Edata, allow_parallel_edges>;
    
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef std::bidirectional_iterator_tag iterator_category;
    
    iterator_base ();
    iterator_base (const _Iterator& i);
    iterator_base (const iterator_base& i);
    
    bool operator== (const iterator_base& i) const;
    bool operator!= (const iterator_base& i) const;
    iterator_base& operator++ ();
    iterator_base operator++ (int);
    iterator_base& operator-- ();
    iterator_base operator-- (int);
    bool operator< (const iterator_base& i) const;
    
  protected:
    /// The base iterator stored.
    _Iterator _it;
      
  };
  
  /// @brief Iterator for graph_vertex_t. 
  ///
  /// Accesses only vertices. All the operator are provided by the
  /// iterator_base class (exept * and ->).
  class vertex_iterator : public iterator_base <_base_vertex_it>
  {
  public:
    typedef iterator_base <_base_vertex_it> Base;
    typedef Vdata value_type;
    typedef vertex_t_* pointer;
    typedef vertex_t_& reference;
    
    vertex_iterator ();
    vertex_iterator (const _base_vertex_it& v);
    vertex_iterator (const vertex_iterator& v);
    vertex_iterator (const Base& i);
    
    reference operator* () const;
    pointer operator-> () const;
  };
  
  /// @brief Iterator for graph_edge_t. 
  ///
  /// Accesses only edges in insertion the order. 
  /// All the operator are provided by the iterator_base class (exept * and ->).
  class edge_iterator : public iterator_base <_base_edge_it>
  {
  public:
    typedef iterator_base <_base_edge_it> Base;
    typedef Edata value_type;
    typedef edge_t_* pointer;
    typedef edge_t_& reference;
    
    edge_iterator ();
    edge_iterator (const _base_edge_it& e);
    edge_iterator (const edge_iterator& e);
    edge_iterator (const Base& i);
    
    reference operator* () const;
    pointer operator-> () const;
  };
  
private:
  typedef typename _MapType <vertex_t_, 
                             edge_iterator, 
                             allow_parallel_edges>::map_type map_type;
  typedef typename map_type::iterator _base_adj_edge_it;

public:
  /// @brief Iterator for graph_edge_t. 
  ///
  /// Accesses only out edges or in edges of a are provided by the 
  /// iterator_base class (exept * and ->).
  class adj_edge_iterator : public iterator_base <_base_adj_edge_it>
  {
  public:
    typedef iterator_base <_base_adj_edge_it> Base;
    typedef Edata value_type;
    typedef edge_t_* pointer;
    typedef edge_t_& reference;
    
    adj_edge_iterator ();
    adj_edge_iterator (const _base_adj_edge_it& e);
    adj_edge_iterator (const adj_edge_iterator& e);
    adj_edge_iterator (const Base& i);
    
    reference operator* () const;
    pointer operator-> () const;
    edge_iterator get_edge_it () const;
  };
  
  typedef std::pair <adj_edge_iterator, adj_edge_iterator> edge_iterator_range;

  /// Constructor.
  explicit graph_t ();
  
  /// Destructor. This calls each of the contained element's destructors if
  /// are not pointers, and deallocates all the storage capacity allocated by 
  /// the graph_t container.
  ~graph_t ();
  
  /// Delete all vertices and edges. No memory is freed if the vertex or edge
  /// data are pointers.
  void clear ();
  
  /// Returns the number of vertices
  size_t number_of_vertices () const;
  
  /// Return the number of edges
  size_t number_of_edges () const;
  
  /// Returns an iterator to the begin of vertex list
  vertex_iterator vertices_begin ();
  
  /// Returns an iterator one past the end of vertex list
  vertex_iterator vertices_end ();
  
  /// Returns an iterator to the begin of edge list
  edge_iterator edges_begin ();
  
  /// Returns an iterator one past the end of edge list
  edge_iterator edges_end ();
  
  /// Get an edge between source and target if is present. Notice that if the 
  /// graph allows parallel edges, this function returns an iterator to only 
  /// one of the edges that connect the two vertex. To obtain the entire range 
  /// of edges between the two vertex use getall_edges. The time complexity is 
  /// O(log E), where E is the number of edges in the graph.
  /// @param source the vertex source of the edge to found
  /// @param target the vertex source of the edge to found
  /// @return an iterator to the edge founded if is present, an iterator to
  ///  edges_end otherwise.
  edge_iterator get_edge (vertex_iterator source, 
                          vertex_iterator target);
  
  /// Get all the edges between source and target if are present. If the graph
  /// not allows parallel edges the operation is also permitted. The time 
  /// complexity is O(log E), where E is the number of edges in the graph.
  /// @param source the vertex source of the edge to found
  /// @param target the vertex source of the edge to found
  /// @return a pair, where its member pair::first is an iterator to the lower 
  /// bound of the range of the edges between the two vertex, and pair::second 
  /// is an iterator to the upper bound of the range of the edges between the 
  /// two vertex.
  edge_iterator_range getall_edges (vertex_iterator source, 
                                    vertex_iterator target);
  
  /// Checks if an edge between source and target exists. The time
  /// complexity is O(log E), where E is the number of edges in the graph.
  /// @param source the vertex source of the edge to found
  /// @param target the vertex source of the edge to found
  /// @return true if the edge is found, false otherwise
  bool exists_edge (vertex_iterator source, 
                    vertex_iterator target);
  
  /// Adds a new vertex in the graph. Operates in constant time.
  /// Builds the vertex data with default constructor.
  /// @return an iterator to the new vertex added.
  vertex_iterator add_vertex ();
  
  /// Adds a new vertex in the graph. Operates in constant time.
  /// @param vertex_data the value to be stored in the vertex (a copy).
  /// @return an iterator to the new vertex added.
  vertex_iterator add_vertex (const Vdata& vertex_data);
  
  /// Adds a new edge in the graph if is not already present, throws an 
  /// exception otherwise. The time complexity is O(log E), where E is the 
  /// number of edges in the graph. Builds the edge data with default
  /// constructor.
  /// @param source the vertex source of the edge to be added
  /// @param target the vertex source of the edge to be added
  /// @return an iterator to the edge added in the graph
  /// @throw graph_exception if an edge between source and target is already
  ///  present and the graph not allow parallel edges.
  edge_iterator add_edge (vertex_iterator source, 
                          vertex_iterator target)
                          throw (graph_exception);
                          
  /// Adds a new edge in the graph if is not already present, throws an 
  /// exception otherwise. The time complexity is O(log E), where E is the 
  /// number of edges in the graph.
  /// @param source the vertex source of the edge to be added
  /// @param target the vertex source of the edge to be added
  /// @param edge_data the data to be stored in the edge (a copy).
  /// @return an iterator to the edge added in the graph
  /// @throw graph_exception if an edge between source and target is already
  ///  present and the graph not allow parallel edges.
  edge_iterator add_edge (vertex_iterator source, 
                          vertex_iterator target, 
                          const Edata& edge_data)
                          throw (graph_exception);
  
  /// Move an edge at new point of the graph. If new_source and new_target
  /// are equal to previous source and target, no work are made to move
  /// the edge. All the adj_edge_iterator pointing to the edge are invalidated.
  /// All the edge_iterator pointing to the edge are valid. The time complexity 
  /// is O(log E), where E is the number of edges in the graph.
  /// @param edge the edge to move
  /// @param new_source the new source of the edge
  /// @param new_target the new target of the edge
  void move_edge (edge_iterator edge,
                  vertex_iterator new_source, 
                  vertex_iterator new_target);
  
  /// Remove a vertex from the graph. It removes also all the edges incident 
  /// with the vertex. The time complexity is O(log E), where E is the 
  /// number of edges in the graph.
  /// @param v the vertex to be removed
  void remove_vertex (vertex_iterator v);
  
  /// Remove an edge from the graph. The time complexity is O(log E), where 
  /// E is the  number of edges in the graph.
  /// @param e the vertex to be removed
  void remove_edge (edge_iterator e);
  
  /// Remove an edge from the graph. The time complexity is O(log E), where 
  /// E is the  number of edges in the graph.
  /// @param e the vertex to be removed
  void remove_edge (const adj_edge_iterator& e);
  
  /// Remove all the out edges of the given vertex. The time complexity is 
  /// O(log E), where E is the  number of edges in the graph.
  /// @param v the vertex
  void remove_out_edges (vertex_iterator v);
  
  /// Remove all the in edges of the given vertex. The time complexity is 
  /// O(log E), where E is the  number of edges in the graph.
  /// @param v the vertex
  void remove_in_edges (vertex_iterator v);

private:
  /// The vertex list.
  std::list <vertex_t_*> _vertices;
  
  /// The edge list.
  std::list <edge_t_*> _edges;

};

/// @brief Vertex class.
///
/// Stores by value a copy of the template value passed by the constructor.
/// @tparam Vdata the data stored in the vertices. You can provide the special
///  value NoData.
/// @tparam Edata the data stored in the edges. You can provide the special
///  value NoData.
/// @tparam allow_parallel_edges if is set to true the graph allows parallel
///  edges.
template <typename Vdata, typename Edata, bool allow_parallel_edges>
class graph_vertex_t
{ 
  friend class graph_t <Vdata, Edata, allow_parallel_edges>;
  friend class _PeHelper<Vdata, Edata, allow_parallel_edges>;
  typedef graph_t <Vdata, Edata, allow_parallel_edges> graph_t_;
  typedef graph_vertex_t<Vdata, Edata, allow_parallel_edges> vertex_t_;
  typedef typename graph_t_::adj_edge_iterator adj_edge_iterator_;
  typedef typename graph_t_::edge_iterator edge_iterator_;
  typedef typename graph_t_::vertex_iterator vertex_iterator_;
  typedef typename _MapType <graph_vertex_t <Vdata, Edata, allow_parallel_edges>, 
                             edge_iterator_, 
                             allow_parallel_edges>::map_type map_type;
  typedef typename map_type::iterator adj_multimap_it_;
  typedef typename std::pair <adj_edge_iterator_, adj_edge_iterator_> adj_edge_range_;
  
public:
  /// Return the number of out edges
  size_t out_degree () const;
  
  /// Return the number of in edges
  size_t in_degree () const;
  
  /// Return an iterator to the begin of out edges list.
  adj_edge_iterator_ out_edges_begin ();
  
  /// Return an iterator one past the end of out edges list.
  adj_edge_iterator_ out_edges_end ();
  
  /// Return an iterator to the begin of in edges list.
  adj_edge_iterator_ in_edges_begin ();
  
  /// Return an iterator one past the end of in edges list.
  adj_edge_iterator_ in_edges_end ();
  
  /// Search an edge that connect the vertex given as parameter to this 
  /// vertex. Notice that if the graph allows parallel edges this function 
  /// returns an iterator to only one of the edges that connect the two 
  /// vertex. To obtain the entire range of edges between the two vertex use 
  /// getall_in_edges. The time complexity is O(log E), where E is the number 
  /// of edges in the graph.
  /// @param from the source vertex of the edge to be found
  /// @return an iterator to an edge that connect the vertex from to this 
  ///  vertex if exists, an iterator to in_edges_end otherwise.
  adj_edge_iterator_ get_in_edge (const vertex_iterator_& from);
  
  /// Search all the edges that connect the vertex given as parameter to this 
  /// vertex. The time complexity is O(log E), where E is the number of edges
  /// in the graph.
  /// @param from the source vertex of the edges to be found
  /// @return a pair, where its member pair::first is an iterator to the lower 
  /// bound of the range of the edges between the two vertex, and pair::second 
  /// is an iterator to the upper bound of the range of the edges between the 
  /// two vertex.
  adj_edge_range_ getall_in_edges (const vertex_iterator_& from);
  
  /// Search an edge that connect this vertex to the vertex given as 
  /// parameter. Notice that if the graph allows parallel edges this function 
  /// returns an iterator to only one of the edges that connect the two 
  /// vertex. To obtain the entire range of edges between the two vertex use 
  /// getall_in_edges. The time complexity is O(log E), where E is the number 
  /// of edges in the graph.
  /// @param to the target vertex of the edge to be found
  /// @return an iterator to an edge that connect this vertex to the vertex
  ///  given as parameter if exists, an iterator that compares equal to 
  ///  out_edges_end otherwise.
  adj_edge_iterator_ get_out_edge (const vertex_iterator_& to);
  
  /// Search all the edges that connect this vertex to the vertex given as 
  /// parameter. The time complexity is O(log E), where E is the number of 
  /// edges in the graph.
  /// @param to the target vertex of the edges to be found
  /// @return a pair, where its member pair::first is an iterator to the lower 
  /// bound of the range of the edges between the two vertex, and pair::second 
  /// is an iterator to the upper bound of the range of the edges between the 
  /// two vertex.
  adj_edge_range_ getall_out_edges (const vertex_iterator_& to);
  
  /// The data stored in the vertex
  Vdata data;

private:
  /// Disable user construction. Use the add_vertex provided by graph_t class
  graph_vertex_t ();
  graph_vertex_t (const Vdata& data_);
  graph_vertex_t (const graph_vertex_t& v);
  graph_vertex_t& operator= (const graph_vertex_t& v);
  
  /// The multimap that associates the target vertex to an iterator to the
  /// respective edge
  map_type _out_edges;
  
  /// The multimap that associates the source vertex to an iterator to the
  /// respective edge
  map_type _in_edges;
};

/// Helper struct for the edge type
template <typename Vdata, typename Edata, bool allow_parallel_edges>
struct _EdgeConfig;

/// If no parallel edges are requred, no further data required
template <typename Vdata, typename Edata>
struct _EdgeConfig <Vdata, Edata, false>
{ };

/// If parallel edges are requred, two iterator to the adjacency lists of the
/// source and target vertex are stored.
template <typename Vdata, typename Edata>
struct _EdgeConfig <Vdata, Edata, true>
{ 
  typedef graph_t<Vdata, Edata, true> graph_t_;
  typedef typename graph_t_::adj_edge_iterator adj_edge_iterator_;
  
  adj_edge_iterator_ _source_out_iter;
  adj_edge_iterator_ _target_in_iter;
};

/// @brief Edge class.
///
/// Stores by value a copy of the template value passed by the constructor.
/// @tparam Vdata the data stored in the vertices. You can provide the special
///  value NoData.
/// @tparam Edata the data stored in the edges. You can provide the special
///  value NoData.
/// @tparam allow_parallel_edges if is set to true the graph allows parallel
///  edges.
template <typename Vdata, typename Edata, bool allow_parallel_edges>
class graph_edge_t : private _EdgeConfig <Vdata, Edata, allow_parallel_edges>
{
  friend class graph_t <Vdata, Edata, allow_parallel_edges>;
  friend class _PeHelper<Vdata, Edata, allow_parallel_edges>;
  typedef graph_t <Vdata, Edata, allow_parallel_edges> graph_t_;
  typedef typename graph_t_::adj_edge_iterator adj_edge_iterator_;
  typedef typename graph_t_::edge_iterator edge_iterator_;
  typedef typename graph_t_::vertex_iterator vertex_iterator_;

public:
  /// Return an iterator to the source of the edge.
  vertex_iterator_ get_source () const;
  
  /// Return an iterator to the target of the edge.
  vertex_iterator_ get_target () const;
  
  /// The data stored in the edge
  Edata data;
  
private:
  /// Disable user construction. Use the add_edge provided by graph_t class
  graph_edge_t (const vertex_iterator_& source, 
                const vertex_iterator_& target);
  
  graph_edge_t (const vertex_iterator_& source, 
                const vertex_iterator_& target, 
                const Edata& data_);
  
  graph_edge_t (const graph_edge_t& e);
  graph_edge_t& operator= (const graph_edge_t& e);
  
  /// An iterator to the source vertex
  vertex_iterator_ _source;
  
  /// An iterator to the target vertex
  vertex_iterator_ _target;
};

/// Helper class for adding edges to a graph. Do not use it directly.
/// Adds an edge with allows_parallel_edges = false
template <typename Vdata, typename Edata>
class _PeHelper <Vdata, Edata, false>
{
private:
  friend class graph_t <Vdata, Edata, false>;
  friend class graph_vertex_t <Vdata, Edata, false>;
  
  typedef graph_t <Vdata, Edata, false> graph_t_;
  typedef typename graph_t_::adj_edge_iterator adj_edge_iterator_;
  typedef typename graph_t_::edge_iterator edge_iterator_;
  typedef typename graph_t_::vertex_iterator vertex_iterator_;
  typedef graph_vertex_t <Vdata, Edata, false> vertex_t_;
  typedef graph_edge_t <Vdata, Edata, false> edge_t_;
  typedef typename _MapType <vertex_t_, 
                             edge_iterator_, 
                             false>::map_type map_type;
  
  /// Disable user construction
  _PeHelper ();
  _PeHelper (const _PeHelper& a);
  _PeHelper& operator= (const _PeHelper& a);
  
  /// Add an adge (parallel edges are not allowed!)
  static edge_iterator_ _add_edge (graph_t_& g,
                                   const vertex_iterator_& source, 
                                   const vertex_iterator_& target, 
                                   const Edata& edge_data)
                                   throw (graph_exception)
  {
    vertex_t_* dest = &(*target);
    vertex_t_* sour = &(*source);
    // check if the edge is already present
    typename graph_t_::_base_adj_edge_it it = source->_out_edges.find (dest);
    if (it != source->_out_edges.end())
      throw graph_exception ("The edge is already present");  
    // insert the edge in the graph edge list
    //default construction
    edge_t_* edge = new edge_t_ (source, target, edge_data);
    g._edges.push_back (edge);
    // insert the edge in the adjacency list of the source and the target
    edge_iterator_ e = edge_iterator_ (--g._edges.end());
    source->_out_edges.insert (it, typename map_type::value_type (dest, e));
    target->_in_edges.insert (typename map_type::value_type (sour, e));
    
    return e;
  }
  
  /// Add an adge with default construction (parallel edges are not allowed!)
  static edge_iterator_ _add_edge (graph_t_& g,
                                   const vertex_iterator_& source, 
                                   const vertex_iterator_& target)
                                   throw (graph_exception)
  {
    vertex_t_* dest = &(*target);
    vertex_t_* sour = &(*source);
    // check if the edge is already present
    typename graph_t_::_base_adj_edge_it it = source->_out_edges.find (dest);
    if (it != source->_out_edges.end())
      throw graph_exception ("The edge is already present");  
    // insert the edge in the graph edge list
    //default construction
    edge_t_* edge = new edge_t_ (source, target);
    g._edges.push_back (edge);
    // insert the edge in the adjacency list of the source and the target
    edge_iterator_ e = edge_iterator_ (--g._edges.end());
    source->_out_edges.insert (it, typename map_type::value_type (dest, e));
    target->_in_edges.insert (typename map_type::value_type (sour, e));
    
    return e;
  }
  
  /// Move and edge if parallel edges are not allowed
  static void _move_edge (graph_t_& g,
                          const edge_iterator_& edge,
                          const vertex_iterator_& new_source, 
                          const vertex_iterator_& new_target)
                          throw (graph_exception)
  {
    if (g.exists_edge (new_source, new_target))
      throw graph_exception ("Cannot move the edge: "
                             "an edge is already present in the position");
    vertex_iterator_ old_source = edge->get_source();
    vertex_iterator_ old_target = edge->get_target();
    if (new_source != old_source)
    {
      edge->_source = new_source;
      old_source->_out_edges.erase (* old_target._it);
      new_source->_out_edges.insert (typename map_type::value_type (* new_target._it, edge));
    }
    
    if (new_target != old_target)
    {
      edge->_target = new_target;
      old_target->_in_edges.erase (* old_source._it);
      new_target->_in_edges.insert (typename map_type::value_type (* new_source._it, edge));
    }
  }
  
  /// Remove an edge if parallel edges are not allowed
  static void _remove_edge (graph_t_& g, const edge_iterator_& e)
  {
    vertex_iterator_ target = e->get_target();
    vertex_iterator_ source = e->get_source();
    adj_edge_iterator_ out_e = source->get_out_edge (target);
    assert (out_e != source->out_edges_end());
    adj_edge_iterator_ in_e = target->get_in_edge (source);
    assert (in_e != target->in_edges_end());
    // erase the edge
    delete * e._it;
    g._edges.erase (e._it);
    // remove the edge from the adjacency list of the source and target
    source->_out_edges.erase (out_e._it);
    target->_in_edges.erase (in_e._it);
  }
  
};

/// Helper class for adding edges to a graph. Do not use it directly.
/// Adds an edge with allows_parallel_edges = true
template <typename Vdata, typename Edata>
class _PeHelper <Vdata, Edata, true>
{
private:
  friend class graph_t <Vdata, Edata, true>;
  friend class graph_vertex_t <Vdata, Edata, true>;
  
  typedef graph_t <Vdata, Edata, true> graph_t_;
  typedef typename graph_t_::adj_edge_iterator adj_edge_iterator_;
  typedef typename graph_t_::edge_iterator edge_iterator_;
  typedef typename graph_t_::vertex_iterator vertex_iterator_;
  typedef graph_vertex_t <Vdata, Edata, true> vertex_t_;
  typedef graph_edge_t <Vdata, Edata, true> edge_t_;
  typedef typename _MapType <vertex_t_, 
                             edge_iterator_, 
                             true>::map_type map_type;
  
  /// Disable user construction
  _PeHelper ();
  _PeHelper (const _PeHelper& a);
  _PeHelper& operator= (const _PeHelper& a);
  
  /// Add an adge (parallel edges are allowed!)
  static edge_iterator_ _add_edge (graph_t_& g,
                            const vertex_iterator_& source, 
                            const vertex_iterator_& target, 
                            const Edata& edge_data)
  {
    vertex_t_* dest = &(*target);
    vertex_t_* sour = &(*source);
    // insert the edge in the graph edge list
    //default construction
    edge_t_* edge = new edge_t_ (source, target, edge_data);
    g._edges.push_back (edge);
    // insert the edge in the adjacency list of the source and the target
    edge_iterator_ e = edge_iterator_ (--g._edges.end());
    typename map_type::iterator source_out_iter = 
      source->_out_edges.insert (typename map_type::value_type (dest, e));
    typename map_type::iterator target_in_iter = 
      target->_in_edges.insert (typename map_type::value_type (sour, e));
    // put in the edge the iterators in the adjacency lists of source and target
    edge->_source_out_iter = source_out_iter;
    edge->_target_in_iter = target_in_iter;
    
    return e;
  }
  
  /// Add an adge with default construction (parallel edges are allowed!)
  static edge_iterator_ _add_edge (graph_t_& g,
                                   const vertex_iterator_& source, 
                                   const vertex_iterator_& target)
  {
    vertex_t_* dest = &(*target);
    vertex_t_* sour = &(*source);
    // insert the edge in the graph edge list
    //default construction
    edge_t_* edge = new edge_t_ (source, target);
    g._edges.push_back (edge);
    // insert the edge in the adjacency list of the source and the target
    edge_iterator_ e = edge_iterator_ (--g._edges.end());
    typename map_type::iterator source_out_iter = 
      source->_out_edges.insert (typename map_type::value_type (dest, e));
    typename map_type::iterator target_in_iter = 
      target->_in_edges.insert (typename map_type::value_type (sour, e));
    // put in the edge the iterators in the adjacency lists of source and target
    edge->_source_out_iter = source_out_iter;
    edge->_target_in_iter = target_in_iter;
    
    return e;
  }
  
  /// Search for the out edge iterator of the source of the given edge.
  /// @return an iterator to the edge in the adjacency list of its source
  inline static adj_edge_iterator_ _find_out_iterator (const edge_iterator_& e)
  {
    return e->_source_out_iter;
  }
  
  /// Search for the in edge iterator of the target of the given edge.
  /// @return an iterator to the edge in the adjacency list of its target
  inline static adj_edge_iterator_ _find_in_iterator (const edge_iterator_& e)
  {
    return e->_target_in_iter;
  }
  
  /// Move and edge if parallel edges are allowed
  static void _move_edge (graph_t_& g,
                          const edge_iterator_& edge,
                          const vertex_iterator_& new_source, 
                          const vertex_iterator_& new_target)
                          throw (graph_exception)
  {
    vertex_iterator_ old_source = edge->get_source();
    vertex_iterator_ old_target = edge->get_target();
    if (new_source != old_source)
    {
      adj_edge_iterator_ it = _find_out_iterator (edge);
      old_source->_out_edges.erase (it._it);
      typename map_type::iterator source_out_iter = 
        new_source->_out_edges.insert (typename map_type::value_type (* new_target._it, edge));
      edge->_source_out_iter = source_out_iter;
    }
    
    if (new_target != old_target)
    {
      adj_edge_iterator_ it = _find_in_iterator (edge);
      old_target->_in_edges.erase (it._it);
      typename map_type::iterator target_in_iter = 
        new_target->_in_edges.insert (typename map_type::value_type (* new_source._it, edge));
      edge->_target_in_iter = target_in_iter;
    }
    
    if (new_source != old_source)
      edge->_source = new_source;
    if (new_target != old_target)
      edge->_target = new_target;
  }
  
  /// Remove an edge if parallel edges are allowed
  static void _remove_edge (graph_t_& g, const edge_iterator_& e)
  {
    typedef typename map_type::iterator map_it;
    
    vertex_t_* target = & (*e->get_target());
    vertex_t_* source = & (*e->get_source());
    
    // search for the correct out edge iterator in the source
    adj_edge_iterator_ it = _find_out_iterator (e);
    map_it out_e = it._it;
    // search for the correct in edge iterator in the target
    it = _find_in_iterator (e);
    map_it in_e = it._it;
    
    // erase the edge
    delete * e._it;
    g._edges.erase (e._it);
    // remove the edge from the adjacency list of the source and target
    source->_out_edges.erase (out_e);
    target->_in_edges.erase (in_e);
  }
  
};


// ===================== template implementation ===============================


// ----------------------------- iterators -------------------------------------

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
iterator_base () : _it() {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
iterator_base (const _Iterator& v)
  : _it (v) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
iterator_base (const iterator_base <_Iterator>& v)
  : _it (v._it) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline bool 
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator== (const iterator_base <_Iterator>& v) const
{
  return _it == v._it;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline bool 
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator!= (const iterator_base <_Iterator>& v) const
{
  return _it != v._it;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>& 
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator++ ()
{
  ++_it;
  return *this;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator++ (int)
{
  iterator_base<_Iterator> tmp = *this;
  ++_it;
  return tmp;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>& 
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator-- ()
{
  --_it;
  return *this;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator-- (int)
{
  iterator_base<_Iterator> tmp = *this;
  --_it;
  return tmp;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
template <typename _Iterator>
inline bool 
graph_t<Vdata, Edata, allow_parallel_edges>::iterator_base <_Iterator>::
operator< (const iterator_base& i) const
{
  return *_it < *i._it;
}


template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator::
vertex_iterator () 
  : Base () {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator::
vertex_iterator (const _base_vertex_it& v)  
  : Base (v) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator::
vertex_iterator (const vertex_iterator& v)  
  : Base (v._it) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>  
inline graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator::
vertex_iterator (const Base& i)
  : Base (i) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_vertex_t <Vdata, Edata, allow_parallel_edges>& 
graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator::
operator* () const
{
  return **(Base::_it);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_vertex_t <Vdata, Edata, allow_parallel_edges>* 
graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator::
operator-> () const
{
  return &(operator*());
}


template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator::
edge_iterator () : Base () {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator::
edge_iterator (const _base_edge_it& e)  
  : Base (e) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator::
edge_iterator (const edge_iterator& e) 
  : Base (e._it) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator::
edge_iterator (const Base& i) 
  : Base (i) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_edge_t <Vdata, Edata, allow_parallel_edges>& 
graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator::
operator* () const
{
  return **(Base::_it);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_edge_t <Vdata, Edata, allow_parallel_edges>* 
graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator::operator-> () const
{
  return &(operator*());
}


template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
adj_edge_iterator () 
  : Base () {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
adj_edge_iterator (const _base_adj_edge_it& e)  
  : Base (e) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
adj_edge_iterator (const adj_edge_iterator& e)  
  : Base (e._it) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
adj_edge_iterator (const Base& i)
  : Base (i) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_edge_t<Vdata, Edata, allow_parallel_edges>& 
graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
operator* () const
{
  return *(Base::_it->second);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_edge_t<Vdata, Edata, allow_parallel_edges>* 
graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
operator-> () const
{
  return &(operator*());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator
graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator::
get_edge_it () const
{
  return Base::_it->second;
}


// ------------------------------- vertex --------------------------------------

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
graph_vertex_t ()
  : data () {}
  
template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
graph_vertex_t (const Vdata& data_)
  : data (data_) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline size_t
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
out_degree () const
{
  return _out_edges.size ();
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline size_t
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
in_degree () const
{
  return _in_edges.size ();
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator 
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
out_edges_begin ()
{
  return adj_edge_iterator_ (_out_edges.begin());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator 
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
out_edges_end ()
{
  return adj_edge_iterator_ (_out_edges.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator 
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
in_edges_begin ()
{
  return adj_edge_iterator_ (_in_edges.begin());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator 
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
in_edges_end ()
{
  return adj_edge_iterator_ (_in_edges.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator 
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
get_in_edge (const vertex_iterator_& from)
{
  vertex_t_* src = &(*from);
  adj_multimap_it_ it = _in_edges.find (src);
  return adj_edge_iterator_ (it);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_vertex_t<Vdata, Edata, allow_parallel_edges>::adj_edge_range_
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
getall_in_edges (const vertex_iterator_& from)
{
  vertex_t_* src = &(*from);
  std::pair <adj_multimap_it_, adj_multimap_it_> range = _in_edges.equal_range (src);
  return adj_edge_range_ (range.first, range.second);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::adj_edge_iterator 
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
get_out_edge (const vertex_iterator_& to)
{
  vertex_t_* dest = &(*to);
  adj_multimap_it_ it = _out_edges.find (dest);
  return adj_edge_iterator_ (it);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_vertex_t<Vdata, Edata, allow_parallel_edges>::adj_edge_range_
graph_vertex_t<Vdata, Edata, allow_parallel_edges>::
getall_out_edges (const vertex_iterator_& to)
{
  vertex_t_* dest = &(*to);
  std::pair <adj_multimap_it_, adj_multimap_it_> range = _in_edges.equal_range (dest);
  return adj_edge_range_ (range.first, range.second);
}


// -------------------------------- edge ---------------------------------------

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_edge_t<Vdata, Edata, allow_parallel_edges>::
graph_edge_t (const vertex_iterator_& source, 
              const vertex_iterator_& target) :
  data (), _source (source), _target (target) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline graph_edge_t<Vdata, Edata, allow_parallel_edges>::
graph_edge_t (const vertex_iterator_& source, 
              const vertex_iterator_& target,
              const Edata& data_) 
  : data (data_), _source (source), _target (target) {}

template <typename Vdata, typename Edata, bool allow_parallel_edges> 
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator 
graph_edge_t<Vdata, Edata, allow_parallel_edges>::get_source () const
{
  return _source;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator 
graph_edge_t<Vdata, Edata, allow_parallel_edges>::get_target () const
{
  return _target;
}

// ------------------------------- graph ---------------------------------------

template <typename Vdata, typename Edata, bool allow_parallel_edges>
graph_t<Vdata, Edata, allow_parallel_edges>::graph_t () { }

template <typename Vdata, typename Edata, bool allow_parallel_edges>
graph_t<Vdata, Edata, allow_parallel_edges>::~graph_t ()
{
  for (_base_vertex_it it = _vertices.begin();
       it != _vertices.end(); ++it)
    delete *it;
  for (_base_edge_it it = _edges.begin();
       it != _edges.end(); ++it)
    delete *it;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
void
graph_t<Vdata, Edata, allow_parallel_edges>::clear ()
{
  for (_base_vertex_it it = _vertices.begin();
       it != _vertices.end(); ++it)
  {
    (*it)->_out_edges.clear ();
    (*it)->_in_edges.clear ();
    delete *it;
  }
  for (_base_edge_it it = _edges.begin();
       it != _edges.end(); ++it)
    delete *it;
  
  _vertices.clear ();
  _edges.clear ();
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline size_t
graph_t<Vdata, Edata, allow_parallel_edges>::number_of_vertices () const
{
  return _vertices.size ();
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline size_t
graph_t<Vdata, Edata, allow_parallel_edges>::number_of_edges () const
{
  return _edges.size ();
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::vertices_begin ()
{
  return vertex_iterator (_vertices.begin());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::vertices_end ()
{
  return vertex_iterator (_vertices.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::edges_begin ()
{
  return edge_iterator (_edges.begin());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
inline typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::edges_end ()
{
  return edge_iterator (_edges.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::
get_edge (vertex_iterator source, 
          vertex_iterator target) 
{
  vertex_t_* dest = &(*target);
  _base_adj_edge_it it = source->_out_edges.find (dest);
  if (it == source->_out_edges.end())
    return edge_iterator (_edges.end());
  else
    return it->second;
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator_range
graph_t<Vdata, Edata, allow_parallel_edges>::
getall_edges (vertex_iterator source, 
              vertex_iterator target)
{
  vertex_t_* dest = &(*target);
  std::pair <_base_adj_edge_it, _base_adj_edge_it> range = source->_out_edges.equal_range (dest);
  return edge_iterator_range (range.first, range.second);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
bool 
graph_t<Vdata, Edata, allow_parallel_edges>::
exists_edge (vertex_iterator source, 
             vertex_iterator target)
{
  vertex_t_* dest = &(*target);
  _base_adj_edge_it it = source->_out_edges.find (dest);
  return (it != source->_out_edges.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::add_vertex ()
{
  // default construction
  vertex_t_* vertex = new vertex_t_ ();
  _vertices.push_back (vertex);
  return (--_vertices.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::vertex_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::
add_vertex (const Vdata& vertex_data)
{
  vertex_t_* vertex = new vertex_t_ (vertex_data);
  _vertices.push_back (vertex);
  return (--_vertices.end());
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::
add_edge (vertex_iterator source, 
          vertex_iterator target)
          throw (graph_exception)
{
  return _PeHelper<Vdata, Edata, allow_parallel_edges>::
           _add_edge (*this, source, target);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
typename graph_t<Vdata, Edata, allow_parallel_edges>::edge_iterator 
graph_t<Vdata, Edata, allow_parallel_edges>::
add_edge (vertex_iterator source, 
          vertex_iterator target, 
          const Edata& edge_data)
          throw (graph_exception)
{
  return _PeHelper<Vdata, Edata, allow_parallel_edges>::
           _add_edge (*this, source, target, edge_data);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
void
graph_t<Vdata, Edata, allow_parallel_edges>::
move_edge (edge_iterator edge,
           vertex_iterator new_source, 
           vertex_iterator new_target)
{
  _PeHelper<Vdata, Edata, allow_parallel_edges>::
    _move_edge (*this, edge, new_source, new_target); 
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
void 
graph_t<Vdata, Edata, allow_parallel_edges>::
remove_vertex (vertex_iterator v)
{
  // remove out edges
  remove_out_edges (v);
  // remove in edges
  remove_in_edges (v);
  // remove the vertex
  delete * v._it;
  _vertices.erase (v._it);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>    
void 
graph_t<Vdata, Edata, allow_parallel_edges>::
remove_edge (edge_iterator e)
{
  _PeHelper<Vdata, Edata, allow_parallel_edges>::
    _remove_edge (*this, e);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
void 
graph_t<Vdata, Edata, allow_parallel_edges>::
remove_edge (const adj_edge_iterator& e)
{
  remove_edge (e._it->second);
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
void 
graph_t<Vdata, Edata, allow_parallel_edges>::
remove_out_edges (vertex_iterator v)
{
  for (adj_edge_iterator it = v->out_edges_begin();
       it != v->out_edges_end(); ++it)
  {
    // get iterator of in_edges of target
    vertex_iterator dest_it = it->get_target();
    adj_edge_iterator in_e = dest_it->get_in_edge (v);
    // get iterator in the edges list
    edge_iterator e = in_e._it->second;
    // clean up memory
    delete * e._it;
    _edges.erase (e._it);
    dest_it->_in_edges.erase (in_e._it);
  }
  v->_out_edges.clear ();
}

template <typename Vdata, typename Edata, bool allow_parallel_edges>
void 
graph_t<Vdata, Edata, allow_parallel_edges>::
remove_in_edges (vertex_iterator v)
{
  for (adj_edge_iterator it = v->in_edges_begin();
       it != v->in_edges_end(); ++it)
  {
    // get iterator of out_edges of source
    vertex_iterator src_it = it->get_source();
    adj_edge_iterator out_e = src_it->get_out_edge (v);
    // get iterator in the edges list
    edge_iterator e = out_e._it->second;
    // clean up memory
    delete * e._it;
    _edges.erase (e._it);
    src_it->_out_edges.erase (out_e._it);
  }
  v->_in_edges.clear ();
}

} // namespace utils

#endif
