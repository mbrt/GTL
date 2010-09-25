// =============================================================================
// 
//       Filename:  bgl_adaptor.hh
// 
//    Description:  Boost Graph Library adaptor for gtl::graph_t class
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

#ifndef BGL_ADAPTOR_HH
#define BGL_ADAPTOR_HH

#include <boost/graph/graph_traits.hpp>
#include <boost/property_map.hpp>
#include <boost/graph/properties.hpp>
#include <utility>

#include "graph.hh"
#include "property_map.hh"


// The functions and classes in this file allows the user to
// treat a gtl::graph_t object as a boost graph "as is". No
// wrapper is needed for the graph_t object.

namespace boost {

/// The traversal category
struct gtl_traversal_category : 
  public virtual bidirectional_graph_tag,
  public virtual adjacency_graph_tag,
  public virtual vertex_list_graph_tag,
  public virtual edge_list_graph_tag { };

/// Graph traits specialization for GTL graphs
template <typename V, typename E, bool p, typename A>
struct graph_traits <gtl::graph_t<V, E, p, A> >
{
  typedef gtl::graph_t<V, E, p, A> G;

  typedef typename G::vertex_descriptor       vertex_descriptor;
  typedef typename G::edge_descriptor         edge_descriptor;
  typedef typename G::adjacency_iterator      adjacency_iterator;
  typedef typename G::out_edge_iterator       out_edge_iterator;
  typedef typename G::in_edge_iterator        in_edge_iterator;
  typedef typename G::vertex_iterator         vertex_iterator;
  typedef typename G::edge_iterator           edge_iterator;
 
  typedef directed_tag                        directed_category;
  typedef typename gtl::impl::static_if_<p,
     allow_parallel_edge_tag,
     disallow_parallel_edge_tag>::type        edge_parallel_category;
  typedef gtl_traversal_category              traversal_category;
 
  typedef size_t                              vertices_size_type;
  typedef size_t                              edges_size_type;
  typedef size_t                              degree_size_type;
 
  static vertex_descriptor null_vertex () {
    return vertex_descriptor(); 
  }
};


// ----------------------------- functions ------------------------------------

template <typename V, typename E, bool p, typename A>
inline
typename gtl::graph_t<V, E, p, A>::vertex_descriptor
source (typename gtl::graph_t<V, E, p, A>::edge_descriptor e,
        const gtl::graph_t<V, E, p, A>& g) {
  return const_cast<gtl::graph_t<V, E, p, A>&>(g).source (e);
}

template <typename V, typename E, bool p, typename A>
inline
typename gtl::graph_t<V, E, p, A>::vertex_descriptor
target (typename gtl::graph_t<V, E, p, A>::edge_descriptor e,
        const gtl::graph_t<V, E, p, A>& g) {
  return const_cast<gtl::graph_t<V, E, p, A>&>(g).target (e);
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_iterator,
           typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_iterator>
vertices (const gtl::graph_t <V, E, p, A>& g) {
  return const_cast<gtl::graph_t <V, E, p, A>&>(g).vertices ();
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::edge_iterator,
           typename graph_traits<gtl::graph_t<V, E, p, A> >::edge_iterator>
edges (const gtl::graph_t <V, E, p, A>& g) {
  return const_cast<gtl::graph_t <V, E, p, A>&>(g).edges ();
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::out_edge_iterator,
           typename graph_traits<gtl::graph_t<V, E, p, A> >::out_edge_iterator>
out_edges (typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
           const gtl::graph_t <V, E, p, A>& g) {
  return const_cast<gtl::graph_t<V, E, p, A>&>(g).out_edges (v);
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::in_edge_iterator,
           typename graph_traits<gtl::graph_t<V, E, p, A> >::in_edge_iterator>
in_edges (typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
          const gtl::graph_t <V, E, p, A>& g) {
  return const_cast<gtl::graph_t<V, E, p, A>&>(g).in_edges (v);
}

template <typename V, typename E, bool p, typename A>
inline
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::adjacency_iterator,
           typename graph_traits<gtl::graph_t<V, E, p, A> >::adjacency_iterator>
adjacent_vertices (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    const gtl::graph_t<V, E, p, A>& g) {
  return const_cast<gtl::graph_t<V, E, p, A>&>(g).adjacent_vertices (v);
}

template <typename V, typename E, bool p, typename A>
inline 
typename graph_traits<gtl::graph_t<V, E, p, A> >::vertices_size_type
num_vertices (const gtl::graph_t<V, E, p, A>& g) {
  return g.num_vertices ();
}

template <typename V, typename E, bool p, typename A>
inline 
typename graph_traits<gtl::graph_t<V, E, p, A> >::edges_size_type
num_edges (const gtl::graph_t<V, E, p, A>& g) {
  return g.num_edges ();
}

template <typename V, typename E, bool p, typename A>
inline 
typename graph_traits<gtl::graph_t<V, E, p, A> >::degree_size_type
out_degree (
    typename graph_traits <gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    const gtl::graph_t<V, E, p, A>& g) {
  return g.out_degree (v);
}

template <typename V, typename E, bool p, typename A>
inline 
typename graph_traits<gtl::graph_t<V, E, p, A> >::degree_size_type
in_degree (
    typename graph_traits <gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    const gtl::graph_t<V, E, p, A>& g) {
  return g.in_degree (v);
}

template <typename V, typename E, bool p, typename A>
inline
typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor
add_vertex (gtl::graph_t<V, E, p, A>& g) {
  return g.add_vertex ();
}

template <typename V, typename E, bool p, typename A>
inline 
typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor
add_vertex (const V& data, gtl::graph_t<V, E, p, A>& g) {
  return g.add_vertex (data);
}

template <typename V, typename E, bool p, typename A>
inline void
clear_vertex (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    gtl::graph_t<V, E, p, A>& g) {
  g.clear_vertex (v);
}

template <typename V, typename E, bool p, typename A>
inline void
remove_vertex (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    gtl::graph_t<V, E, p, A>& g) {
  g.remove_vertex (v);
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::edge_descriptor, 
           bool>
add_edge (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor u,
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    gtl::graph_t<V, E, p, A>& g) {
  return g.add_edge (u, v);
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::edge_descriptor, 
           bool>
add_edge (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor u,
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    const E& data,
    gtl::graph_t<V, E, p, A>& g) {
  return g.add_edge (u, v, data);
}

template <typename V, typename E, bool p, typename A>
inline void
remove_edge (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::edge_descriptor e,
    gtl::graph_t<V, E, p, A>& g) {
  g.remove_edge (e);
}

template <typename V, typename E, bool p, typename A>
inline void
remove_edge (
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor u,
    typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_descriptor v,
    gtl::graph_t<V, E, p, A>& g) {
  g.remove_edge (u, v);
}

// ---------------------------- property map ----------------------------------


/// This adaptor can be passed as property map to a boost graph library 
/// algorithm. Internally store a pointer to a gtl property map.
/// @tparam Container the container type
template <typename GtlMap>
class gtl_map_adaptor
  : public put_get_helper <typename GtlMap::value_type&, 
                           gtl_map_adaptor<GtlMap> >
{
public:
  typedef lvalue_property_map_tag category;
  typedef typename GtlMap::key_type key_type;
  typedef typename GtlMap::value_type value_type;
  typedef value_type& reference;
  
  gtl_map_adaptor () : m_c(NULL) {}
  gtl_map_adaptor (GtlMap& c) : m_c(&c) {}

  reference operator[] (const key_type& k) const {
    return (*m_c)[k];
  }

private:
  GtlMap* m_c;
};


/// Returns an adaptor from a gtl property map to a bgl property map. The 
/// returned map can be used in any algorithm in the boost graph library.
/// @param gtl_pmap the gtl property map to pass
template <typename GtlMap>
inline gtl_map_adaptor<GtlMap>
make_gtl_map_adaptor (GtlMap& gtl_pmap) {
  return gtl_map_adaptor<GtlMap> (gtl_pmap);
}

/// Default color traits class
template <>
struct color_traits<gtl::default_color_t>
{
  static gtl::default_color_t white() { return gtl::white_color; }
  static gtl::default_color_t gray()  { return gtl::gray_color; }
  static gtl::default_color_t black() { return gtl::black_color; }
  static gtl::default_color_t red()   { return gtl::red_color; }
  static gtl::default_color_t green() { return gtl::green_color; }
};


} // namespace boost

#endif // BGL_ADAPTOR_HH
