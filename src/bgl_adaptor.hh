// =============================================================================
// 
//       Filename:  bgl_adaptor.hh
// 
//    Description:  A template set of algorithms for graph classes
//
//         Author:  Michele Bertasi 
//        Contact:  michele.bertasi@gmail.com
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
#include <utility>

#include "graph.hh"


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
std::pair <typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_iterator,
           typename graph_traits<gtl::graph_t<V, E, p, A> >::vertex_iterator>
vertices (const gtl::graph_t <V, E, p, A>& g) {
  return const_cast<gtl::graph_t <V, E, p, A>&>(g).vertices ();
}

template <typename V, typename E, bool p, typename A>
inline 
std::pair <typename gtl::graph_t<V, E, p, A>::out_edge_iterator,
           typename gtl::graph_t<V, E, p, A>::out_edge_iterator>
out_edges (typename gtl::graph_t<V, E, p, A>::vertex_descriptor v,
           const gtl::graph_t <V, E, p, A>& g) {
  return const_cast<gtl::graph_t<V, E, p, A>&>(g).out_edges (v);
}

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
inline size_t
out_degree (typename gtl::graph_t<V, E, p, A>::vertex_descriptor v,
            const gtl::graph_t<V, E, p, A>& g) {
  return g.out_degree (v);
}


// ---------------------------- property map ----------------------------------


template <typename Container>
class std_container_adaptor
  : public put_get_helper <typename Container::value_type::second_type&, 
                           std_container_adaptor<Container> >
{
public:
  typedef lvalue_property_map_tag category;
  typedef typename Container::key_type key_type;
  typedef typename Container::value_type::second_type value_type;
  typedef value_type& reference;
  
  std_container_adaptor () : m_c(NULL) {}
  std_container_adaptor (Container& c) : m_c(&c) {}

  reference operator[] (const key_type& k) const {
    return (*m_c)[k];
  }

private:
  Container* m_c;
};


} // namespace boost

#endif // BGL_ADAPTOR_HH
