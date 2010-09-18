// =============================================================================
// 
//       Filename:  graph_internals.hh
// 
//    Description:  Internal graph classes. Do not use this header directly
//
//         Author:  Michele Bertasi 
//                  Giuseppe Di Guglielmo
//        Contact:  giuseppe.diguglielmo@univr.it
//                  michele.bertasi@studenti.univr.it
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

#ifndef GRAPH_INTERNALS_HH
#define GRAPH_INTERNALS_HH

#include <set>
#include <list>
#include <tr1/unordered_map>


namespace utils
{

// Forward references
template <typename Vdata, typename Edata, bool parallel, typename Allocator>
class graph_t;
template <typename Vdata, typename Config>
class graph_vertex_t_;
template <typename Edata, typename Config>
class graph_edge_t_;

/// An empty struct that contains no data
struct NoData {};

/// Used for internal graph data types and functions
namespace internals {

struct unspecified;

/// Returns one of its two arguments, T1 or T2, depending on the value C.
template<bool c, typename T1, typename T2>
struct static_if_
{
  typedef unspecified type;
};

template <typename T1, typename T2>
struct static_if_<true, T1, T2>
{
  typedef T1 type;
};

template <typename T1, typename T2>
struct static_if_<false, T1, T2>
{
  typedef T2 type;
};

/// Base for the edge_t_ class
template <typename OutEdgeIterator, 
          typename InEdgeIterator, 
          bool parallel>
class _EdgeBase;

template <typename OutEdgeIterator, typename InEdgeIterator>
class _EdgeBase<OutEdgeIterator, InEdgeIterator, false>
{
protected:
  void out_edge_position (OutEdgeIterator) {}
  void in_edge_position (InEdgeIterator) {}
  
  template <typename Helper>
  void remover_out (typename Helper::Container& container,
                    typename Helper::value_type val)
  {
    Helper::remove (container, val);
  }
  
  template <typename Helper>
  void remover_in (typename Helper::Container& container,
                   typename Helper::value_type val)
  {
    Helper::remove (container, val);
  }
};

template <typename OutEdgeIterator, typename InEdgeIterator>
class _EdgeBase<OutEdgeIterator, InEdgeIterator, true>
{
protected:
  /// The iterator of the source vertex to this edge
  OutEdgeIterator _out_edge_iter;

  /// The iterator of the target vertex to this edge
  InEdgeIterator _in_edge_iter;
  
  void out_edge_position (OutEdgeIterator it) {
    _out_edge_iter = it;
  }

  void in_edge_position (InEdgeIterator it) {
    _in_edge_iter = it;
  }
  
  template <typename Helper>
  void remover_out (typename Helper::Container& container,
                    typename Helper::value_type)
  {
    Helper::remove (container, _out_edge_iter);
  }
  
  template <typename Helper>
  void remover_in (typename Helper::Container& container,
                   typename Helper::value_type)
  {
    Helper::remove (container, _in_edge_iter);
  }
};

/// Used as operator< in in-edge lists
template <typename VertexIt, typename EdgeIt>
struct compare_in_
  : std::binary_function<const EdgeIt&, const EdgeIt&, bool> {
  bool operator() (const EdgeIt& a, const EdgeIt& b) const {
    VertexIt a_source = (a.internal_value())->_source;
    VertexIt b_source = (b.internal_value())->_source;
    return a_source.internal_value() < b_source.internal_value();
  }
};

/// Used as operator< in out-edge lists
template <typename VertexIt, typename EdgeIt>
struct compare_out_ 
  : std::binary_function<const EdgeIt&, const EdgeIt&, bool> {
  bool operator() (const EdgeIt& a, const EdgeIt& b) const {
    VertexIt a_target = (a.internal_value())->_target;
    VertexIt b_target = (b.internal_value())->_target;
    return a_target.internal_value() < b_target.internal_value();
  }
};

/// The hasher function, based on the standard hash function on integers
template <typename Descriptor>
struct _descriptor_hash : public std::unary_function<Descriptor, size_t>
{
  std::tr1::hash<unsigned long> __hasher;

  inline size_t operator() (const Descriptor& x) const {
    return __hasher (reinterpret_cast<unsigned long> (x.internal_value()));
  }
};


/// Base class for vertex and edge iterators
/// @tparam T the value type
/// @tparam _Iterator the base iterator stored.
template <typename T, typename _Iterator>
class iterator_base_ : public _Iterator
{
  typedef _Iterator Base;
  template <typename Vdata, typename Edata, bool parallel, typename Allocator>
  friend class graph_t;

public:
  typedef T value_type;
  typedef T* pointer;
  typedef T& reference;
  
  iterator_base_ ()
    : Base() {}
  
  iterator_base_ (const _Iterator& i)
    : Base(i) {}

  reference operator* () const
  { return (T&)(*this); }
  
  pointer operator-> () const
  { return &(operator*()); }
};

/// Base class for the adjacency iterators with no check in the increment
/// and decrement operators
/// @tparam T the value type
/// @tparam _Iterator the base iterator stored.
/// @tparam Container the container type
template <typename T, typename _Iterator, typename Container>
class adjacency_iterator_no_check_ : public _Iterator
{
  typedef _Iterator Base;

public:
  typedef T value_type;
  typedef T* pointer;
  typedef T& reference;

  adjacency_iterator_no_check_ ()
    : Base() {}
  
  adjacency_iterator_no_check_ (const _Iterator& i)
    : Base(i) {}

  adjacency_iterator_no_check_ (const _Iterator& i, const Container&)
    : Base(i) {}

  reference operator* () const
  { return (Base::operator*().internal_value())->_target; }

  pointer operator-> () const
  { return &(operator*()); }
};

/// Base class for the adjacency iterators with check in the increment
/// and decrement operators (if the reached vertex is equal to previous)
/// @tparam T the value type
/// @tparam _Iterator the base iterator stored.
/// @tparam Container the container type
template <typename T, typename _Iterator, typename Container>
class adjacency_iterator_check_ 
  : public adjacency_iterator_no_check_<T, _Iterator, Container>
{
  typedef adjacency_iterator_no_check_<T, _Iterator, Container> Base;
  typedef adjacency_iterator_check_<T, _Iterator, Container> self;

public:
  adjacency_iterator_check_ () 
    : _cont(NULL) {}
  
  adjacency_iterator_check_ (const _Iterator& i, const Container& cont)
    : Base(i), _cont(&cont) {}

  self& operator++ () {
    T prev = Base::operator*();
    do
      Base::operator++();
    while (*this != _cont->end() && 
             Base::operator*() == prev);
    return *this;
  }

  self operator++ (int) {
    self tmp = *this;
    operator++();
    return tmp;
  }

  self& operator-- () {
    if (*this == _cont->end()) {
      Base::operator--();
      return *this;
    }
    else {
      T prev = Base::operator*();
      do
        Base::operator--();
      while (Base::operator*() == prev);
      return *this;
    }
  }

  self operator-- (int) {
    self tmp = *this;
    operator--();
    return tmp;
  }

private:
  const Container* _cont;
};

/// Base class for vertex and edge descriptors
/// @tparam T the value type
/// @tparam _Iterator the base iterator stored.
template <typename T, typename _Iterator>
class descriptor_base_ : protected _Iterator
{
  typedef _Iterator Base;
  typedef descriptor_base_<T, _Iterator> self;

  template <typename Vdata, typename Edata, bool parallel, typename Allocator>
  friend class graph_t;
  template <typename Vdata, typename Config>
  friend class graph_vertex_t_;
  template <typename Vdata, typename Edata, bool parallel>
  friend class _EdgeBase;
  template <typename VertexIt, typename EdgeIt>
  friend struct compare_out_;
  template <typename VertexIt, typename EdgeIt>
  friend struct compare_in_;
  template <typename _T, typename _I, typename C>
  friend class adjacency_iterator_no_check_;
  friend struct _descriptor_hash<self>;

public:
  typedef T value_type;
  typedef T* pointer;
  typedef T& reference;
  typedef typename Base::reference internal_type;
  typedef _descriptor_hash<self> hasher;
  
  descriptor_base_ ()
    : Base() {}

  descriptor_base_ (const _Iterator& i)
    : Base(i) {}

  reference operator* () const
  { return (Base::operator*())->data; }
  
  pointer operator-> () const
  { return &(operator*()); }

  bool operator== (const descriptor_base_& other) const
  { return Base::operator== (other); }
  
  bool operator!= (const descriptor_base_& other) const
  { return Base::operator!= (other); }

  bool operator< (const descriptor_base_& other) const
  { return internal_value() < other.internal_value(); }

protected:
  internal_type internal_value() const
  { return Base::operator*(); }

};

/// Specialization for descriptors in wich the value stored is NoData.
/// No operator * and -> are provided.
/// @tparam _Iterator the base iterator stored.
template <typename _Iterator>
class descriptor_base_<NoData, _Iterator> : private _Iterator
{
  typedef descriptor_base_<NoData, _Iterator> self;
  typedef _Iterator Base;

  template <typename Vdata, typename Edata, bool parallel, typename Allocator>
  friend class graph_t;
  template <typename Vdata, typename Config>
  friend class graph_vertex_t_;
  template <typename Vdata, typename Edata, bool parallel>
  friend class _EdgeBase;
  template <typename VertexIt, typename EdgeIt>
  friend struct compare_out_;
  template <typename VertexIt, typename EdgeIt>
  friend struct compare_in_;
  friend struct _descriptor_hash<self>;

public:
  typedef void value_type;
  typedef typename Base::reference internal_type;
  typedef _descriptor_hash<self> hasher;

  descriptor_base_ ()
    : Base() {}

  descriptor_base_ (const _Iterator& i)
    : Base(i) {}
  
  bool operator== (const descriptor_base_& other) const
  { return Base::operator== (other); }
  
  bool operator!= (const descriptor_base_& other) const
  { return Base::operator!= (other); }
  
  bool operator< (const descriptor_base_& other) const
  { return internal_value() < other.internal_value(); }

protected:
  internal_type internal_value() const
  { return Base::operator*(); }

};

/// Adaptor for different types of containers
template <typename Container_>
struct _ContainerAdaptor
{
  typedef Container_ Container;
  typedef typename Container::iterator iterator;
  typedef typename Container::value_type value_type;
  
  template <typename T, typename Cmp, typename A>
  static std::pair<iterator, bool> 
  insert (std::set<T, Cmp, A>& container, value_type e) {
    return container.insert(e);
  }

  template <typename T, typename Cmp, typename A>
  static std::pair<iterator, bool> 
  insert (std::multiset<T, Cmp, A>& container, value_type e) {
    return std::make_pair<iterator, bool> (container.insert(e), true);
  }

  template <typename T, typename Cmp, typename A>
  static size_t
  remove (std::multiset<T, Cmp, A>& container, iterator e) {
    container.erase(e);
    return 1;
  }
  
  template <typename T, typename Cmp, typename A>
  static size_t
  remove (std::set<T, Cmp, A>& container, iterator e) {
    container.erase(e);
    return 1;
  }
  
  template <typename T, typename Cmp, typename A>
  static size_t
  remove (std::multiset<T, Cmp, A>& container, value_type e) {
    return container.erase(e);
  }
  
  template <typename T, typename Cmp, typename A>
  static size_t
  remove (std::set<T, Cmp, A>& container, value_type e) {
    return container.erase(e);
  }
};

/// Function that delete a pointer
template <typename Alloc>
struct deleter
{
  Alloc& alloc;
  
  deleter (Alloc& alloc_) 
    : alloc(alloc_) {};
  
  void operator() (typename Alloc::pointer p) {
    alloc.destroy (p);
    alloc.deallocate (p, 1);
  }
};

/// Base for the Config_ class
template <typename Vdata, typename Edata, bool parallel, typename Allocator>
struct _Config 
{
  typedef _Config<Vdata, Edata, parallel, Allocator> self;
  // graph type
  typedef graph_t<Vdata, Edata, parallel, Allocator> graph_t_;
  // vertex and edge types
  typedef graph_vertex_t_<Vdata, self> vertex_t;
  typedef graph_edge_t_<Edata, self> edge_t;
  // allocator types
  typedef typename Allocator::template rebind<vertex_t>::other vertex_alloc;
  typedef typename Allocator::template rebind<edge_t>::other edge_alloc;
  // vertices and edges list types
  typedef std::list<vertex_t*, vertex_alloc> vertex_list_type;
  typedef std::list<edge_t*, edge_alloc> edge_list_type;
  // internal iterator base
  typedef typename vertex_list_type::iterator _base_vertex_it;
  typedef typename edge_list_type::iterator _base_edge_it;
  // descriptors
  typedef descriptor_base_<Vdata, _base_vertex_it> vertex_descriptor;
  typedef descriptor_base_<Edata, _base_edge_it> edge_descriptor;
  // iterators
  typedef iterator_base_<vertex_descriptor, _base_vertex_it> vertex_iterator;
  typedef iterator_base_<edge_descriptor, _base_edge_it> edge_iterator;
  // comparision function objects
  typedef compare_out_<vertex_descriptor, edge_descriptor> compare_out; 
  typedef compare_in_<vertex_descriptor, edge_descriptor> compare_in;
  // allocatorfor in and out edge adjacency lists
  typedef typename Allocator::template rebind<edge_descriptor>::other 
    adj_alloc_;
  // in and out edge adjacency list
  typedef typename static_if_ <parallel,                          // if
    std::multiset<edge_descriptor, compare_out, adj_alloc_>,      // then
    std::set<edge_descriptor, compare_out, adj_alloc_> >          // else
      ::type out_edge_list_type;
  typedef typename static_if_ <parallel,                          // if
    std::multiset<edge_descriptor, compare_in, adj_alloc_>,       // then
    std::set<edge_descriptor, compare_in, adj_alloc_> >           // else
      ::type in_edge_list_type;
  // iterators
  typedef typename out_edge_list_type::iterator out_edge_iterator;
  typedef typename in_edge_list_type::iterator in_edge_iterator;
  typedef adjacency_iterator_no_check_<vertex_descriptor, out_edge_iterator,
                                       out_edge_list_type>
    adjacency_iterator_base_;
  typedef typename static_if_ <parallel,
    adjacency_iterator_check_<vertex_descriptor, out_edge_iterator,
                              out_edge_list_type>,
    adjacency_iterator_base_>
      ::type adjacency_iterator;
  // in and out edge adjacency list adaptors
  typedef _ContainerAdaptor<out_edge_list_type> OutEdgesAdaptor;
  typedef _ContainerAdaptor<in_edge_list_type> InEdgesAdaptor;
  // base class for edge
  typedef _EdgeBase<out_edge_iterator, 
                    in_edge_iterator, 
                    parallel> EdgeBase;
  // vertex and edge descriptor hash functions
  typedef _descriptor_hash<vertex_descriptor> vertex_hash;
  typedef _descriptor_hash<edge_descriptor> edge_hash;
};

} // namespace internals
} // namespace utils

#endif
