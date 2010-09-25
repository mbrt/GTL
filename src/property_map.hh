// =============================================================================
// 
//       Filename:  property_map.hh
// 
//    Description:  A template set of property maps for graphs
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

#ifndef GTL_PROPERTY_MAP_HH
#define GTL_PROPERTY_MAP_HH

#include <map>

namespace gtl {

enum default_color_t { 
  white_color, gray_color, black_color, green_color, red_color 
};

/// Default color traits class
template <typename Color>
struct color_traits
{
  static default_color_t white() { return white_color; }
  static default_color_t gray()  { return gray_color; }
  static default_color_t black() { return black_color; }
  static default_color_t red()   { return red_color; }
  static default_color_t green() { return green_color; }
};


/// Associates descriptors (vertex or edges) to properties. The property type
/// must be copy constructible. A copy of the given value is stored in the map.
/// @tparam Descriptor the descriptor type
/// @tparam Value the value to be stored for each descriptor
/// @tparam Map the map type that associates descriptors to values
template <typename Descriptor,
          typename Value,
          typename Map = std::map<Descriptor, Value> >
class property_map_external_t
{
public:
  typedef Descriptor key_type;
  typedef Value value_type;
  typedef Map container_type;

  /// Assign to the descriptor property the given value
  void put (Descriptor d, Value v)
  { _map[d] = v; }

  /// Returns the descriptor property (a copy)
  Value get (Descriptor d)
  { return _map[d]; }

  Value& operator[] (Descriptor d)
  { return _map[d]; }

private:
  Map _map;

};

/// If the vertex (or edge) type is a struct (or a class), this map holds a 
/// pointer to a given member. When put and get functions are called the value 
/// stored in the vertex (or the edge) is used. The access time is constant and
/// consists of one pointer sum and a dereference (therefore in constant time).
/// It can be used as property map (when possible is suggested to use this class
/// instead of the property_map_external_t, that uses a map or an hash-map that 
/// are slower)
/// @tparam Descriptor the descriptor type
/// @tparam Value the value to be stored for each descriptor
/// @tparam Data the type of the vertex (or edge) data stored (it must be a 
///  struct or a class, that contains the desired member
template <typename Descriptor,
          typename Value,
          typename Data = typename Descriptor::value_type>
class property_map_internal_t
{
public:
  typedef Descriptor key_type;
  typedef Value value_type;
  
  property_map_internal_t (Value Data::*member) : m_member (member) {}

  /// Assign to the descriptor property the given value
  void put (Descriptor d, Value v)
  { (*d).*m_member = v; }

  /// Returns the descriptor property (a copy)
  Value get (Descriptor d)
  { return (*d).*m_member; }

  Value& operator[] (Descriptor d)
  { return (*d).*m_member; }

private:
  Value Data::*m_member;

};

} // namespace gtl

#endif // GTL_PROPERTY_MAP_HH
