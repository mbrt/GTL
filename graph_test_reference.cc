
#include "graph_reference.hh"

#include <iostream>
#include <set>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/tr1/tuple.hpp>
//#include <boost/foreach.hpp>

//#define foreach BOOST_FOREACH

struct Foo
{
  Foo (int a_, int b_) : a (a_), b (b_) {};
  
  int method (int i)
  {
    std::cout << "a: " << a << " b: " << b << std::endl;
    return i;
  }
  
  int a;
  int b;
};

std::ostream& operator<< (std::ostream& out, const Foo& f)
{
  out << "a: " << f.a << " b: " << f.b << std::endl;
  return out;
}



using namespace std;
using namespace utils;
typedef utils::graph_t <Foo, int> graph;

template <typename Graph>
void printVertex (typename Graph::vertex_iterator v)
{
  cout << "  data: " << v->data << endl;
  cout << "  out edges:\n";
  if (v->out_degree() == 0)
    cout << "  empty!\n";
  else
    for (typename Graph::adj_edge_iterator it = v->out_edges_begin();
         it != v->out_edges_end(); ++it)
      cout << "  " << it->data << endl;
  cout << "in edges:\n";
  if (v->in_degree() == 0)
    cout << "  empty!\n";
  else
    for (typename Graph::adj_edge_iterator it = v->in_edges_begin();
         it != v->in_edges_end(); ++it)
      cout << "  " << it->data << endl;
}

template <typename EdgeIt>
void printEdge (EdgeIt e)
{
  cout
    << "  data:\n  "
    << e->data << endl 
    << "  source:\n  "
    << e->get_source()->data << endl
    << "  destination:\n  "
    << e->get_target()->data << endl;
}

struct BData {
  int id;
};


int main ()
{
//  typedef boost::adjacency_list <boost::multisetS, boost::listS, 
//                                 boost::bidirectionalS,
//                                 BData, BData> boost_graph_t;
//  typedef boost::graph_traits<boost_graph_t>::vertex_descriptor Vertex;
//  typedef boost::graph_traits<boost_graph_t>::edge_descriptor Edge;
//  
//  boost_graph_t boost_g;
//  
//  Vertex bv1 = boost::add_vertex (boost_g);
//  boost_g[bv1].id = 1;
//  Vertex bv2 = boost::add_vertex (boost_g);
//  boost_g[bv2].id = 2;
//  
//  std::pair<Edge, bool> edge_pair = boost::add_edge (bv1, bv2, boost_g);
//  assert (edge_pair.second);
//  Edge be1 = edge_pair.first;
//  boost_g[be1].id = 1;
//  
//  edge_pair = boost::add_edge (bv1, bv1, boost_g);
//  assert (edge_pair.second);
//  Edge be4 = edge_pair.first;
//  boost_g[be4].id = 4;
//  
//  edge_pair = boost::add_edge (bv1, bv2, boost_g);
//  assert (edge_pair.second);
//  Edge be2 = edge_pair.first;
//  boost_g[be2].id = 2;
//  
//  edge_pair = boost::add_edge (bv2, bv1, boost_g);
//  assert (edge_pair.second);
//  Edge be3 = edge_pair.first;
//  boost_g[be3].id = 3;
//  
//  boost::graph_traits<boost_graph_t>::out_edge_iterator it, end;
//  std::cout << "Boost edge range\n";
//  foreach (Edge e, boost::edge_range (bv1, bv2, boost_g)) {
//    int id = boost_g[e].id;
//    std::cout << "Edge id:" << id << std::endl;
//    assert (id == 1 || id == 2);
//  }
//  
//  boost::remove_edge (be1, boost_g);
//  boost::remove_edge (be2, boost_g);
//  boost::remove_vertex (bv1, boost_g);
//  boost::remove_vertex (bv2, boost_g);
  
  graph g;
  Foo f (1, 2);
  Foo h (2, 3);
  int p = 5;
  graph::vertex_iterator v = g.add_vertex (f);
  graph::vertex_iterator s = g.add_vertex (h);
  graph::edge_iterator e = g.add_edge (v, s, p);
  try {
    g.add_edge (v, s, p);
    assert (false && "The edge is already present but inserted!!\n");
  }
  catch (utils::graph_exception& exc)
  {
    cout << "Ok the duplicated edge is not inserted!\n";
  }
  
  cout << "\nVertex v:\n";
  printVertex<graph> (v);
  
  cout << "Vertex s:\n";
  printVertex<graph> (s);
  
  cout << "Edge:\n";
  printEdge (e);
  
  graph::adj_edge_iterator adj = v->get_out_edge (s);
  cout << "v out:\n";
  printEdge (adj);
  cout << "  number of edge out: " << v->out_degree() << endl;
  
  adj = s->get_in_edge (v);
  cout << "s in:\n";
  printEdge (adj);
  cout << "  number of edge in: " << s->in_degree() << endl;
  
  cout << "Check an edge present\n";
  assert (g.exists_edge (v, s));
  cout << "passed\n Check an edge not present\n";
  assert (!g.exists_edge (s, v));
  cout << "passed\n";
  
  cout << "Iterate vertex:\n";
  for (graph::vertex_iterator it = g.vertices_begin(); it != g.vertices_end(); ++it)
    cout << it->data;
  cout << "Iterate edges:\n";
  for (graph::edge_iterator it = g.edges_begin(); it != g.edges_end(); ++it)
    cout << it->data << endl;
  
  cout << "remove vertex s\n";
  g.remove_vertex (s);
  
  cout << "Iterate vertex:\n";
  for (graph::vertex_iterator it = g.vertices_begin(); it != g.vertices_end(); ++it)
    cout << it->data;
  cout << "Iterate edges:\n";
  for (graph::edge_iterator it = g.edges_begin(); it != g.edges_end(); ++it)
    cout << it->data << endl;
  
  cout << "Add vertex and edges\n";
  s = g.add_vertex (h);
  e = g.add_edge (v, s, p);
  graph::edge_iterator a = g.add_edge (s, v, 7);
  
  cout << "Vertexes num: " << g.number_of_vertices()
    << "\nEdges num: " << g.number_of_edges() << endl;
  
  cout << "Iterate vertex:\n";
  for (graph::vertex_iterator it = g.vertices_begin(); it != g.vertices_end(); ++it)
    cout << it->data;
  cout << "Iterate edges:\n";
  for (graph::edge_iterator it = g.edges_begin(); it != g.edges_end(); ++it)
    cout << it->data << endl;
  
  cout << "Try to move an edge in the same place of another edge\n";
  try {
    g.move_edge (e, s, v);
    assert (false && "Exception not throw: ERROR!");
  }
  catch (graph_exception& exc)
  {
    cout << "Exception throwed: OK\n";
  }
  
  cout << "\nremove edge (v, s)\n";
  adj = s->get_in_edge (v);
  g.remove_edge (adj);
  
  cout << "Iterate vertex:\n";
  for (graph::vertex_iterator it = g.vertices_begin(); it != g.vertices_end(); ++it)
    cout << it->data;
  cout << "Iterate edges:\n";
  for (graph::edge_iterator it = g.edges_begin(); it != g.edges_end(); ++it)
    cout << it->data << endl;
  
  cout << "remove edge (s, v)\n";
  e = g.get_edge (s, v);
  g.remove_edge (e);
  
  cout << "Iterate vertex:\n";
  for (graph::vertex_iterator it = g.vertices_begin(); it != g.vertices_end(); ++it)
    cout << it->data;
  cout << "Iterate edges:\n";
  for (graph::edge_iterator it = g.edges_begin(); it != g.edges_end(); ++it)
    cout << it->data << endl;
  
  cout << "Insert new vertex and edge\n";
  s = g.add_vertex (h);
  v = g.add_vertex (f);
  e = g.add_edge (v, s, p);
  
  cout << "Clear the graph\n";
  g.clear ();
  assert (g.number_of_vertices() == 0 && g.number_of_edges() == 0);
  cout << "done\n";
  
  typedef graph_t <map <string, int>, NoData> TestG;
  TestG test_g;
  TestG::vertex_iterator x = test_g.add_vertex ();
  TestG::vertex_iterator y = test_g.add_vertex ();
  TestG::edge_iterator z = test_g.add_edge (x, y);
  
  typedef graph_t <int, int, true> G;
  G gr;
  G::vertex_iterator gv = gr.add_vertex (1);
  G::vertex_iterator gs = gr.add_vertex (2);
  G::edge_iterator ge = gr.add_edge (gv, gs, 3);
  cout << "Before moving:\n";
  cout << "v:\n";
  printVertex<G> (gv);
  cout << "s:\n";
  printVertex<G> (gs);
  cout << "e:\n";
  printEdge (ge);
  gr.move_edge (ge, gs, gv);
  cout << "After moving:\n";
  cout << "v:\n";
  printVertex<G> (gv);
  cout << "s:\n";
  printVertex<G> (gs);
  cout << "e:\n";
  printEdge (ge);
  G::edge_iterator ge2 = gr.add_edge (gs, gv, 4);
  cout << "After adding:\n";
  cout << "v:\n";
  printVertex<G> (gv);
  cout << "s:\n";
  printVertex<G> (gs);
  G::edge_iterator_range range = gr.getall_edges (gs, gv);
  for (G::adj_edge_iterator it = range.first;
        it != range.second; ++it)
    printEdge (it);
  cout << "Move an edge in the same place\n";
  gr.move_edge (ge2, gs, gv);
  cout << "passed\n";
  
  // color map example
  map <G::vertex_iterator, bool> color_map;
  for (G::vertex_iterator it = gr.vertices_begin();
        it != gr.vertices_end(); ++it)
    color_map [it] = false;
  
  G gr2;
  G::vertex_iterator v1 = gr2.add_vertex (1);
  G::vertex_iterator v2 = gr2.add_vertex (2);
  G::vertex_iterator v3 = gr2.add_vertex (3);
  G::vertex_iterator v4 = gr2.add_vertex (4);
  
  G::edge_iterator e1 = gr2.add_edge (v1, v2, 1);
  G::edge_iterator e2 = gr2.add_edge (v1, v3, 2);
  G::edge_iterator e3 = gr2.add_edge (v1, v4, 3);
  G::edge_iterator e4 = gr2.add_edge (v4, v1, 4);
  G::edge_iterator e5 = gr2.add_edge (v3, v1, 5);
  
  assert (v1->out_degree () == 3 && "Insertion failed");
  assert (v1->in_degree () == 2 && "Insertion failed");
  
  cout << "Vertex removing\n";
  gr2.remove_vertex (v1);
  assert (gr2.number_of_edges () == 0 && "Edges not removed");
  assert (gr2.number_of_vertices () == 3 && "Vertices not removed");
  
  v1 = gr2.add_vertex (1);
  e1 = gr2.add_edge (v1, v2, 1);
  e2 = gr2.add_edge (v1, v3, 2);
  e3 = gr2.add_edge (v1, v4, 3);
  e4 = gr2.add_edge (v4, v1, 4);
  e5 = gr2.add_edge (v3, v1, 5);
  
  gr2.remove_out_edges (v1);
  assert (v1->out_degree () == 0 && "remove out edges failed");
  assert (v1->in_degree () == 2 && "In edges also removed??");
  gr2.remove_in_edges (v1);
  assert (v1->in_degree () == 0 && "remove out edges failed");
  cout << "passed\n";
  
  cout << "Remove parallel edges:\n";
  gr2.clear ();
  v1 = gr2.add_vertex (1);
  v2 = gr2.add_vertex (2);
  e1 = gr2.add_edge (v1, v2, 1);
  e2 = gr2.add_edge (v1, v2, 2);
  e3 = gr2.add_edge (v1, v2, 3);
  e4 = gr2.add_edge (v2, v1, 4);
  e5 = gr2.add_edge (v2, v1, 5);
  
  gr2.remove_edge (e2);
  assert (v1->out_degree () == 2);
  assert (v2->in_degree () == 2);
  assert (gr2.number_of_edges () == 4);
  
  cout << "Out edges after the remove: \n";
  for (G::adj_edge_iterator it = v1->out_edges_begin(); 
       it != v1->out_edges_end(); 
       ++it)
  {
    int i = it->data;
    cout << "Edge value: " << i << endl;
    assert (i == 1 || i == 3);
  }
  
  cout << "In edges after the remove: \n";
  for (G::adj_edge_iterator it = v2->in_edges_begin(); 
       it != v2->in_edges_end(); 
       ++it)
  {
    int i = it->data;
    cout << "Edge value: " << i << endl;
    assert (i == 1 || i == 3);
  }
  
  gr2.remove_edge (e1);
  assert (gr2.get_edge (v1, v2)->data == 3);
  cout << "passed\n";
  
  cout << "Remove vertex with parallel edges:\n";
  gr2.clear ();
  v1 = gr2.add_vertex (1);
  v2 = gr2.add_vertex (2);
  v3 = gr2.add_vertex (3);
  e1 = gr2.add_edge (v1, v2, 1);
  e2 = gr2.add_edge (v1, v2, 2);
  e3 = gr2.add_edge (v1, v2, 3);
  e4 = gr2.add_edge (v2, v1, 4);
  e5 = gr2.add_edge (v2, v1, 5);
  G::edge_iterator e6 = gr2.add_edge (v1, v3, 6);
  G::edge_iterator e7 = gr2.add_edge (v2, v3, 7);
  G::edge_iterator e8 = gr2.add_edge (v3, v1, 8);
  
  cout << "Before removing:\n";
  for (G::edge_iterator it = gr2.edges_begin(); 
       it != gr2.edges_end(); 
       ++it)
  {
    cout << "Edge " << it->data << ": "
      << "source: " << it->get_source()->data
      << " target: " << it->get_target()->data
      << endl;
  }
  
  cout << "After removing:\n";
  gr2.remove_edge (e3);
  gr2.remove_vertex (v1);
  for (G::edge_iterator it = gr2.edges_begin(); 
       it != gr2.edges_end(); 
       ++it)
  {
    cout << "Edge " << it->data << ": "
      << "source: " << it->get_source()->data
      << " target: " << it->get_target()->data
      << endl;
  }
  assert (gr2.number_of_edges() == 1);
  e1 = gr2.edges_begin ();
  assert (e1->get_source() == v2 &&
          e1->get_target() == v3 &&
          e1->data == 7);
  cout << "passed\n";
  
  cout << "Moving edges with parallel graph\n";
  v1 = gr2.add_vertex (1);
  v2 = gr2.add_vertex (2);
  e1 = gr2.add_edge (v1, v2, 1);
  e2 = gr2.add_edge (v1, v2, 2);
  e3 = gr2.add_edge (v1, v2, 3);
  e4 = gr2.add_edge (v2, v1, 4);
  e5 = gr2.add_edge (v2, v1, 5);
  
  gr2.move_edge (e2, v2, v1);
  
  assert (e2->get_source()->data == 2);
  assert (e2->get_target()->data == 1);
  for (G::adj_edge_iterator it = v1->out_edges_begin(); 
       it != v1->out_edges_end(); 
       ++it)
  {
    assert (it->get_source()->data == 1);
    assert (it->get_target()->data == 2);
    int data = it->data;
    cout << data << endl;
    assert (data == 1 || data == 3);
  }
  for (G::adj_edge_iterator it = v2->in_edges_begin(); 
       it != v2->in_edges_end(); 
       ++it)
  {
    assert (it->get_source()->data == 1);
    assert (it->get_target()->data == 2);
    int data = it->data;
    cout << data << endl;
    assert (data == 1 || data == 3);
  }
  for (G::adj_edge_iterator it = v2->out_edges_begin(); 
       it != v2->out_edges_end(); 
       ++it)
  {
    assert (it->get_source()->data == 2);
    assert (it->get_target()->data == 1);
    int data = it->data;
    cout << data << endl;
    assert (data == 2 || data == 4 || data == 5);
  }
  for (G::adj_edge_iterator it = v1->in_edges_begin(); 
       it != v1->in_edges_end(); 
       ++it)
  {
    assert (it->get_source()->data == 2);
    assert (it->get_target()->data == 1);
    int data = it->data;
    cout << data << endl;
    assert (data == 2 || data == 4 || data == 5);
  }
  
  cout << "passed\n";
  
  return 0;  
  
}
