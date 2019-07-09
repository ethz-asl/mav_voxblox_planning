#ifndef VOXBLOX_SKELETON_BOOST_GRAPH_H
#define VOXBLOX_SKELETON_BOOST_GRAPH_H

//
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <sys/time.h>
#include <vector>
#include <list>

namespace voxblox {

class BoostGraphPlanner {

// auxiliary types
  typedef voxblox::Point location;
  typedef voxblox::FloatingPoint cost;

// euclidean distance heuristic
  template <class Graph, class CostType, class LocMap>
  class distance_heuristic : public astar_heuristic<Graph, CostType>
  {
  public:
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(LocMap l, Vertex goal)
        : m_location(l), m_goal(goal) {}
    CostType operator()(Vertex u) {
      return (m_location[m_goal] - m_location[u]).norm();
    }
  private:
    LocMap m_location;
    Vertex m_goal;
  };

// exception for termination
  struct found_goal {};

// visitor that terminates when we find the goal
  template <class Vertex>
  class astar_goal_visitor : public boost::default_astar_visitor
  {
  public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
      if(u == m_goal)
        throw found_goal();
    }
  private:
    Vertex m_goal;
  };

  // specify some types
  typedef adjacency_list<listS, vecS, undirectedS, no_property,
  property<edge_weight_t, cost> > BoostGraph;
  typedef property_map<BoostGraph, edge_weight_t>::type BoostWeightMap;
  typedef BoostGraph::vertex_descriptor BoostVertex;
  typedef BoostGraph::edge_descriptor BoostEdgeDescriptor;
  typedef std::pair<int, int> BoostEdge;

 public:
  BoostGraphPlanner();
  void addGraph(SparseGraph* graph) {
    CHECK_NOTNULL(graph);
    //todo

    // specify data
    enum nodes { Troy  };
    const char *name[] = { "New York"  };
    location locations[] = { {42.73, 73.68}  };
    BoostEdge edge_array[] = { BoostEdge(Troy,Utica)  };
    unsigned int num_edges = sizeof(edge_array) / sizeof(BoostEdge);
    cost weights[] = { 96  };

    // create graph
    BoostGraph g(N);
    BoostWeightMap weightmap = get(edge_weight, g);
    for(std::size_t j = 0; j < num_edges; ++j) {
      BoostEdgeDescriptor e;
      bool inserted;
      boost::tie(e, inserted) = boost::add_edge(edge_array[j].first,
                                                edge_array[j].second, g);
      weightmap[e] = weights[j];
    }
  }

  getPathBetweenVertices(int64_t start_vertex_id, int64_t end_vertex_id,
      std::vector<int64_t>* vertex_path) {
    vector<BoostGraph::vertex_descriptor> p(num_vertices(g));
    vector<cost> d(num_vertices(g));
    try {
      // call astar named parameter interface
      astar_search_tree
          (g, start,
           distance_heuristic<BoostGraph, cost, location*>
               (locations, goal),
           predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
               distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
               visitor(astar_goal_visitor<BoostVertex>(goal)));


    } catch(found_goal fg) { // found a path to the goal
      list<BoostVertex> shortest_path;
      for(BoostVertex v = goal;; v = p[v]) {
        shortest_path.push_front(v);
        if(p[v] == v)
          break;
      }
      cout << "Shortest path from " << name[start] << " to "
           << name[goal] << ": ";
      list<BoostVertex>::iterator spi = shortest_path.begin();
      cout << name[start];
      for(++spi; spi != shortest_path.end(); ++spi)
        cout << " -> " << name[*spi];
      cout << endl << "Total travel time: " << d[goal] << endl;
      return 0;
    }

    cout << "Didn't find a path from " << name[start] << "to"
         << name[goal] << "!" << endl;
    return 0;
  };

 private:
  graph_;

};

}

#endif //VOXBLOX_SKELETON_BOOST_GRAPH_H
