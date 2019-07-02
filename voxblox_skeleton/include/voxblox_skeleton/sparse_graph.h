#ifndef VOXBLOX_SKELETON_SPARSE_GRAPH_H
#define VOXBLOX_SKELETON_SPARSE_GRAPH_H

#include <map>
#include <voxblox/core/common.h>

namespace voxblox {

  struct GraphVertex {
    int64_t vertex_id = -1;
    Point point = Point::Zero();
    float distance = 0.0f;

    std::vector<int64_t> edge_list;

    int subgraph_id = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct GraphEdge {
    int64_t edge_id = -1;
    int64_t start_vertex = -1;
    int64_t end_vertex = -1;
    Point start_point = Point::Zero();
    Point end_point = Point::Zero();
    float start_distance = 0.0f;
    float end_distance = 0.0f;

    int subgraph_id = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class SparseGraph {
    typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint>
        Transformation;

  public:
    SparseGraph();

    int64_t addVertex(const GraphVertex& vertex);
    // Add Edge does all the heavy lifting: connects the vertices using their
    // internal edge lists, and updates the start and end points within the
    // edge.
    int64_t addEdge(const GraphEdge& edge);

    // Removal operations... Takes care of breaking all previous connections.
    void removeVertex(int64_t vertex_id);
    void removeEdge(int64_t edge_id);

    bool hasVertex(int64_t id) const;
    bool hasEdge(int64_t id) const;

    const GraphVertex& getVertex(int64_t id) const;
    const GraphEdge& getEdge(int64_t id) const;

    GraphVertex& getVertex(int64_t id);
    GraphEdge& getEdge(int64_t id);

    // Accessors to just get all the vertex and edge IDs.
    void getAllVertexIds(std::vector<int64_t>* vertex_ids) const;
    void getAllEdgeIds(std::vector<int64_t>* edge_ids) const;

    void clear();

    // Returns true if the vertices have a direct (i.e., ONE edge) connection
    // to each other.
    bool areVerticesDirectlyConnected(int64_t vertex_id_1,
                                      int64_t vertex_id_2) const;

    // Only const access to the vertex and edge maps, mostly for kD-tree use.
    // To modify the stuff, use add and remove vertex/edge, since this preserves
    // the consistency of the graph.
    const std::map<int64_t, GraphVertex>& getVertexMap() const {
      return vertex_map_;
    }
    const std::map<int64_t, GraphEdge>& getEdgeMap() const {
      return edge_map_;
    }

    // These are the barebones version: does not enforce any connections, used
    // only for de-serializing sparse skeleton graphs. Also sets ids to the ids
    // already in the structure.
    void addSerializedVertex(const GraphVertex& vertex);
    void addSerializedEdge(const GraphEdge& edge);

    // transforms vertex locations into different frame
    void transformFrame(Transformation T_G_S);

  private:
    // Vertex and edge IDs are separate.
    std::map<int64_t, GraphVertex> vertex_map_;
    std::map<int64_t, GraphEdge> edge_map_;

    int64_t next_vertex_id_;
    int64_t next_edge_id_;
  };

}

#endif //VOXBLOX_SKELETON_SPARSE_GRAPH_H
