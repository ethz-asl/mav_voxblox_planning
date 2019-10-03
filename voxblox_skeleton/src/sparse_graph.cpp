#include <voxblox_skeleton/sparse_graph.h>
#include <iostream>
namespace voxblox {

SparseSkeletonGraph::SparseSkeletonGraph()
    : next_vertex_id_(0), next_edge_id_(0) {}

int64_t SparseSkeletonGraph::addVertex(const GraphVertex& vertex) {
  int64_t vertex_id = next_vertex_id_++;
  // Returns an iterator...
  auto iter_pair = vertex_map_.emplace(std::make_pair(vertex_id, vertex));
  iter_pair.first->second.vertex_id = vertex_id;
  return vertex_id;
}

int64_t SparseSkeletonGraph::addEdge(const GraphEdge& edge) {
  int64_t edge_id = next_edge_id_++;
  // Returns an iterator...
  auto iter_pair = edge_map_.emplace(std::make_pair(edge_id, edge));
  iter_pair.first->second.edge_id = edge_id;

  // Now hook it up to the vertices.
  GraphVertex& start_vertex = vertex_map_.at(edge.start_vertex);
  GraphVertex& end_vertex = vertex_map_.at(edge.end_vertex);

  start_vertex.edge_list.push_back(edge_id);
  end_vertex.edge_list.push_back(edge_id);

  iter_pair.first->second.start_point = start_vertex.point;
  iter_pair.first->second.end_point = end_vertex.point;

  return edge_id;
}

bool SparseSkeletonGraph::hasVertex(int64_t id) const {
  if (vertex_map_.find(id) != vertex_map_.end()) {
    return true;
  }
  return false;
}

bool SparseSkeletonGraph::hasEdge(int64_t id) const {
  if (edge_map_.find(id) != edge_map_.end()) {
    return true;
  }
  return false;
}

const GraphVertex& SparseSkeletonGraph::getVertex(int64_t id) const {
  return vertex_map_.find(id)->second;
}

const GraphEdge& SparseSkeletonGraph::getEdge(int64_t id) const {
  return edge_map_.find(id)->second;
}

GraphVertex& SparseSkeletonGraph::getVertex(int64_t id) {
  return vertex_map_.find(id)->second;
}

GraphEdge& SparseSkeletonGraph::getEdge(int64_t id) {
  return edge_map_.find(id)->second;
}

GraphVertex* SparseSkeletonGraph::getVertexPtr(int64_t id) {
  return &vertex_map_.at(id);
}
GraphEdge* SparseSkeletonGraph::getEdgePtr(int64_t id) {
  return &edge_map_.at(id);
}

void SparseSkeletonGraph::clear() {
  next_vertex_id_ = 0;
  next_edge_id_ = 0;

  vertex_map_.clear();
  edge_map_.clear();
}

void SparseSkeletonGraph::getAllVertexIds(
    std::vector<int64_t>* vertex_ids) const {
  vertex_ids->reserve(vertex_map_.size());
  for (const std::pair<int64_t, GraphVertex>& kv : vertex_map_) {
    vertex_ids->push_back(kv.first);
  }
}

void SparseSkeletonGraph::getAllEdgeIds(std::vector<int64_t>* edge_ids) const {
  edge_ids->reserve(edge_map_.size());
  for (const std::pair<int64_t, GraphEdge>& kv : edge_map_) {
    edge_ids->push_back(kv.first);
  }
}

void SparseSkeletonGraph::removeVertex(int64_t vertex_id) {
  // Find the vertex.
  std::map<int64_t, GraphVertex>::iterator iter =
      vertex_map_.find(vertex_id);
  if (iter == vertex_map_.end()) {
    return;
  }
  const GraphVertex& vertex = iter->second;

  // Remove all edges that are connected to it.
  for (int64_t edge_id : vertex.edge_list) {
    removeEdge(edge_id);
  }

  // Remove the vertex.
  vertex_map_.erase(iter);
}

void SparseSkeletonGraph::removeEdge(int64_t edge_id) {
  // Find the edge.
  std::map<int64_t, GraphEdge>::iterator iter = edge_map_.find(edge_id);
  if (iter == edge_map_.end()) {
    return;
  }
  const GraphEdge& edge = iter->second;

  // Remove this edge from both vertices.
  if (hasVertex(edge.start_vertex)) {
    GraphVertex& vertex_1 = getVertex(edge.start_vertex);
    for (size_t i = 0; i < vertex_1.edge_list.size(); i++) {
      if (vertex_1.edge_list[i] == edge_id) {
        vertex_1.edge_list.erase(vertex_1.edge_list.begin() + i);
        break;
      }
    }
  }
  if (hasVertex(edge.end_vertex)) {
    GraphVertex& vertex_2 = getVertex(edge.end_vertex);
    for (size_t i = 0; i < vertex_2.edge_list.size(); i++) {
      if (vertex_2.edge_list[i] == edge_id) {
        vertex_2.edge_list.erase(vertex_2.edge_list.begin() + i);
        break;
      }
    }
  }

  edge_map_.erase(iter);
}

bool SparseSkeletonGraph::areVerticesDirectlyConnected(
    int64_t vertex_id_1, int64_t vertex_id_2) const {
  const GraphVertex& vertex_1 = getVertex(vertex_id_1);
  // Iterate over all edges of vertex 1.
  for (int64_t edge_id : vertex_1.edge_list) {
    // Check if any connect to vertex 2.
    const GraphEdge& edge = getEdge(edge_id);
    if (edge.start_vertex == vertex_id_2 || edge.end_vertex == vertex_id_2) {
      return true;
    }
  }

  return false;
}

void SparseSkeletonGraph::addSerializedVertex(const GraphVertex& vertex) {
  vertex_map_[vertex.vertex_id] = vertex;
}
void SparseSkeletonGraph::addSerializedEdge(const GraphEdge& edge) {
  edge_map_[edge.edge_id] = edge;
}

void SparseSkeletonGraph::transformFrame(const Transformation& T_G_S) {
  // transform vertices
  for (std::pair<const int64_t, GraphVertex>& kv : vertex_map_) {
    kv.second.point = T_G_S * kv.second.point;
  }
  // transform edges
  for (std::pair<const int64_t, GraphEdge>& kv : edge_map_) {
    kv.second.start_point = T_G_S * kv.second.start_point;
    kv.second.end_point = T_G_S * kv.second.end_point;
  }
}

}
