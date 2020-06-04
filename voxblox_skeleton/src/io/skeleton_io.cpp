#include <voxblox/utils/protobuf_utils.h>

#include "voxblox_skeleton/skeleton.h"

#include "voxblox_skeleton/io/skeleton_io.h"

namespace voxblox {
namespace io {

bool saveSparseSkeletonGraphToFile(const std::string& filename,
                                   const SparseSkeletonGraph& graph) {
  // Then fill in all the edges.
  CHECK(!filename.empty());
  std::fstream outfile;
  // Will APPEND to the current file in case outputting multiple layers on the
  // same file, depending on the flag.
  std::ios_base::openmode file_flags =
      std::fstream::out | std::fstream::binary | std::fstream::trunc;
  outfile.open(filename, file_flags);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << filename;
    return false;
  }

  SkeletonGraphProto proto;

  // First save all the vertices.
  const std::map<int64_t, SkeletonVertex>& vertices = graph.getVertexMap();

  for (const std::pair<int64_t, SkeletonVertex>& kv : vertices) {
    proto.add_vertices();
    convertVertexToProto(kv.second,
                         proto.mutable_vertices(proto.vertices_size() - 1));
  }

  // Now it's time for edges.
  const std::map<int64_t, SkeletonEdge>& edges = graph.getEdgeMap();

  for (const std::pair<int64_t, SkeletonEdge>& kv : edges) {
    proto.add_edges();
    convertEdgeToProto(kv.second, proto.mutable_edges(proto.edges_size() - 1));
  }

  if (!utils::writeProtoMsgToStream(proto, &outfile)) {
    LOG(ERROR) << "Could not write layer header message.";
    outfile.close();
    return false;
  }

  return true;
}

bool loadSparseSkeletonGraphFromFile(const std::string& filename,
                                     SparseSkeletonGraph* graph) {
  CHECK(!filename.empty());
  CHECK_NOTNULL(graph);
  graph->clear();

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(filename, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << filename;
    return false;
  }

  // Byte offset result, used to keep track where we are in the file if
  // necessary.
  uint64_t tmp_byte_offset = 0;

  SkeletonGraphProto proto;
  if (!utils::readProtoMsgFromStream(&proto_file, &proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read skeleton graph proto from file!";
    return false;
  }
  for (size_t i = 0; i < proto.vertices_size(); i++) {
    SkeletonVertex vertex;
    convertProtoToVertex(proto.vertices(i), &vertex);
    graph->addSerializedVertex(vertex);
  }
  for (size_t i = 0; i < proto.edges_size(); i++) {
    SkeletonEdge edge;
    convertProtoToEdge(proto.edges(i), &edge);
    graph->addSerializedEdge(edge);
  }
  return true;
}

void convertVertexToProto(const SkeletonVertex& vertex,
                          SkeletonVertexProto* proto) {
  CHECK_NOTNULL(proto);
  proto->set_vertex_id(vertex.vertex_id);
  proto->set_point_x(vertex.point.x());
  proto->set_point_y(vertex.point.y());
  proto->set_point_z(vertex.point.z());
  proto->set_subgraph_id(vertex.subgraph_id);
  for (const int64_t& edge_id : vertex.edge_list) {
    proto->add_edge_list(edge_id);
  }
}

void convertEdgeToProto(const SkeletonEdge& edge, SkeletonEdgeProto* proto) {
  CHECK_NOTNULL(proto);
  proto->set_edge_id(edge.edge_id);
  proto->set_start_vertex(edge.start_vertex);
  proto->set_end_vertex(edge.end_vertex);
  proto->set_start_point_x(edge.start_point.x());
  proto->set_start_point_y(edge.start_point.y());
  proto->set_start_point_z(edge.start_point.z());
  proto->set_end_point_x(edge.end_point.x());
  proto->set_end_point_y(edge.end_point.y());
  proto->set_end_point_z(edge.end_point.z());
  proto->set_start_distance(edge.start_distance);
  proto->set_end_distance(edge.end_distance);
}

void convertProtoToVertex(const SkeletonVertexProto& proto,
                          SkeletonVertex* vertex) {
  CHECK_NOTNULL(vertex);
  vertex->vertex_id = proto.vertex_id();
  vertex->point << proto.point_x(), proto.point_y(), proto.point_z();
  vertex->subgraph_id = proto.subgraph_id();

  vertex->edge_list.reserve(proto.edge_list_size());
  for (size_t i = 0; i < proto.edge_list_size(); i++) {
    vertex->edge_list.push_back(proto.edge_list(i));
  }
}

void convertProtoToEdge(const SkeletonEdgeProto& proto, SkeletonEdge* edge) {
  CHECK_NOTNULL(edge);
  edge->edge_id = proto.edge_id();
  edge->start_vertex = proto.start_vertex();
  edge->end_vertex = proto.end_vertex();

  edge->start_point << proto.start_point_x(), proto.start_point_y(),
      proto.start_point_z();
  edge->end_point << proto.end_point_x(), proto.end_point_y(),
      proto.end_point_z();

  edge->start_distance = proto.start_distance();
  edge->end_distance = proto.end_distance();
}

}  // namespace io
}  // namespace voxblox
