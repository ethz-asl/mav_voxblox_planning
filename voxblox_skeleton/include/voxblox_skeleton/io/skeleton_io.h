#ifndef VOXBLOX_SKELETON_IO_SKELETON_IO_H_
#define VOXBLOX_SKELETON_IO_SKELETON_IO_H_

#include "./skeleton.pb.h"

#include "voxblox_skeleton/skeleton.h"
#include <voxblox_skeleton/sparse_graph_planner.h>

namespace voxblox {
namespace io {

bool saveSparseGraphToFile(const std::string& filename,
                                   const SparseGraph& graph);

bool loadSparseGraphFromFile(const std::string& filename,
                                     SparseGraph* graph);

void convertVertexToProto(const GraphVertex& vertex,
                          GraphVertexProto* proto);
void convertEdgeToProto(const GraphEdge& edge, GraphEdgeProto* proto);

void convertProtoToVertex(const GraphVertexProto& proto,
                          GraphVertex* vertex);
void convertProtoToEdge(const GraphEdgeProto& proto, GraphEdge* edge);

}  // namespace io
}  // namespace voxblox

#endif  // VOXBLOX_SKELETON_IO_SKELETON_IO_H_
