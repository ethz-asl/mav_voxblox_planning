#ifndef VOXBLOX_SKELETON_IO_SKELETON_IO_H_
#define VOXBLOX_SKELETON_IO_SKELETON_IO_H_

#include "./skeleton.pb.h"

#include "voxblox_skeleton/skeleton.h"

namespace voxblox {
namespace io {

bool saveSparseSkeletonGraphToFile(const std::string& filename,
                                   const SparseSkeletonGraph& graph);

bool loadSparseSkeletonGraphFromFile(const std::string& filename,
                                     SparseSkeletonGraph* graph);

void convertVertexToProto(const SkeletonVertex& vertex,
                          SkeletonVertexProto* proto);
void convertEdgeToProto(const SkeletonEdge& edge, SkeletonEdgeProto* proto);

void convertProtoToVertex(const SkeletonVertexProto& proto,
                          SkeletonVertex* vertex);
void convertProtoToEdge(const SkeletonEdgeProto& proto, SkeletonEdge* edge);

}  // namespace io
}  // namespace voxblox

#endif  // VOXBLOX_SKELETON_IO_SKELETON_IO_H_
