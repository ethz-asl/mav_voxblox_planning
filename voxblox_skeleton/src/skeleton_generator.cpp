#include "voxblox_skeleton/io/skeleton_io.h"

#include "voxblox_skeleton/skeleton_generator.h"

namespace voxblox {

SkeletonGenerator::SkeletonGenerator()
    : min_separation_angle_(0.785),
      generate_by_layer_neighbors_(false),
      num_neighbors_for_edge_(18),
      check_edges_on_construction_(false),
      vertex_pruning_radius_(0.35),
      min_gvd_distance_(0.4),
      cleanup_style_(kSimplify) {
  // Initialize the template matchers.
  pruning_template_matcher_.setDeletionTemplates();
  corner_template_matcher_.setCornerTemplates();
}

SkeletonGenerator::SkeletonGenerator(Layer<EsdfVoxel>* esdf_layer)
    : SkeletonGenerator() {
  setEsdfLayer(esdf_layer);
  CHECK_NOTNULL(esdf_layer);
}

void SkeletonGenerator::setEsdfLayer(Layer<EsdfVoxel>* esdf_layer) {
  CHECK_NOTNULL(esdf_layer);
  esdf_layer_ = esdf_layer;

  voxels_per_side_ = esdf_layer_->voxels_per_side();
  voxel_size_ = esdf_layer_->voxel_size();

  // Make a skeleton layer to store the intermediate skeleton steps, along with
  // the lists.
  skeleton_layer_.reset(new Layer<SkeletonVoxel>(
      esdf_layer_->voxel_size(), esdf_layer_->voxels_per_side()));
  skeleton_planner_.setSkeletonLayer(skeleton_layer_.get());
  skeleton_planner_.setEsdfLayer(esdf_layer_);
  skeleton_planner_.setMinEsdfDistance(min_gvd_distance_);

  CHECK_EQ(voxel_size_, skeleton_layer_->voxel_size());
  CHECK_EQ(voxels_per_side_, skeleton_layer_->voxels_per_side());
}

void SkeletonGenerator::updateSkeletonFromLayer() {
  CHECK(skeleton_layer_);

  timing::Timer update_timer("skeleton/update_from_layer");

  // Clear whatever's in there.
  skeleton_.getSkeletonPoints().clear();
  skeleton_.getEdgePoints().clear();
  skeleton_.getVertexPoints().clear();

  // Now iterate over the whole thing, and stick any face/edge/vertex points
  // in there.
  BlockIndexList blocks;
  skeleton_layer_->getAllAllocatedBlocks(&blocks);

  for (const BlockIndex& block_index : blocks) {
    const Block<SkeletonVoxel>::Ptr skeleton_block =
        skeleton_layer_->getBlockPtrByIndex(block_index);

    const size_t num_voxels_per_block = skeleton_block->num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const SkeletonVoxel& skeleton_voxel =
          skeleton_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          skeleton_block->computeVoxelIndexFromLinearIndex(lin_index);

      if (!skeleton_voxel.is_face) {
        continue;
      }
      Point coords =
          skeleton_block->computeCoordinatesFromVoxelIndex(voxel_index);
      SkeletonPoint skeleton_point;
      skeleton_point.distance = skeleton_voxel.distance;
      skeleton_point.num_basis_points = skeleton_voxel.num_basis_points;
      skeleton_point.point = coords;
      skeleton_.getSkeletonPoints().push_back(skeleton_point);
      if (skeleton_voxel.is_edge) {
        skeleton_.getEdgePoints().push_back(skeleton_point);
      }
      if (skeleton_voxel.is_vertex) {
        skeleton_.getVertexPoints().push_back(skeleton_point);
      }
    }
  }
}

void SkeletonGenerator::generateSkeleton() {
  timing::Timer generate_timer("skeleton/gvd");

  // Clear the skeleton and start over.
  skeleton_.getSkeletonPoints().clear();

  // Iterate over all blocks in the ESDF...
  // So should be wavefront ends, but can just check parent direction.
  // Maybe a minimum angle between parent directions of neighbors?
  BlockIndexList blocks;
  esdf_layer_->getAllAllocatedBlocks(&blocks);

  for (const BlockIndex& block_index : blocks) {
    const Block<EsdfVoxel>::Ptr& esdf_block =
        esdf_layer_->getBlockPtrByIndex(block_index);
    Block<SkeletonVoxel>::Ptr skeleton_block =
        skeleton_layer_->allocateBlockPtrByIndex(block_index);

    const size_t num_voxels_per_block = esdf_block->num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const EsdfVoxel& esdf_voxel =
          esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);

      if (!esdf_voxel.observed || esdf_voxel.distance < min_gvd_distance_ ||
          esdf_voxel.fixed) {
        continue;
      }

      Point coords = esdf_block->computeCoordinatesFromVoxelIndex(voxel_index);

      // Get the floating-point distance of this voxel, normalize it as long
      // as it's not 0.
      Eigen::Vector3f parent_dir = esdf_voxel.parent.cast<float>();

      // Parent-less voxel (probably in max-distance area), just skip.
      if (parent_dir.norm() < 1e-6) {
        continue;
      }
      parent_dir.normalize();

      // See what the neighbors are pointing to. You can choose what
      // connectivity you want.
      AlignedVector<VoxelKey> neighbors;
      Neighborhood<>::getFromBlockAndVoxelIndex(block_index, voxel_index,
                                                voxels_per_side_, &neighbors);

      // Just go though the 6-connectivity set of this to start.
      SkeletonPoint skeleton_point;
      bool on_skeleton = false;
      for (size_t i = 0; i < neighbors.size(); ++i) {
        // Get this voxel with way too many checks.
        // Get the block for this voxel.
        BlockIndex neighbor_block_index = neighbors[i].first;
        VoxelIndex neighbor_voxel_index = neighbors[i].second;
        Block<EsdfVoxel>::Ptr neighbor_block;
        if (neighbor_block_index == block_index) {
          neighbor_block = esdf_block;
        } else {
          neighbor_block =
              esdf_layer_->getBlockPtrByIndex(neighbor_block_index);
        }
        if (!neighbor_block) {
          continue;
        }
        CHECK(neighbor_block->isValidVoxelIndex(neighbor_voxel_index))
            << "Neigbor voxel index: " << neighbor_voxel_index.transpose();

        EsdfVoxel& neighbor_voxel =
            neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

        if (!neighbor_voxel.observed ||
            neighbor_voxel.distance < min_gvd_distance_ ||
            neighbor_voxel.fixed) {
          continue;
        }

        // Get the relative distance: that is, if this voxel was on top of
        // ours, what direction would it point in, roughly?
        // Make sure this is a double so we can normalize it.
        Eigen::Vector3f relative_direction =
            neighbor_voxel.parent.cast<float>() +
            Neighborhood<>::kOffsets.col(i).cast<float>();

        if (relative_direction.norm() < 1e-6) {
          // This is pointing at us! We're its parent. No way is this a
          // skeleton point.
          continue;
        }
        relative_direction.normalize();

        // Compute the dot product between the two...
        float dot_prod = relative_direction.dot(parent_dir);
        if (acos(dot_prod) >= min_separation_angle_) {
          // Then this is a ridge or something! Probably. Who knows.
          on_skeleton = true;
          skeleton_point.num_basis_points++;
          skeleton_point.basis_directions.push_back(relative_direction);
        }
      }
      if (on_skeleton) {
        skeleton_point.distance =
            esdf_block->getVoxelByVoxelIndex(voxel_index).distance;

        skeleton_point.point = coords;
        skeleton_.getSkeletonPoints().push_back(skeleton_point);

        // Also add it to the layer.
        SkeletonVoxel& skeleton_voxel =
            skeleton_block->getVoxelByVoxelIndex(voxel_index);
        skeleton_voxel.distance = skeleton_point.distance;
        skeleton_voxel.num_basis_points = skeleton_point.num_basis_points;
        if (!generate_by_layer_neighbors_) {
          skeleton_voxel.is_face = (skeleton_voxel.num_basis_points == 9);
          skeleton_voxel.is_edge = (skeleton_voxel.num_basis_points >= 12);
          skeleton_voxel.is_vertex = (skeleton_voxel.num_basis_points == 16);

          if (skeleton_voxel.is_edge) {
            skeleton_.getEdgePoints().push_back(skeleton_point);
          }
          if (skeleton_voxel.is_vertex) {
            skeleton_.getVertexPoints().push_back(skeleton_point);
          }
        } else {
          // If we're generating by layer neighbors, don't bother setting
          // vertex vs. edge yet.
          skeleton_voxel.is_face = true;
        }
      }
    }
  }

  LOG(INFO)
      << "[GVD] Finished finding GVD candidates. Number of skeleton points: "
      << skeleton_.getSkeletonPoints().size()
      << " edges: " << skeleton_.getEdgePoints().size();
  if (generate_by_layer_neighbors_) {
    generateEdgesByLayerNeighbors();
  }
  // Keep going until a certain small percentage remains...
  size_t num_pruned = 1;
  while (num_pruned > 0) {
    num_pruned = pruneDiagramEdges();
  }

  generateVerticesByLayerNeighbors();
  pruneDiagramVertices();
}

void SkeletonGenerator::generateEdgesByLayerNeighbors() {
  timing::Timer generate_timer("skeleton/neighbor_gvd_edge");

  // Rather than iterate over the entire layer, let's just go over all the
  // points in the skeleton.
  const AlignedVector<SkeletonPoint>& skeleton_points =
      skeleton_.getSkeletonPoints();

  // Then figure out how to connect them to other vertices by following the
  // skeleton layer voxels.
  for (const SkeletonPoint& point : skeleton_points) {
    // Get the voxel.
    BlockIndex block_index =
        skeleton_layer_->computeBlockIndexFromCoordinates(point.point);
    Block<SkeletonVoxel>::Ptr block_ptr;
    block_ptr = skeleton_layer_->getBlockPtrByIndex(block_index);
    CHECK(block_ptr);
    VoxelIndex voxel_index =
        block_ptr->computeVoxelIndexFromCoordinates(point.point);
    SkeletonVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);

    // Now just get the neighbors and count how many are on the skeleton.
    AlignedVector<VoxelKey> neighbors;
    Neighborhood<>::getFromBlockAndVoxelIndex(block_index, voxel_index,
                                              voxels_per_side_, &neighbors);

    int num_neighbors_on_medial_axis = 0;
    for (size_t i = 0; i < neighbors.size(); ++i) {
      // Get the block for this voxel.
      BlockIndex neighbor_block_index = neighbors[i].first;
      VoxelIndex neighbor_voxel_index = neighbors[i].second;
      Block<SkeletonVoxel>::Ptr neighbor_block;
      if (neighbor_block_index == block_index) {
        neighbor_block = block_ptr;
      } else {
        neighbor_block =
            skeleton_layer_->getBlockPtrByIndex(neighbor_block_index);
      }
      if (!neighbor_block) {
        continue;
      }
      SkeletonVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

      if (neighbor_voxel.is_face) {
        num_neighbors_on_medial_axis++;
      }
    }
    if (num_neighbors_on_medial_axis >= num_neighbors_for_edge_) {
      voxel.is_edge = true;
      skeleton_.getEdgePoints().push_back(point);
    }
  }
}

size_t SkeletonGenerator::pruneDiagramEdges() {
  timing::Timer timer("skeleton/prune_edges");

  // Go through all edge points, checking them against the templates. Remove
  // any that fit the template, and mark them in removal indices.
  std::vector<size_t> removal_indices;

  const AlignedList<SkeletonPoint>& edge_points = skeleton_.getEdgePoints();

  size_t j = 0;
  for (const SkeletonPoint& edge : edge_points) {
    // Get the voxel.
    BlockIndex block_index =
        skeleton_layer_->computeBlockIndexFromCoordinates(edge.point);
    Block<SkeletonVoxel>::Ptr block_ptr;
    block_ptr = skeleton_layer_->getBlockPtrByIndex(block_index);
    CHECK(block_ptr);
    VoxelIndex voxel_index =
        block_ptr->computeVoxelIndexFromCoordinates(edge.point);
    SkeletonVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
    if (!voxel.is_edge) {
      // this is already deleted.
      continue;
    }

    timing::Timer neighbor_timer("skeleton/prune_edges/neighbors");
    // Now just get the neighbors and count how many are on the skeleton.
    AlignedVector<VoxelKey> neighbors;
    Neighborhood<>::getFromBlockAndVoxelIndex(block_index, voxel_index,
                                              voxels_per_side_, &neighbors);

    std::bitset<27> neighbor_bitset;
    for (size_t i = 0; i < neighbors.size(); ++i) {
      // Get the block for this voxel.
      BlockIndex neighbor_block_index = neighbors[i].first;
      VoxelIndex neighbor_voxel_index = neighbors[i].second;
      Block<SkeletonVoxel>::Ptr neighbor_block;
      if (neighbor_block_index == block_index) {
        neighbor_block = block_ptr;
      } else {
        neighbor_block =
            skeleton_layer_->getBlockPtrByIndex(neighbor_block_index);
      }
      if (!neighbor_block) {
        continue;
      }
      SkeletonVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

      if (neighbor_voxel.is_edge) {
        neighbor_bitset[mapNeighborIndexToBitsetIndex(i)] = true;
      }
    }
    neighbor_timer.Stop();
    timing::Timer template_timer("skeleton/prune_edges/template");
    if (pruning_template_matcher_.fitsTemplates(neighbor_bitset)) {
      if (isSimplePoint(neighbor_bitset) && !isEndPoint(neighbor_bitset)) {
        voxel.is_edge = false;
        removal_indices.push_back(j);
      }
    }
    template_timer.Stop();
    j++;
  }

  size_t num_removed = removal_indices.size();
  timing::Timer removal_timer("skeleton/prune_edges/removal");

  AlignedList<SkeletonPoint>& non_const_edge_points = skeleton_.getEdgePoints();

  auto iter = non_const_edge_points.begin();
  size_t current_index = 0;
  for (size_t index : removal_indices) {
    while (index > current_index) {
      iter++;
      current_index++;
    }
    iter = non_const_edge_points.erase(iter);
    current_index++;
  }
  return num_removed;
}

size_t SkeletonGenerator::mapNeighborIndexToBitsetIndex(
    size_t neighbor_index) const {
  // This is the mapping between 6-connectivity first, then 18- then 26-
  // to just [0 1 2; 3 4 5; 6 7 8] etc. in 3D.
  switch (neighbor_index) {
    case 24:
      return 0;
    case 12:
      return 1;
    case 20:
      return 2;
    case 15:
      return 3;
    case 4:
      return 4;
    case 14:
      return 5;
    case 22:
      return 6;
    case 10:
      return 7;
    case 18:
      return 8;
    case 9:
      return 9;
    case 3:
      return 10;
    case 7:
      return 11;
    case 1:
      return 12;
    // 13 is skipped since it's the center voxel, and therefore not in the
    // neighbor index.
    case 0:
      return 14;
    case 8:
      return 15;
    case 2:
      return 16;
    case 6:
      return 17;
    case 25:
      return 18;
    case 13:
      return 19;
    case 21:
      return 20;
    case 17:
      return 21;
    case 5:
      return 22;
    case 16:
      return 23;
    case 23:
      return 24;
    case 11:
      return 25;
    case 19:
      return 26;
    default:
      return 13;  // This is the center pixel (not used).
  };
}

void SkeletonGenerator::generateVerticesByLayerNeighbors() {
  timing::Timer generate_timer("skeleton/neighbor_gvd_vertex");

  // Rather than iterate over the entire layer, let's just go over all the
  // points in the skeleton.
  const AlignedList<SkeletonPoint>& edge_points = skeleton_.getEdgePoints();

  // Then figure out how to connect them to other vertices by following the
  // skeleton layer voxels.
  for (const SkeletonPoint& point : edge_points) {
    // Get the voxel.
    BlockIndex block_index =
        skeleton_layer_->computeBlockIndexFromCoordinates(point.point);
    Block<SkeletonVoxel>::Ptr block_ptr;
    block_ptr = skeleton_layer_->getBlockPtrByIndex(block_index);
    CHECK(block_ptr);
    VoxelIndex voxel_index =
        block_ptr->computeVoxelIndexFromCoordinates(point.point);
    SkeletonVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);

    // Now just get the neighbors and count how many are on the skeleton.
    AlignedVector<VoxelKey> neighbors;
    Neighborhood<>::getFromBlockAndVoxelIndex(block_index, voxel_index,
                                              voxels_per_side_, &neighbors);

    int num_neighbors_on_edges = 0;
    // Just go though the 6-connectivity set of this to start.
    for (size_t i = 0; i < neighbors.size(); ++i) {
      // Get this voxel with way too many checks.
      // Get the block for this voxel.
      BlockIndex neighbor_block_index = neighbors[i].first;
      VoxelIndex neighbor_voxel_index = neighbors[i].second;
      Block<SkeletonVoxel>::Ptr neighbor_block;
      if (neighbor_block_index == block_index) {
        neighbor_block = block_ptr;
      } else {
        neighbor_block =
            skeleton_layer_->getBlockPtrByIndex(neighbor_block_index);
      }
      if (!neighbor_block) {
        continue;
      }
      SkeletonVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

      if (neighbor_voxel.is_edge) {
        num_neighbors_on_edges++;
      }
    }
    if ((num_neighbors_on_edges >= 3 || num_neighbors_on_edges == 1) &&
        !voxel.is_vertex) {
      voxel.is_vertex = true;
      voxel.is_edge = true;
      skeleton_.getVertexPoints().push_back(point);
    }
  }
}

void SkeletonGenerator::generateSparseGraph() {
  timing::Timer generate_timer("skeleton/graph");

  graph_.clear();

  // Start with all the vertices.
  // Put them into the graph structure.
  const AlignedVector<SkeletonPoint>& vertex_points =
      skeleton_.getVertexPoints();
  // Store all the IDs we need to iterate over later.
  std::vector<int64_t> vertex_ids;
  vertex_ids.reserve(vertex_points.size());

  for (const SkeletonPoint& point : vertex_points) {
    SkeletonVertex vertex;
    vertex.point = point.point;
    vertex.distance = point.distance;
    int64_t vertex_id = graph_.addVertex(vertex);
    vertex_ids.push_back(vertex_id);

    // Also set the vertex ids in the layer (uuuughhhh)
    Block<SkeletonVoxel>::Ptr block_ptr;
    block_ptr = skeleton_layer_->getBlockPtrByCoordinates(vertex.point);
    CHECK(block_ptr);
    SkeletonVoxel& voxel = block_ptr->getVoxelByCoordinates(vertex.point);
    voxel.vertex_id = vertex_id;
  }

  // The floodfill queue contains the global voxel index and vertex id parent.
  // This allows us to easily do breadth-first search....
  AlignedQueue<std::pair<GlobalIndex, int64_t>> floodfill_queue;
  Neighborhood<>::IndexMatrix neighbors;
  const FloatingPoint grid_size_inv = 1.0 / voxel_size_;
  for (const int64_t vertex_id : vertex_ids) {
    SkeletonVertex& vertex = graph_.getVertex(vertex_id);

    // Get the global index of this voxel.
    GlobalIndex global_index =
        getGridIndexFromPoint<GlobalIndex>(vertex.point, grid_size_inv);

    Neighborhood<>::getFromGlobalIndex(global_index, &neighbors);

    for (int col_idx = 0; col_idx < neighbors.cols(); ++col_idx) {
      floodfill_queue.emplace(neighbors.col(col_idx), vertex_id);
    }
  }

  // Now floodfill to our hearts' desire!
  while (!floodfill_queue.empty()) {
    // Get the next voxel in the queue.
    std::pair<GlobalIndex, int64_t> kv = floodfill_queue.front();
    floodfill_queue.pop();

    int64_t connected_vertex_id = -1;

    SkeletonVoxel* neighbor_voxel =
        skeleton_layer_->getVoxelPtrByGlobalIndex(kv.first);
    if (neighbor_voxel == nullptr) {
      continue;
    }
    // This works because all vertices are also edges.
    if (!neighbor_voxel->is_edge) {
      continue;
    }
    // Two different options. Let's first handle that it's NOT a vertex.
    if (!neighbor_voxel->is_vertex) {
      // If it's an edge, check if it's already checked (i.e., colored by us).
      // There's no way we should have to re-color or merge subgraphs if we
      // are doing flood-fill correctly so only 1 case here.
      if (neighbor_voxel->vertex_id == kv.second) {
        continue;
      }
      if (neighbor_voxel->vertex_id != -1) {
        // There is another vertex here.
        connected_vertex_id = neighbor_voxel->vertex_id;
      } else {
        // Otherwise mark this thing as visited and throw its neighbors into
        // the fray.
        neighbor_voxel->vertex_id = kv.second;
        Neighborhood<>::getFromGlobalIndex(kv.first, &neighbors);
        for (int col_idx = 0; col_idx < neighbors.cols(); ++col_idx) {
          floodfill_queue.emplace(neighbors.col(col_idx), kv.second);
        }
      }
    } else {
      // Now it MUST be a vertex.
      connected_vertex_id = neighbor_voxel->vertex_id;
      if (connected_vertex_id == kv.second) {
        continue;
      }
    }

    if (connected_vertex_id >= 0) {
      SkeletonVertex& connected_vertex = graph_.getVertex(connected_vertex_id);

      // TODO(helenol): some more checks, for instance merge directly adjacent
      // vertices.
      // Before adding an edge, make sure it's not already in there...
      bool already_exists = false;
      for (int64_t edge_id : connected_vertex.edge_list) {
        SkeletonEdge& edge = graph_.getEdge(edge_id);
        if (edge.start_vertex == kv.second || edge.end_vertex == kv.second) {
          already_exists = true;
          break;
        }
      }
      if (already_exists) {
        continue;
      }

      // Ok it's new, let's add this little guy.
      SkeletonEdge edge;
      edge.start_vertex = kv.second;
      edge.end_vertex = connected_vertex_id;
      edge.start_distance = 0.0;
      edge.end_distance = 0.0;
      // Start and end are filled in by the addEdge function.
      int64_t edge_id = graph_.addEdge(edge);
    }
  }

  generate_timer.Stop();

  if (cleanup_style_ == kMatchUnderlyingDiagram) {
    splitEdges();
    repairGraph();
  } else if (cleanup_style_ == kSimplify) {
    simplifyGraph();
  }

  LOG(INFO) << "[Sparse Graph] Vertices: " << graph_.getVertexMap().size()
            << " Edges: " << graph_.getEdgeMap().size();
}

// Checks whether a point is simple, i.e., if its removal would not affect
// the connectivity of its neighbors.
// Uses a SIMILAR numbering to our bitset definitions, except without counting
// 13 (the middle point). So it's a bitset of 26 rather than 27.
// Adapted from Skeletonize3D in ImageJ.
// http://imagejdocu.tudor.lu/doku.php?id=plugin:morphology:skeletonize3d:start
bool SkeletonGenerator::isSimplePoint(const std::bitset<27>& neighbors) const {
  // copy neighbors for labeling
  std::vector<int> cube(26, 0);
  int i;
  for (i = 0; i < 13; i++) {  // i =  0..12 -> cube[0..12]
    cube[i] = neighbors[i];
  }
  // i != 13 : ignore center pixel when counting (see [Lee94])
  for (i = 14; i < 27; i++) {  // i = 14..26 -> cube[13..25]
    cube[i - 1] = neighbors[i];
  }
  // set initial label
  int label = 2;
  // for all points in the neighborhood
  for (i = 0; i < 26; i++) {
    if (cube[i] == 1)  // voxel has not been labeled yet
    {
      // start recursion with any octant that contains the point i
      switch (i) {
        case 0:
        case 1:
        case 3:
        case 4:
        case 9:
        case 10:
        case 12:
          octreeLabeling(1, label, &cube);
          break;
        case 2:
        case 5:
        case 11:
        case 13:
          octreeLabeling(2, label, &cube);
          break;
        case 6:
        case 7:
        case 14:
        case 15:
          octreeLabeling(3, label, &cube);
          break;
        case 8:
        case 16:
          octreeLabeling(4, label, &cube);
          break;
        case 17:
        case 18:
        case 20:
        case 21:
          octreeLabeling(5, label, &cube);
          break;
        case 19:
        case 22:
          octreeLabeling(6, label, &cube);
          break;
        case 23:
        case 24:
          octreeLabeling(7, label, &cube);
          break;
        case 25:
          octreeLabeling(8, label, &cube);
          break;
      }
      label++;
      if (label - 2 >= 2) {
        return false;
      }
    }
  }
  return true;
}

// This is a recursive method that calculates the number of connected
// components in the 3D neighborhood after the center pixel would
// have been removed.
// From Skeletonize3D from ImageJ.
void SkeletonGenerator::octreeLabeling(int octant, int label,
                                       std::vector<int>* cube) const {
  // check if there are points in the octant with value 1
  if (octant == 1) {
    // set points in this octant to current label
    // and recursive labeling of adjacent octants
    if ((*cube)[0] == 1) {
      (*cube)[0] = label;
    }
    if ((*cube)[1] == 1) {
      (*cube)[1] = label;
      octreeLabeling(2, label, cube);
    }
    if ((*cube)[3] == 1) {
      (*cube)[3] = label;
      octreeLabeling(3, label, cube);
    }
    if ((*cube)[4] == 1) {
      (*cube)[4] = label;
      octreeLabeling(2, label, cube);
      octreeLabeling(3, label, cube);
      octreeLabeling(4, label, cube);
    }
    if ((*cube)[9] == 1) {
      (*cube)[9] = label;
      octreeLabeling(5, label, cube);
    }
    if ((*cube)[10] == 1) {
      (*cube)[10] = label;
      octreeLabeling(2, label, cube);
      octreeLabeling(5, label, cube);
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[12] == 1) {
      (*cube)[12] = label;
      octreeLabeling(3, label, cube);
      octreeLabeling(5, label, cube);
      octreeLabeling(7, label, cube);
    }
  }
  if (octant == 2) {
    if ((*cube)[1] == 1) {
      (*cube)[1] = label;
      octreeLabeling(1, label, cube);
    }
    if ((*cube)[4] == 1) {
      (*cube)[4] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(3, label, cube);
      octreeLabeling(4, label, cube);
    }
    if ((*cube)[10] == 1) {
      (*cube)[10] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(5, label, cube);
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[2] == 1) {
      (*cube)[2] = label;
    }
    if ((*cube)[5] == 1) {
      (*cube)[5] = label;
      octreeLabeling(4, label, cube);
    }
    if ((*cube)[11] == 1) {
      (*cube)[11] = label;
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[13] == 1) {
      (*cube)[13] = label;
      octreeLabeling(4, label, cube);
      octreeLabeling(6, label, cube);
      octreeLabeling(8, label, cube);
    }
  }
  if (octant == 3) {
    if ((*cube)[3] == 1) {
      (*cube)[3] = label;
      octreeLabeling(1, label, cube);
    }
    if ((*cube)[4] == 1) {
      (*cube)[4] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(2, label, cube);
      octreeLabeling(4, label, cube);
    }
    if ((*cube)[12] == 1) {
      (*cube)[12] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(5, label, cube);
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[6] == 1) {
      (*cube)[6] = label;
    }
    if ((*cube)[7] == 1) {
      (*cube)[7] = label;
      octreeLabeling(4, label, cube);
    }
    if ((*cube)[14] == 1) {
      (*cube)[14] = label;
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[15] == 1) {
      (*cube)[15] = label;
      octreeLabeling(4, label, cube);
      octreeLabeling(7, label, cube);
      octreeLabeling(8, label, cube);
    }
  }
  if (octant == 4) {
    if ((*cube)[4] == 1) {
      (*cube)[4] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(2, label, cube);
      octreeLabeling(3, label, cube);
    }
    if ((*cube)[5] == 1) {
      (*cube)[5] = label;
      octreeLabeling(2, label, cube);
    }
    if ((*cube)[13] == 1) {
      (*cube)[13] = label;
      octreeLabeling(2, label, cube);
      octreeLabeling(6, label, cube);
      octreeLabeling(8, label, cube);
    }
    if ((*cube)[7] == 1) {
      (*cube)[7] = label;
      octreeLabeling(3, label, cube);
    }
    if ((*cube)[15] == 1) {
      (*cube)[15] = label;
      octreeLabeling(3, label, cube);
      octreeLabeling(7, label, cube);
      octreeLabeling(8, label, cube);
    }
    if ((*cube)[8] == 1) {
      (*cube)[8] = label;
    }
    if ((*cube)[16] == 1) {
      (*cube)[16] = label;
      octreeLabeling(8, label, cube);
    }
  }
  if (octant == 5) {
    if ((*cube)[9] == 1) {
      (*cube)[9] = label;
      octreeLabeling(1, label, cube);
    }
    if ((*cube)[10] == 1) {
      (*cube)[10] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(2, label, cube);
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[12] == 1) {
      (*cube)[12] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(3, label, cube);
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[17] == 1) {
      (*cube)[17] = label;
    }
    if ((*cube)[18] == 1) {
      (*cube)[18] = label;
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[20] == 1) {
      (*cube)[20] = label;
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[21] == 1) {
      (*cube)[21] = label;
      octreeLabeling(6, label, cube);
      octreeLabeling(7, label, cube);
      octreeLabeling(8, label, cube);
    }
  }
  if (octant == 6) {
    if ((*cube)[10] == 1) {
      (*cube)[10] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(2, label, cube);
      octreeLabeling(5, label, cube);
    }
    if ((*cube)[11] == 1) {
      (*cube)[11] = label;
      octreeLabeling(2, label, cube);
    }
    if ((*cube)[13] == 1) {
      (*cube)[13] = label;
      octreeLabeling(2, label, cube);
      octreeLabeling(4, label, cube);
      octreeLabeling(8, label, cube);
    }
    if ((*cube)[18] == 1) {
      (*cube)[18] = label;
      octreeLabeling(5, label, cube);
    }
    if ((*cube)[21] == 1) {
      (*cube)[21] = label;
      octreeLabeling(5, label, cube);
      octreeLabeling(7, label, cube);
      octreeLabeling(8, label, cube);
    }
    if ((*cube)[19] == 1) {
      (*cube)[19] = label;
    }
    if ((*cube)[22] == 1) {
      (*cube)[22] = label;
      octreeLabeling(8, label, cube);
    }
  }
  if (octant == 7) {
    if ((*cube)[12] == 1) {
      (*cube)[12] = label;
      octreeLabeling(1, label, cube);
      octreeLabeling(3, label, cube);
      octreeLabeling(5, label, cube);
    }
    if ((*cube)[14] == 1) {
      (*cube)[14] = label;
      octreeLabeling(3, label, cube);
    }
    if ((*cube)[15] == 1) {
      (*cube)[15] = label;
      octreeLabeling(3, label, cube);
      octreeLabeling(4, label, cube);
      octreeLabeling(8, label, cube);
    }
    if ((*cube)[20] == 1) {
      (*cube)[20] = label;
      octreeLabeling(5, label, cube);
    }
    if ((*cube)[21] == 1) {
      (*cube)[21] = label;
      octreeLabeling(5, label, cube);
      octreeLabeling(6, label, cube);
      octreeLabeling(8, label, cube);
    }
    if ((*cube)[23] == 1) {
      (*cube)[23] = label;
    }
    if ((*cube)[24] == 1) {
      (*cube)[24] = label;
      octreeLabeling(8, label, cube);
    }
  }
  if (octant == 8) {
    if ((*cube)[13] == 1) {
      (*cube)[13] = label;
      octreeLabeling(2, label, cube);
      octreeLabeling(4, label, cube);
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[15] == 1) {
      (*cube)[15] = label;
      octreeLabeling(3, label, cube);
      octreeLabeling(4, label, cube);
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[16] == 1) {
      (*cube)[16] = label;
      octreeLabeling(4, label, cube);
    }
    if ((*cube)[21] == 1) {
      (*cube)[21] = label;
      octreeLabeling(5, label, cube);
      octreeLabeling(6, label, cube);
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[22] == 1) {
      (*cube)[22] = label;
      octreeLabeling(6, label, cube);
    }
    if ((*cube)[24] == 1) {
      (*cube)[24] = label;
      octreeLabeling(7, label, cube);
    }
    if ((*cube)[25] == 1) {
      (*cube)[25] = label;
    }
  }
}

bool SkeletonGenerator::isEndPoint(const std::bitset<27>& neighbors) const {
  if (neighbors.count() == 1) {
    return true;
  }
  // Check first that we have at most 1 6-connected component.
  std::bitset<27> neighbor_mask_6 =
      corner_template_matcher_.get6ConnNeighborMask();
  if ((neighbor_mask_6 & neighbors).count() > 1) {
    return false;
  }
  if (corner_template_matcher_.fitsTemplates(neighbors)) {
    return true;
  }
  return false;
}

void SkeletonGenerator::pruneDiagramVertices() {
  timing::Timer generate_timer("skeleton/prune_vertices");

  // Ok, first set up a kdtree/nanoflann instance using the skeleton point
  // wrapper.
  // We want it dynamic because we are gonna be dropping hella vertices.
  // Or do we need to?
  const int kDim = 3;
  const int kMaxLeaf = 10;

  const AlignedVector<SkeletonPoint>& const_vertices =
      skeleton_.getVertexPoints();
  const size_t num_vertices = const_vertices.size();

  // Create the adapter.
  SkeletonPointVectorAdapter adapter(const_vertices);
  // construct a kd-tree index:
  SkeletonPointKdTree kd_tree(
      kDim, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));

  // This would be buildIndex if we were doing the non-dynamic version...
  // kd_tree.addPoints(0, adapter.kdtree_get_point_count());
  kd_tree.buildIndex();

  // Keep a set (for fast look-ups) of all the indices that are slated for
  // deletion. Will delete afterwards, so as to not mess up the data
  // structure.
  std::set<size_t> deletion_index;

  // Go through all the vertices, check which ones have neighbors within a
  // a radius, and cut all but the longest distance ones.
  for (size_t i = 0; i < num_vertices; ++i) {
    if (deletion_index.count(i) > 0) {
      // Already deleted! Nothing to do.
      continue;
    }
    // Pair from index and distance.
    std::vector<std::pair<size_t, FloatingPoint>> returned_matches;
    nanoflann::SearchParams params;  // Defaults are fine.

    size_t num_matches =
        kd_tree.radiusSearch(const_vertices[i].point.data(),
                             vertex_pruning_radius_ * vertex_pruning_radius_,
                             returned_matches, params);

    // Go through all the matches... Figure out if we actually need to delete
    // something. Keep track of our favorite to keep.
    size_t num_valid_matches = 0;
    float largest_vertex_distance = const_vertices[i].distance;
    size_t favorite_vertex_index = i;
    for (const std::pair<size_t, FloatingPoint>& match : returned_matches) {
      if (match.first == i) {
        continue;
      }
      if (deletion_index.count(match.first) > 0) {
        continue;
      }
      // Ok I guess now it's actually a valid match.
      num_valid_matches++;
      if (const_vertices[match.first].distance > largest_vertex_distance) {
        // Then we should delete the current favorite, which starts out
        // as ourselves. :(
        deletion_index.insert(favorite_vertex_index);
        largest_vertex_distance = const_vertices[match.first].distance;
        favorite_vertex_index = match.first;
      } else {
        // Current favorite is still better, delete the match.
        deletion_index.insert(match.first);
      }
    }
  }

  LOG(INFO) << "[Prune] Number of vertices before prune: " << num_vertices
            << " Number of deleted vertices: " << deletion_index.size();

  // Go through everything in the deletion index and remove it from the list.
  // As always, have to go backwards to preserve voxel indices.

  AlignedVector<SkeletonPoint>& non_const_vertices =
      skeleton_.getVertexPoints();

  for (std::set<size_t>::reverse_iterator rit = deletion_index.rbegin();
       rit != deletion_index.rend(); ++rit) {
    size_t index_to_delete = *rit;

    // Make a copy! NOT a reference, as we're gonna delete it.
    SkeletonPoint point = const_vertices[index_to_delete];

    non_const_vertices.erase(non_const_vertices.begin() + index_to_delete);

    // Get the voxel.
    BlockIndex block_index =
        skeleton_layer_->computeBlockIndexFromCoordinates(point.point);
    Block<SkeletonVoxel>::Ptr block_ptr;
    block_ptr = skeleton_layer_->getBlockPtrByIndex(block_index);
    CHECK(block_ptr);
    VoxelIndex voxel_index =
        block_ptr->computeVoxelIndexFromCoordinates(point.point);
    SkeletonVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
    voxel.is_vertex = false;
    voxel.is_edge = true;
  }
}

void SkeletonGenerator::splitEdges() {
  timing::Timer split_timer("skeleton/split_edges");

  std::vector<int64_t> edge_ids;
  graph_.getAllEdgeIds(&edge_ids);

  splitSpecificEdges(edge_ids);
}

void SkeletonGenerator::splitSpecificEdges(
    const std::vector<int64_t>& starting_edge_ids) {
  std::vector<int64_t> edge_ids = starting_edge_ids;

  // This is a number from a butt.
  const FloatingPoint kMaxThreshold = 2 * voxel_size_;
  const FloatingPoint kVertexSearchRadus =
      vertex_pruning_radius_ * vertex_pruning_radius_;
  const int kMaxAstarIterations = 500;
  skeleton_planner_.setMaxIterations(kMaxAstarIterations);

  timing::Timer kdtree_timer("skeleton/split_edges/kdtree");
  // Build the kD Tree of the vertices at the current moment.
  // Create the adapter.
  DirectSkeletonVertexMapAdapter adapter(graph_.getVertexMap());

  // construct a kd-tree index:
  const int kDim = 3;
  const int kMaxLeaf = 10;

  DynamicVertexGraphKdTree kd_tree(
      kDim, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));
  kd_tree.addPoints(0, adapter.kdtree_get_point_count() - 1);

  kdtree_timer.Stop();

  size_t num_vertices_added = 0;

  // Need to use a regular (not range-based) for loop since we'll add a bunch
  // of new edges to the end.
  for (size_t i = 0; i < edge_ids.size(); ++i) {
    int64_t edge_id = edge_ids[i];
    if (!graph_.hasEdge(edge_id)) {
      // This edge already got deleted.
      continue;
    }
    // Get the start and end points.
    SkeletonEdge& edge = graph_.getEdge(edge_id);
    const Point& start = edge.start_point;
    const Point& end = edge.end_point;

    // Get the shortest path through the graph.
    size_t max_d_ind = 0;
    AlignedVector<Point> coordinate_path;
    FloatingPoint max_d = getMaxEdgeDistanceFromStraightLine(
        start, end, &coordinate_path, &max_d_ind);

    if (max_d <= -1.0) {
      graph_.removeEdge(edge_id);
    }

    // Impose some minimum distance...
    if (max_d > kMaxThreshold) {
      // Pick the point with the max deviation to insert a new voxel there.
      SkeletonVertex new_vertex;
      new_vertex.point = coordinate_path[max_d_ind];

      // Ok, let's check if there's something already close enough...
      // I guess it would be best if this was a dynamic k-D tree, since we're
      // adding/subtracting vertices at will.
      // Also not necessarily indexed in order.
      // Pair from index and distance.
      std::vector<std::pair<size_t, FloatingPoint>> returned_matches;
      nanoflann::SearchParams params;  // Defaults are fine.
      size_t num_results = 1;
      nanoflann::KNNResultSet<FloatingPoint> result_set(num_results);
      FloatingPoint squared_distance;
      size_t ret_index = 0;
      result_set.init(&ret_index, &squared_distance);

      kd_tree.findNeighbors(result_set, new_vertex.point.data(), params);

      if (squared_distance < kVertexSearchRadus) {
        auto iter = graph_.getVertexMap().find(ret_index);
        if (iter != graph_.getVertexMap().end()) {
          const SkeletonVertex& vertex_candidate = iter->second;

          if (vertex_candidate.vertex_id != edge.start_vertex &&
              vertex_candidate.vertex_id != edge.end_vertex) {
            if (graph_.areVerticesDirectlyConnected(vertex_candidate.vertex_id,
                                                    edge.start_vertex) &&
                graph_.areVerticesDirectlyConnected(vertex_candidate.vertex_id,
                                                    edge.end_vertex)) {
            } else {
              // Try to find a connection from start vertex -> this and then
              // from this to end vertex.
              AlignedVector<Point> start_path, end_path;

              timing::Timer path_timer("skeleton/split_edges/search_path");
              bool success_start = skeleton_planner_.getPathOnDiagram(
                  start, vertex_candidate.point, &start_path);
              bool success_end = skeleton_planner_.getPathOnDiagram(
                  vertex_candidate.point, end, &end_path);
              path_timer.Stop();

              if (success_start && success_end) {
                size_t max_start_index = 0, max_end_index = 0;
                FloatingPoint max_d_start =
                    getMaxEdgeDistanceOnPath(start, vertex_candidate.point,
                                             start_path, &max_start_index);
                FloatingPoint max_d_end = getMaxEdgeDistanceOnPath(
                    vertex_candidate.point, end, end_path, &max_end_index);

                if (max_d_start < max_d && max_d_end < max_d) {
                  // Only connect to this if it ACTUALLY lowers the costs!

                  // Remove the existing edge, add two new edges.
                  SkeletonEdge new_edge_1, new_edge_2;
                  new_edge_1.start_vertex = edge.start_vertex;
                  new_edge_1.end_vertex = vertex_candidate.vertex_id;
                  new_edge_2.start_vertex = vertex_candidate.vertex_id;
                  new_edge_2.end_vertex = edge.end_vertex;

                  int64_t edge_id_1 = graph_.addEdge(new_edge_1);
                  int64_t edge_id_2 = graph_.addEdge(new_edge_2);

                  // Make sure two new edges are going to be iteratively checked
                  // again.
                  edge_ids.push_back(edge_id_1);
                  edge_ids.push_back(edge_id_2);

                  graph_.removeEdge(edge_id);
                  continue;
                }
              }
            }
          }
        }
      }

      // Get this point from the diagram.
      // Check if it's already a vertex!
      SkeletonVoxel* voxel =
          skeleton_layer_->getVoxelPtrByCoordinates(new_vertex.point);
      if (voxel != nullptr) {
        if (voxel == nullptr || (voxel->is_vertex && voxel->vertex_id > -1)) {
          continue;
        }
      }

      int64_t vertex_id = graph_.addVertex(new_vertex);
      num_vertices_added++;

      // Get the point from the diagram and label it.
      voxel->is_vertex = true;
      voxel->vertex_id = vertex_id;

      // Remove the existing edge, add two new edges.
      SkeletonEdge new_edge_1, new_edge_2;
      new_edge_1.start_vertex = edge.start_vertex;
      new_edge_1.end_vertex = vertex_id;
      new_edge_2.start_vertex = vertex_id;
      new_edge_2.end_vertex = edge.end_vertex;

      int64_t edge_id_1 = graph_.addEdge(new_edge_1);
      int64_t edge_id_2 = graph_.addEdge(new_edge_2);
      // Make sure two new edges are going to be iteratively checked again.

      edge_ids.push_back(edge_id_1);
      edge_ids.push_back(edge_id_2);

      graph_.removeEdge(edge_id);
      if (num_vertices_added % 1 == 0) {
        timing::Timer kdtree_add_timer("skeleton/split_edges/kdtree_add");
        kd_tree.addPoints(adapter.kdtree_get_point_count() - 1,
                          adapter.kdtree_get_point_count() - 1);
        kdtree_add_timer.Stop();
      }
    }
  }

  skeleton_planner_.setMaxIterations(0);
  LOG(INFO) << "[Split Edges] Num vertices added: " << num_vertices_added;
}

FloatingPoint SkeletonGenerator::getMaxEdgeDistanceFromStraightLine(
    const Point& start, const Point& end, AlignedVector<Point>* coordinate_path,
    size_t* max_index) {
  timing::Timer split_timer("skeleton/split_edges/get_max_dist");

  CHECK_NOTNULL(coordinate_path);
  CHECK_NOTNULL(max_index);
  *max_index = 0;
  bool success =
      skeleton_planner_.getPathOnDiagram(start, end, coordinate_path);

  if (!success) {
    LOG(INFO) << "Something is wrong!";
    return -1.0;
  }

  FloatingPoint max_d =
      getMaxEdgeDistanceOnPath(start, end, *coordinate_path, max_index);
  return max_d;
}

FloatingPoint SkeletonGenerator::getMaxEdgeDistanceOnPath(
    const Point& start, const Point& end,
    const AlignedVector<Point>& coordinate_path, size_t* max_index) {
  // Ok now figure out what the straight-line path between the two points
  // would be.
  FloatingPoint max_d = 0.0;
  for (size_t j = 0; j < coordinate_path.size(); ++j) {
    // Project every point in the coordinate path onto this.
    // From http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    const Point& point = (coordinate_path)[j];
    FloatingPoint d =
        ((end - start).cross(start - point)).norm() / (end - start).norm();

    if (d > max_d) {
      max_d = d;
      *max_index = j;
    }
  }
  return max_d;
}

void SkeletonGenerator::setSkeletonLayer(Layer<SkeletonVoxel>* skeleton_layer) {
  skeleton_layer_.reset(skeleton_layer);
  skeleton_planner_.setSkeletonLayer(skeleton_layer_.get());
}

void SkeletonGenerator::repairGraph() {
  timing::Timer repair_timer("skeleton/repair_graph");

  // Go over all the vertices in the sparse graph and flood fill to label
  // unconnected components.
  std::vector<int64_t> vertex_ids;
  graph_.getAllVertexIds(&vertex_ids);
  std::map<int, int64_t> subgraph_vertex_examples;
  std::vector<int64_t> new_edge_ids;

  int last_subgraph = 0;
  for (const int64_t vertex_id : vertex_ids) {
    SkeletonVertex& vertex = graph_.getVertex(vertex_id);
    if (vertex.subgraph_id > 0) {
      // This vertex is already labelled.
      continue;
    }
    int subgraph_id = ++last_subgraph;
    int num_labelled = recursivelyLabel(vertex_id, subgraph_id);

    if (num_labelled == 1) {
      graph_.removeVertex(vertex_id);
    } else {
      subgraph_vertex_examples[subgraph_id] = vertex_id;
    }
  }

  // Ok now we presumably have more than 1 disconnected subgraph...
  if (subgraph_vertex_examples.size() <= 1) {
    // Nope only one subgraph, we're done!
    return;
  }

  LOG(INFO) << "[Subgraph] Number of disconnected subgraphs: "
            << subgraph_vertex_examples.size();

  // Go through all combinations of subgraphs until we're connected.
  for (const std::pair<int, int64_t>& subgraph1 : subgraph_vertex_examples) {
    const SkeletonVertex& vertex1 = graph_.getVertex(subgraph1.second);
    if (vertex1.subgraph_id != subgraph1.first) {
      // This already got absorbed into another subgraph.
      continue;
    }

    for (const std::pair<int, int64_t>& subgraph2 : subgraph_vertex_examples) {
      if (subgraph1.first == subgraph2.first) {
        continue;
      }
      const SkeletonVertex& vertex2 = graph_.getVertex(subgraph2.second);
      if (vertex2.subgraph_id != subgraph2.first) {
        // This already got absorbed into another subgraph.
        continue;
      }

      // Ok now presumably we have two different subgraphs that we're gonna try
      // to connect.
      AlignedVector<Point> coordinate_path;
      bool success = skeleton_planner_.getPathOnDiagram(
          vertex1.point, vertex2.point, &coordinate_path);
      if (success) {
        tryToFindEdgesInCoordinatePath(coordinate_path, subgraph1.first,
                                       subgraph2.first, &new_edge_ids);
      }
    }
  }
  LOG(INFO) << "[Subgraph] Trying to check if we need to split "
            << new_edge_ids.size() << " new edges.";

  timing::Timer split_timer("skeleton/repair_graph/split");
  splitSpecificEdges(new_edge_ids);
  split_timer.Stop();

  // Check how many subgraphs we finally have.
  int num_subgraphs = 0;
  graph_.getAllVertexIds(&vertex_ids);
  std::set<int> unique_subgraphs;

  for (const int64_t vertex_id : vertex_ids) {
    SkeletonVertex& vertex = graph_.getVertex(vertex_id);
    if (vertex.subgraph_id > 0) {
      unique_subgraphs.insert(vertex.subgraph_id);
    }
  }

  LOG(INFO) << "[Subgraph] Final number of disconnected subgraphs: "
            << unique_subgraphs.size();
}

int SkeletonGenerator::recursivelyLabel(int64_t vertex_id, int subgraph_id) {
  int num_labelled = 1;
  SkeletonVertex& vertex = graph_.getVertex(vertex_id);
  if (vertex.subgraph_id == subgraph_id) {
    return 0;
  }
  vertex.subgraph_id = subgraph_id;
  for (int64_t edge_id : vertex.edge_list) {
    const SkeletonEdge& edge = graph_.getEdge(edge_id);
    int64_t neighbor_vertex_id = -1;
    if (edge.start_vertex == vertex_id) {
      neighbor_vertex_id = edge.end_vertex;
    } else {
      neighbor_vertex_id = edge.start_vertex;
    }
    num_labelled += recursivelyLabel(neighbor_vertex_id, subgraph_id);
  }
  return num_labelled;
}

void SkeletonGenerator::tryToFindEdgesInCoordinatePath(
    const AlignedVector<Point>& coordinate_path, int subgraph_id_start,
    int subgraph_id_end, std::vector<int64_t>* new_edge_ids) {
  CHECK_NOTNULL(new_edge_ids);
  // Just appends to new edge Ids, doesn't clear 'em.
  // Go through the coordinate path, looking up vertices in the sparse graph
  // any time that their IDs are set.
  // When two graph-adjacent vertices have a different subgraph index, add
  // an edge between them.
  int last_subgraph_id = subgraph_id_start;
  int64_t last_vertex_id = -1;

  if (coordinate_path.empty()) {
    return;
  }

  for (const Point& coord : coordinate_path) {
    // Look up the voxel in the map.
    SkeletonVoxel* voxel = skeleton_layer_->getVoxelPtrByCoordinates(coord);
    if (voxel == nullptr) {
      continue;
    }
    if (voxel->is_vertex && voxel->vertex_id >= 0) {
      int64_t vertex_id = voxel->vertex_id;
      // Look up this vertex.
      if (graph_.hasVertex(vertex_id)) {
        SkeletonVertex& vertex = graph_.getVertex(vertex_id);
        if (vertex.subgraph_id > 0) {
          // If this is the same subgraph ID as the previous one, then just
          // store it as the latest.
          if (vertex.subgraph_id == last_subgraph_id) {
            last_vertex_id = vertex_id;
          } else if (last_vertex_id != -1) {
            // Ok this is a different subgraph! Let's put in an edge.
            SkeletonEdge new_edge;
            new_edge.start_vertex = last_vertex_id;
            new_edge.end_vertex = vertex_id;
            int64_t edge_id = graph_.addEdge(new_edge);
            new_edge_ids->push_back(edge_id);
            // Probably the right time to recursively label stuff too.
            recursivelyLabel(vertex_id, last_subgraph_id);
          } else {
            // I'm very confused.
          }

          last_vertex_id = vertex_id;
          // last_subgraph_id = vertex.subgraph_id;
          if (last_subgraph_id == subgraph_id_end) {
            // Ok we're done.
            return;
          }
        }
      }
    }
  }
}

void SkeletonGenerator::simplifyGraph() {
  // This will have to do a couple of things.
  // (1) Verify each edge along the ESDF does not reach a distance below min
  // gvd.
  // (2) For vertices with only 2 neighbors, try to skip the vertex along the
  // straight line.
  // (3) For vertices that are end-points, try to reconnect them to something
  // nice.
  // (4) Label subgraphs, and if there's really near-by vertices with a
  // different subgraph, try to connect along the ESDF.

  // Make sure we don't waste time looking too far.
  const int kMaxAstarIterations = 100;
  skeleton_planner_.setMaxIterations(kMaxAstarIterations);

  simplifyVertices();

  reconnectSubgraphsAlongEsdf();
}

void SkeletonGenerator::simplifyVertices() {
  timing::Timer simplify_timer("skeleton/simplify_vertices");

  // Build a kd tree again.
  constexpr int kMaxLeaf = 10;
  DirectSkeletonVertexMapAdapter adapter(graph_.getVertexMap());
  // construct a kd-tree index:
  VertexGraphKdTree kd_tree(
      3, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));
  kd_tree.buildIndex();

  constexpr int kNumNeighbors = 10;
  nanoflann::SearchParams params;  // Defaults are fine.
  std::vector<size_t> ret_index(kNumNeighbors);
  std::vector<FloatingPoint> out_dist_sqr(kNumNeighbors);

  std::vector<int64_t> vertex_ids;
  graph_.getAllVertexIds(&vertex_ids);

  size_t vertex_removal_candidates = 0;
  size_t vertices_removed = 0;
  size_t edges_added = 0;

  // We're a bit more generous here.
  const FloatingPoint kMaxThreshold = 2 * voxel_size_;

  for (const int64_t vertex_id : vertex_ids) {
    SkeletonVertex& vertex = graph_.getVertex(vertex_id);
    if (vertex.edge_list.size() == 1) {
      // Find it a friend! Find only friends with only 1 neighbor.
      // kD tree lookup here.
      size_t num_results = kd_tree.knnSearch(vertex.point.data(), kNumNeighbors,
                                             &ret_index[0], &out_dist_sqr[0]);
      for (size_t i = 0; i < num_results; i++) {
        const SkeletonVertex& neighbor_vertex = graph_.getVertex(ret_index[i]);
        if (neighbor_vertex.edge_list.size() == 1) {
          timing::Timer path_timer("skeleton/simplify_vertices/path_finding");

          // We don't want stuff that's easily linked in the diagram, or we
          // would have already gotten this.
          AlignedVector<Point> coordinate_path;
          bool success = skeleton_planner_.getPathOnDiagram(
              vertex.point, neighbor_vertex.point, &coordinate_path);
          if (success) {
            continue;
          }

          coordinate_path.clear();
          success = skeleton_planner_.getPathInEsdf(
              vertex.point, neighbor_vertex.point, &coordinate_path);

          if (success) {
            size_t max_index;
            FloatingPoint max_d =
                getMaxEdgeDistanceOnPath(vertex.point, neighbor_vertex.point,
                                         coordinate_path, &max_index);
            if (max_d > kMaxThreshold) {
              continue;
            }

            SkeletonEdge new_edge;
            new_edge.start_vertex = vertex_id;
            new_edge.end_vertex = neighbor_vertex.vertex_id;
            int64_t edge_id = graph_.addEdge(new_edge);

            edges_added++;
            break;
          }
        }
      }
    }
  }

  for (const int64_t vertex_id : vertex_ids) {
    const SkeletonVertex& vertex = graph_.getVertex(vertex_id);
    if (vertex.edge_list.size() == 2) {
      vertex_removal_candidates++;

      // Try to see if we can cut this!
      const SkeletonEdge& edge1 = graph_.getEdge(vertex.edge_list[0]);
      const SkeletonEdge& edge2 = graph_.getEdge(vertex.edge_list[1]);

      int64_t vertex_id1 = edge1.start_vertex;
      if (vertex_id1 == vertex_id) {
        vertex_id1 = edge1.end_vertex;
      }
      int64_t vertex_id2 = edge2.start_vertex;
      if (vertex_id2 == vertex_id) {
        vertex_id2 = edge2.end_vertex;
      }

      const SkeletonVertex& vertex1 = graph_.getVertex(vertex_id1);
      const SkeletonVertex& vertex2 = graph_.getVertex(vertex_id2);

      AlignedVector<Point> coordinate_path;
      bool success = skeleton_planner_.getPathInEsdf(
          vertex1.point, vertex2.point, &coordinate_path);
      if (!success) {
        continue;
      }
      size_t max_index;
      FloatingPoint max_d = getMaxEdgeDistanceOnPath(
          vertex1.point, vertex2.point, coordinate_path, &max_index);

      if (max_d <= kMaxThreshold) {
        SkeletonEdge new_edge;
        new_edge.start_vertex = vertex_id1;
        new_edge.end_vertex = vertex_id2;
        int64_t edge_id = graph_.addEdge(new_edge);
        graph_.removeVertex(vertex_id);
        vertices_removed++;
      }
    }
  }
  LOG(INFO) << "[Simplify Vertices] Vertex removals: " << vertices_removed
            << " / " << vertex_removal_candidates
            << ", Edges added: " << edges_added;
}

void SkeletonGenerator::reconnectSubgraphsAlongEsdf() {
  timing::Timer reconnect_timer("skeleton/reconnect");

  // Subgraph merging is done differently here... Just accumlate all the ones
  // that map to the same thing. Always map to the lowest.
  std::map<int, int> subgraph_map;

  // Go over all the vertices in the sparse graph and flood fill to label
  // unconnected components.
  std::vector<int64_t> vertex_ids;
  graph_.getAllVertexIds(&vertex_ids);
  std::map<int, int64_t> subgraph_vertex_examples;

  int last_subgraph = 0;
  for (const int64_t vertex_id : vertex_ids) {
    SkeletonVertex& vertex = graph_.getVertex(vertex_id);
    if (vertex.subgraph_id > 0) {
      // This vertex is already labelled.
      continue;
    }
    int subgraph_id = ++last_subgraph;
    int num_labelled = recursivelyLabel(vertex_id, subgraph_id);

    if (num_labelled == 1) {
      graph_.removeVertex(vertex_id);
    } else {
      subgraph_vertex_examples[subgraph_id] = vertex_id;
      subgraph_map[subgraph_id] = subgraph_id;
    }
  }

  // Ok now we presumably have more than 1 disconnected subgraph...
  if (subgraph_vertex_examples.size() <= 1) {
    // Nope only one subgraph, we're done!
    return;
  }

  LOG(INFO) << "[Subgraph] Number of disconnected subgraphs: "
            << subgraph_vertex_examples.size();

  size_t potential_edge_candidates = 0;
  size_t diagram_candidates = 0;

  // Next, build a KD Tree of all the vertices.
  // Create the adapter.
  constexpr int kMaxLeaf = 10;
  DirectSkeletonVertexMapAdapter adapter(graph_.getVertexMap());
  // construct a kd-tree index:
  VertexGraphKdTree kd_tree(
      3, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));
  kd_tree.buildIndex();

  constexpr int kNumNeighbors = 5;
  nanoflann::SearchParams params;  // Defaults are fine.
  std::vector<size_t> ret_index(kNumNeighbors);
  std::vector<FloatingPoint> out_dist_sqr(kNumNeighbors);

  // Look up the nearest kNumNeighbors in the kD tree for each vertex.
  for (const int64_t vertex_id : vertex_ids) {
    if (!graph_.hasVertex(vertex_id)) {
      continue;
    }
    const SkeletonVertex& vertex = graph_.getVertex(vertex_id);

    // kD tree lookup here.
    size_t num_results = kd_tree.knnSearch(vertex.point.data(), kNumNeighbors,
                                           &ret_index[0], &out_dist_sqr[0]);
    for (size_t i = 0; i < num_results; i++) {
      CHECK(graph_.hasVertex(ret_index[i]));
      const SkeletonVertex& neighbor_vertex = graph_.getVertex(ret_index[i]);
      if (subgraph_map[neighbor_vertex.subgraph_id] ==
          subgraph_map[vertex.subgraph_id]) {
        continue;
      }
      potential_edge_candidates++;

      // Ok now presumably we have two different subgraphs that we're gonna
      // try to connect.
      timing::Timer path_timer("skeleton/reconnect/path_finding");

      AlignedVector<Point> coordinate_path;
      bool success = skeleton_planner_.getPathInEsdf(
          vertex.point, neighbor_vertex.point, &coordinate_path);
      if (success) {
        diagram_candidates++;
        SkeletonEdge new_edge;
        new_edge.start_vertex = vertex_id;
        new_edge.end_vertex = neighbor_vertex.vertex_id;
        int64_t edge_id = graph_.addEdge(new_edge);

        mergeSubgraphs(vertex.subgraph_id, neighbor_vertex.subgraph_id,
                       &subgraph_map);
      }
      path_timer.Stop();
    }
  }
  LOG(INFO) << "[Subgraph] Potential edge candidates: "
            << potential_edge_candidates
            << " diagram candidates: " << diagram_candidates;

  // Recolor!
  timing::Timer recolor_timer("skeleton/reconnect/recolor");
  std::set<int> unique_subgraphs;

  for (const int64_t vertex_id : vertex_ids) {
    SkeletonVertex& vertex = graph_.getVertex(vertex_id);
    vertex.subgraph_id = subgraph_map[vertex.subgraph_id];
    unique_subgraphs.insert(vertex.subgraph_id);
  }
  recolor_timer.Stop();

  LOG(INFO) << "[Subgraph] Final number of disconnected subgraphs: "
            << unique_subgraphs.size();
}

void SkeletonGenerator::mergeSubgraphs(int subgraph_1, int subgraph_2,
                                       std::map<int, int>* subgraph_map) const {
  // Figure out the lowest thing that these subgraphs actually map to.
  int subgraph_1a = (*subgraph_map)[subgraph_1];
  int subgraph_2a = (*subgraph_map)[subgraph_2];

  int new_subgraph = std::min(subgraph_1a, subgraph_2a);
  int old_subgraph = std::max(subgraph_1a, subgraph_2a);

  for (std::pair<const int, int>& kv : (*subgraph_map)) {
    if (kv.second == old_subgraph) {
      kv.second = new_subgraph;
    }
  }
}

bool SkeletonGenerator::loadSparseGraphFromFile(const std::string& filename) {
  return io::loadSparseSkeletonGraphFromFile(filename, &graph_);
}

bool SkeletonGenerator::saveSparseGraphToFile(const std::string& filename) {
  return io::saveSparseSkeletonGraphToFile(filename, graph_);
}

}  // namespace voxblox
