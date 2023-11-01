/**
 * @file CORA_problem.h
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <CORA/CORA_types.h>
#include <CORA/Measurements.h>
#include <CORA/Symbol.h>
#include <map>
#include <utility>
#include <vector>

namespace CORA {
class Problem {
private:
  /** dimension of the pose and landmark variables e.g., SO(dim_) */
  const size_t dim_;

  /** rank of the relaxation e.g., the latent embedding space of Stiefel
   * manifold */
  size_t relaxation_rank_;

  // maps from pose symbol to pose index (e.g., x1 -> 0, x2 -> 1, etc.)
  std::map<Symbol, int> pose_symbol_idxs_;

  // maps from landmark symbol to landmark index (e.g., l1 -> 0, l2 -> 1, etc.)
  std::map<Symbol, int> landmark_symbol_idxs_;

  /** maps from range measurement symbol pair to range measurement index
   * (e.g., r1 -> 0, r2 -> 1, etc.) */
  std::map<std::pair<Symbol, Symbol>, int> range_measurement_symbol_idxs_;

  // the range measurements that are used to construct the problem
  std::vector<RangeMeasurement> range_measurements_;

  // the relative pose measurements that are used to construct the problem
  std::vector<RelativePoseMeasurement> rel_pose_measurements_;

  // the pose priors that are used to construct the problem
  std::vector<PosePrior> pose_priors_;

  // the landmark priors that are used to construct the problem
  std::vector<LandmarkPrior> landmark_priors_;

public:
  Problem(size_t dim, size_t relaxation_rank)
      : dim_(dim), relaxation_rank_(relaxation_rank) {
    // relaxation rank must be >= dim
    assert(relaxation_rank >= dim);
  }

  ~Problem() {}

  void addPoseVariable(Symbol pose_id);
  void addLandmarkVariable(Symbol landmark_id);
  void addRangeMeasurement(RangeMeasurement range_measurement);
  void addRelativePoseMeasurement(RelativePoseMeasurement rel_pose_measure);
  void addPosePrior(PosePrior pose_prior);
  void addLandmarkPrior(LandmarkPrior landmark_prior);

  size_t numPoses() const { return pose_symbol_idxs_.size(); }
  size_t numLandmarks() const { return landmark_symbol_idxs_.size(); }
  size_t numRangeMeasurements() const {
    return range_measurement_symbol_idxs_.size();
  }
}; // class Problem

} // namespace CORA
