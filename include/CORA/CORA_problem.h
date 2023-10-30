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

namespace CORA {
class Problem {
private:
  size_t num_poses_;
  size_t num_landmarks_;
  size_t num_range_measurements_;
  const size_t dim_;
  size_t relaxation_rank_;

public:
  Problem(size_t dim, size_t relaxation_rank)
      : num_poses_(0),
        num_landmarks_(0),
        num_range_measurements_(0),
        dim_(dim),
        relaxation_rank_(relaxation_rank) {
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
}; // class Problem

} // namespace CORA
