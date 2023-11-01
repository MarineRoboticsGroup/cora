/**
 * @file CORA_problem.cpp
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <CORA/CORA_problem.h>

namespace CORA {
void Problem::addPoseVariable(Symbol pose_id) {
  pose_symbol_idxs_.insert(std::make_pair(pose_id, pose_symbol_idxs_.size()));
}

void Problem::addLandmarkVariable(Symbol landmark_id) {
  landmark_symbol_idxs_.insert(
      std::make_pair(landmark_id, landmark_symbol_idxs_.size()));
}

void Problem::addRangeMeasurement(RangeMeasurement range_measurement) {
  range_measurement_symbol_idxs_.insert(
      std::make_pair(range_measurement.getSymbolPair(),
                     range_measurement_symbol_idxs_.size()));
  range_measurements_.push_back(range_measurement);
}

void Problem::addRelativePoseMeasurement(
    RelativePoseMeasurement rel_pose_measure) {
  rel_pose_measurements_.push_back(rel_pose_measure);
}

void Problem::addPosePrior(PosePrior pose_prior) {
  pose_priors_.push_back(pose_prior);
}

void Problem::addLandmarkPrior(LandmarkPrior landmark_prior) {
  landmark_priors_.push_back(landmark_prior);
}

}; // namespace CORA
