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
  // TODO(alanpapalia): implement this function
  throw NotImplementedException();
}

void Problem::addLandmarkVariable(Symbol landmark_id) {
  // TODO(alanpapalia): implement this function
  throw NotImplementedException();
}

void Problem::addRangeMeasurement(RangeMeasurement range_measurement) {
  // TODO(alanpapalia): implement this function
  throw NotImplementedException();
}

void Problem::addRelativePoseMeasurement(
    RelativePoseMeasurement rel_pose_measure) {
  // TODO(alanpapalia): implement this function
  throw NotImplementedException();
}

void Problem::addPosePrior(PosePrior pose_prior) {
  // TODO(alanpapalia): implement this function
  throw NotImplementedException();
}

void Problem::addLandmarkPrior(LandmarkPrior landmark_prior) {
  // TODO(alanpapalia): implement this function
  throw NotImplementedException();
}

}; // namespace CORA
