//
// Created by tim on 11/12/23.
//

#include "CORA/CORA_vis.h"
#include "CORA/CORA_problem.h"
namespace CORA {

CORAVis::CORAVis() = default;
using TNTStatus = Optimization::Riemannian::TNTStatus;
/**
 * @brief Visualize the result trajectory using TonioViz
 *
 * @description This function takes the result of the CORA algorithm and plots
 * the trajectory, landmark location, and the ranges
 * @param result The result of the CORA algorithm
 */

void CORAVis::visualize(const Problem &problem, const CoraTntResult &result) {
  auto aligned_sol_matrix = problem.alignEstimateToOrigin(result.x);
  auto status = result.status;

  switch (status) {
  case TNTStatus::Gradient:
    std::cout << "Solution terminated due to gradient" << std::endl;
    break;
  case TNTStatus::IterationLimit:
    std::cout << "Solution terminated due to iteration limit" << std::endl;
    break;
  case TNTStatus::ElapsedTime:
    std::cout << "Solution terminated due to elapsed time" << std::endl;
    break;
  case TNTStatus::PreconditionedGradient:
    std::cout << "Solution terminated due to preconditioned gradient"
              << std::endl;
    break;
  case TNTStatus::RelativeDecrease:
    std::cout << "Solution terminated due to relative decrease" << std::endl;
    break;
  case TNTStatus::Stepsize:
    std::cout << "Solution terminated due to step size" << std::endl;
    break;
  case TNTStatus::TrustRegion:
    std::cout << "Solution terminated due to trust region" << std::endl;
    break;
  case TNTStatus::UserFunction:
    std::cout << "Solution terminated due to user function" << std::endl;
    break;
  default:
    std::cout << "Solution terminated due to unknown reason" << std::endl;
    break;
  }

  auto pose_sym_to_idx = problem.getPoseSymbolMap();
  auto landmark_sym_to_idx = problem.getLandmarkSymbolMap();
  auto range_measurements = problem.getRangeMeasurements();
  auto dim = problem.dim();
  // Iterate through all poses and draw using TonioViz
  for (auto [pose_sym, pose_idx] : pose_sym_to_idx) {
    auto rotation = aligned_sol_matrix.block(pose_idx, 0, dim, dim);
    auto translation =
        aligned_sol_matrix.block(problem.numPosesDim() + pose_idx, 0, 1, dim);

    // Convert rotation and translation to SE3 as a Matrix4d
    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    pose_matrix.block(0, 0, dim, dim) = rotation;
    pose_matrix.block(0, dim, dim, 1) = translation.transpose();

    viz.AddVizPose(pose_matrix, 0.1, 2.0, pose_sym.index());
  }

  // Draw all landmarks
  // Draw ranges as lines to landmark
  viz.RenderWorld();
}

} // namespace CORA
