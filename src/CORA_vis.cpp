//
// Created by tim on 11/12/23.
//

#include "CORA/CORA_vis.h"
#include "CORA/CORA_problem.h"
namespace CORA {

CORAVis::CORAVis() {
  mrg::VisualizerParams params;
  params.frustum_scale = 0.3; // size of frustum in m
  params.landtype = mrg::LandmarkDrawType::kCross;
  params.f = 600.0f; // focal length in px
  params.rangetype = mrg::RangeDrawType::kLine;
  viz = std::make_unique<mrg::Visualizer>(params);
}
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
  // Iterate through all poses and draw using TonioViz
  for (auto [pose_sym, pose_idx] : pose_sym_to_idx) {
    auto pose = getPose(problem, aligned_sol_matrix, pose_sym);
    viz->AddVizPose(pose, 0.5, 4.0);
  }

  for (auto [landmark_sym, landmark_idx] : landmark_sym_to_idx) {
    viz->AddVizLandmark(getPoint(problem, aligned_sol_matrix, landmark_sym));
  }

  for (const auto &range_measurement : range_measurements) {
    auto p1 = getPoint(problem, aligned_sol_matrix, range_measurement.first_id);
    auto p2 =
        getPoint(problem, aligned_sol_matrix, range_measurement.second_id);
    mrg::Range range{p1(0), p1(1), range_measurement.r, p2(0), p2(1)};
    viz->AddRangeMeasurement(range);
  }
  viz->RenderWorld();
}

Eigen::Matrix4d CORAVis::getPose(const Problem &problem,
                                 const Eigen::MatrixXd &solution_matrix,
                                 const Symbol &pose_sym) {
  auto dim = problem.dim();
  auto rotation_idx = problem.getRotationIdx(pose_sym);
  auto rotation = solution_matrix.block(rotation_idx * dim, 0, dim, dim);
  auto translation =
      solution_matrix.block(problem.getTranslationIdx(pose_sym), 0, 1, dim);
  // Convert rotation and translation to SE3 as a Matrix4d
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
  pose_matrix.block(0, 0, dim, dim) = rotation;
  pose_matrix.block(0, 3, dim, 1) = translation.transpose();
  return pose_matrix;
}

Eigen::Vector3d CORAVis::getPoint(const Problem &problem,
                                  const Eigen::MatrixXd &solution_matrix,
                                  const Symbol &point_sym) {
  return solution_matrix
      .block(problem.getTranslationIdx(point_sym), 0, 1, problem.dim())
      .transpose();
}

} // namespace CORA
