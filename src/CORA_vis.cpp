//
// Created by tim on 11/12/23.
//

#include "CORA/CORA_vis.h"
#include "CORA/CORA_problem.h"
#include <thread> // NOLINT [build/c++11]
#include <utility>
namespace CORA {

CORAVis::CORAVis() {}

using TNTStatus = Optimization::Riemannian::TNTStatus;

/**
 * @brief Start both the rendering thread and the data provider thread
 *
 * @param problem CORA problem to visualize
 * @param results Vector of results, which will be looped over and rendered
 * @param rate_hz Rate at which the results will be rendered
 */

void CORAVis::run(const Problem &problem, std::vector<CoraTntResult> results,
                  double rate_hz, bool verbose) {
  mrg::VisualizerParams params;
  params.frustum_scale = 0.3; // size of frustum in m
  params.landtype = mrg::LandmarkDrawType::kCross;
  params.f = 600.0f; // focal length in px
  params.rangetype = mrg::RangeDrawType::kLine;
  params.range_color = {0.0f, 0.71f, 0.16f};  // Medium green
  params.landmark_color = {0.0f, 0.0f, 0.0f}; // Black

  auto viz = std::make_shared<mrg::Visualizer>(params);
  auto render_thread = std::thread(&CORAVis::renderLoop, this, viz);
  dataPlaybackLoop(std::shared_ptr<mrg::Visualizer>(viz), problem,
                   std::move(results), rate_hz, verbose);
  render_thread.join();
}

void CORAVis::dataPlaybackLoop(const std::shared_ptr<mrg::Visualizer> &viz,
                               const Problem &problem,
                               std::vector<CoraTntResult> results,
                               double rate_hz, bool verbose) {
  auto result_idx{0};
  while (alive) {
    auto result = results.at(result_idx);
    auto status = result.status;
    auto aligned_sol_matrix = problem.alignEstimateToOrigin(result.x);

    if (verbose) {
      std::cout << "Result index: " << result_idx << std::endl;
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
        std::cout << "Solution terminated due to relative decrease"
                  << std::endl;
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
    }

    auto pose_sym_to_idx = problem.getPoseSymbolMap();
    auto landmark_sym_to_idx = problem.getLandmarkSymbolMap();
    auto range_measurements = problem.getRangeMeasurements();
    // Iterate through all poses and draw using TonioViz
    viz->Clear();
    for (auto [pose_sym, pose_idx] : pose_sym_to_idx) {
      auto pose = getPose(problem, aligned_sol_matrix, pose_sym);
      viz->AddVizPose(pose, 0.5, 4.0, static_cast<int>(pose_sym.chr()));
    }

    for (auto [landmark_sym, landmark_idx] : landmark_sym_to_idx) {
      viz->AddVizLandmark(getPoint(problem, aligned_sol_matrix, landmark_sym));
    }

    for (const auto &range_measurement : range_measurements) {
      auto p1 =
          getPoint(problem, aligned_sol_matrix, range_measurement.first_id);
      auto p2 =
          getPoint(problem, aligned_sol_matrix, range_measurement.second_id);

      if (pose_sym_to_idx.find(range_measurement.first_id) ==
          pose_sym_to_idx.end()) {
        // P1 is the landmark, P2 is the pose. We need to flip them
        std::swap(p1, p2);
      }

      mrg::Range range{p1(0), p1(1), range_measurement.r, p2(0), p2(1)};
      viz->AddRangeMeasurement(range);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(1 / rate_hz));

    result_idx++;
    if (result_idx >= static_cast<int>(results.size())) {
      result_idx = 0;
    }
  }
  alive = false;
}

void CORAVis::renderLoop(const std::shared_ptr<mrg::Visualizer> &viz) {
  viz->RenderWorld();
  alive = false;
}

Eigen::Matrix4d CORAVis::getPose(const Problem &problem,
                                 const Matrix &solution_matrix,
                                 const Symbol &pose_sym) {
  auto dim = problem.dim();
  auto rotation_idx = problem.getRotationIdx(pose_sym);
  auto rotation = solution_matrix.block(rotation_idx * dim, 0, dim, dim);
  auto translation =
      solution_matrix.block(problem.getTranslationIdx(pose_sym), 0, 1, dim);
  // Convert rotation and translation to SE3 as a Matrix4d
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();

  // we need to transpose the rotation matrix b/c the solution matrix is
  // actually [R0, R1, ..., Rn, t0, t1, ..., tn]^T so each rotation block is
  // actually R^T
  pose_matrix.block(0, 0, dim, dim) = rotation.transpose();
  pose_matrix.block(0, 3, dim, 1) = translation.transpose();
  return pose_matrix;
}

Eigen::Vector3d CORAVis::getPoint(const Problem &problem,
                                  const Matrix &solution_matrix,
                                  const Symbol &point_sym) {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  point.block(0, 0, problem.dim(), 1) =
      solution_matrix
          .block(problem.getTranslationIdx(point_sym), 0, 1, problem.dim())
          .transpose();
  return point;
}

} // namespace CORA
