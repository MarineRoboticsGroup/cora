//
// Created by tim on 11/12/23.
//

#include "CORA/CORA_vis.h"
#include "CORA/CORA_problem.h"
#include <ncurses.h>
#include <thread> // NOLINT [build/c++11]
#include <utility>
namespace CORA {

CORAVis::CORAVis() {}

using TNTStatus = Optimization::Riemannian::TNTStatus;

std::vector<Matrix>
projectAndAlignIterates(const Problem &problem,
                        const std::vector<Matrix> &iterates) {
  std::vector<Matrix> aligned_iterates;
  for (const auto &iterate : iterates) {
    auto aligned_sol_matrix = problem.alignEstimateToOrigin(projectSolution(
        problem, iterate)); // project and align the solution to the origin
    aligned_iterates.push_back(aligned_sol_matrix);
  }
  return aligned_iterates;
}

/**
 * @brief Start both the rendering thread and the data provider thread
 *
 * @param problem CORA problem to visualize
 * @param results Vector of results, which will be looped over and rendered
 * @param rate_hz Rate at which the results will be rendered
 */

void CORAVis::run(const Problem &problem, std::vector<Matrix> iterates,
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
                   projectAndAlignIterates(problem, iterates), rate_hz,
                   verbose);
  render_thread.join();
}

void CORAVis::dataPlaybackLoop(const std::shared_ptr<mrg::Visualizer> &viz,
                               const Problem &problem,
                               std::vector<Matrix> iterates, double rate_hz,
                               bool verbose) {
  auto soln_idx{0};
  int curr_loop_cnt = 0;
  int max_num_loops = 3;

  // we will use double buffering to render the poses and landmarks
  // so that we don't have to clear the screen every time

  // Iterate through all poses and draw using TonioViz
  // set it up so only drawing <=100 poses total (skip the right number of
  // poses)
  int num_poses = problem.numPoses();
  int num_poses_to_skip = static_cast<int>(num_poses / 650);
  int num_ranges = problem.numRangeMeasurements();
  int num_ranges_to_skip = static_cast<int>(num_ranges / 500);
  while (alive) {
    while (soln_idx >= static_cast<int>(iterates.size())) {
      // pause for 2 seconds and then restart
      std::this_thread::sleep_for(std::chrono::duration<double>(2.0));
      soln_idx = 0;
      curr_loop_cnt++;
      if (curr_loop_cnt >= max_num_loops) {
        alive = false;
        break;
      }
    }

    auto soln = iterates.at(soln_idx);
    auto aligned_sol_matrix = problem.alignEstimateToOrigin(soln);

    auto pose_sym_to_idx = problem.getPoseSymbolMap();
    auto landmark_sym_to_idx = problem.getLandmarkSymbolMap();
    auto range_measurements = problem.getRangeMeasurements();
    // Ready false
    viz->setReadyToRender(false);
    viz->Clear();

    std::vector<mrg::VizPose> viz_poses = {};
    for (auto [pose_sym, pose_idx] : pose_sym_to_idx) {
      if (num_poses_to_skip > 0 && pose_idx % num_poses_to_skip != 0) {
        continue;
      }
      viz_poses.emplace_back(getPose(problem, aligned_sol_matrix, pose_sym));
    }
    viz->AddVizPoses(viz_poses);

    for (auto [landmark_sym, landmark_idx] : landmark_sym_to_idx) {
      viz->AddVizLandmark(getPoint(problem, aligned_sol_matrix, landmark_sym));
    }

    int range_measurement_idx = -1;
    for (const auto &range_measurement : range_measurements) {
      range_measurement_idx++;
      if (num_ranges_to_skip > 0 &&
          range_measurement_idx % num_ranges_to_skip != 0) {
        continue;
      }
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

    // Ready true
    viz->setReadyToRender(true);

    std::this_thread::sleep_for(std::chrono::duration<double>(1 / rate_hz));

    soln_idx++;
  }
  alive = false;
}

void CORAVis::renderLoop(const std::shared_ptr<mrg::Visualizer> &viz) {
  viz->RenderWorld();
  alive = false;
}

mrg::VizPose CORAVis::getPose(const Problem &problem,
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
  return std::make_tuple(pose_matrix, 0.5, 4.0);
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
