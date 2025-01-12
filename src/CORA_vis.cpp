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

std::vector<Matrix>
CORAVis::projectAndAlignIterates(const Problem &problem,
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

  auto aligned_iterates = projectAndAlignIterates(problem, iterates);
  dataPlaybackLoop(std::shared_ptr<mrg::Visualizer>(viz), problem,
                   aligned_iterates, rate_hz, verbose);
  render_thread.join();
}

void CORAVis::dataPlaybackLoop(const std::shared_ptr<mrg::Visualizer> &viz,
                               const Problem &problem,
                               std::vector<Matrix> iterates, double rate_hz,
                               bool verbose) {
  if (iterates.empty()) {
    throw std::runtime_error("iterates is empty");
  }
  if (rate_hz <= 0) {
    throw std::runtime_error("rate_hz must be > 0");
  }

  auto soln_idx{0};
  int curr_loop_cnt = 0;
  int max_num_loops = 2;

  // try to load max_num_loops from the environment
  if (const char *env_p = std::getenv("CORA_MAX_LOOPS")) {
    max_num_loops = std::stoi(env_p);
    std::cout << "Using max_num_loops from environment variable: "
              << max_num_loops << std::endl;
  } else {
    std::cout << "Using default max_num_loops: " << max_num_loops << std::endl;
  }

  // we will use double buffering to render the poses and landmarks
  // so that we don't have to clear the screen every time

  // Iterate through all poses and draw using TonioViz
  // set it up so only drawing <=100 poses total (skip the right number of
  // poses)
  int num_poses = problem.numPoses();
  int num_poses_to_show = 5000;
  int num_poses_to_skip = static_cast<int>(num_poses / num_poses_to_show);
  int num_ranges = problem.numRangeMeasurements();
  int num_ranges_to_show = 2000;
  int num_ranges_to_skip = static_cast<int>(num_ranges / num_ranges_to_show);

  auto landmark_sym_to_idx = problem.getLandmarkSymbolMap();
  auto range_measurements = problem.getRangeMeasurements();

  auto pose_sym_to_idx = problem.getPoseSymbolMap();
  // isolate the different pose chains by the character of the symbols
  std::set<char> pose_chain_chars;
  for (const auto &[pose_sym, pose_idx] : pose_sym_to_idx) {
    pose_chain_chars.insert(pose_sym.chr());
  }

  int num_pose_chains = pose_chain_chars.size();
  std::vector<std::map<Symbol, int>> pose_chain_sym_to_idx(num_pose_chains);
  for (const auto &[pose_sym, pose_idx] : pose_sym_to_idx) {
    char pose_char = pose_sym.chr();
    int pose_chain_idx = std::distance(pose_chain_chars.begin(),
                                       pose_chain_chars.find(pose_char));
    pose_chain_sym_to_idx[pose_chain_idx][pose_sym] = pose_idx;
  }

  while (alive && curr_loop_cnt < max_num_loops) {
    auto soln = iterates.at(soln_idx);

    // Ready false
    viz->setReadyToRender(false);
    viz->Clear();

    // add all poses
    for (int pose_chain_idx = 0; pose_chain_idx < num_pose_chains;
         pose_chain_idx++) {
      auto pose_sym_to_idx = pose_chain_sym_to_idx[pose_chain_idx];
      std::vector<mrg::VizPose> viz_poses = {};
      for (auto [pose_sym, pose_idx] : pose_sym_to_idx) {
        if (num_poses_to_skip > 0 && pose_idx % num_poses_to_skip != 0) {
          continue;
        }
        if (pose_sym == problem.getOriginSymbol()) {
          continue;
        }

        viz_poses.emplace_back(getPose(problem, soln, pose_sym));
      }
      viz->AddVizPoses(viz_poses, pose_chain_idx);
    }
    // std::vector<mrg::VizPose> viz_poses = {};
    // for (auto [pose_sym, pose_idx] : pose_sym_to_idx) {
    //   if (num_poses_to_skip > 0 && pose_idx % num_poses_to_skip != 0) {
    //     continue;
    //   }
    //   if (pose_sym == problem.getOriginSymbol()) {
    //     continue;
    //   }
    //   viz_poses.emplace_back(getPose(problem, soln, pose_sym));
    // }
    // viz->AddVizPoses(viz_poses);

    // add all landmarks
    for (auto [landmark_sym, landmark_idx] : landmark_sym_to_idx) {
      viz->AddVizLandmark(getPoint(problem, soln, landmark_sym));
    }

    // add all range measurements
    int range_measurement_idx = -1;
    for (const auto &range_measurement : range_measurements) {
      range_measurement_idx++;
      if (num_ranges_to_skip > 0 &&
          range_measurement_idx % num_ranges_to_skip != 0) {
        continue;
      }
      auto p1 = getPoint(problem, soln, range_measurement.first_id);
      auto p2 = getPoint(problem, soln, range_measurement.second_id);

      if (pose_sym_to_idx.find(range_measurement.first_id) ==
          pose_sym_to_idx.end()) {
        // P1 is the landmark, P2 is the pose. We need to flip them
        std::swap(p1, p2);
      }

      Eigen::Vector3d p1_vec = Eigen::Vector3d::Zero();
      p1_vec.block(0, 0, problem.dim(), 1) = p1;
      Eigen::Vector3d p2_vec = Eigen::Vector3d::Zero();
      p2_vec.block(0, 0, problem.dim(), 1) = p2;
      mrg::Range range{p1_vec, p2_vec, range_measurement.r};
      viz->AddRangeMeasurement(range);
    }

    // Ready true
    viz->setReadyToRender(true);

    std::this_thread::sleep_for(std::chrono::duration<double>(1 / rate_hz));

    soln_idx++;

    if (soln_idx >= static_cast<int>(iterates.size())) {
      // pause for 2 seconds and then restart
      std::this_thread::sleep_for(std::chrono::duration<double>(2.0));
      soln_idx = 0;
      curr_loop_cnt++;
    }
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
