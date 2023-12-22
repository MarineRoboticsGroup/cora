//
// Created by tim on 11/12/23.
//

#pragma once

#include <CORA/CORA.h>
#include <CORA/CORA_types.h>
#include <atomic>
#include <memory>
#include <vector>

#include "tonioviz/Visualizer.h"

namespace CORA {
class CORAVis {
public:
  CORAVis();
  void run(const Problem &problem, std::vector<Matrix> iterates, double rate_hz,
           bool verbose = true);
  ~CORAVis() = default;

private:
  std::atomic<bool> alive{true}; // Shared between threads, will be false if
                                 // either thread terminates

  static mrg::VizPose getPose(const Problem &problem,
                              const Matrix &solution_matrix,
                              const Symbol &pose_sym);
  static Eigen::Vector3d getPoint(const Problem &problem,
                                  const Matrix &solution_matrix,
                                  const Symbol &point_sym);

  void dataPlaybackLoop(const std::shared_ptr<mrg::Visualizer> &viz,
                        const Problem &problem, std::vector<Matrix> iterates,
                        double rate_hz, bool verbose);

  void renderLoop(const std::shared_ptr<mrg::Visualizer> &viz);
};

} // namespace CORA
