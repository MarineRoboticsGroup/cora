//
// Created by tim on 11/12/23.
//

#pragma once

#include <CORA/CORA.h>
#include <CORA/CORA_types.h>
#include <memory>
#include <vector>

#include "tonioviz/Visualizer.h"

namespace CORA {
class CORAVis {
public:
  CORAVis();
  void run(const Problem &problem, std::vector<CoraTntResult> results,
           double rate_hz);

private:
  std::unique_ptr<mrg::Visualizer> viz;

  static Eigen::Matrix4d getPose(const Problem &problem,
                                 const Matrix &solution_matrix,
                                 const Symbol &pose_sym);
  static Eigen::Vector3d getPoint(const Problem &problem,
                                  const Matrix &solution_matrix,
                                  const Symbol &point_sym);
  void renderLoop();
  void dataPlaybackLoop(const Problem &problem,
                        std::vector<CoraTntResult> results, double rate_hz);
  void visualize(const Problem &problem, const CoraTntResult &result);
};

} // namespace CORA
