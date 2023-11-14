//
// Created by tim on 11/12/23.
//

#ifndef CORA_CORA_VIS_H
#define CORA_CORA_VIS_H

#include <CORA/CORA.h>
#include <CORA/CORA_types.h>
#include <memory>

#include "tonioviz/Visualizer.h"

namespace CORA {
class CORAVis {
public:
  CORAVis();

  void visualize(const Problem &problem, const CoraTntResult &result);

private:
  std::unique_ptr<mrg::Visualizer> viz;

  static Eigen::Matrix4d getPose(const Problem &problem,
                                 const Matrix &solution_matrix,
                                 const Symbol &pose_sym);
  static Eigen::Vector3d getPoint(const Problem &problem,
                                  const Matrix &solution_matrix,
                                  const Symbol &point_sym);
};

} // namespace CORA

#endif // CORA_CORA_VIS_H
