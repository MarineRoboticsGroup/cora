//
// Created by tim on 11/12/23.
//

#ifndef CORA_CORA_VIS_H
#define CORA_CORA_VIS_H

#include "CORA.h"
#include "tonioviz/Visualizer.h"

namespace CORA {
class CORAVis {
public:
  CORAVis();

  void visualize(const Problem &problem, const CoraTntResult &result);

private:
  mrg::Visualizer viz{};
};

} // namespace CORA

#endif // CORA_CORA_VIS_H
