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

private:
  mrg::Visualizer viz{};
  void visualize(const Problem &problem, const CoraTntResult &result);
};

} // namespace CORA

#endif // CORA_CORA_VIS_H
