//
// Created by tim on 11/13/23.
//
#include <CORA/CORA.h>
#include <CORA/CORA_vis.h>

int main() {
  CORA::Problem problem = CORA::parsePyfgTextToProblem(
      "./bin/example_data/factor_graph_small.pyfg");
  problem.updateProblemData();

  CORA::Matrix x0 = problem.getRandomInitialGuess();

  CORA::CoraTntResult res;
  res = solveCORA(problem, x0);

  std::cout << "Testing with Random initialization" << std::endl;
  // Visualize the result
  CORA::CORAVis viz{};
  viz.run(problem, {res}, 1.0, true);
  return 0;
}
