#include <CORA/CORA.h>
#include <CORA/pyfg_text_parser.h>
#include <test_utils.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace CORA {

void printResults(CoraTntResult res, Problem problem, std::string data_subdir) {
  std::cout << "_____________________________________" << std::endl;
  std::cout << "Data subdir: " << data_subdir << std::endl;
  std::cout << "Cost (soln): " << res.f << std::endl;
  std::cout << "Rgrad norm (soln): " << res.gradfx_norm << std::endl;
  std::cout << "Run time: " << res.elapsed_time << std::endl;
  // std::cout << "Estimated state: " << std::endl << res.x << std::endl;
  std::cout << "Outer Iterations: " << res.iterates.size() << std::endl;
  std::cout << "Inner Iterations: " << res.inner_iterations.size() << std::endl;

  std::cout << "Objective values:" << std::endl;
  for (Scalar val : res.objective_values) {
    std::cout << val << "; ";
  }
  std::cout << "\nPreconditioned Riemannian gradient norms:" << std::endl;
  for (Scalar val : res.preconditioned_gradient_norms) {
    std::cout << val << "; ";
  }
  std::cout << "\nRiemannian gradient norms:" << std::endl;
  for (Scalar val : res.gradient_norms) {
    std::cout << val << "; ";
  }
  std::cout << "\nUpdate step norms:" << std::endl;
  for (Scalar val : res.update_step_norms) {
    std::cout << val << "; ";
  }
  std::cout << "\n_____________________________________" << std::endl;
  std::cout << "\n\n\n" << std::endl;
}

CoraTntResult testScenario(std::string data_subdir) {
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  problem.updateProblemData();

  Matrix x0 = problem.getRandomInitialGuess();

  std::string X_gt_fpath = getTestDataFpath(data_subdir, "X_gt.mm");
  Matrix X_gt = readMatrixMarketFile(X_gt_fpath).toDense();

  checkMatrixShape("CORA-solve::" + data_subdir + "::X_gt",
                   problem.getDataMatrixSize(), 2, X_gt.rows(), X_gt.cols());

  CoraTntResult res;
  res = solveCORA(problem, x0).first;

  // some print output for debugging
  // std::cout << "Testing with Random initialization" << std::endl;
  // printResults(res, problem, x0, data_subdir);

  return res;
}

TEST_CASE("Test solve RA-SLAM", "[CORA-solve::small_ra_slam_problem]") {
  std::string data_subdir = "small_ra_slam_problem";
  auto results = testScenario(data_subdir);

  /*** Some additional content for playing around **/
  // auto prob = getProblem(data_subdir);
  // prob.updateProblemData();
  // Matrix aligned_x = prob.alignEstimateToOrigin(results.x);
  // printResults(results, prob, data_subdir);
  // std::cout << "solution:\n" << results.x << std::endl;
  // std::cout << "aligned solution:\n" << aligned_x << std::endl;
}

TEST_CASE("Test solve single-range", "[CORA-solve::single_range]") {
  std::string data_subdir = "single_range";
  testScenario(data_subdir);
}

TEST_CASE("Test solve single-rpm", "[CORA-solve::single_rpm]") {
  std::string data_subdir = "single_rpm";
  CoraTntResult res = testScenario(data_subdir);
}

} // namespace CORA
