#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace CORA {
TEST_CASE("Test solve", "[CORA-solve::small_ra_slam_problem]") {
  std::string data_subdir = "small_ra_slam_problem";
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  problem.updateProblemData();

  Matrix x0 = problem.getRandomInitialGuess();

  std::string X_gt_fpath = getTestDataFpath(data_subdir, "X_gt.mm");
  Matrix X_gt = readMatrixMarketFile(X_gt_fpath).toDense();

  checkMatrixShape("CORA-solve::small-ra-slam-problem::X_gt",
                   problem.getDataMatrixSize(), 2, X_gt.rows(), X_gt.cols());

  // just check if it runs
  Matrix x = solveCORA(problem, X_gt);
}
} // namespace CORA
