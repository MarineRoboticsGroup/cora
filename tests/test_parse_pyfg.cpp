//
// Created by Tim Magoun on 10/31/23.
//

#include <CORA/CORA_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace CORA {
TEST_CASE("Loading PyFg", "[Read PyFg]") {
  std::string small_sample_subdir = "small_ra_slam_problem";
  std::string pyfg_path =
      getTestDataFpath(small_sample_subdir, "factor_graph.pyfg");

  Problem problem = parsePyfgTextToProblem(pyfg_path);
  std::string err_msg =
      checkSubmatricesAreCorrect(problem, small_sample_subdir);
  REQUIRE(err_msg == "");

  // // DEBUGGING
  // SparseMatrix mat_diff = data_matrix - expected_matrix;
  // mat_diff.prune(0.5, 1.0);
  // std::cout << "data_matrix - expected_matrix = \n"
  //           << mat_diff.toDense() << std::endl;
  // printMatrixSparsityPattern(mat_diff.toDense());
  // // DEBUGGING
}
} // namespace CORA
