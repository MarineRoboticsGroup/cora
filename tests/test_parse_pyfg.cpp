//
// Created by Tim Magoun on 10/31/23.
//

#include <CORA/CORA_test_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace CORA {
TEST_CASE("Loading PyFg", "[ParsePyFG::small_ra_slam_problem]") {
  std::string data_subdir = "small_ra_slam_problem";
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  std::string err_msg = checkSubmatricesAreCorrect(problem, data_subdir);
  REQUIRE(err_msg.empty());
}

TEST_CASE("Loading PyFg", "[ParsePyFG::single_range]") {
  std::string data_subdir = "single_range";
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  std::string err_msg = checkSubmatricesAreCorrect(problem, data_subdir);
  REQUIRE(err_msg.empty());
}

TEST_CASE("Loading PyFg", "[ParsePyFG::single_rpm]") {
  std::string data_subdir = "single_rpm";
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  std::string err_msg = checkSubmatricesAreCorrect(problem, data_subdir);
  REQUIRE(err_msg.empty());
}

} // namespace CORA
