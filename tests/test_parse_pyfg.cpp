//
// Created by Tim Magoun on 10/31/23.
//

#include <CORA/pyfg_text_parser.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Loading PyFg", "[Read PyFg]") {
  auto filename = "/tmp/cora-plus-plus/examples/factor_graph.pyfg";
  auto problem = CORA::parsePyfgTextToProblem(filename);
  REQUIRE(true);
}
