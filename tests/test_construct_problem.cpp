/**
 * @file test_construct_problem.cpp
 * @author Alan Papalia (apapalia@mit.edu)
 * @brief unit tests for the CORA::Problem class
 * @version 0.1
 * @date 2023-11-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <CORA/CORA_problem.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Correctly constructing single-odometry data matrix",
          "[problem::data_matrix]") {
  CORA::Problem problem(3, 5);
  problem.addPoseVariable("x1");
  problem.addPoseVariable("x2");
  problem.printProblem();
}
