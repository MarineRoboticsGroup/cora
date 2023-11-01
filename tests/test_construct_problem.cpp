/**
 * @file test_construct_problem.cpp
 * @author Alan Papalia (apapalia@mit.edu)
 * @brief unit tests for the Problem class
 * @version 0.1
 * @date 2023-11-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <CORA/CORA_problem.h>
#include <catch2/catch_test_macros.hpp>

namespace CORA {

TEST_CASE("constructing single-odometry data matrix",
          "[problem::data_matrix::single-odom]") {
  int dim = 3;
  int rank = 5;
  Problem problem(dim, rank, Formulation::Explicit);
  Symbol x1("x1");
  Symbol x2("x2");
  problem.addPoseVariable(x1);
  problem.addPoseVariable(x2);

  // test when odometry measurement is identity (no rotation, no translation)
  Matrix rotId = Matrix::Identity(3, 3);
  Vector tranZero = Vector::Zero(3);
  Matrix covId = Matrix::Identity(6, 6);
  RelativePoseMeasurement rpm_identity(x1, x2, rotId, tranZero, covId);
  problem.addRelativePoseMeasurement(rpm_identity);
  problem.constructDataMatrix();
  SparseMatrix data_matrix = problem.getDataMatrix();

  Matrix expected_state_vals = Matrix::Zero((dim + 1) * 2, dim);
  expected_state_vals.block(0, 0, dim, dim) = rotId;
  expected_state_vals.block(dim, 0, dim, dim) = rotId;
  Matrix expected_translations = Matrix::Zero(2, dim);
  expected_translations.row(0) = Vector::Random(dim);
  expected_translations.row(1) = expected_translations.row(0);

  expected_state_vals.block(2 * dim, 0, 2, dim) = expected_translations;

  // the expected state values should be in null space of the data matrix
  Matrix mat_prod = data_matrix * expected_state_vals;
  REQUIRE(mat_prod.norm() < 1e-12);

  // we should be able to multiply the state values by any (d x d) orthogonal
  // matrix and still be in the null space of the data matrix

  // we can get a random orthogonal matrix by taking the left singular vectors
  // of a random matrix
  Matrix A(dim, dim);
  Eigen::JacobiSVD<Matrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Matrix random_ortho = svd.matrixU();

  mat_prod = data_matrix * (expected_state_vals * random_ortho);
  REQUIRE(mat_prod.norm() < 1e-12);
}

TEST_CASE("constructing single-range data matrix",
          "[problem::data_matrix::single-range::landmark-only]") {
  int dim = 3;
  int rank = 5;
  Problem problem(dim, rank, Formulation::Explicit);
  Symbol l1("l1");
  Symbol l2("l2");
  problem.addLandmarkVariable(l1);
  problem.addLandmarkVariable(l2);
  Scalar range_dist = 2.0;
  RangeMeasurement range_measure = RangeMeasurement(l1, l2, range_dist, 0.5);
  problem.addRangeMeasurement(range_measure);

  problem.constructDataMatrix();
  SparseMatrix data_matrix = problem.getDataMatrix();

  /**
   * @brief We will construct an expected state matrix with l1 and l2 range_dist
   * apart. The direction shouldn't matter but the first row of the state matrix
   * should be the direction of l1 to l2.
   *
   * The expected state matrix ordering: [offset direction; l1; l2]
   *
   */
  Vector l1_pos = Vector::Random(dim);
  Vector l2_offset_dir = Vector::Random(dim);
  l2_offset_dir.normalize();
  Vector l2_pos = l1_pos + l2_offset_dir * range_dist;

  Matrix expected_state_vals = Matrix::Zero(3, dim);
  expected_state_vals.row(0) = l2_offset_dir;
  expected_state_vals.row(1) = l1_pos;
  expected_state_vals.row(2) = l2_pos;

  // the expected state values should be in null space of the data matrix
  Matrix mat_prod = data_matrix * expected_state_vals;
  REQUIRE(mat_prod.norm() < 1e-12);
}
} // namespace CORA
