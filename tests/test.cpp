#include <CORA/CORA_preconditioners.h>
#include <CORA/CORA_types.h>

#include <cstdint>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>

uint32_t factorial(uint32_t number) {
  return number <= 1 ? number : factorial(number - 1) * number;
}

double get_random_double() { return static_cast<double>(rand_r()) / RAND_MAX; }

TEST_CASE("Factorials are computed", "[factorial]") {
  REQUIRE(factorial(1) == 1);
  REQUIRE(factorial(2) == 2);
  REQUIRE(factorial(3) == 6);
  REQUIRE(factorial(10) == 3'628'800);
}

/**
 * @brief Tests the case in which the matrix is a single block and deterministic
 * (i.e., we are just factoring a single matrix, which we have hand-coded)
 *
 */
TEST_CASE("Single Cholesky solves work okay for fixed matrix", "[cholesky]") {
  int block_size = 3;
  CORA::SparseMatrix A(block_size, block_size);

  // generate a known symmetric positive definite matrix by filling in the
  // upper triangular part of A with random values and then copying the upper
  // triangular part to the lower triangular part
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 0.5;
  A.insert(0, 2) = 0.25;
  A.insert(1, 1) = 1.0;
  A.insert(1, 2) = 0.5;
  A.insert(2, 2) = 1.0;

  // copy the upper triangular part to the lower triangular part
  for (int row = 0; row < A.rows(); row++) {
    for (int col = row + 1; col < A.cols(); col++) {
      A.insert(col, row) = A.coeff(row, col);
    }
  }

  // make sure the matrix is compressed before we use it
  A.makeCompressed();

  // factorize the matrix
  CORA::Vector block_sizes(1);
  block_sizes << block_size;
  CORA::CholFactorPtrVector block_cholesky_factors =
      CORA::getBlockCholeskyFactorization(A, block_sizes);

  // get the inverse of the matrix to test against
  CORA::Matrix A_inv = A.toDense().inverse();

  // test on identity matrix
  CORA::Matrix I = CORA::Matrix::Identity(block_size, block_size);
  CORA::Matrix I_x =
      CORA::blockCholeskySolve(block_cholesky_factors, block_size, I);
  REQUIRE(I_x.isApprox(A_inv));

  // solve on itself should be identity
  CORA::Matrix A_x =
      CORA::blockCholeskySolve(block_cholesky_factors, block_size, A.toDense());
  REQUIRE(A_x.isApprox(CORA::Matrix::Identity(block_size, block_size)));

  // test vector to solve against
  CORA::Vector b(block_size);
  b << 1.0, 2.0, 3.0;
  CORA::Matrix x =
      CORA::blockCholeskySolve(block_cholesky_factors, block_size, b);
  CORA::Vector expected_x = A_inv * b;
  REQUIRE(x.isApprox(expected_x));
}

/**
 * @brief Tests the case in which the matrix is a single block and random (i.e.,
 * we are just factoring a single matrix, which we are generating randomly)
 *
 */
TEST_CASE("Single Cholesky solves work okay for random matrices",
          "[cholesky]") {
  // random value between 1 and 30
  int block_size = GENERATE(
      take(10, filter([](int i) { return i % 2 == 1; }, random(1, 100))));

  // generate a random symmetric positive definite matrix by filling in the
  // upper triangular part of A with random values and then copying the upper
  // triangular part to the lower triangular part
  CORA::SparseMatrix A(block_size, block_size);

  // randomly fill A and then multiply A*A' to make it symmetric positive
  // definite
  for (int row = 0; row < A.rows(); row++) {
    for (int col = row; col < A.cols(); col++) {
      A.insert(row, col) = get_random_double();
    }
  }
  A.makeCompressed();
  A = A * A.transpose();
  // add a small amount to the diagonal to make it positive definite
  for (int row = 0; row < A.rows(); row++) {
    A.coeffRef(row, row) += 0.1;
  }

  // factorize the matrix
  CORA::Vector block_sizes(1);
  block_sizes << block_size;
  CORA::CholFactorPtrVector block_cholesky_factors =
      CORA::getBlockCholeskyFactorization(A, block_sizes);

  // get the inverse of the matrix to test against
  CORA::Matrix A_inv = A.toDense().inverse();

  // test on identity matrix
  CORA::Matrix I = CORA::Matrix::Identity(block_size, block_size);
  CORA::Matrix I_x =
      CORA::blockCholeskySolve(block_cholesky_factors, block_size, I);

  bool inverseCorrect = I_x.isApprox(A_inv);
  REQUIRE(inverseCorrect);
  if (!inverseCorrect) {
    INFO("I_x: " << I_x);
    INFO("A_inv: " << A_inv);
  }

  // solve on itself should be identity
  CORA::Matrix A_x =
      CORA::blockCholeskySolve(block_cholesky_factors, block_size, A.toDense());
  REQUIRE(A_x.isApprox(CORA::Matrix::Identity(block_size, block_size)));

  // test vector to solve against
  CORA::Vector b(block_size);
  // randomly fill b
  for (int i = 0; i < b.size(); i++) {
    b(i) = get_random_double();
  }

  CORA::Matrix x =
      CORA::blockCholeskySolve(block_cholesky_factors, block_size, b);
  CORA::Vector expected_x = A_inv * b;
  REQUIRE(x.isApprox(expected_x));
}

/**
 * @brief Test the case in which we take several blocks of the same size
 * and factorize them. The matrix is deterministic (i.e., we are just factoring
 * a single matrix, which we have hand-coded)
 */
TEST_CASE("Block Cholesky solves work okay for fixed matrix",
          "[block cholesky]") {
  int mat_size = 6;
  CORA::SparseMatrix A(mat_size, mat_size);

  // fill the diagonal with 1's
  for (int i = 0; i < mat_size; i++) {
    A.insert(i, i) = 1.0;
  }

  // fill the upper triangular part with 0.5's
  for (int row = 0; row < A.rows(); row++) {
    for (int col = row + 1; col < A.cols(); col++) {
      A.insert(row, col) = 0.5;
      A.insert(col, row) = 0.5;
    }
  }

  // make sure the matrix is compressed before we use it
  A.makeCompressed();

  // set the block sizes
  CORA::Vector block_sizes(2);
  block_sizes << 3, 3;

  // factorize the matrix
  CORA::CholFactorPtrVector block_cholesky_factors =
      CORA::getBlockCholeskyFactorization(A, block_sizes);

  // get just the block diagonals
  CORA::SparseMatrix A_block_diags(mat_size, mat_size);
  int start_idx = 0;
  for (int block_idx = 0; block_idx < block_sizes.size(); block_idx++) {
    int block_size = block_sizes(block_idx);
    for (int row_idx = start_idx; row_idx < start_idx + block_size; row_idx++) {
      for (int col_idx = start_idx; col_idx < start_idx + block_size;
           col_idx++) {
        A_block_diags.insert(row_idx, col_idx) = A.coeff(row_idx, col_idx);
      }
    }
    start_idx += block_size;
  }

  // check that the block diagonals are different from the original matrix
  REQUIRE(!A_block_diags.isApprox(A));

  // get the inverses of the blocks to test against
  CORA::Matrix A_block_diags_inv = A_block_diags.toDense().inverse();

  // test on identity matrix
  CORA::Matrix I = CORA::Matrix::Identity(mat_size, mat_size);
  CORA::Matrix I_x =
      CORA::blockCholeskySolve(block_cholesky_factors, mat_size, I);
  REQUIRE(I_x.isApprox(A_block_diags_inv));

  // solve on block diagonals should be identity
  CORA::Matrix A_x =
      CORA::blockCholeskySolve(block_cholesky_factors, mat_size, A_block_diags);
  REQUIRE(A_x.isApprox(CORA::Matrix::Identity(mat_size, mat_size)));

  // test vector to solve against
  CORA::Vector b(mat_size);
  b << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  CORA::Matrix x =
      CORA::blockCholeskySolve(block_cholesky_factors, mat_size, b);
  CORA::Vector expected_x = A_block_diags_inv * b;
  REQUIRE(x.isApprox(expected_x));
}
