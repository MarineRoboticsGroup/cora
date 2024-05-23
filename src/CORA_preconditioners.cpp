/**
 * @file CORA_preconditioners.cpp
 * @author Alan Papalia (apapalia@mit.edu)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <CORA/CORA_preconditioners.h>

namespace CORA {

CholFactorPtrVector getBlockCholeskyFactorization(const SparseMatrix &A,
                                                  const VectorXi &block_sizes) {
  if (block_sizes.sum() != A.rows()) {
    throw std::invalid_argument(
        "The block sizes must sum to A.rows() for the "
        "CORA block Cholesky preconditioner. Block sizes sum: " +
        std::to_string(block_sizes.sum()) +
        ", A.rows(): " + std::to_string(A.rows()));
  }

  // for each block along the diagonal of A compute the Cholesky decomposition
  // of the block and store the Cholesky factor and the permutation
  // ordering. The default AMD ordering is used to minimize fill-in during
  // the Cholesky decomposition.
  int block_start = 0;
  CholFactorPtrVector block_cholesky_factors;
  for (int block_idx = 0; block_idx < block_sizes.size(); block_idx++) {
    int blockSize = block_sizes[block_idx];
    // extract the block from A
    SparseMatrix block =
        A.block(block_start, block_start, blockSize, blockSize);

    // compute the Cholesky decomposition of the block
    block_cholesky_factors.emplace_back(
        std::make_shared<CholeskyFactorization>(block));
  }

  return block_cholesky_factors;
}

Matrix blockCholeskySolve(const CholFactorPtrVector &block_chol_factor_ptrs,
                          const Matrix &rhs) {
  // sum # rows of each block
  int num_result_rows = 0;
  for (auto &block_chol_factor_ptr : block_chol_factor_ptrs) {
    num_result_rows += block_chol_factor_ptr->rows();
  }

  bool rhs_same_num_rows = rhs.rows() == num_result_rows;
  bool rhs_one_more_row = rhs.rows() == num_result_rows + 1;

  // rhs and cholesky factor must have either the same number of rows or one
  // more row than the cholesky factor
  if (!rhs_same_num_rows && !rhs_one_more_row) {
    throw std::invalid_argument(
        "The number of rows in the right-hand side must be equal to the sum "
        "of the number of rows in the block Cholesky factors or one more row "
        "than the sum of the number of rows in the block Cholesky factors.");
  }

  Matrix result(rhs.rows(), rhs.cols());
  int block_start = 0;
  for (auto &block_chol_factor_ptr : block_chol_factor_ptrs) {
    // perform the solves in a block-wise ordering
    int block_size = block_chol_factor_ptr->rows();
    result.block(block_start, 0, block_size, rhs.cols()) =
        block_chol_factor_ptr->solve(
            rhs.block(block_start, 0, block_size, rhs.cols()));
    block_start += block_size;
  }

  // if rhs has one more row then set the last row of the result to zero
  if(rhs_one_more_row) {
    result.bottomRows(1).setZero();
  }

  return result;
}

} // namespace CORA
