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
    throw std::invalid_argument("The block sizes must sum to A.rows() for the "
                                "CORA block Cholesky preconditioner");
  }

  // for each block along the diagonal of A compute the Cholesky decomposition
  // of the block and store the Cholesky factor and the permutation
  // ordering. The default AMD ordering is used to minimize fill-in during
  // the Cholesky decomposition.
  int block_start = 0;
  CholFactorPtrVector block_cholesky_factors;
  for (int block_idx = 0; block_idx < block_sizes.size(); block_idx++) {
    int blockSize = block_sizes(block_idx);
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

  // check that the number of rows of the result is the same as the number of
  // rows of the input vector
  if (num_result_rows != rhs.rows()) {
    throw std::invalid_argument(
        "The number of rows of the result must be the same as the number of "
        "rows of the input vector for the CORA block Cholesky "
        "preconditioner");
  }

  Matrix result(num_result_rows, rhs.cols());
  int block_start = 0;
  for (auto &block_chol_factor_ptr : block_chol_factor_ptrs) {
    // perform the solves in a block-wise ordering
    int block_size = block_chol_factor_ptr->rows();
    result.block(block_start, 0, block_size, rhs.cols()) =
        block_chol_factor_ptr->solve(
            rhs.block(block_start, 0, block_size, rhs.cols()));
    block_start += block_size;
  }

  return result;
}

Matrix tangent_space_projection(const Matrix &Y, const Matrix &Ydot) {
  throw NotImplementedException();
  return Ydot - Y * Y.transpose() * Ydot;
}

} // namespace CORA
