/**
 * @file CORA_preconditioners.h
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <Eigen/CholmodSupport>

#include <CORA/CORA_types.h>
#include <memory>
#include <vector>

#include "Optimization/Riemannian/Concepts.h"

namespace CORA {

using CholeskyFactorization = Eigen::CholmodDecomposition<SparseMatrix>;
using CholFactorPtr = std::shared_ptr<CholeskyFactorization>;
using CholFactorPtrVector = std::vector<CholFactorPtr>;

/**
 * @brief Generate a block-diagonal preconditioner for the given matrix A based
 * on Cholesky decompositions of each block. This assumes that the matrix A is
 * the un-marginalized data matrix, so the last row/column of A is ignored. When
 * applying the preconditioner, the last row/column of the input vector is
 * mapped to zero. In SLAM, this corresponds to pinning the last translation
 * to the origin. See Appendix D of the CORA paper for more details.
 *
 * @param A the matrix to precondition (should be symmetric positive definite)
 * @param block_sizes the sizes of the blocks (should sum to A.rows() - 1)
 * @return Optimization::Riemannian::LinearOperator<Matrix, Matrix>
 */
CholFactorPtrVector getBlockCholeskyFactorization(const SparseMatrix &A,
                                                  const Vector &block_sizes) {
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
    int block_size = block_sizes[block_idx];

    // extract the block from A
    SparseMatrix block =
        A.block(block_start, block_start, block_sizes[block_idx],
                block_sizes[block_idx]);

    // compute the Cholesky decomposition of the block
    block_cholesky_factors.emplace_back(
        std::make_shared<CholeskyFactorization>(block));
  }

  return block_cholesky_factors;
}

Matrix blockCholeskySolve(CholFactorPtrVector block_chol_factor_ptrs,
                          int chol_size, const Matrix &rhs) {
  Matrix result(chol_size, rhs.cols());
  int block_start = 0;
  for (int block_idx = 0; block_idx < block_chol_factor_ptrs.size();
       block_idx++) {
    int block_size = block_chol_factor_ptrs[block_idx]->rows();

    // print out the block Cholesky factor
    result.block(block_start, 0, block_size, rhs.cols()) =
        block_chol_factor_ptrs[block_idx]->solve(
            rhs.block(block_start, 0, block_size, rhs.cols()));
    block_start += block_size;
  }
  return result;
}

} // namespace CORA
