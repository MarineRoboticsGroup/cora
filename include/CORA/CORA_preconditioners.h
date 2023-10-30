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

#include <CORA/CORA_types.h>
#include <vector>

#include <Eigen/Sparse>
#include <SuiteSparseQR.hpp>

#include "Optimization/Riemannian/Concepts.h"
#include "Preconditioners/LSChol/include/LSChol/LSChol.h"

namespace CORA {

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
Optimization::Riemannian::LinearOperator<Matrix, Matrix>
constructCoraBlockCholeskyPreconditioner(const SparseMatrix &A,
                                         const Vector &block_sizes) {
  // TODO(magoun): implement this function
  throw NotImplementedException();

  // check that the block sizes sum to A.rows() - 1
  if (block_sizes.sum() != A.rows() - 1) {
    throw std::invalid_argument(
        "The block sizes must sum to A.rows() - 1 for the "
        "CORA block Cholesky preconditioner");
  }

  // set the cholmod ordering method to COLAMD
  int ordering_method = CHOLMOD_COLAMD;

  /**
   * @brief compute Cholesky decomposition with permutation ordering to minimize
   * fill-in store the Cholesky factor and the permutation return a
   * LinearOperator that applies the block-wise Cholesky factors
   */

  // for each block along the diagonal of A compute the Cholesky decomposition
  // of the block and store the Cholesky factor and the permutation
  // ordering. The permutation ordering is used to minimize fill-in during
  // the Cholesky decomposition. The Cholesky factor and the permutation
  // ordering are stored in a LinearOperator that applies the block-wise
  // Cholesky factors. The LinearOperator is returned.
  int block_start = 0;
  std::vector<SuiteSparseQR_factorization<Matrix>> block_cholesky_factors;
  for (int block_idx = 0; block_idx < block_sizes.size(); block_idx++) {
    int block_size = block_sizes[block_idx];

    // extract the block from A
    Eigen::SparseMatrix<double> block =
        A.block(block_start, block_start, block_sizes[block_idx],
                block_sizes[block_idx]);
    SuiteSparseQR_factorization<Eigen::SparseMatrix<double>>
        block_cholesky_factor =
            SuiteSparseQR_factorization<Eigen::SparseMatrix<double>>(
                block, ordering_method);

    // store the block Cholesky factor and the permutation ordering
    block_cholesky_factors.push_back(block_cholesky_factor);
  }
}

/**
 * @brief Similar to constructCoraBlockCholeskyPreconditioner, but uses the
 * marginalized data matrix, so does not ignore the last row/column of A
 *
 * @param A the matrix to precondition (should be symmetric positive definite)
 * @param block_sizes the sizes of the blocks (should sum to A.rows())
 * @return Optimization::Riemannian::LinearOperator<Matrix, Matrix>
 */
Optimization::Riemannian::LinearOperator<Matrix, Matrix>
constructMarginalizedCoraBlockCholeskyPreconditioner(
    const SparseMatrix &A, const Vector &block_sizes) {
  // TODO(magoun): implement this function
  throw NotImplementedException();

  /**
   * @brief compute Cholesky decomposition with permutation ordering to minimize
   * fill-in store the Cholesky factor and the permutation return a
   * LinearOperator that applies the block-wise Cholesky factors
   */
}

} // namespace CORA
