/**
 * @file CORA_types.h
 * @brief A file containing the basic types used by the CORA library.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Optimization/Riemannian/TNT.h"

namespace CORA {

typedef double Scalar;

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> DiagonalMatrix;

/** Per SE-Sync:
 * We use row-major storage order to take advantage of fast (sparse-matrix) *
 * (dense-vector) multiplications when OpenMP is available (cf. the Eigen
 * documentation page on "Eigen and Multithreading") */
typedef Eigen::SparseMatrix<Scalar, Eigen::RowMajor> SparseMatrix;

// manifold operations
enum class StiefelRetraction { QR, Polar };
enum class ObliqueRetraction { Normalize };

/** The preconditioner applied to the inner tCG solver. */
enum class Preconditioner { None, Jacobi, BlockCholesky, RegularizedCholesky };

/** The initialization method used for the CORA algorithm. */
enum class Initialization { Random, Odometry };

/** A typedef for an "instrumentation function" that can be passed into
 * the Riemannian TNT solver. */
typedef Optimization::Riemannian::TNTUserFunction<Matrix, Matrix, Scalar,
                                                  Matrix>
    InstrumentationFunction;

} // namespace CORA