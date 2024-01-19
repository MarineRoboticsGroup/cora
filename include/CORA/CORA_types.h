/**
 * @file CORA_types.h
 * @brief A file containing the basic types used by the CORA library.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <string>

#include "Optimization/Riemannian/TNT.h"

class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(std::string const &str)
      : std::logic_error(str + " not implemented") {}
};

using Index = Eigen::Index;

class MatrixShapeException : public std::logic_error {
public:
  MatrixShapeException(const std::string &func_name, Index exp_rows,
                       Index exp_cols, Index act_rows, Index act_cols)
      : std::logic_error(func_name + ": " + "expected matrix of shape (" +
                         std::to_string(exp_rows) + ", " +
                         std::to_string(exp_cols) + ") but got (" +
                         std::to_string(act_rows) + ", " +
                         std::to_string(act_cols) + ")") {}
};
inline void checkMatrixShape(const std::string &func_name, Index exp_rows,
                             Index exp_cols, Index act_rows, Index act_cols) {
  if (exp_rows != act_rows || exp_cols != act_cols) {
    throw MatrixShapeException(func_name, exp_rows, exp_cols, act_rows,
                               act_cols);
  }
}

namespace CORA {

typedef double Scalar;

typedef Eigen::VectorXi VectorXi;
typedef Eigen::Index Index;
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> DiagonalMatrix;

enum class Formulation {
  // The CORA problem in which translations are explicitly represented
  Explicit,
  // The SE-Sync problem in which translations are marginalized out
  Implicit
};

struct CertResults {
  bool is_certified;
  Scalar theta;
  Vector x;
  Matrix all_eigvecs;
  size_t num_iters;
};

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
