/**
 * @file CORA_problem.h
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <CORA/CORA_preconditioners.h>
#include <CORA/CORA_types.h>
#include <CORA/Measurements.h>
#include <CORA/ObliqueManifold.h>
#include <CORA/StiefelProduct.h>
#include <CORA/Symbol.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace CORA {

/**
 * @brief the submatrices that are used to construct the data matrix.
 * All of these matrices are sparse and most have sparsity structures
 * relating to the underlying graph structure of the problem.
 */
struct CoraDataSubmatrices {
  SparseMatrix range_incidence_matrix;
  SparseMatrix range_precision_matrix;
  SparseMatrix range_dist_matrix;
  SparseMatrix rel_pose_incidence_matrix;
  SparseMatrix rel_pose_translation_data_matrix;
  SparseMatrix rotation_conn_laplacian;
  SparseMatrix rel_pose_translation_precision_matrix;
  SparseMatrix rel_pose_rotation_precision_matrix;
  SparseMatrix pose_prior_precision_matrix;
  SparseMatrix landmark_prior_precision_matrix;

  CoraDataSubmatrices() = default;
};

struct PreconditionerMatrices {
  CholFactorPtrVector block_chol_factor_ptrs_;
  DiagonalMatrix jacobi_preconditioner_;
  SparseMatrix block_jacobi_preconditioner_;
};

struct Manifolds {
  StiefelProduct stiefel_prod_manifold_;
  ObliqueManifold oblique_manifold_;

  void incrementRank() {
    stiefel_prod_manifold_.incrementRank();
    oblique_manifold_.incrementRank();
  }
};

class Problem {
private:
  /** dimension of the pose and landmark variables e.g., SO(dim_) */
  const int64_t dim_;

  /** rank of the relaxation e.g., the latent embedding space of Stiefel
   * manifold */
  int64_t relaxation_rank_;

  // maps from pose symbol to pose index (e.g., x1 -> 0, x2 -> 1, etc.)
  std::map<Symbol, int> pose_symbol_idxs_;

  // maps from landmark symbol to landmark index (e.g., l1 -> 0, l2 -> 1, etc.)
  std::map<Symbol, int> landmark_symbol_idxs_;

  // the range measurements that are used to construct the problem
  std::vector<RangeMeasurement> range_measurements_;

  // the relative pose measurements that are used to construct the problem
  std::vector<RelativePoseMeasurement> rel_pose_measurements_;

  // the pose priors that are used to construct the problem
  std::vector<PosePrior> pose_priors_;

  // the landmark priors that are used to construct the problem
  std::vector<LandmarkPrior> landmark_priors_;

  // the non-Euclidean manifolds that make up the problem
  Manifolds manifolds_;

  // the formulation of the problem (e.g., translation-explicit vs -implicit)
  Formulation formulation_;

  // the preconditioner to use for solving the problem
  Preconditioner preconditioner_;

  // the preconditioner matrices
  PreconditionerMatrices preconditioner_matrices_;

  // the submatrices that are used to construct the data matrix
  CoraDataSubmatrices data_submatrices_;

  // a flag to check if any data has been modified since last call to
  // updateProblemData()
  bool problem_data_up_to_date_ = false;
  void checkUpToDate() const {
    if (!problem_data_up_to_date_) {
      throw std::runtime_error(
          "The data matrix must be constructed before the objective function "
          "can be evaluated. This error may be due to the fact that data has "
          "been modified since the last call to updateProblemData()");
    }
  }

  // function to fill all of the submatrices built from range measurements.
  // Should only be called from updateProblemData()
  void fillRangeSubmatrices();

  // function to fill all of the submatrices built from relative pose
  // measurements. Should only be called from updateProblemData()
  void fillRelPoseSubmatrices();

  // function to construct the rotation connection Laplacian. Should only be
  // called from fillRelPoseSubmatrices()
  void fillRotConnLaplacian();

  /**
   * @brief function to fill in the full data matrix from the *already computed*
   * submatrices. Should only be called from updateProblemData()
   *
   * The data matrix Q is a symmetric block matrix of the form:
   *            dn                r                 n + l
   * __________________________________________________________________
   * |     Lrho + Sigma   |       0         |   T^T * Omega_t * A_t   |  dn
   * |        ****        | Omega_r * D * D |     D * Omega_r * A_r   |  r
   * |        ****        |     ****        |       L_r + L_t         |  n + l
   * _________________________________________________________________
   *
   * We will alternatively write this as:
   *
   * __________________________________________________________________
   * |         Q11        |        0         |         Q13             |  dn
   * |        ****        |       Q22        |         Q23             |  r
   * |        ****        |       ****       |         Q33             |  n + l
   * _________________________________________________________________
   *
   *
   *  where we ignore the lower-triangular section due to symmetry
   *
   *  where we define the matrices as follows:
   *
   *  Lrho = rotation connection Laplacian
   *  Sigma = T^T Omega_t T
   *  T = rel_pose_translation_data_matrix
   *  Omega_t = rel_pose_translation_precision_matrix
   *  Omega_r = range_precision_matrix
   *  D = range_dist_matrix
   *  L_r = A_r^T * Omega_r * A_r
   *  L_t = A_t^T * Omega_t * A_t
   *  A_r = range_incidence_matrix
   *  A_t = rel_pose_incidence_matrix
   *
   */
  void fillDataMatrix();

  void updatePreconditioner();

  Matrix dataMatrixProduct(const Matrix &Y) const;

  Index getRotationIdx(const Symbol &pose_symbol) const;
  Index getRangeIdx(const SymbolPair &range_symbol_pair) const;
  Index getTranslationIdx(const Symbol &trans_symbol) const;

public:
  Problem(int64_t dim, int64_t relaxation_rank,
          Formulation formulation = Formulation::Explicit,
          Preconditioner preconditioner = Preconditioner::BlockCholesky)
      : dim_(dim),
        relaxation_rank_(relaxation_rank),
        formulation_(formulation),
        preconditioner_(preconditioner),
        manifolds_(Manifolds()) {
    // relaxation rank must be >= dim
    assert(relaxation_rank >= dim);
    manifolds_.oblique_manifold_ = ObliqueManifold(relaxation_rank, 0);
    manifolds_.stiefel_prod_manifold_ = StiefelProduct(dim, relaxation_rank, 0);
  }

  ~Problem() = default;

  void addPoseVariable(const Symbol &pose_id);
  void addPoseVariable(std::string pose_id) {
    addPoseVariable(Symbol(std::move(pose_id)));
  }
  void addPoseVariable(Key pose_key) { addPoseVariable(Symbol(pose_key)); }

  void addLandmarkVariable(const Symbol &landmark_id);
  void addLandmarkVariable(std::string landmark_id) {
    addLandmarkVariable(Symbol(std::move(landmark_id)));
  }
  void addLandmarkVariable(Key landmark_key) {
    addLandmarkVariable(Symbol(landmark_key));
  }

  void addRangeMeasurement(const RangeMeasurement &range_measurement);
  void
  addRelativePoseMeasurement(const RelativePoseMeasurement &rel_pose_measure);
  void addPosePrior(const PosePrior &pose_prior);
  void addLandmarkPrior(const LandmarkPrior &landmark_prior);

  void printProblem() const;

  // function to get read-only references to the data submatrices
  const CoraDataSubmatrices &getDataSubmatrices() {
    if (!problem_data_up_to_date_) {
      updateProblemData();
    }
    return data_submatrices_;
  }

  // the data matrix that is used to construct the problem
  SparseMatrix data_matrix_;

  void updateProblemData();
  SparseMatrix getDataMatrix();

  // the full size of the full (explicit problem) data matrix
  size_t getDataMatrixSize() const;

  size_t numPoses() const { return pose_symbol_idxs_.size(); }
  size_t numLandmarks() const { return landmark_symbol_idxs_.size(); }
  size_t numRangeMeasurements() const { return range_measurements_.size(); }
  size_t numTranslationalStates() const { return numPoses() + numLandmarks(); }

  /*****  Riemannian optimization functions  *******/

  Matrix getRandomInitialGuess() const;
  void incrementRank() {
    relaxation_rank_++;
    manifolds_.incrementRank();
  }

  Scalar evaluateObjective(const Matrix &Y) const;
  Matrix Euclidean_gradient(const Matrix &Y) const;
  Matrix Riemannian_gradient(const Matrix &Y) const;
  Matrix Riemannian_gradient(const Matrix &Y, const Matrix &NablaF_Y) const;
  Matrix Riemannian_Hessian_vector_product(const Matrix &Y,
                                           const Matrix &NablaF_Y,
                                           const Matrix &Ydot) const;
  Matrix tangent_space_projection(const Matrix &Y, const Matrix &Ydot) const;
  Matrix precondition(const Matrix &V) const;
  Matrix projectToManifold(const Matrix &A) const;
  Matrix retract(const Matrix &Y, const Matrix &V) const;
}; // class Problem

} // namespace CORA
