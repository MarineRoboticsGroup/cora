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
  void setRank(int r) {
    stiefel_prod_manifold_.setRank(r);
    oblique_manifold_.setRank(r);
  }
};

class Problem {
private:
  /** dimension of the pose and landmark variables e.g., SO(dim_) */
  const int dim_;

  /** whether to pin the last translation when applying preconditioner */
  const bool pin_last_translation_ = true;

  /** rank of the relaxation e.g., the latent embedding space of Stiefel
   * manifold */
  int relaxation_rank_;

  // maps from pose symbol to pose index (e.g., x1 -> 0, x2 -> 1, etc.)
  std::map<Symbol, int> pose_symbol_idxs_;

  // maps from landmark symbol to landmark index (e.g., l1 -> 0, l2 -> 1, etc.)
  std::map<Symbol, int> landmark_symbol_idxs_;

  // the range measurements that are used to construct the problem
  std::vector<RangeMeasurement> range_measurements_;

  // the relative pose measurements that are used to construct the problem
  std::vector<RelativePoseMeasurement> rel_pose_pose_measurements_;

  // the pose-landmark measurements that are used to construct the problem
  std::vector<RelativePoseLandmarkMeasurement> rel_pose_landmark_measurements_;

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

  void fillImplicitFormulationMatrices();

  void updatePreconditioner();

  Matrix dataMatrixProduct(const Matrix &Y) const;

public:
  Problem(int dim, int relaxation_rank,
          Formulation formulation = Formulation::Explicit,
          Preconditioner preconditioner = Preconditioner::RegularizedCholesky)
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
  void addRelativePoseLandmarkMeasurement(
      const RelativePoseLandmarkMeasurement &rel_pose_landmark_measure);
  void addPosePrior(const PosePrior &pose_prior);
  void addLandmarkPrior(const LandmarkPrior &landmark_prior);
  inline int getNumPosePriors() const { return pose_priors_.size(); }
  inline int getNumLandmarkPriors() const { return landmark_priors_.size(); }

  // Indexing helpers
  Index getRotationIdx(const Symbol &pose_symbol) const;
  Index getRangeIdx(const SymbolPair &range_symbol_pair) const;
  Index getTranslationIdx(const Symbol &trans_symbol) const;

  void printProblem() const;

  // function to get read-only references to the data submatrices
  const CoraDataSubmatrices &getDataSubmatrices() {
    if (!problem_data_up_to_date_) {
      updateProblemData();
    }
    return data_submatrices_;
  }

  // get pose symbols that start with a given character
  std::vector<Symbol> getPoseSymbols(unsigned char chr) const;

  // Get copies of pose and landmark symbol maps
  std::map<Symbol, int> getPoseSymbolMap() const { return pose_symbol_idxs_; }
  std::map<Symbol, int> getLandmarkSymbolMap() const {
    return landmark_symbol_idxs_;
  }
  // Get copy of range measurements
  std::vector<RangeMeasurement> getRangeMeasurements() const {
    return range_measurements_;
  }

  // Get copy of relative pose measurements
  inline std::vector<RelativePoseMeasurement> getRPMs() const {
    return rel_pose_pose_measurements_;
  }

  // the data matrix that is used to construct the problem
  SparseMatrix data_matrix_;

  // the elements that we will use to compute matrix products in the implicit
  // form
  SparseMatrix Qmain_;
  SparseMatrix TransOffDiagRed_;
  CholFactorPtr LtransCholRed_;

  // the most recent minimum eigenvectors computed by LOBPCG for certification
  CertResults last_cert_results_;
  bool last_cert_results_valid_ = false;

  void updateProblemData();
  SparseMatrix getDataMatrix();

  // the full size of the full (explicit problem) data matrix
  int getDataMatrixSize() const;

  int getExpectedVariableSize() const;

  Formulation getFormulation() const { return formulation_; }
  inline int dim() const { return dim_; }
  inline int numPoses() const {
    return static_cast<int>(pose_symbol_idxs_.size());
  }
  inline int numPosePoseMeasurements() const {
    return static_cast<int>(rel_pose_pose_measurements_.size());
  }
  inline int numPoseLandmarkMeasurements() const {
    return static_cast<int>(rel_pose_landmark_measurements_.size());
  }
  inline int numLandmarks() const {
    return static_cast<int>(landmark_symbol_idxs_.size());
  }
  inline int numRangeMeasurements() const {
    return static_cast<int>(range_measurements_.size());
  }
  inline int numTranslationalStates() const {
    return numPoses() + numLandmarks();
  }

  // Offset calculations
  inline int numPosesDim() const { return dim() * numPoses(); }
  inline int rotAndRangeMatrixSize() const {
    return numPosesDim() + numRangeMeasurements();
  }

  /*****  Riemannian optimization functions  *******/

  inline size_t getRelaxationRank() const { return relaxation_rank_; }
  Matrix getRandomInitialGuess() const;
  void incrementRank() {
    relaxation_rank_++;
    manifolds_.incrementRank();
  }
  void setRank(int r) {
    relaxation_rank_ = r;
    manifolds_.setRank(r);
  }
  void setPreconditioner(Preconditioner preconditioner) {
    preconditioner_ = preconditioner;
  }
  void setFormulation(Formulation formulation) { formulation_ = formulation; }

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

  /********** Certification **************/

  using LambdaBlocks = std::pair<Matrix, Vector>;

  /**
   * @brief Check if a solution is certified. If not, compute a direction of
   * negative curvature and its associated Rayleigh quotient.
   *
   * @param Y the rank-r solution to certify
   * @param eta the regularization parameter (tolerance on PSDness of the
   * certificate matrix)
   * @param nx the block size to use in LOBPCG
   * @param max_LOBPCG_iters the maximum number of LOBPCG iterations to use
   * @param max_fill_factor the maximum fill factor to use in the incomplete
   * factorization-based preconditioner
   * @param drop_tol the drop tolerance to use in the incomplete
   * factorization-based preconditioner
   * @return CertResults
   */
  CertResults certify_solution(const Matrix &Y, Scalar eta, size_t nx,
                               const Matrix &eigvec_bootstrap,
                               size_t max_LOBPCG_iters = 500,
                               Scalar max_fill_factor = 3,
                               Scalar drop_tol = 1e-3) const;

  /** Given the d x dn block matrix containing the diagonal blocks of Lambda,
   * this function computes and returns the matrix Lambda itself */
  SparseMatrix
  compute_Lambda_from_Lambda_blocks(const LambdaBlocks &Lambda_blocks) const;

  /** Given a critical point Y of the rank-r relaxation, this function computes
   * and returns a d x dn matrix comprised of d x d block elements of the
   * associated block-diagonal Lagrange multiplier matrix associated with the
   * orthonormality constraints on the generalized orientations of the poses
   * (cf. eq. (119) in the SE-Sync tech report) */
  LambdaBlocks compute_Lambda_blocks(const Matrix &Y) const;

  /**
   * @brief Get the certificate matrix as Q - Lambda. If this matrix is PSD,
   * then the solution is certified.
   *
   * @param Lambda the Lagrange multiplier matrix
   * @return SparseMatrix
   */
  SparseMatrix get_certificate_matrix(const Matrix &Y) const;

  /************** Utilities **********************/

  /**
   * @brief Given an estimate Y with d columns (i.e., not relaxed), this
   * function aligns the estimate to the origin by rotating the first dxd block
   * to the identity matrix and offsetting the translational states such that
   * the first translational variable is at the origin.
   *
   * @param Y the estimate to align
   * @return Matrix the aligned estimate
   */
  Matrix alignEstimateToOrigin(const Matrix &Y) const;
}; // class Problem

} // namespace CORA
