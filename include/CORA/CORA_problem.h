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

#include <CORA/CORA_types.h>
#include <CORA/Measurements.h>
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

  CoraDataSubmatrices() {}
};

class Problem {
private:
  /** dimension of the pose and landmark variables e.g., SO(dim_) */
  const size_t dim_;

  /** rank of the relaxation e.g., the latent embedding space of Stiefel
   * manifold */
  size_t relaxation_rank_;

  // maps from pose symbol to pose index (e.g., x1 -> 0, x2 -> 1, etc.)
  std::map<Symbol, int> pose_symbol_idxs_;

  // maps from landmark symbol to landmark index (e.g., l1 -> 0, l2 -> 1, etc.)
  std::map<Symbol, int> landmark_symbol_idxs_;

  /** maps from range measurement symbol pair to range measurement index
   * (e.g., r1 -> 0, r2 -> 1, etc.) */
  std::map<SymbolPair, int> range_measurement_symbol_idxs_;

  // the range measurements that are used to construct the problem
  std::vector<RangeMeasurement> range_measurements_;

  // the relative pose measurements that are used to construct the problem
  std::vector<RelativePoseMeasurement> rel_pose_measurements_;

  // the pose priors that are used to construct the problem
  std::vector<PosePrior> pose_priors_;

  // the landmark priors that are used to construct the problem
  std::vector<LandmarkPrior> landmark_priors_;

  // the formulation of the problem (e.g., translation-explicit vs -implicit)
  Formulation formulation_;

  // the submatrices that are used to construct the data matrix
  CoraDataSubmatrices data_submatrices_;

  // a flag to check if range data has been modified since last call to
  // fillRangeSubmatrices()
  bool range_submatrices_up_to_date_ = false;

  void set_range_submatrices_up_to_date(bool up_to_date) {
    range_submatrices_up_to_date_ = up_to_date;
    if (!up_to_date) {
      data_matrix_up_to_date_ = false;
    }
  }

  // a flag to check if relative pose data has been modified since last call to
  // fillRelPoseSubmatrices()
  bool rel_pose_submatrices_up_to_date_ = false;

  void set_rel_pose_submatrices_up_to_date(bool up_to_date) {
    rel_pose_submatrices_up_to_date_ = up_to_date;
    if (!up_to_date) {
      data_matrix_up_to_date_ = false;
    }
  }

  // a flag to check if any data has been modified since last call to
  // constructDataMatrix()
  bool data_matrix_up_to_date_ = false;

  // the full size of the data matrix
  size_t getDataMatrixSize() const;

  // function to fill all of the submatrices built from range measurements.
  // Should only be called from constructDataMatrix()
  void fillRangeSubmatrices();

  // function to fill all of the submatrices built from relative pose
  // measurements. Should only be called from constructDataMatrix()
  void fillRelPoseSubmatrices();

  // function to construct the rotation connection Laplacian. Should only be
  // called from fillRelPoseSubmatrices()
  void fillRotConnLaplacian();

  template <typename... Matrices>
  DiagonalMatrix diagMatrixMult(const DiagonalMatrix &first,
                                const Matrices &...matrices);

  size_t getRotationIdx(Symbol pose_symbol) const;
  size_t getRangeIdxInExplicitDataMatrix(SymbolPair range_symbol_pair) const;
  size_t getTranslationIdxInExplicitDataMatrix(Symbol trans_symbol) const;

public:
  Problem(size_t dim, size_t relaxation_rank,
          Formulation formulation = Formulation::Explicit)
      : dim_(dim),
        relaxation_rank_(relaxation_rank),
        formulation_(formulation) {
    // relaxation rank must be >= dim
    assert(relaxation_rank >= dim);
  }

  ~Problem() {}

  void addPoseVariable(Symbol pose_id);
  void addPoseVariable(std::string pose_id) {
    addPoseVariable(Symbol(pose_id));
  }
  void addPoseVariable(Key pose_key) { addPoseVariable(Symbol(pose_key)); }

  void addLandmarkVariable(Symbol landmark_id);
  void addLandmarkVariable(std::string landmark_id) {
    addLandmarkVariable(Symbol(landmark_id));
  }
  void addLandmarkVariable(Key landmark_key) {
    addLandmarkVariable(Symbol(landmark_key));
  }

  void addRangeMeasurement(RangeMeasurement range_measurement);
  void addRelativePoseMeasurement(RelativePoseMeasurement rel_pose_measure);
  void addPosePrior(PosePrior pose_prior);
  void addLandmarkPrior(LandmarkPrior landmark_prior);

  void printProblem() const;

  // function to get read-only references to the data submatrices
  const CoraDataSubmatrices &getDataSubmatrices() {
    if (!data_matrix_up_to_date_) {
      constructDataMatrix();
    }
    return data_submatrices_;
  }

  // the data matrix that is used to construct the problem
  SparseMatrix data_matrix_;

  /**
   * @brief The data matrix Q is a symmetric block matrix of the form:
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
  void constructDataMatrix();
  SparseMatrix getDataMatrix();

  size_t numPoses() const { return pose_symbol_idxs_.size(); }
  size_t numLandmarks() const { return landmark_symbol_idxs_.size(); }
  size_t numRangeMeasurements() const {
    return range_measurement_symbol_idxs_.size();
  }
  size_t numTranslationalStates() const { return numPoses() + numLandmarks(); }
}; // class Problem

} // namespace CORA
