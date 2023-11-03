/**
 * @file CORA_problem.cpp
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <CORA/CORA_problem.h>

namespace CORA {
void Problem::addPoseVariable(Symbol pose_id) {
  pose_symbol_idxs_.insert(std::make_pair(pose_id, pose_symbol_idxs_.size()));
}

void Problem::addLandmarkVariable(Symbol landmark_id) {
  landmark_symbol_idxs_.insert(
      std::make_pair(landmark_id, landmark_symbol_idxs_.size()));
}

void Problem::addRangeMeasurement(RangeMeasurement range_measurement) {
  range_measurement_symbol_idxs_.insert(
      std::make_pair(range_measurement.getSymbolPair(),
                     range_measurement_symbol_idxs_.size()));
  range_measurements_.push_back(range_measurement);

  // mark range measurements as not up to date
  set_range_submatrices_up_to_date(false);
}

void Problem::addRelativePoseMeasurement(
    RelativePoseMeasurement rel_pose_measure) {
  rel_pose_measurements_.push_back(rel_pose_measure);

  // mark relative pose measurements as not up to date
  set_rel_pose_submatrices_up_to_date(false);
}

void Problem::addPosePrior(PosePrior pose_prior) {
  pose_priors_.push_back(pose_prior);
}

void Problem::addLandmarkPrior(LandmarkPrior landmark_prior) {
  landmark_priors_.push_back(landmark_prior);
}

void Problem::fillRangeSubmatrices() {
  // need to account for the fact that the indices will be offset by the
  // dimension of the rotation and the range variables that precede the
  // translations
  size_t translation_offset = numPoses() * dim_ + numRangeMeasurements();

  size_t num_translations = numTranslationalStates();
  size_t num_range_measurements = range_measurements_.size();

  // initialize the submatrices to the correct sizes
  data_submatrices_.range_incidence_matrix =
      SparseMatrix(num_range_measurements, num_translations);
  data_submatrices_.range_dist_matrix =
      SparseMatrix(num_range_measurements, num_range_measurements);
  data_submatrices_.range_precision_matrix =
      SparseMatrix(num_range_measurements, num_range_measurements);

  // for diagonal matrices, get the diagonal vector and set the values
  for (int measure_idx = 0; measure_idx < range_measurements_.size();
       measure_idx++) {
    RangeMeasurement measure = range_measurements_[measure_idx];

    // update the diagonal matrices
    data_submatrices_.range_dist_matrix.insert(measure_idx, measure_idx) =
        measure.r;
    data_submatrices_.range_precision_matrix.insert(measure_idx, measure_idx) =
        measure.getPrecision();

    // update the incidence matrix
    int id1 = getTranslationIdxInExplicitDataMatrix(measure.first_id) -
              translation_offset;
    int id2 = getTranslationIdxInExplicitDataMatrix(measure.second_id) -
              translation_offset;
    data_submatrices_.range_incidence_matrix.insert(measure_idx, id1) = -1.0;
    data_submatrices_.range_incidence_matrix.insert(measure_idx, id2) = 1.0;
  }

  // mark range measurements as updated
  set_range_submatrices_up_to_date(true);
}

void Problem::fillRelPoseSubmatrices() {
  fillRotConnLaplacian();

  // initialize the submatrices to the correct sizes
  data_submatrices_.rel_pose_incidence_matrix =
      SparseMatrix(rel_pose_measurements_.size(), numTranslationalStates());
  data_submatrices_.rel_pose_translation_data_matrix =
      SparseMatrix(rel_pose_measurements_.size(), dim_ * numPoses());
  data_submatrices_.rel_pose_translation_precision_matrix = SparseMatrix(
      rel_pose_measurements_.size(), rel_pose_measurements_.size());
  data_submatrices_.rel_pose_rotation_precision_matrix = SparseMatrix(
      rel_pose_measurements_.size(), rel_pose_measurements_.size());

  // need to account for the fact that the indices will be offset by the
  // dimension of the rotation and the range variables that precede the
  // translations
  int translation_offset = numPoses() * dim_ + numRangeMeasurements();

  // for diagonal matrices, get the diagonal vector and set the values
  for (int measure_idx = 0; measure_idx < rel_pose_measurements_.size();
       measure_idx++) {
    RelativePoseMeasurement rpm = rel_pose_measurements_[measure_idx];

    // fill in precision matrices
    data_submatrices_.rel_pose_translation_precision_matrix.insert(
        measure_idx, measure_idx) = rpm.getTransPrecision();
    data_submatrices_.rel_pose_rotation_precision_matrix.insert(
        measure_idx, measure_idx) = rpm.getRotPrecision();

    // fill in incidence matrix
    int id1 = getTranslationIdxInExplicitDataMatrix(rpm.first_id) -
              translation_offset;
    int id2 = getTranslationIdxInExplicitDataMatrix(rpm.second_id) -
              translation_offset;
    data_submatrices_.rel_pose_incidence_matrix.insert(measure_idx, id1) = -1.0;
    data_submatrices_.rel_pose_incidence_matrix.insert(measure_idx, id2) = 1.0;

    // fill in translation data matrix where the id1-th (1xdim_) block is -rpm.t
    // and all other blocks are 0
    for (int k = 0; k < dim_; k++) {
      data_submatrices_.rel_pose_translation_data_matrix.insert(
          measure_idx, id1 * dim_ + k) = -rpm.t(k);
    }
  }

  // mark relative pose measurements as updated
  set_rel_pose_submatrices_up_to_date(true);
}

void Problem::fillRotConnLaplacian() {
  size_t d = dim_;

  // Each measurement contributes 2*d elements along the diagonal of the
  // connection Laplacian, and 2*d^2 elements on a pair of symmetric
  // off-diagonal blocks

  size_t measurement_stride = 2 * (d + d * d);

  std::vector<Eigen::Triplet<Scalar>> triplets;
  triplets.reserve(measurement_stride * rel_pose_measurements_.size());

  size_t i, j;
  for (const RelativePoseMeasurement &measurement : rel_pose_measurements_) {
    i = getRotationIdx(measurement.first_id);
    j = getRotationIdx(measurement.second_id);

    // Elements of ith block-diagonal
    for (size_t k = 0; k < d; k++)
      triplets.emplace_back(d * i + k, d * i + k,
                            measurement.getRotPrecision());

    // Elements of jth block-diagonal
    for (size_t k = 0; k < d; k++)
      triplets.emplace_back(d * j + k, d * j + k,
                            measurement.getRotPrecision());

    // Elements of ij block
    for (size_t r = 0; r < d; r++)
      for (size_t c = 0; c < d; c++)
        triplets.emplace_back(i * d + r, j * d + c,
                              -measurement.getRotPrecision() *
                                  measurement.R(r, c));

    // Elements of ji block
    for (size_t r = 0; r < d; r++)
      for (size_t c = 0; c < d; c++)
        triplets.emplace_back(j * d + r, i * d + c,
                              -measurement.getRotPrecision() *
                                  measurement.R(c, r));
  }

  size_t num_poses = numPoses();

  // Construct and return a sparse matrix from these triplets
  data_submatrices_.rotation_conn_laplacian =
      SparseMatrix(d * num_poses, d * num_poses);
  data_submatrices_.rotation_conn_laplacian.setFromTriplets(triplets.begin(),
                                                            triplets.end());
}

template <typename... Matrices>
DiagonalMatrix diagMatrixMult(const DiagonalMatrix &first,
                              const Matrices &...matrices) {
  // Check dimensions
  const int numRows = first.rows();
  auto checkDimensions = [numRows](const DiagonalMatrix &mat) {
    assert(mat.rows() == numRows);
  };
  (checkDimensions(matrices), ...);

  DiagonalMatrix result = first; // Start with the first matrix
  auto multiplyIntoResult = [&result](const DiagonalMatrix &mat) {
    for (int i = 0; i < result.rows(); i++) {
      result.diagonal()(i) *= mat.diagonal()(i);
    }
  };
  (multiplyIntoResult(matrices), ...);

  return result;
}

void Problem::printProblem() const {
  // print out all of the pose variables
  if (numPoses()) {
    std::cout << "Pose variables:" << std::endl;
    for (auto pose_symbol_idx_pair : pose_symbol_idxs_) {
      std::cout << pose_symbol_idx_pair.first.string() << " -> "
                << pose_symbol_idx_pair.second << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No pose variables" << std::endl;
  }

  // print out all of the landmark variables
  if (numLandmarks()) {
    std::cout << "\nLandmark variables:" << std::endl;
    for (auto landmark_symbol_idx_pair : landmark_symbol_idxs_) {
      std::cout << landmark_symbol_idx_pair.first.string() << " -> "
                << landmark_symbol_idx_pair.second << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No landmark variables" << std::endl;
  }

  // print out all of the range measurements
  if (numRangeMeasurements()) {
    std::cout << "\nRange measurements:" << std::endl;
    for (auto range_measurement : range_measurements_) {
      std::cout << range_measurement.first_id.string() << " -> "
                << range_measurement.second_id.string() << " "
                << range_measurement.r << " " << range_measurement.cov
                << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No range measurements" << std::endl;
  }

  // print out all of the relative pose measurements
  if (rel_pose_measurements_.size()) {
    std::cout << "\nRelative pose measurements:" << std::endl;
    for (auto rel_pose_measurement : rel_pose_measurements_) {
      std::cout << rel_pose_measurement.first_id.string() << " -> "
                << rel_pose_measurement.second_id.string() << std::endl;
      std::cout << "Rot:\n" << rel_pose_measurement.R << std::endl;
      std::cout << "Trans: " << rel_pose_measurement.t.transpose() << std::endl;
      std::cout << "Cov:\n" << rel_pose_measurement.cov << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No relative pose measurements" << std::endl;
  }

  // print out all of the pose priors
  if (pose_priors_.size()) {
    std::cout << "\nPose priors:" << std::endl;
    for (auto pose_prior : pose_priors_) {
      std::cout << pose_prior.id.string() << std::endl;
      std::cout << pose_prior.R << std::endl;
      std::cout << pose_prior.t << std::endl;
      std::cout << pose_prior.cov << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No pose priors" << std::endl;
  }

  // print out all of the landmark priors
  if (landmark_priors_.size()) {
    std::cout << "\nLandmark priors:" << std::endl;
    for (auto landmark_prior : landmark_priors_) {
      std::cout << landmark_prior.id.string() << std::endl;
      std::cout << landmark_prior.p << std::endl;
      std::cout << landmark_prior.cov << std::endl;
      std::cout << std::endl;
    }
  } else {
    std::cout << "No landmark priors" << std::endl;
  }
}

SparseMatrix Problem::getDataMatrix() {
  if (data_matrix_.nonZeros() == 0 || !data_matrix_up_to_date_) {
    constructDataMatrix();
  }
  return data_matrix_;
}

void Problem::constructDataMatrix() {
  // fill the submatrices
  if (!range_submatrices_up_to_date_) {
    fillRangeSubmatrices();
  }
  if (!rel_pose_submatrices_up_to_date_) {
    fillRelPoseSubmatrices();
  }

  size_t data_matrix_size = getDataMatrixSize();
  data_matrix_ = SparseMatrix(data_matrix_size, data_matrix_size);

  size_t n = numPoses();
  size_t dn = dim_ * n;
  size_t r = numRangeMeasurements();
  size_t l = numLandmarks();

  /**
   * @brief From here we form the subblocks of the data matrix Q
   * and then assemble them into the full data matrix by adding the
   * triplets together.
   */

  // Q11
  // upper-left dn x dn block is:
  // rotation connection Laplacian + T^T * Omega_t * T
  SparseMatrix Q11 =
      data_submatrices_.rotation_conn_laplacian +
      data_submatrices_.rel_pose_translation_data_matrix.transpose() *
          data_submatrices_.rel_pose_translation_precision_matrix *
          data_submatrices_.rel_pose_translation_data_matrix;

  // Q13
  // upper-right dn x (n+l) block is: T^T * Omega_t * A_t
  SparseMatrix Q13 =
      data_submatrices_.rel_pose_translation_data_matrix.transpose() *
      data_submatrices_.rel_pose_translation_precision_matrix *
      data_submatrices_.rel_pose_incidence_matrix;

  // Q22
  // the next (r x r) block on the diagonal is: Omega_r * D * D
  SparseMatrix OmegaRD = data_submatrices_.range_precision_matrix *
                         data_submatrices_.range_dist_matrix;
  SparseMatrix Q22 = OmegaRD * data_submatrices_.range_dist_matrix;

  // Q23
  // the next (r x (n+l)) block to the right of the (r x r) block on the
  // diagonal is D * Omega_r * A_r
  SparseMatrix Q23 = OmegaRD * data_submatrices_.range_incidence_matrix;

  // Q33
  // the bottom-right block on the diagonal is: L_r + L_t
  SparseMatrix Q33 = (data_submatrices_.rel_pose_incidence_matrix.transpose() *
                      data_submatrices_.rel_pose_translation_precision_matrix *
                      data_submatrices_.rel_pose_incidence_matrix) +
                     (data_submatrices_.range_incidence_matrix.transpose() *
                      data_submatrices_.range_precision_matrix *
                      data_submatrices_.range_incidence_matrix);

  /**
   * @brief Now we join all of the triplets together, properly offsetting the
   * indices of the triplets to account for the fact that the submatrices are
   * located in different parts of the data matrix.
   */
  std::vector<Eigen::Triplet<Scalar>> combined_triplets;
  combined_triplets.reserve(Q11.nonZeros() + Q13.nonZeros() + Q22.nonZeros() +
                            Q23.nonZeros() + Q33.nonZeros());

  // lambda function to add triplets to the combined triplets vector
  auto addTriplets = [&combined_triplets](const SparseMatrix &matrix,
                                          size_t row_offset,
                                          size_t col_offset) {
    for (int k = 0; k < matrix.outerSize(); ++k) {
      for (SparseMatrix::InnerIterator it(matrix, k); it; ++it) {
        combined_triplets.emplace_back(it.row() + row_offset,
                                       it.col() + col_offset, it.value());
      }
    }
  };

  // Q11, Q13, Q22, Q23, Q33
  addTriplets(Q11, 0, 0);
  addTriplets(Q13, 0, dn + r);
  addTriplets(Q22, dn, dn);
  addTriplets(Q23, dn, dn + r);
  addTriplets(Q33, dn + r, dn + r);

  // also add Q13 and Q23 transposed to the triplets
  addTriplets(Q13.transpose(), dn + r, 0);
  addTriplets(Q23.transpose(), dn + r, dn);

  // construct the data matrix
  data_matrix_.setFromTriplets(combined_triplets.begin(),
                               combined_triplets.end());

  // mark the data matrix as up to date
  data_matrix_up_to_date_ = true;
}

size_t Problem::getDataMatrixSize() const {
  if (formulation_ == Formulation::Explicit) {
    return (numPoses() * (dim_ + 1)) + numLandmarks() + numRangeMeasurements();
  } else if (formulation_ == Formulation::Implicit) {
    return (numPoses() * dim_) + numRangeMeasurements();
  } else {
    throw std::invalid_argument("Unknown formulation");
  }
}

size_t Problem::getRotationIdx(Symbol pose_symbol) const {
  auto rotation_search_it = pose_symbol_idxs_.find(pose_symbol);
  if (rotation_search_it != pose_symbol_idxs_.end()) {
    return rotation_search_it->second;
  }

  // if we get here, we didn't find the pose symbol
  throw std::invalid_argument("Unknown pose symbol");
}

size_t
Problem::getRangeIdxInExplicitDataMatrix(SymbolPair range_symbol_pair) const {
  // all range measurements come after the rotations
  size_t rot_offset = numPoses() * dim_;

  // search for the range symbol
  auto range_search_it = range_measurement_symbol_idxs_.find(range_symbol_pair);

  if (range_search_it != range_measurement_symbol_idxs_.end()) {
    return range_search_it->second + rot_offset;
  }

  // if we get here, we didn't find the range symbol
  throw std::invalid_argument("Unknown range symbol");
}

size_t
Problem::getTranslationIdxInExplicitDataMatrix(Symbol trans_symbol) const {
  // all translations come after the rotations and range measurement variables
  // (unit spheres) so we need to offset by the dimension of the rotations and
  // the number of range measurements
  size_t idx_offset = numPoses() * dim_ + numRangeMeasurements();

  // is a pose translation
  auto pose_search_it = pose_symbol_idxs_.find(trans_symbol);
  if (pose_search_it != pose_symbol_idxs_.end()) {
    return pose_search_it->second + idx_offset;
  }

  // is a landmark translation
  auto landmark_search_it = landmark_symbol_idxs_.find(trans_symbol);
  if (landmark_search_it != landmark_symbol_idxs_.end()) {
    // need to offset by the number of poses because the landmark variables
    // come after the pose translations
    return landmark_search_it->second + idx_offset + numPoses();
  }

  // if we get here, we didn't find the translation symbol
  throw std::invalid_argument("Unknown translation symbol");
}
}; // namespace CORA
