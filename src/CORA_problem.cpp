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
}

void Problem::addRelativePoseMeasurement(
    RelativePoseMeasurement rel_pose_measure) {
  rel_pose_measurements_.push_back(rel_pose_measure);
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
  data_submatrices_.range_dist_matrix = DiagonalMatrix(num_range_measurements);
  data_submatrices_.range_precision_matrix =
      DiagonalMatrix(num_range_measurements);

  // for diagonal matrices, get the diagonal vector and set the values
  DiagonalMatrix::DiagonalVectorType &dist_diagonal =
      data_submatrices_.range_dist_matrix.diagonal();
  DiagonalMatrix::DiagonalVectorType &precision_diagonal =
      data_submatrices_.range_precision_matrix.diagonal();

  for (int measure_idx = 0; measure_idx < range_measurements_.size();
       measure_idx++) {
    RangeMeasurement measure = range_measurements_[measure_idx];

    // update the diagonal matrices
    dist_diagonal(measure_idx) = measure.r;
    precision_diagonal(measure_idx) = measure.getPrecision();

    // update the incidence matrix
    int id1 = getTranslationIdxInExplicitDataMatrix(measure.first_id) -
              translation_offset;
    int id2 = getTranslationIdxInExplicitDataMatrix(measure.second_id) -
              translation_offset;
    data_submatrices_.range_incidence_matrix.insert(measure_idx, id1) = 1.0;
    data_submatrices_.range_incidence_matrix.insert(measure_idx, id2) = -1.0;
  }
}

void Problem::fillRelPoseSubmatrices() {
  // initialize the submatrices to the correct sizes
  data_submatrices_.rel_pose_incidence_matrix =
      SparseMatrix(rel_pose_measurements_.size(), numTranslationalStates());
  data_submatrices_.rel_pose_translation_data_matrix =
      SparseMatrix(rel_pose_measurements_.size(), dim_ * numPoses());
  data_submatrices_.rel_pose_translation_precision_matrix =
      DiagonalMatrix(rel_pose_measurements_.size());
  data_submatrices_.rel_pose_rotation_precision_matrix =
      DiagonalMatrix(rel_pose_measurements_.size());

  // need to account for the fact that the indices will be offset by the
  // dimension of the rotation and the range variables that precede the
  // translations
  int translation_offset = numPoses() * dim_ + numRangeMeasurements();

  // for diagonal matrices, get the diagonal vector and set the values
  DiagonalMatrix::DiagonalVectorType &translation_precision_diagonal =
      data_submatrices_.rel_pose_translation_precision_matrix.diagonal();
  DiagonalMatrix::DiagonalVectorType &rotation_precision_diagonal =
      data_submatrices_.rel_pose_rotation_precision_matrix.diagonal();

  for (int measure_idx = 0; measure_idx < rel_pose_measurements_.size();
       measure_idx++) {
    RelativePoseMeasurement rpm = rel_pose_measurements_[measure_idx];

    // fill in precision matrices
    translation_precision_diagonal(measure_idx) = rpm.getTransPrecision();
    rotation_precision_diagonal(measure_idx) = rpm.getRotPrecision();

    // fill in incidence matrix
    int id1 = getTranslationIdxInExplicitDataMatrix(rpm.first_id) -
              translation_offset;
    int id2 = getTranslationIdxInExplicitDataMatrix(rpm.second_id) -
              translation_offset;
    data_submatrices_.rel_pose_incidence_matrix.insert(measure_idx, id1) = 1.0;
    data_submatrices_.rel_pose_incidence_matrix.insert(measure_idx, id2) = -1.0;

    // fill in translation data matrix where the id1-th (1xdim_) block is -rpm.t
    // and all other blocks are 0
    for (int k = 0; k < dim_; k++) {
      data_submatrices_.rel_pose_translation_data_matrix.insert(
          measure_idx, id1 * dim_ + k) = -rpm.t(k);
    }
  }
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

void Problem::constructDataMatrix() {
  size_t data_matrix_size = getDataMatrixSize();
  data_matrix_ = SparseMatrix(data_matrix_size, data_matrix_size);

  throw NotImplementedException();
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
