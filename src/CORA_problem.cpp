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
#include <CORA/CORA_utils.h>

namespace CORA {
void Problem::addPoseVariable(const Symbol &pose_id) {
  if (pose_symbol_idxs_.find(pose_id) != pose_symbol_idxs_.end()) {
    throw std::invalid_argument("Pose variable already exists");
  }
  pose_symbol_idxs_.insert(std::make_pair(pose_id, pose_symbol_idxs_.size()));
  problem_data_up_to_date_ = false;
  manifolds_.stiefel_prod_manifold_.addNewFrame();
}

void Problem::addLandmarkVariable(const Symbol &landmark_id) {
  if (landmark_symbol_idxs_.find(landmark_id) != landmark_symbol_idxs_.end()) {
    throw std::invalid_argument("Landmark variable already exists");
  }
  landmark_symbol_idxs_.insert(
      std::make_pair(landmark_id, landmark_symbol_idxs_.size()));
}

void Problem::addRangeMeasurement(const RangeMeasurement &range_measurement) {
  if (std::find(range_measurements_.begin(), range_measurements_.end(),
                range_measurement) != range_measurements_.end()) {
    throw std::invalid_argument("Range measurement already exists");
  }
  range_measurements_.push_back(range_measurement);
  problem_data_up_to_date_ = false;
  manifolds_.oblique_manifold_.addNewSphere();
}

void Problem::addRelativePoseMeasurement(
    const RelativePoseMeasurement &rel_pose_measure) {
  if (std::find(rel_pose_measurements_.begin(), rel_pose_measurements_.end(),
                rel_pose_measure) != rel_pose_measurements_.end()) {
    throw std::invalid_argument("Relative pose measurement already exists");
  }
  rel_pose_measurements_.push_back(rel_pose_measure);
  problem_data_up_to_date_ = false;
}

void Problem::addPosePrior(const PosePrior &pose_prior) {
  if (std::find(pose_priors_.begin(), pose_priors_.end(), pose_prior) !=
      pose_priors_.end()) {
    throw std::invalid_argument("Pose prior already exists");
  }
  pose_priors_.push_back(pose_prior);
  problem_data_up_to_date_ = false;
}

void Problem::addLandmarkPrior(const LandmarkPrior &landmark_prior) {
  if (std::find(landmark_priors_.begin(), landmark_priors_.end(),
                landmark_prior) != landmark_priors_.end()) {
    throw std::invalid_argument("Landmark prior already exists");
  }
  landmark_priors_.push_back(landmark_prior);
  problem_data_up_to_date_ = false;
}

void Problem::fillRangeSubmatrices() {
  // need to account for the fact that the indices will be offset by the
  // dimension of the rotation and the range variables that precede the
  // translations
  auto translation_offset = rotAndRangeMatrixSize();
  auto num_range_measurements = numRangeMeasurements();
  auto num_translations = numTranslationalStates();
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
    auto id1 = getTranslationIdx(measure.first_id) - translation_offset;
    auto id2 = getTranslationIdx(measure.second_id) - translation_offset;
    data_submatrices_.range_incidence_matrix.insert(measure_idx, id1) = -1.0;
    data_submatrices_.range_incidence_matrix.insert(measure_idx, id2) = 1.0;
  }
}

void Problem::fillRelPoseSubmatrices() {
  fillRotConnLaplacian();
  auto num_pose_measurements = numPoseMeasurements();
  auto num_translations = numTranslationalStates();

  // need to account for the fact that the indices will be offset by the
  // dimension of the rotation and the range variables that precede the
  // translations
  auto translation_offset = rotAndRangeMatrixSize();
  // initialize the submatrices to the correct sizes
  data_submatrices_.rel_pose_incidence_matrix =
      SparseMatrix(num_pose_measurements, num_translations);
  data_submatrices_.rel_pose_translation_data_matrix =
      SparseMatrix(num_pose_measurements, numPosesDim());
  data_submatrices_.rel_pose_translation_precision_matrix =
      SparseMatrix(num_pose_measurements, num_pose_measurements);
  data_submatrices_.rel_pose_rotation_precision_matrix =
      SparseMatrix(num_pose_measurements, num_pose_measurements);

  // for diagonal matrices, get the diagonal vector and set the values
  for (int measure_idx = 0; measure_idx < num_pose_measurements;
       measure_idx++) {
    RelativePoseMeasurement rpm = rel_pose_measurements_[measure_idx];

    // fill in precision matrices
    data_submatrices_.rel_pose_translation_precision_matrix.insert(
        measure_idx, measure_idx) = rpm.getTransPrecision();
    data_submatrices_.rel_pose_rotation_precision_matrix.insert(
        measure_idx, measure_idx) = rpm.getRotPrecision();

    // fill in incidence matrix
    Index id1 = getTranslationIdx(rpm.first_id) - translation_offset;
    Index id2 = getTranslationIdx(rpm.second_id) - translation_offset;
    data_submatrices_.rel_pose_incidence_matrix.insert(measure_idx, id1) = -1.0;
    data_submatrices_.rel_pose_incidence_matrix.insert(measure_idx, id2) = 1.0;

    // fill in translation data matrix where the id1-th (1 x dim_) block is
    // -rpm.t and all other blocks are 0
    for (int k = 0; k < dim_; k++) {
      data_submatrices_.rel_pose_translation_data_matrix.insert(
          measure_idx, id1 * dim_ + k) = -rpm.t(k);
    }
  }
}

void Problem::fillRotConnLaplacian() {
  auto d{dim_};

  // Each measurement contributes 2*d elements along the diagonal of the
  // connection Laplacian, and 2*d^2 elements on a pair of symmetric
  // off-diagonal blocks

  size_t measurement_stride = 2 * (d + d * d);

  std::vector<Eigen::Triplet<Scalar>> triplets;
  triplets.reserve(measurement_stride * numPoseMeasurements());

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
    for (Index r = 0; r < d; r++)
      for (Index c = 0; c < d; c++)
        triplets.emplace_back(i * d + r, j * d + c,
                              -measurement.getRotPrecision() *
                                  measurement.R(r, c));

    // Elements of ji block
    for (Index r = 0; r < d; r++)
      for (Index c = 0; c < d; c++)
        triplets.emplace_back(j * d + r, i * d + c,
                              -measurement.getRotPrecision() *
                                  measurement.R(c, r));
  }

  // Construct and return a sparse matrix from these triplets
  data_submatrices_.rotation_conn_laplacian =
      SparseMatrix(numPosesDim(), numPosesDim());
  data_submatrices_.rotation_conn_laplacian.setFromTriplets(triplets.begin(),
                                                            triplets.end());
}

template <typename... Matrices>
DiagonalMatrix diagMatrixMult(const DiagonalMatrix &first,
                              const Matrices &...matrices) {
  // Check dimensions
  const auto numRows = first.rows();
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
  if (numPoses() > 0) {
    std::cout << "Pose variables:" << std::endl;
    for (const auto &[pose_sym, pose_idx] : pose_symbol_idxs_) {
      std::cout << pose_sym.string() << " -> " << pose_idx << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No pose variables" << std::endl;
  }

  // print out all of the landmark variables
  if (numLandmarks() > 0) {
    std::cout << "\nLandmark variables:" << std::endl;
    for (const auto &[landmark_sym, landmark_idx] : landmark_symbol_idxs_) {
      std::cout << landmark_sym.string() << " -> " << landmark_idx << std::endl;
    }
    std::cout << std::endl;
  } else {
    std::cout << "No landmark variables" << std::endl;
  }

  // print out all of the range measurements
  if (numRangeMeasurements() > 0) {
    std::cout << "\nRange measurements:" << std::endl;
    for (const auto &range_measurement : range_measurements_) {
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
  if (numPoseMeasurements() > 0) {
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
  if (!pose_priors_.empty()) {
    std::cout << "\nPose priors:" << std::endl;
    for (const auto &pose_prior : pose_priors_) {
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
  if (!landmark_priors_.empty()) {
    std::cout << "\nLandmark priors:" << std::endl;
    for (const auto &landmark_prior : landmark_priors_) {
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
  if (data_matrix_.nonZeros() == 0 || !problem_data_up_to_date_) {
    updateProblemData();
  }
  return data_matrix_;
}

void Problem::updateProblemData() {
  // update the relevant submatrices
  fillRangeSubmatrices();
  fillRelPoseSubmatrices();
  fillDataMatrix();
  updatePreconditioner();
  problem_data_up_to_date_ = true;
}

void Problem::updatePreconditioner() {
  if (preconditioner_ == Preconditioner::BlockCholesky) {
    // blocks are rots: n*d, ranges: r, and translations: n + l
    std::vector<int> block_size_vec = {numPosesDim(), numRangeMeasurements(),
                                       numPoses() + numLandmarks()};
    // drop any values that are 0
    block_size_vec.erase(
        std::remove(block_size_vec.begin(), block_size_vec.end(), 0),
        block_size_vec.end());

    // convert to VectorXi
    VectorXi block_sizes(block_size_vec.size());
    for (int i = 0; i < block_size_vec.size(); i++) {
      block_sizes(i) = block_size_vec[i];
    }

    SparseMatrix epsilonPosDefUpdate =
        SparseMatrix(data_matrix_.rows(), data_matrix_.cols());
    epsilonPosDefUpdate.setIdentity();
    epsilonPosDefUpdate *= 1e-3;
    preconditioner_matrices_.block_chol_factor_ptrs_ =
        getBlockCholeskyFactorization(data_matrix_ + epsilonPosDefUpdate,
                                      block_sizes);
  } else if (preconditioner_ == Preconditioner::RegularizedCholesky) {
    // add a small value to the diagonal of the data matrix to ensure that it is
    // positive definite
    SparseMatrix epsilonPosDefUpdate =
        SparseMatrix(data_matrix_.rows(), data_matrix_.cols());
    epsilonPosDefUpdate.setIdentity();
    epsilonPosDefUpdate *= 1e-2;
    VectorXi block_sizes(1);
    block_sizes(0) = data_matrix_.rows();
    preconditioner_matrices_.block_chol_factor_ptrs_ =
        getBlockCholeskyFactorization(data_matrix_ + epsilonPosDefUpdate,
                                      block_sizes);
  } else {
    throw std::invalid_argument("The desired preconditioner is not "
                                "implemented");
  }
}

void Problem::fillDataMatrix() {
  auto data_matrix_size = getDataMatrixSize();
  data_matrix_ = SparseMatrix(data_matrix_size, data_matrix_size);

  /**
   * @brief From here we form the sub blocks of the data matrix Q
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

  auto rot_mat_sz = numPosesDim();
  auto rot_range_mat_sz = rotAndRangeMatrixSize();
  // Q11, Q13, Q22, Q23, Q33
  addTriplets(Q11, 0, 0);
  addTriplets(Q13, 0, rot_range_mat_sz);
  addTriplets(Q22, rot_mat_sz, rot_mat_sz);
  addTriplets(Q23, rot_mat_sz, rot_range_mat_sz);
  addTriplets(Q33, rot_range_mat_sz, rot_range_mat_sz);

  // also add Q13 and Q23 transposed to the triplets
  addTriplets(Q13.transpose(), rot_range_mat_sz, 0);
  addTriplets(Q23.transpose(), rot_range_mat_sz, rot_mat_sz);

  // construct the data matrix
  data_matrix_.setFromTriplets(combined_triplets.begin(),
                               combined_triplets.end());
}

Matrix Problem::dataMatrixProduct(const Matrix &Y) const {
  if (formulation_ == Formulation::Explicit) {
    checkMatrixShape("Problem::dataMatrixProduct::Y", getDataMatrixSize(),
                     relaxation_rank_, Y.rows(), Y.cols());
    return data_matrix_ * Y;
  } else {
    throw std::invalid_argument("Implicit formulation not implemented");
  }
}

Scalar Problem::evaluateObjective(const Matrix &Y) const {
  checkUpToDate();
  return 0.5 * (Y.transpose() * dataMatrixProduct(Y)).trace();
}

Matrix Problem::Euclidean_gradient(const Matrix &Y) const {
  checkUpToDate();
  Matrix egrad = dataMatrixProduct(Y);
  checkMatrixShape("Problem::Euclidean_gradient", Y.rows(), Y.cols(),
                   egrad.rows(), egrad.cols());
  return egrad;
}

Matrix Problem::Riemannian_gradient(const Matrix &Y) const {
  return Riemannian_gradient(Y, Euclidean_gradient(Y));
}

Matrix Problem::Riemannian_gradient(const Matrix &Y,
                                    const Matrix &NablaF_Y) const {
  checkUpToDate();
  return tangent_space_projection(Y, NablaF_Y);
}

Matrix Problem::tangent_space_projection(const Matrix &Y,
                                         const Matrix &Ydot) const {
  // similar to projectToManifold, we treat this projection block-wise. The
  // first n*d columns are Stiefel elements, and thus use the Stiefel
  // projection. The next r columns are range measurements, and thus use the
  // oblique projection. The remaining columns are translational variables, and
  // thus belong to the Euclidean manifold and do not need projection.

  // check that Y and Ydot have the correct dimensions
  checkMatrixShape("Problem::tangent_space_projection::Y", getDataMatrixSize(),
                   relaxation_rank_, Y.rows(), Y.cols());
  checkMatrixShape("Problem::tangent_space_projection::Ydot",
                   getDataMatrixSize(), relaxation_rank_, Ydot.rows(),
                   Ydot.cols());

  Matrix result = Ydot;

  // Stiefel component
  auto rot_mat_sz = numPosesDim();
  result.block(0, 0, rot_mat_sz, relaxation_rank_) =
      manifolds_.stiefel_prod_manifold_
          .projectToTangentSpace(
              Y.block(0, 0, rot_mat_sz, relaxation_rank_).transpose(),
              result.block(0, 0, rot_mat_sz, relaxation_rank_).transpose())
          .transpose();

  // Oblique component
  int r = numRangeMeasurements();
  result.block(rot_mat_sz, 0, r, relaxation_rank_) =
      manifolds_.oblique_manifold_
          .projectToTangentSpace(
              Y.block(rot_mat_sz, 0, r, relaxation_rank_).transpose(),
              result.block(rot_mat_sz, 0, r, relaxation_rank_).transpose())
          .transpose();

  // remaining component is untouched
  return result;
}

Matrix Problem::Riemannian_Hessian_vector_product(const Matrix &Y,
                                                  const Matrix &nablaF_Y,
                                                  const Matrix &dotY) const {
  checkMatrixShape("Problem::Riemannian_Hessian_vector_product::Y",
                   getDataMatrixSize(), relaxation_rank_, Y.rows(), Y.cols());
  checkMatrixShape("Problem::Riemannian_Hessian_vector_product::nablaF_Y",
                   getDataMatrixSize(), relaxation_rank_, nablaF_Y.rows(),
                   nablaF_Y.cols());
  checkMatrixShape("Problem::Riemannian_Hessian_vector_product::dotY",
                   getDataMatrixSize(), relaxation_rank_, dotY.rows(),
                   dotY.cols());

  Matrix H_dotY = dataMatrixProduct(dotY);

  auto rot_mat_sz = numPosesDim();
  // Stiefel component
  H_dotY.block(0, 0, rot_mat_sz, relaxation_rank_) =
      manifolds_.stiefel_prod_manifold_
          .projectToTangentSpace(
              Y.block(0, 0, rot_mat_sz, relaxation_rank_).transpose(),
              H_dotY.block(0, 0, rot_mat_sz, relaxation_rank_).transpose() -
                  manifolds_.stiefel_prod_manifold_.SymBlockDiagProduct(
                      dotY.block(0, 0, rot_mat_sz, relaxation_rank_)
                          .transpose(),
                      Y.block(0, 0, rot_mat_sz, relaxation_rank_),
                      nablaF_Y.block(0, 0, rot_mat_sz, relaxation_rank_)
                          .transpose()))
          .transpose();

  // Oblique component
  int r = numRangeMeasurements();
  Vector diagQXXT = (nablaF_Y.array() * Y.array()).rowwise().sum();
  // weight the rows of dotY by the diagonal of QXXT (which is a vector)
  Matrix weightedDotY = dotY.array().colwise() * diagQXXT.array();
  Matrix euclidean_hessian = H_dotY.block(rot_mat_sz, 0, r, relaxation_rank_) =
      manifolds_.oblique_manifold_
          .projectToTangentSpace(
              Y.block(rot_mat_sz, 0, r, relaxation_rank_).transpose(),
              (H_dotY.block(rot_mat_sz, 0, r, relaxation_rank_) -
               weightedDotY.block(rot_mat_sz, 0, r, relaxation_rank_))
                  .transpose())
          .transpose();

  return H_dotY;
}

Matrix Problem::precondition(const Matrix &V) const {
  checkMatrixShape("Problem::precondition::input", getDataMatrixSize(),
                   relaxation_rank_, V.rows(), V.cols());
  Matrix res;
  if (preconditioner_ == Preconditioner::BlockCholesky ||
      preconditioner_ == Preconditioner::RegularizedCholesky) {
    res =
        blockCholeskySolve(preconditioner_matrices_.block_chol_factor_ptrs_, V);
  } else {
    throw std::invalid_argument("The desired preconditioner is not "
                                "implemented");
  }
  checkMatrixShape("Problem::precondition::result", getDataMatrixSize(),
                   relaxation_rank_, res.rows(), res.cols());

  // check for NaNs in res
  if (res.hasNaN()) {
    std::cout << "NaNs in preconditioned vector:\n" << res << std::endl;
    throw std::runtime_error("NaNs in preconditioned vector");
  }
  return res;
}

Matrix Problem::projectToManifold(const Matrix &A) const {
  checkMatrixShape("Problem::projectToManifold", getDataMatrixSize(),
                   relaxation_rank_, A.rows(), A.cols());

  Matrix result = A;

  // the first n*d rows are obtained from
  // manifolds_.stiefel_prod.projectToManifoldresult(1:n*d, :))

  auto rot_mat_sz = numPosesDim();
  result.block(0, 0, rot_mat_sz, relaxation_rank_) =
      manifolds_.stiefel_prod_manifold_
          .projectToManifold(
              result.block(0, 0, rot_mat_sz, relaxation_rank_).transpose())
          .transpose();

  // the next r rows are obtained from
  // manifolds_.oblique_manifold.retract(Y(n*d+1:n*d+r, :), V(n*d+1:n*d+r, :))
  int r = numRangeMeasurements();
  result.block(rot_mat_sz, 0, r, relaxation_rank_) =
      manifolds_.oblique_manifold_
          .projectToManifold(
              result.block(rot_mat_sz, 0, r, relaxation_rank_).transpose())
          .transpose();

  // if there are remaining rows, they should be translational variables and
  // thus belong to the Euclidean manifold and do not need rounding.

  return result;
}

Matrix Problem::retract(const Matrix &Y, const Matrix &V) const {
  return projectToManifold(Y + V);
}

int Problem::getDataMatrixSize() const {
  if (formulation_ == Formulation::Explicit) {
    return (numPoses() * (dim_ + 1)) + numLandmarks() + numRangeMeasurements();
  } else if (formulation_ == Formulation::Implicit) {
    return rotAndRangeMatrixSize();
  } else {
    throw std::invalid_argument("Unknown formulation");
  }
}

Index Problem::getRotationIdx(const Symbol &pose_symbol) const {
  auto rotation_search_it = pose_symbol_idxs_.find(pose_symbol);
  if (rotation_search_it != pose_symbol_idxs_.end()) {
    return rotation_search_it->second;
  }

  // if we get here, we didn't find the pose symbol
  throw std::invalid_argument("Unknown pose symbol");
}

Index Problem::getRangeIdx(const SymbolPair &range_symbol_pair) const {
  // all range measurements come after the rotations
  auto rot_offset = numPosesDim();

  // search for the range symbol
  auto range_search_it = std::find_if(
      range_measurements_.begin(), range_measurements_.end(),
      [&range_symbol_pair](const RangeMeasurement &rangeMeasurement) {
        return rangeMeasurement.hasSymbolPair(range_symbol_pair);
      });

  // if we found the range symbol, return the index of the symbol in
  // range_measurements_ plus the offset
  if (range_search_it != range_measurements_.end()) {
    return std::distance(range_measurements_.begin(), range_search_it) +
           rot_offset;
  }

  // if we get here, we didn't find the range symbol
  throw std::invalid_argument("Unknown range symbol");
}

Index Problem::getTranslationIdx(const Symbol &trans_symbol) const {
  // all translations come after the rotations and range measurement variables
  // (unit spheres) so we need to offset by the dimension of the rotations and
  // the number of range measurements
  size_t idx_offset = rotAndRangeMatrixSize();

  // is a pose translation
  auto pose_search_it = pose_symbol_idxs_.find(trans_symbol);
  if (pose_search_it != pose_symbol_idxs_.end()) {
    return static_cast<Index>(pose_search_it->second + idx_offset);
  }

  // is a landmark translation
  auto landmark_search_it = landmark_symbol_idxs_.find(trans_symbol);
  if (landmark_search_it != landmark_symbol_idxs_.end()) {
    // need to offset by the number of poses because the landmark variables
    // come after the pose translations
    return static_cast<Index>(landmark_search_it->second + idx_offset +
                              numPoses());
  }

  // if we get here, we didn't find the translation symbol
  throw std::invalid_argument("Unknown translation symbol");
}

Matrix Problem::getRandomInitialGuess() const {
  // assert that the problem data must be up to date
  assert(problem_data_up_to_date_);
  Matrix x0 = Matrix::Random(getDataMatrixSize(), relaxation_rank_);
  return projectToManifold(x0);
}

CertResults Problem::certify_solution(const Matrix &Y, Scalar eta, size_t nx,
                                      size_t max_LOBPCG_iters,
                                      Scalar max_fill_factor,
                                      Scalar drop_tol) const {
  /// Construct certificate matrix S

  LambdaBlocks Lambda_blocks;
  SparseMatrix S;

  // We compute the certificate matrix corresponding to the *full* (i.e.
  // translation-explicit) form of the problem
  Lambda_blocks = compute_Lambda_blocks(Y);
  S = data_matrix_ - compute_Lambda_from_Lambda_blocks(Lambda_blocks);

  /// Test positive-semidefiniteness of certificate matrix S using fast
  /// verification method
  auto num_eigvecs =
      std::min(std::max(Eigen::Index(nx), Y.cols() + 2), S.rows());
  if (S.rows() < Y.cols()) {
    throw std::invalid_argument(
        "The number of rows of S must be greater than or "
        "equal to the number of columns of Y");
  }
  Matrix init_eigvec_guess = Matrix::Random(S.rows(), num_eigvecs);
  init_eigvec_guess.block(0, 0, S.rows(), relaxation_rank_) = Y;
  CertResults results = fast_verification(
      S, eta, init_eigvec_guess, max_LOBPCG_iters, max_fill_factor, drop_tol);

  if (!results.is_certified && (formulation_ == Formulation::Implicit)) {
    // Extract the (leading) portion of the tangent vector corresponding to the
    // rotational and spherical variables
    Vector v = results.x.head(rotAndRangeMatrixSize()).normalized();
    results.x = v;

    // Compute x's Rayleight quotient with the simplified certificate matrix
    SparseMatrix Lambda = compute_Lambda_from_Lambda_blocks(Lambda_blocks);
    Vector Sx = dataMatrixProduct(results.x) - Lambda * results.x;
    results.theta = results.x.dot(Sx);
  }

  return results;
}

Problem::LambdaBlocks Problem::compute_Lambda_blocks(const Matrix &Y) const {
  // Compute S * Y, where S is the data matrix defining the quadratic form
  // for the specific version of the SE-Sync problem we're solving
  Matrix QY = dataMatrixProduct(Y);

  // Preallocate storage for diagonal blocks of Lambda
  Matrix stiefel_Lambda_blocks(dim_, numPosesDim());

  for (auto i = 0; i < numPoses(); ++i) {
    Matrix P = QY.block(i * dim_, 0, dim_, Y.cols()) *
               Y.block(i * dim_, 0, dim_, Y.cols()).transpose();
    stiefel_Lambda_blocks.block(0, i * dim_, dim_, dim_) =
        .5 * (P + P.transpose());
  }

  Vector oblique_Lambda_blocks(numRangeMeasurements());
  auto rot_mat_sz = numPosesDim();
  Vector oblique_inner_prods =
      (Y.block(rot_mat_sz, 0, numRangeMeasurements(), Y.cols()).array() *
       QY.block(rot_mat_sz, 0, numRangeMeasurements(),
                Y.cols())
           .array())
          .rowwise()
          .sum(); // (r x 1) vector of inner products

  return std::make_pair(stiefel_Lambda_blocks, oblique_inner_prods);
}

SparseMatrix Problem::compute_Lambda_from_Lambda_blocks(
    const LambdaBlocks &Lambda_blocks) const {
  std::vector<Eigen::Triplet<Scalar>> elements;
  elements.reserve(dim_ * numPosesDim() + numRangeMeasurements());

  // add the symmetric diagonal blocks for the Stiefel constraints
  for (auto i = 0; i < numPoses(); ++i) { // block index
    for (auto r = 0; r < dim_; ++r) {     // block row index
      for (auto c = 0; c < dim_; ++c) {   // block column index
        elements.emplace_back(i * dim_ + r, i * dim_ + c,
                              Lambda_blocks.first(r, i * dim_ + c));
      }
    }
  }

  auto rot_mat_sz = numPosesDim();
  // add the diagonal block for the Oblique constraints
  for (auto i = 0; i < numRangeMeasurements(); ++i) {
    elements.emplace_back(rot_mat_sz + i, rot_mat_sz + i,
                          Lambda_blocks.second(i));
  }

  // add additional zeros if we're using the explicit formulation
  int Lambda_size = rotAndRangeMatrixSize();
  if (formulation_ == Formulation::Explicit) {
    Lambda_size += numLandmarks() + numPoses();
  }

  SparseMatrix Lambda(Lambda_size, Lambda_size);
  Lambda.setFromTriplets(elements.begin(), elements.end());
  return Lambda;
}

SparseMatrix Problem::get_certificate_matrix(const Matrix &Y) const {
  LambdaBlocks Lambda_blocks = compute_Lambda_blocks(Y);
  return data_matrix_ - compute_Lambda_from_Lambda_blocks(Lambda_blocks);
}

Matrix Problem::alignEstimateToOrigin(const Matrix &Y) const {
  if (formulation_ == Formulation::Implicit) {
    throw std::invalid_argument(
        "Problem::alignEstimateToOrigin not implemented for implicit "
        "formulation");
  }

  if (Y.cols() != dim_) {
    throw std::invalid_argument(
        "Problem::alignEstimateToOrigin: Y must have "
        "the same number of columns as the dimension. This method is not "
        "implemented for a relaxed solution.");
  }

  // start by rotating everything such that the first dxd block is the identity
  Matrix first_rot = Y.block(0, 0, dim_, dim_);
  Matrix Y_aligned = Y * first_rot.transpose();

  // now uniformly translate all of the translation variables such that the
  // first translation variable is the origin
  auto rot_range_mat_sz = rotAndRangeMatrixSize();
  Vector first_translation =
      Y_aligned.block(rot_range_mat_sz, 0, 1, dim_).transpose();
  Y_aligned.block(rot_range_mat_sz, 0, numPoses() + numLandmarks(), dim_) =
      Y_aligned.block(rot_range_mat_sz, 0, numPoses() + numLandmarks(), dim_)
          .rowwise() -
      first_translation.transpose();

  return Y_aligned;
}

} // namespace CORA
