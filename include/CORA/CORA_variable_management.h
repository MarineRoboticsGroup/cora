
#pragma once

#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace CORA {

/**
 * @brief A small helper container to keep initial guesses for variables.
 *
 * Users can populate pose rotations/translations and landmark translations
 * keyed by `Symbol`. The data can then be packed into an `x0` matrix that
 * `solveCORA` accepts.
 */
class Values {
public:
  Values() = default;
  void clear();

  // Pose initialization: rotation (d x d) and translation (d,)
  void setPose(const Symbol &sym, const Matrix &R,
                             const Vector &t);
  bool hasPose(const Symbol &sym) const;
  const Matrix *getPoseRotation(const Symbol &sym) const;
  const Vector *getPoseTranslation(const Symbol &sym) const;

  // Landmark initialization: position vector (d,)
  void setLandmark(const Symbol &sym, const Vector &p);
  bool hasLandmark(const Symbol &sym) const;
  const Vector *getLandmark(const Symbol &sym) const;

  inline std::map<Symbol, Matrix> getAllPoseRotations() const {
    return pose_rotations_;
  }
  inline std::map<Symbol, Vector> getAllPoseTranslations() const {
    return pose_translations_;
  }
  inline std::map<Symbol, Vector> getAllLandmarkPositions() const {
    return landmark_positions_;
  }

private:
  std::map<Symbol, Matrix> pose_rotations_;
  std::map<Symbol, Vector> pose_translations_;
  std::map<Symbol, Vector> landmark_positions_;
};

/**
 * @brief Generate an `Values` from an initialization matrix `x0`.
 * Checks that all variables are initialized to be feasible (i.e., are members
 * of the correct manifold).
 */
Values getValuesFromVarMatrix(const Problem &problem,
                                           const Matrix &x0);

/**
 * @brief Convert an `Values` into an initialization matrix `x0`. Any
 * values not specified in the map will be set randomly.
 */
Matrix getVarMatrixFromValues(const Problem &problem,
                                const Values &inits);

/**
 * @brief Update an `Values` `inits` based on values in an
 * initialization matrix `x0`.
 */
void updateValuesFromVarMatrix(const Problem &problem,
                                 Values &inits, const Matrix &x0);

/**
 * @brief Update the range measurement rows of an initialization matrix `x0`
 * based on its current translation values. If the formulation is implicit,
 * the translation-explicit form is computed first and the 'optimal' translations
 * are used to compute the ranges.
 */
void updateVarMatrixRangesBasedOnTranslationVals(const Problem &problem,
                                                  Matrix &x0);

/**
 * @brief Update an initialization matrix `x0` based on values in an
 * `Values` `inits`.
 */
void updateVarMatrixFromValues(const Problem &problem,
                                 const Values &inits, Matrix &x0);

/**
 * @brief Generate a random initialization map for the problem. The map should
 * initialize all poses and landmarks.
 */
Values getRandomValues(const Problem &problem);

/**
 * @brief Generate a random initialization matrix for the problem. The matrix
 * should initialize all poses and landmarks.
 */
Matrix getRandomVarMatrix(const Problem &problem);

} // namespace CORA
