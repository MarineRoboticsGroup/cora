
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
class InitializationMap {
public:
  InitializationMap() = default;
  void clear();

  // Pose initialization: rotation (d x d) and translation (d,)
  void setPoseInitialization(const Symbol &sym, const Matrix &R,
                             const Vector &t);
  bool hasPoseInitialization(const Symbol &sym) const;
  const Matrix *getPoseRotation(const Symbol &sym) const;
  const Vector *getPoseTranslation(const Symbol &sym) const;

  // Landmark initialization: position vector (d,)
  void setLandmarkInitialization(const Symbol &sym, const Vector &p);
  bool hasLandmarkInitialization(const Symbol &sym) const;
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
 * @brief Generate an `InitializationMap` from an initialization matrix `x0`.
 * Checks that all variables are initialized to be feasible (i.e., are members
 * of the correct manifold).
 */
InitializationMap getInitMapFromInitMatrix(const Problem &problem,
                                           const Matrix &x0);

/**
 * @brief Convert an `InitializationMap` into an initialization matrix `x0`. Any
 * values not specified in the map will be set randomly.
 */
Matrix getInitMatrixFromInitMap(const Problem &problem,
                                const InitializationMap &inits);

/**
 * @brief Update an `InitializationMap` `inits` based on values in an
 * initialization matrix `x0`.
 */
void updateInitMapFromInitMatrix(const Problem &problem,
                                 InitializationMap &inits, const Matrix &x0);

/**
 * @brief Update the range measurement rows of an initialization matrix `x0`
 * based on its current translation values.
 */
void updateInitMatrixRangesBasedOnTranslationVals(const Problem &problem,
                                                  Matrix &x0);

/**
 * @brief Update an initialization matrix `x0` based on values in an
 * `InitializationMap` `inits`.
 */
void updateInitMatrixFromInitMap(const Problem &problem,
                                 const InitializationMap &inits, Matrix &x0);

/**
 * @brief Generate a random initialization map for the problem. The map should
 * initialize all poses and landmarks.
 */
InitializationMap getRandomInitMap(const Problem &problem);

/**
 * @brief Generate a random initialization matrix for the problem. The matrix
 * should initialize all poses and landmarks.
 */
Matrix getRandomInitMatrix(const Problem &problem);

} // namespace CORA
