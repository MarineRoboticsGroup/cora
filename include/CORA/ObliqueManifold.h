/**
 * @brief this lightweight class models the geometry of the oblique manifold,
 * the product of n copies of the unit sphere in R^r.  Elements of this manifold
 * (and its tangent spaces) are represented as r x n matrices of type
 * 'MatrixType'
 *
 * @author Alan Papalia (apapalia@mit.edu)
 */

#pragma once

#include <Eigen/Dense>

#include <random> // For sampling random points on the manifold

#include "CORA/CORA_types.h"

namespace CORA {

class ObliqueManifold {
private:
  // Dimension of ambient Euclidean space containing the unit vectors
  size_t r_;

  // Number of unit vectors in the product
  size_t n_;

public:
  /// CONSTRUCTORS AND MUTATORS

  // Default constructor -- sets all dimensions to 0
  ObliqueManifold() = default;

  ObliqueManifold(size_t r, size_t n) : r_(r), n_(n) {}

  void set_r(size_t r) { r_ = r; }
  void set_n(size_t n) { n_ = n; }

  /// GEOMETRY

  /** Given a generic matrix A in R^{r x n}, this function projects A onto the
   * oblique manifold by normalizing each column of A to have unit norm. Note
   * that this is a computationally cheap operation but is not a second-order
   * retraction. See Boumal "Optimization on Smooth Manifolds" for more details.
   */
  Matrix projectToManifold(const Matrix &A) const;

  /**
   * @brief Compute the Frobenius inner product of A and B in R^{r x n}.
   *
   * @param A the first matrix
   * @param B the second matrix
   * @return Scalar the inner product of A and B
   */
  Scalar innerProduct(const Matrix &A, const Matrix &B) const;

  /**
   * @brief Projects a matrix A in R^{r x n} onto the tangent space T_Y(M) of
   * the oblique manifold at Y in R^{r x n}.
   *
   * @param A the matrix to project
   * @param Y the point defining the tangent space
   * @return Matrix
   */
  Matrix projectToTangentSpace(const Matrix &A, const Matrix &Y) const;

  /** Given an element Y in M and a tangent vector V in T_Y(M), this function
   * computes the retraction along V at Y using the projectToManifold method
   *
   */
  Matrix retract(const Matrix &Y, const Matrix &V) const;

  /** Sample a random point on M, using the (optional) passed seed to initialize
   * the random number generator.  */
  Matrix random_sample(const std::default_random_engine::result_type &seed =
                           std::default_random_engine::default_seed) const;
};

} // namespace CORA
