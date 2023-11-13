/**
 * @file MatrixManifold.h
 * @author Alan Papalia (apapalia@mit.edu)
 * @brief A virtual class for matrix manifolds. Mostly contains pure virtual
 * functions that are implemented in the derived classes. This function does
 * contain an inner product function that is used by all derived classes.
 * @version 0.1
 * @date 2023-11-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include <CORA/CORA_types.h>
#include <Eigen/Dense>

namespace CORA {

class MatrixManifold {
public:
  /// CONSTRUCTORS AND MUTATORS

  // Default constructor -- sets all dimensions to 0
  MatrixManifold() = default;

  /// GEOMETRY

  /**
   * @brief projectToTangentSpaceects a matrix A onto the manifold M.
   *
   * @param A the matrix to project
   * @return Matrix the projection of A onto M
   */
  virtual Matrix projectToManifold(const Matrix &A) const = 0;

  /**
   * @brief projectToTangentSpaceects a matrix V onto the tangent space T_Y(M)
   * of the manifold at point Y
   *
   * @param Y the point defining the tangent space
   * @param V the matrix to project
   * @return Matrix the projection of V onto T_Y(M)
   */
  virtual Matrix projectToTangentSpace(const Matrix &Y,
                                       const Matrix &V) const = 0;

  /**
   * @brief Computes the trace inner product of A and B
   *
   * @param A the first matrix
   * @param B the second matrix
   * @return Scalar the inner product of A and B
   */
  Scalar innerProduct(const Matrix &A, const Matrix &B) const {
    // get the inner product by taking the row-wise summation of the Hadamard
    // product of A and B
    return (A.array() * B.array()).sum();
  }

  virtual Matrix retract(const Matrix &Y, const Matrix &V) const = 0;
};

} // namespace CORA
