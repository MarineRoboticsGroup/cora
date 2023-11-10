// cppcheck-suppress syntaxError
#include <CORA/CORA_test_utils.h>
#include <CORA/CORA_types.h>
#include <CORA/ObliqueManifold.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace CORA {

TEST_CASE("Testing unit-sphere functions", "[sphere]") {
  int num_spheres = 1;
  int sphere_dim = 2;
  ObliqueManifold M = ObliqueManifold(sphere_dim, num_spheres);

  // generate a random point on the sphere
  Matrix Y = M.random_sample();

  // check that Y is a unit vector
  REQUIRE_THAT(Y.norm(), Catch::Matchers::WithinRel(1.0));

  // check that the inner product of Y with itself is 1
  Scalar inner_prod_self = M.innerProduct(Y, Y);
  REQUIRE_THAT(inner_prod_self, Catch::Matchers::WithinRel(1.0));

  // generate a random vector in the tangent space
  Matrix V = Matrix::Random(sphere_dim, num_spheres);

  // check that the projection to the tangent space works
  Matrix V_on_tangent_space = M.projectToTangentSpace(Y, V);

  // in general, the projection to the tangent space should not be zero
  REQUIRE_FALSE(V_on_tangent_space.isZero());

  // the inner product of V_on_tangent_space and Y should be zero
  Scalar inner_prod_tangent = M.innerProduct(V_on_tangent_space, Y);
  REQUIRE_THAT(inner_prod_tangent, Catch::Matchers::WithinAbs(0.0, 1e-6));

  // check that the retraction of Y + V_on_tangent_space is a unit vector
  Matrix Y_retract = M.retract(Y, V_on_tangent_space);
  REQUIRE_THAT(Y_retract.norm(), Catch::Matchers::WithinRel(1.0));

  // check that the retraction is different from Y
  REQUIRE_FALSE(Y_retract.isApprox(Y));
}

TEST_CASE("Testing oblique manifold functions", "[oblique]") {
  int num_spheres = 5;
  int sphere_dim = 3;
  ObliqueManifold M = ObliqueManifold(sphere_dim, num_spheres);

  // check that the projection to the manifold works
  Matrix A = Matrix::Random(sphere_dim, num_spheres);
  Matrix Y = M.projectToManifold(A);
  for (int j = 0; j < Y.cols(); j++) {
    REQUIRE_THAT(Y.col(j).norm(), Catch::Matchers::WithinRel(1.0));
  }

  // check that the projection of a random point to the tangent space works
  Matrix rand = Matrix::Random(sphere_dim, num_spheres);
  Matrix V = M.projectToTangentSpace(Y, rand);

  // check that projecting Y to its own tangent space is zero
  Matrix V_zero = M.projectToTangentSpace(Y, Y);
  REQUIRE(V_zero.isZero());

  // in general, the projection to the tangent space should not be zero
  REQUIRE(V.norm() > 1e-6);

  // check that the inner product of V and Y is zero
  Scalar inner_prod_tangent = M.innerProduct(V, Y);
  REQUIRE_THAT(inner_prod_tangent, Catch::Matchers::WithinAbs(0.0, 1e-6));

  // check that the retraction has unit norm columns
  Matrix Y_retract = M.retract(Y, V);
  for (int j = 0; j < Y_retract.cols(); j++) {
    REQUIRE_THAT(Y_retract.col(j).norm(), Catch::Matchers::WithinRel(1.0));
  }

  // check that the retraction is different from Y
  REQUIRE_FALSE(Y_retract.isApprox(Y));
}

} // namespace CORA
