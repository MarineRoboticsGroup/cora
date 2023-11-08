
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>
#include <CORA/ObliqueManifold.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace CORA {

TEST_CASE("Testing oblique manifold functions", "[problem::oblique-manifold]") {
  int num_spheres = 5;
  int sphere_dim = 3;
  ObliqueManifold M = ObliqueManifold(sphere_dim, num_spheres);

  // check that the projection to the manifold works
  Matrix A = Matrix::Random(sphere_dim, num_spheres);
  Matrix A_normalized = M.projectToManifold(A);
  for (int j = 0; j < A_normalized.cols(); j++) {
    REQUIRE_THAT(A_normalized.col(j).norm(), Catch::Matchers::WithinRel(1.0));
  }
}

} // namespace CORA
