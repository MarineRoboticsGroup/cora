#pragma once

#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

// Custom Catch2 matcher to check if two Eigen matrices are approximately equal
template <typename MatrixType>
struct EigenMatrixApproxMatcher : Catch::Matchers::MatcherBase<MatrixType> {
  explicit EigenMatrixApproxMatcher(const MatrixType &expected,
                                    double epsilon = 1e-6)
      : expected_(expected), epsilon_(epsilon) {}

  bool match(const MatrixType &actual) const override {
    return (expected_ - actual).cwiseAbs().maxCoeff() <= epsilon_;
  }

  std::string describe() const override {
    std::ostringstream oss;
    oss << "\nis approximately equal to\n"
        << expected_ << "\nwithin epsilon " << epsilon_;
    return oss.str();
  }

private:
  const MatrixType &expected_;
  double epsilon_;
};

// Convenience function to create the custom matcher
template <typename MatrixType>
EigenMatrixApproxMatcher<MatrixType>
IsApproximatelyEqual(const MatrixType &expected, double epsilon = 1e-6) {
  return EigenMatrixApproxMatcher<MatrixType>(expected, epsilon);
}

// Custom Catch2 matcher to check if two Eigen matrices are approximately equal
// *up to a sign*
template <typename MatrixType>
struct EigenMatrixApproxMatcherUpToSign
    : Catch::Matchers::MatcherBase<MatrixType> {
  explicit EigenMatrixApproxMatcherUpToSign(const MatrixType &expected,
                                            double epsilon = 1e-6)
      : expected_(expected), epsilon_(epsilon) {}

  bool match(const MatrixType &actual) const override {
    return (expected_ - actual).cwiseAbs().maxCoeff() <= epsilon_ ||
           (expected_ + actual).cwiseAbs().maxCoeff() <= epsilon_;
  }

  std::string describe() const override {
    std::ostringstream oss;
    oss << "\nis approximately equal to\n"
        << expected_ << "\nwithin epsilon " << epsilon_;
    return oss.str();
  }

private:
  const MatrixType &expected_;
  double epsilon_;
};

// Convenience function to create the custom matcher
template <typename MatrixType>
EigenMatrixApproxMatcherUpToSign<MatrixType>
IsApproximatelyEqualUpToSign(const MatrixType &expected,
                             double epsilon = 1e-6) {
  return EigenMatrixApproxMatcherUpToSign<MatrixType>(expected, epsilon);
}
