#pragma once

#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/pyfg_text_parser.h>

#include <unsupported/Eigen/SparseExtra>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

/********  MATCHERS    **********/

// Custom Catch2 matcher to check if two Eigen matrices are approximately equal
template <typename MatrixType>
struct EigenMatrixApproxMatcher : Catch::Matchers::MatcherBase<MatrixType> {
  explicit EigenMatrixApproxMatcher(const MatrixType &expected,
                                    double epsilon = 1e-6)
      : expected_(expected), epsilon_(epsilon) {}

  bool match(const MatrixType &actual) const override {
    if (expected_.rows() != actual.rows() ||
        expected_.cols() != actual.cols()) {
      return false;
    }

    if constexpr (std::is_same_v<MatrixType, CORA::SparseMatrix>) {
      MatrixType diff = expected_ - actual;
      for (int k = 0; k < diff.outerSize(); ++k) {
        for (typename MatrixType::InnerIterator it(diff, k); it; ++it) {
          const typename MatrixType::Scalar abs_diff = std::abs(it.value());
          if (abs_diff > epsilon_) {
            return false;
          }
        }
      }
    } else {
      return (expected_ - actual).cwiseAbs().maxCoeff() <= epsilon_;
    }

    return true;
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

/*********   PROBLEM SETUP   ***********/

namespace CORA {

SparseMatrix readMatrixMarketFile(const std::string &filename);
void writeMatrixMarketFile(const SparseMatrix &A, const std::string &filename);

void printMatrixSparsityPattern(const Matrix &matrix);
std::string getTestDataFpath(const std::string &data_subdir,
                             const std::string &fname);
std::string checkSubmatricesAreCorrect(Problem prob,
                                       const std::string &data_subdir);

// load problem data
Problem getProblem(std::string data_subdir);
Matrix getRandInit(std::string data_subdir);
Matrix getGroundTruthState(std::string data_subdir);
Matrix getRandDX(std::string data_subdir);

// expected values
SparseMatrix getExpectedRandCertMatrix(std::string data_subdir);
Scalar getExpectedCost(std::string data_subdir);
Matrix getExpectedEgrad(std::string data_subdir);
Matrix getExpectedRgrad(std::string data_subdir);
Matrix getExpectedHessProd(std::string data_subdir);
} // namespace CORA
