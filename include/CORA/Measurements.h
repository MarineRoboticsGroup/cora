/**
 * @file RelativePoseMeasurement.h
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <CORA/CORA_types.h>
#include <CORA/Symbol.h>
#include <utility>
#include <vector>

namespace CORA {

struct RelativePoseMeasurement {
  Symbol first_id;
  Symbol second_id;

  /** Rotational measurement */
  Matrix R;

  /** Translational measurement */
  Vector t;

  /** Covariance Matrix in order of: translation, rotation */
  Matrix cov;

  RelativePoseMeasurement(const Symbol &first_id, const Symbol &second_id,
                          Matrix R_measurement, Vector t_measurement,
                          Matrix cov)
      : first_id(first_id),
        second_id(second_id),
        R(std::move(R_measurement)),
        t(std::move(t_measurement)),
        cov(std::move(cov)) {}

  /**
   * @brief Computes the rotational (scalar) precision of the measurement
   * from the covariance matrix. This is a bit more sophisticated b/c of
   * the geometry of the rotation manifold.
   *
   * @return Scalar - the rotational precision
   */
  Scalar getRotPrecision() const {
    if (cov.rows() == 6) {
      // for 3D rotations the (information-divergence minimizing) precision is:
      // 3.0 / (2*trace(cov(3:6, 3:6))))
      return 1.5 / (cov(3, 3) + cov(4, 4) + cov(5, 5));
    } else if (cov.rows() == 3) {
      // for 2D rotations, the rotational variance is a scalar, so the
      // precision is just the inverse of the variance
      return 1.0 / cov(2, 2);
    } else {
      throw std::runtime_error(
          "RelativePoseMeasurement::getRotPrecision() only implemented for "
          "2D and 3D rotations");
    }
  }

  /**
   * @brief Compute the translational (scalar) precision of the measurement
   * from the covariance matrix.
   *
   * @return Scalar - the translational precision
   */
  Scalar getTransPrecision() const {
    size_t dim = t.size();
    return dim / (cov.block(0, 0, dim, dim).trace());
  }
};

struct RangeMeasurement {
  Symbol first_id;
  Symbol second_id;

  /** Range measurement */
  Scalar r;

  /** Range measurement covariance */
  Scalar cov;

  RangeMeasurement(const Symbol &first_id, const Symbol &second_id,
                   Scalar r_measurement, Scalar cov)
      : first_id(first_id), second_id(second_id), r(r_measurement), cov(cov) {}

  std::pair<Symbol, Symbol> getSymbolPair() const {
    return std::make_pair(first_id, second_id);
  }

  Scalar getPrecision() const { return 1.0 / cov; }
};

struct PosePrior {
  Symbol id;
  Matrix R;
  Vector t;
  Matrix cov;
};

struct LandmarkPrior {
  Symbol id;
  Vector p;
  Matrix cov;
};

typedef std::vector<CORA::RelativePoseMeasurement> rel_pose_measurements_t;
typedef std::vector<CORA::RangeMeasurement> range_measurements_t;
typedef std::vector<CORA::PosePrior> pose_priors_t;
typedef std::vector<CORA::LandmarkPrior> landmark_priors_t;

} // namespace CORA
