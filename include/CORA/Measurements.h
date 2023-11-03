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

struct Measurement {
  Symbol id;

  explicit Measurement(const Symbol &id) : id(id) {}
  bool operator==(const Measurement &other) const { return id == other.id; }
};

struct PairMeasurement {
  Symbol first_id;
  Symbol second_id;

  PairMeasurement(const Symbol &first_id, const Symbol &second_id)
      : first_id(first_id), second_id(second_id) {}

  SymbolPair getSymbolPair() const {
    return std::make_pair(first_id, second_id);
  }

  bool hasSymbolPair(const SymbolPair &pair) const {
    return (first_id == pair.first && second_id == pair.second) ||
           (first_id == pair.second && second_id == pair.first);
  }

  bool operator==(const PairMeasurement &other) const {
    return hasSymbolPair(other.getSymbolPair());
  }
};

struct RelativePoseMeasurement : PairMeasurement {
  /** Rotational measurement */
  Matrix R;

  /** Translational measurement */
  Vector t;

  /** Covariance Matrix in order of: translation, rotation */
  Matrix cov;

  RelativePoseMeasurement(const Symbol &first_id, const Symbol &second_id,
                          Matrix R_measurement, Vector t_measurement,
                          Matrix cov)
      : PairMeasurement(first_id, second_id),
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
    return static_cast<double>(dim) / (cov.block(0, 0, dim, dim).trace());
  }
};

struct RangeMeasurement : PairMeasurement {
  /** Range measurement */
  Scalar r;

  /** Range measurement covariance */
  Scalar cov;

  RangeMeasurement(const Symbol &first_id, const Symbol &second_id,
                   Scalar r_measurement, Scalar cov)
      : PairMeasurement(first_id, second_id), r(r_measurement), cov(cov) {}

  Scalar getPrecision() const { return 1.0 / cov; }
};

struct PosePrior : Measurement {
  Matrix R;
  Vector t;
  Matrix cov;

  PosePrior(const Symbol &id, Matrix R, Vector t, Matrix cov)
      : Measurement(id),
        R(std::move(R)),
        t(std::move(t)),
        cov(std::move(cov)) {}
};

struct LandmarkPrior : Measurement {
  Vector p;
  Matrix cov;

  LandmarkPrior(const Symbol &id, Vector p, Matrix cov)
      : Measurement(id), p(std::move(p)), cov(std::move(cov)) {}
};

typedef std::vector<CORA::RelativePoseMeasurement> rel_pose_measurements_t;
typedef std::vector<CORA::RangeMeasurement> range_measurements_t;
typedef std::vector<CORA::PosePrior> pose_priors_t;
typedef std::vector<CORA::LandmarkPrior> landmark_priors_t;

} // namespace CORA
