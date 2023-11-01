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

  /** Covariance Matrix */
  Matrix cov;

  RelativePoseMeasurement(const Symbol &first_id, const Symbol &second_id,
                          Matrix R_measurement, Vector t_measurement,
                          Matrix cov)
      : first_id(first_id),
        second_id(second_id),
        R(std::move(R_measurement)),
        t(std::move(t_measurement)),
        cov(std::move(cov)) {}

  Scalar getRotPrecision() const { throw NotImplementedException(); }
  Scalar getTransPrecision() const { throw NotImplementedException(); }
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
