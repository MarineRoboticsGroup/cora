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

  /** Rotational measurement precision */
  Scalar rot_precision;

  /** Translational measurement precision */
  Scalar tran_precision;

  RelativePoseMeasurement(Symbol first_id, Symbol second_id,
                          const Matrix &R_measurement,
                          const Vector &t_measurement, Scalar R_precision,
                          Scalar t_precision)
      : first_id(first_id),
        second_id(second_id),
        R(R_measurement),
        t(t_measurement),
        rot_precision(R_precision),
        tran_precision(t_precision) {}
};

struct RangeMeasurement {
  Symbol first_id;
  Symbol second_id;

  /** Range measurement */
  Scalar r;

  /** Range measurement precision */
  Scalar precision;

  RangeMeasurement(Symbol first_id, Symbol second_id, Scalar r_measurement,
                   Scalar r_precision)
      : first_id(first_id),
        second_id(second_id),
        r(r_measurement),
        precision(r_precision) {}

  std::pair<Symbol, Symbol> getSymbolPair() const {
    return std::make_pair(first_id, second_id);
  }
};

struct PosePrior {
  Symbol id;
  Matrix R;
  Vector t;
  Scalar rot_precision;
  Scalar tran_precision;
};

struct LandmarkPrior {
  Symbol id;
  Vector p;
  Scalar precision;
};

typedef std::vector<CORA::RelativePoseMeasurement> rel_pose_measurements_t;
typedef std::vector<CORA::RangeMeasurement> range_measurements_t;
typedef std::vector<CORA::PosePrior> pose_priors_t;
typedef std::vector<CORA::LandmarkPrior> landmark_priors_t;

} // namespace CORA
