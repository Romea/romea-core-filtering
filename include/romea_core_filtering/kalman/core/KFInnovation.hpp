// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_FILTERING__KALMAN__CORE__KFINNOVATION_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__CORE__KFINNOVATION_HPP_


#include "romea_core_filtering/GaussianDistribution.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
inline void computeInnovationCovariance(
  const Eigen::Matrix<Scalar, StateDIM, StateDIM> & P,
  const Eigen::Matrix<Scalar, StateDIM, ObservationDIM> & H,
  const Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> & R,
  Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> & QInn)
{
  QInn = H * P * H.transpose() + R;
}

//-----------------------------------------------------------------------------
template<typename Scalar>
inline void computeInnovationCovariance(
  const double & P,
  const double & H,
  const double & R,
  double & QInn)
{
  QInn = H * P * H + R;
}


}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__CORE__KFINNOVATION_HPP_
