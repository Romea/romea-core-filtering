// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_CORE_FILTERING__KALMAN__CORE__KFINNOVATION_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__CORE__KFINNOVATION_HPP_


#include "romea_core_filtering/GaussianDistribution.hpp"


namespace romea
{
namespace core
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


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__CORE__KFINNOVATION_HPP_
