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


#ifndef ROMEA_CORE_FILTERING__KALMAN__CORE__KFMAHALANOBIS_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__CORE__KFMAHALANOBIS_HPP_

// std
#include <limits>


// romea
#include "romea_core_filtering/GaussianDistribution.hpp"
#include "romea_core_filtering/kalman/core/KFUpdaterTraits.hpp"


namespace romea
{


//-----------------------------------------------------------------------------
template<typename Scalar, size_t ObservationDIM>
struct KFMahalanobis
{
  static Scalar compute(
    const Eigen::Matrix<Scalar, ObservationDIM, 1> & Inn,
    const Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> & QInn,
    Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> & QInnInverse)
  {
    // Check if QInn_ is inversible
    if (QInn.determinant() < std::numeric_limits<double>::epsilon()) {
      throw std::runtime_error("Innovation covariance cannot be inversed");
    }

    QInnInverse = QInn.ldlt().solve(
      Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM>::Identity());

    return std::sqrt((Inn.transpose() * QInnInverse * Inn)(0, 0));
  }
};


//-----------------------------------------------------------------------------
template<typename Scalar>
struct KFMahalanobis<Scalar, 1>
{
  static Scalar compute(const Scalar & Inn, const Scalar & QInn, Scalar & QInnInverse)
  {
    QInnInverse = 1 / QInn;
    return std::sqrt(Inn * Inn * QInnInverse);
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__CORE__KFMAHALANOBIS_HPP_
