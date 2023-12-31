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


#ifndef ROMEA_CORE_FILTERING__KALMAN__KALMANFILTERUPDATERCORE_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__KALMANFILTERUPDATERCORE_HPP_

// std
#include <limits>

// romea
#include "romea_core_filtering/kalman/core/KFGain.hpp"
#include "romea_core_filtering/kalman/core/KFUpdate.hpp"
#include "romea_core_filtering/kalman/core/KFInnovation.hpp"
#include "romea_core_filtering/kalman/core/KFMahalanobis.hpp"
#include "romea_core_filtering/GaussianState.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
class KFUpdaterCore
{
public:
  using State = GaussianState<Scalar, StateDIM>;

public:
  explicit KFUpdaterCore(const Scalar & maximalMahalanobisDistance)
  : Inn_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::Inn>::zero()),
    QInn_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn>::zero()),
    QInnInverse_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn>::zero()),
    H_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::H>::zero()),
    K_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K>::zero()),
    mahalanobisDistance_(std::numeric_limits<Scalar>::max()),
    maximalMahalanobisDistance_(maximalMahalanobisDistance)
  {
  }

  virtual ~KFUpdaterCore() = default;

protected:
  bool updateState_(State & state)
  {
    mahalanobisDistance_ = KFMahalanobis<Scalar, ObservationDIM>::
      compute(Inn_, QInn_, QInnInverse_);

    if (mahalanobisDistance_ < maximalMahalanobisDistance_) {
      KFGain<Scalar, StateDIM, ObservationDIM>::compute(state.P(), H_, QInnInverse_, K_);
      KFUpdateStateVector<Scalar, StateDIM, ObservationDIM>::compute(state.X(), Inn_, K_);
      KFUpdateStateCovariance<Scalar, StateDIM, ObservationDIM>::compute(state.P(), QInn_, K_);
      return true;
    } else {
      return false;
    }
  }

protected:
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::Inn Inn_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn QInn_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn QInnInverse_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::H H_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K K_;
  Scalar mahalanobisDistance_;
  Scalar maximalMahalanobisDistance_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__KALMANFILTERUPDATERCORE_HPP_
