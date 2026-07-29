// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__BASE__LINEAR_HPP_
#define ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__BASE__LINEAR_HPP_

// std
#include <limits>

// romea
#include "romea_core_filtering/filter/kalman/updater/equation/gain.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/innovation.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/mahalanobis.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/update.hpp"
#include "romea_core_filtering/gaussian/state.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
class LKFUpdaterBase
{
public:
  using Traits = KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>;
  using State = GaussianState<Scalar, StateDIM>;
  using Inn = typename Traits::Inn;
  using QInn = typename Traits::QInn;
  using H = typename Traits::H;
  using K = typename Traits::K;

public:
  explicit LKFUpdaterBase(const Scalar & maximal_mahalanobis_distance)
  : Inn_(Zero<Inn>::zero()),
    QInn_(Zero<QInn>::zero()),
    QInnInverse_(Zero<QInn>::zero()),
    H_(Zero<H>::zero()),
    K_(Zero<K>::zero()),
    mahalanobis_distance_(std::numeric_limits<Scalar>::max()),
    maximal_mahalanobis_distance_(maximal_mahalanobis_distance)
  {
  }

  virtual ~LKFUpdaterBase() = default;

protected:
  bool update_state_(State & state, const QInn & observation_covariance)
  {
    mahalanobis_distance_ =
      KFMahalanobis<Scalar, ObservationDIM>::compute(Inn_, QInn_, QInnInverse_);

    if (mahalanobis_distance_ < maximal_mahalanobis_distance_) {
      KFGain<Scalar, StateDIM, ObservationDIM>::compute(state.P(), H_, QInnInverse_, K_);
      KFUpdateStateVector<Scalar, StateDIM, ObservationDIM>::compute(state.X(), Inn_, K_);
      KFUpdateStateCovariance<Scalar, StateDIM, ObservationDIM>::compute_joseph(
        state.P(), observation_covariance, H_, K_);
      return true;
    } else {
      return false;
    }
  }

protected:
  Inn Inn_;
  QInn QInn_;
  QInn QInnInverse_;
  H H_;
  K K_;
  Scalar mahalanobis_distance_;
  Scalar maximal_mahalanobis_distance_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__BASE__LINEAR_HPP_
