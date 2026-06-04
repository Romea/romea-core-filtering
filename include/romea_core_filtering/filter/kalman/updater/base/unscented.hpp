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

#ifndef ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__BASE__UNSCENTED_HPP_
#define ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__BASE__UNSCENTED_HPP_

// std
#include <limits>

// romea
#include "romea_core_filtering/filter/kalman/updater/equation/innovation.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/mahalanobis.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/update.hpp"
#include "romea_core_filtering/unscented_transform/correlation.hpp"
#include "romea_core_filtering/unscented_transform/forward.hpp"
#include "romea_core_filtering/unscented_transform/inverse.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
class UKFUpdaterBase
{
public:
  using State = GaussianState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, ObservationDIM>;
  using CorrelationMatrix = Eigen::Matrix<Scalar, StateDIM, ObservationDIM>;

public:
  UKFUpdaterBase(
    const double & UTKappa,
    const double & UTAlpha,
    const double & UTBeta,
    const double & maximal_mahalanobis_distance);

  virtual ~UKFUpdaterBase() = default;

protected:
  bool update_state_(State & state);

  bool update_state_(State & state, const Observation & observation);

  void compute_state_sigma_points_(const State & state);

protected:
  UnscentedTransformParameters<Scalar> unscented_transform_parameters_;
  typename GaussianDistribution<Scalar, StateDIM>::SigmaPoints state_sigma_points_;
  typename GaussianDistribution<Scalar, ObservationDIM>::SigmaPoints propagated_sigma_points_;
  GaussianObservation<Scalar, ObservationDIM> propagated_state_;

  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::Inn Inn_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn QInn_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn QInnInverse_;
  typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K K_;

  Scalar mahalanobis_distance_;
  Scalar maximal_mahalanobis_distance_;
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
UKFUpdaterBase<Scalar, StateDIM, ObservationDIM>::UKFUpdaterBase(
  const double & UTKappa,
  const double & UTAlpha,
  const double & UTBeta,
  const double & maximal_mahalanobis_distance)
: unscented_transform_parameters_(StateDIM, UTKappa, UTAlpha, UTBeta),
  state_sigma_points_(2 * StateDIM + 1),
  propagated_sigma_points_(2 * StateDIM + 1),
  propagated_state_(),
  Inn_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::Inn>::zero()),
  QInn_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn>::zero()),
  QInnInverse_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn>::zero()),
  K_(Zero<typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K>::zero()),
  mahalanobis_distance_(std::numeric_limits<Scalar>::max()),
  maximal_mahalanobis_distance_(maximal_mahalanobis_distance)
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
void UKFUpdaterBase<Scalar, StateDIM, ObservationDIM>::compute_state_sigma_points_(
  const State & state)
{
  UnscentedTransformForward<Scalar, StateDIM>::to_sigma_points(
    unscented_transform_parameters_, state, state_sigma_points_);
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
bool UKFUpdaterBase<Scalar, StateDIM, ObservationDIM>::update_state_(
  State & state, const Observation & observation)
{
  UnscentedTransformInverse<Scalar, ObservationDIM>::to_gaussian(
    unscented_transform_parameters_, propagated_sigma_points_, propagated_state_);

  this->Inn_ = observation.Y() - propagated_state_.Y();
  this->QInn_ = observation.R() + propagated_state_.R();
  return update_state_(state);
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
bool UKFUpdaterBase<Scalar, StateDIM, ObservationDIM>::update_state_(State & state)
{
  mahalanobis_distance_ = KFMahalanobis<Scalar, ObservationDIM>::compute(Inn_, QInn_, QInnInverse_);
  if (mahalanobis_distance_ < maximal_mahalanobis_distance_) {
    UKFCorrelation<Scalar, StateDIM, ObservationDIM>::compute(
      unscented_transform_parameters_,
      state,
      propagated_state_,
      state_sigma_points_,
      propagated_sigma_points_,
      K_);

    K_ *= QInnInverse_;
    KFUpdateStateVector<Scalar, StateDIM, ObservationDIM>::compute(state.X(), Inn_, K_);
    KFUpdateStateCovariance<Scalar, StateDIM, ObservationDIM>::compute(state.P(), QInn_, K_);
    return true;
  } else {
    return false;
  }
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__BASE__UNSCENTED_HPP_
