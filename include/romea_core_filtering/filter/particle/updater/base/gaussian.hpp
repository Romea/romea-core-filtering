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

#ifndef ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__BASE__GAUSSIAN_HPP_
#define ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__BASE__GAUSSIAN_HPP_

// std
#include <limits>

// romea
#include "romea_core_filtering/filter/kalman/updater/equation/mahalanobis.hpp"
#include "romea_core_filtering/filter/particle/updater/base/generic.hpp"
#include "romea_core_filtering/gaussian/observation.hpp"

namespace romea {
namespace core {

template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
class PFGaussianUpdaterBase
    : public PFUpdaterBase<Scalar, StateDIM, ObservationDIM> {
 public:
  using State = ParticleFilterState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, ObservationDIM>;
  using Observations = Eigen::Array<Scalar, ObservationDIM, Eigen::Dynamic>;
  using RawMajorVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

 public:
  PFGaussianUpdaterBase(const std::size_t& number_of_particles,
                        const double& maximal_mahalanobis_distance);

  virtual ~PFGaussianUpdaterBase() = default;

 protected:
  bool check_mahalanobis_distance_();

  virtual void compute_innovation_(const Observation& observation,
                                   const RawMajorVector& weights);

  bool update_state_(State& state, const Observation& observation);

 protected:
  Observations apriori_observations_;
  Observations apriori_mean_centered_observations_;
  GaussianObservation<Scalar, ObservationDIM> apriori_observation_;

  Eigen::Matrix<Scalar, ObservationDIM, 1> Inn_;
  Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> QInn_;
  Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> QInnInverse_;
  double maximal_mahalanobis_distance_;
  double mahalanobis_distance_;
};

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
PFGaussianUpdaterBase<Scalar, StateDIM, ObservationDIM>::PFGaussianUpdaterBase(
    const std::size_t& number_of_particles,
    const double& maximal_mahalanobis_distance)
    : PFUpdaterBase<Scalar, StateDIM, ObservationDIM>(number_of_particles),
      apriori_observations_(),
      apriori_mean_centered_observations_(),
      apriori_observation_(),
      Inn_(),
      QInn_(),
      QInnInverse_(),
      maximal_mahalanobis_distance_(maximal_mahalanobis_distance),
      mahalanobis_distance_(std::numeric_limits<double>::max()) {}

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
void PFGaussianUpdaterBase<Scalar, StateDIM, ObservationDIM>::
    compute_innovation_(const Observation& observation,
                        const RawMajorVector& weights) {
  Scalar weight_sum = weights.sum();

  for (size_t i = 0; i < ObservationDIM; ++i) {
    apriori_observation_.Y(i) =
        (apriori_observations_.row(i) * weights).sum() / weight_sum;
    apriori_mean_centered_observations_.row(i) =
        apriori_observations_.row(i) - apriori_observation_.Y(i);
  }

  for (size_t i = 0; i < ObservationDIM; ++i) {
    for (size_t j = i; j < ObservationDIM; ++j) {
      apriori_observation_.R(i, j) = apriori_observation_.R(j, i) =
          (apriori_mean_centered_observations_.row(i) *
           apriori_mean_centered_observations_.row(j) * weights)
              .sum() /
          weight_sum;
    }
  }

  this->Inn_ = observation.Y() - apriori_observation_.Y();
  this->QInn_ = observation.R() + apriori_observation_.R();
}

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
bool PFGaussianUpdaterBase<Scalar, StateDIM, ObservationDIM>::update_state_(
    State& state, const Observation& observation) {
  compute_innovation_(observation, state.weights);
  mahalanobis_distance_ =
      KFMahalanobis<Scalar, ObservationDIM>::compute(Inn_, QInn_, QInnInverse_);
  if (mahalanobis_distance_ < maximal_mahalanobis_distance_) {
    for (int i = 0; i < ObservationDIM; ++i) {
      state.weights *=
          (-QInnInverse_(i, i) *
           (apriori_observations_.row(i) - observation.Y(i)).square() / 2.)
              .exp();

      for (int j = i; j < ObservationDIM; ++j) {
        state.weights *= (-QInnInverse_(i, j) *
                          (apriori_observations_.row(i) - observation.Y(i)) *
                          (apriori_observations_.row(j) - observation.Y(j)))
                             .exp();
      }
    }

    this->resampling_.resampling(
        state, ParticleFilterResamplingScheme::MUTINOMIAL, 0.8);
    return true;
  } else {
    return false;
  }
}

template <typename Scalar, size_t StateDIM>
class PFGaussianUpdaterBase<Scalar, StateDIM, 1>
    : public PFUpdaterBase<Scalar, StateDIM, 1> {
 public:
  using State = ParticleFilterState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, 1>;
  using ObservationVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;
  using WeightVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

 public:
  PFGaussianUpdaterBase(const std::size_t& number_of_particles,
                        const double& maximal_mahalanobis_distance);

  virtual ~PFGaussianUpdaterBase() = default;

 protected:
  bool check_mahalanobis_distance_();

  void compute_innovation_(const Observation& observation,
                           const WeightVector& weights);

  bool update_state_(State& state, const Observation& observation);

 protected:
  ObservationVector apriori_observations_;
  GaussianObservation<Scalar, 1> apriori_observation_;

  double Inn_;
  double QInn_;
  double QInnInverse_;
  double maximal_mahalanobis_distance_;
  double mahalanobis_distance_;
};

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM>
PFGaussianUpdaterBase<Scalar, StateDIM, 1>::PFGaussianUpdaterBase(
    const std::size_t& number_of_particles,
    const double& maximal_mahalanobis_distance)
    : PFUpdaterBase<Scalar, StateDIM, 1>(number_of_particles),
      apriori_observations_(ObservationVector::Zero(number_of_particles)),
      apriori_observation_(),
      Inn_(0),
      QInn_(0),
      QInnInverse_(0),
      maximal_mahalanobis_distance_(maximal_mahalanobis_distance),
      mahalanobis_distance_(std::numeric_limits<double>::max()) {}

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM>
void PFGaussianUpdaterBase<Scalar, StateDIM, 1>::compute_innovation_(
    const Observation& observation, const WeightVector& weights) {
  apriori_observation_.Y() = (apriori_observations_ * weights).sum();
  apriori_observation_.R() =
      ((apriori_observations_ - apriori_observation_.Y()) * weights)
          .square()
          .sum();
  this->Inn_ = observation.Y() - apriori_observation_.Y();
  this->QInn_ = observation.R() + apriori_observation_.R();
}

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM>
bool PFGaussianUpdaterBase<Scalar, StateDIM, 1>::update_state_(
    State& state, const Observation& observation) {
  compute_innovation_(observation, state.weights);
  mahalanobis_distance_ =
      KFMahalanobis<Scalar, 1>::compute(Inn_, QInn_, QInnInverse_);
  if (mahalanobis_distance_ < maximal_mahalanobis_distance_) {
    state.weights *= (-QInnInverse_ *
                      (apriori_observations_ - observation.Y()).square() / 2.)
                         .exp();
    this->resampling_.resampling(
        state, ParticleFilterResamplingScheme::MUTINOMIAL, 0.8);
    return true;
  } else {
    return false;
  }
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__BASE__GAUSSIAN_HPP_
