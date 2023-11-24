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


#ifndef ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERGAUSSIANUPDATERCORE_HPP_
#define ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERGAUSSIANUPDATERCORE_HPP_

// std
#include <limits>

// romea
#include "romea_core_filtering/particle/ParticleFilterUpdaterCore.hpp"
#include "romea_core_filtering/GaussianObservation.hpp"
#include "romea_core_filtering/kalman/core/KFMahalanobis.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
class PFGaussianUpdaterCore : public PFUpdaterCore<Scalar, StateDIM, ObservationDIM>
{
public:
  using State = ParticleFilterState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, ObservationDIM>;
  using Observations = Eigen::Array<Scalar, ObservationDIM, Eigen::Dynamic>;
  using RawMajorVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

public:
  PFGaussianUpdaterCore(
    const std::size_t & numberOfParticles,
    const double & maximalMahalanobisDistance);

  virtual ~PFGaussianUpdaterCore() = default;

protected:
  bool checkMahalanobisDistance_();

  virtual void computeInnovation_(const Observation & observation, const RawMajorVector & weights);

  bool updateState_(State & state, const Observation & observation);

protected:
  Observations aprioriObservations_;
  Observations aprioriMeanCenteredObservations_;
  GaussianObservation<Scalar, ObservationDIM> aprioriObservation_;

  Eigen::Matrix<Scalar, ObservationDIM, 1> Inn_;
  Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> QInn_;
  Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> QInnInverse_;
  double maximalMahalanobisDistance_;
  double mahalanobisDistance_;
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
PFGaussianUpdaterCore<Scalar, StateDIM, ObservationDIM>::PFGaussianUpdaterCore(
  const std::size_t & numberOfParticles,
  const double & maximalMahalanobisDistance)
: PFUpdaterCore<Scalar, StateDIM, ObservationDIM>(numberOfParticles),
  aprioriObservations_(),
  aprioriMeanCenteredObservations_(),
  aprioriObservation_(),
  Inn_(),
  QInn_(),
  QInnInverse_(),
  maximalMahalanobisDistance_(maximalMahalanobisDistance),
  mahalanobisDistance_(std::numeric_limits<double>::max())
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
void
PFGaussianUpdaterCore<Scalar, StateDIM, ObservationDIM>::computeInnovation_(
  const Observation & observation,
  const RawMajorVector & weights)
{
  Scalar weightSum = weights.sum();

  for (size_t i = 0; i < ObservationDIM; ++i) {
    aprioriObservation_.Y(i) = (aprioriObservations_.row(i) * weights).sum() / weightSum;
    aprioriMeanCenteredObservations_.row(i) = aprioriObservations_.row(i) -
      aprioriObservation_.Y(i);
  }

  for (size_t i = 0; i < ObservationDIM; ++i) {
    for (size_t j = i; j < ObservationDIM; ++j) {
      aprioriObservation_.R(i, j) = aprioriObservation_.R(j, i) =
        (aprioriMeanCenteredObservations_.row(i) *
        aprioriMeanCenteredObservations_.row(j) *
        weights).sum() / weightSum;
    }
  }

  this->Inn_ = observation.Y() - aprioriObservation_.Y();
  this->QInn_ = observation.R() + aprioriObservation_.R();
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
bool PFGaussianUpdaterCore<Scalar, StateDIM, ObservationDIM>::updateState_(
  State & state,
  const Observation & observation)
{
  computeInnovation_(observation, state.weights);
  mahalanobisDistance_ = KFMahalanobis<Scalar, ObservationDIM>::compute(Inn_, QInn_, QInnInverse_);
  if (mahalanobisDistance_ < maximalMahalanobisDistance_) {
    for (int i = 0; i < ObservationDIM; ++i) {
      state.weights *= (-QInnInverse_(i, i) *
        (aprioriObservations_.row(i) - observation.Y(i)).square() / 2.).exp();

      for (int j = i; j < ObservationDIM; ++j) {
        state.weights *= (-QInnInverse_(i, j) *
          (aprioriObservations_.row(i) - observation.Y(i)) *
          (aprioriObservations_.row(j) - observation.Y(j))).exp();
      }
    }

    this->resampling_.resampling(state, ParticleFilterResamplingScheme::MUTINOMIAL, 0.8);
    return true;
  } else {
    return false;
  }
}


template<typename Scalar, size_t StateDIM>
class PFGaussianUpdaterCore<Scalar, StateDIM, 1>: public PFUpdaterCore<Scalar, StateDIM, 1>
{
public:
  using State = ParticleFilterState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, 1>;
  using ObservationVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;
  using WeightVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

public:
  PFGaussianUpdaterCore(
    const std::size_t & numberOfParticles,
    const double & maximalMahalanobisDistance);

  virtual ~PFGaussianUpdaterCore() = default;

protected:
  bool checkMahalanobisDistance_();

  void computeInnovation_(const Observation & observation, const WeightVector & weights);

  bool updateState_(State & state, const Observation & observation);

protected:
  ObservationVector aprioriObservations_;
  GaussianObservation<Scalar, 1> aprioriObservation_;

  double Inn_;
  double QInn_;
  double QInnInverse_;
  double maximalMahalanobisDistance_;
  double mahalanobisDistance_;
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM>
PFGaussianUpdaterCore<Scalar, StateDIM, 1>::PFGaussianUpdaterCore(
  const std::size_t & numberOfParticles,
  const double & maximalMahalanobisDistance)
: PFUpdaterCore<Scalar, StateDIM, 1>(numberOfParticles),
  aprioriObservations_(ObservationVector::Zero(numberOfParticles)),
  aprioriObservation_(),
  Inn_(0),
  QInn_(0),
  QInnInverse_(0),
  maximalMahalanobisDistance_(maximalMahalanobisDistance),
  mahalanobisDistance_(std::numeric_limits<double>::max())
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM>
void PFGaussianUpdaterCore<Scalar, StateDIM, 1>::computeInnovation_(
  const Observation & observation,
  const WeightVector & weights)
{
  aprioriObservation_.Y() = (aprioriObservations_ * weights).sum();
  aprioriObservation_.R() =
    ((aprioriObservations_ - aprioriObservation_.Y()) * weights).square().sum();
  this->Inn_ = observation.Y() - aprioriObservation_.Y();
  this->QInn_ = observation.R() + aprioriObservation_.R();
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM>
bool PFGaussianUpdaterCore<Scalar, StateDIM, 1>::updateState_(
  State & state,
  const Observation & observation)
{
  computeInnovation_(observation, state.weights);
  mahalanobisDistance_ = KFMahalanobis<Scalar, 1>::compute(Inn_, QInn_, QInnInverse_);
  if (mahalanobisDistance_ < maximalMahalanobisDistance_) {
    state.weights *= (-QInnInverse_ * (aprioriObservations_ - observation.Y()).square() / 2.).exp();
    this->resampling_.resampling(state, ParticleFilterResamplingScheme::MUTINOMIAL, 0.8);
    return true;
  } else {
    return false;
  }
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERGAUSSIANUPDATERCORE_HPP_
