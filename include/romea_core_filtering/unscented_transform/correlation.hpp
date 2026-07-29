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

#ifndef ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__CORRELATION_HPP_
#define ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__CORRELATION_HPP_

#include "romea_core_filtering/gaussian/observation.hpp"
#include "romea_core_filtering/gaussian/state.hpp"
#include "romea_core_filtering/unscented_transform/parameters.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct UKFCorrelation
{
  using Parameters = UnscentedTransformParameters<Scalar>;
  using State = GaussianState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, ObservationDIM>;
  using StateSigmaPoints = typename State::SigmaPoints;
  using ObservationSigmaPoints = typename Observation::SigmaPoints;
  using CorrelationMatrix = Eigen::Matrix<Scalar, StateDIM, ObservationDIM>;

  static void compute(
    const Parameters & parameters,
    const State & state,
    const Observation & propagatedState,
    const StateSigmaPoints & stateSigmaPoints,
    const ObservationSigmaPoints & propagatedSigmaPoints,
    CorrelationMatrix & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 2 * StateDIM + 1);
    assert(propagatedSigmaPoints.size() == 2 * StateDIM + 1);
    assert(parameters.covariance_weights.size() == 2 * StateDIM + 1);

    const auto & stateFirstMoment = state.first_moment;
    const auto & propagatedFirstMoment = propagatedState.first_moment;
    const auto & covariance_weights = parameters.covariance_weights;

    correlationMatrix.setConstant(0);
    for (size_t n = 0; n < 2 * StateDIM + 1; ++n) {
      correlationMatrix += covariance_weights[n] * (stateSigmaPoints[n] - stateFirstMoment) *
                           (propagatedSigmaPoints[n] - propagatedFirstMoment).transpose();
    }
  }
};

template<typename Scalar, size_t StateDIM>
struct UKFCorrelation<Scalar, StateDIM, 1>
{
  using Parameters = UnscentedTransformParameters<Scalar>;
  using State = GaussianState<Scalar, StateDIM>;
  using Observation = GaussianObservation<Scalar, 1>;
  using StateSigmaPoints = typename State::SigmaPoints;
  using ObservationSigmaPoints = typename Observation::SigmaPoints;
  using CorrelationMatrix = Eigen::Matrix<Scalar, StateDIM, 1>;

  static void compute(
    const Parameters & parameters,
    const State & state,
    const Observation & propagatedState,
    const StateSigmaPoints & stateSigmaPoints,
    const ObservationSigmaPoints & propagatedSigmaPoints,
    CorrelationMatrix & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 2 * StateDIM + 1);
    assert(propagatedSigmaPoints.size() == 2 * StateDIM + 1);
    assert(parameters.covariance_weights.size() == 2 * StateDIM + 1);

    const auto & stateFirstMoment = state.first_moment;
    const auto & propagatedFirstMoment = propagatedState.first_moment;
    const auto & covariance_weights = parameters.covariance_weights;

    correlationMatrix.setConstant(0);
    for (size_t n = 0; n < 2 * StateDIM + 1; ++n) {
      correlationMatrix += covariance_weights[n] * (stateSigmaPoints[n] - stateFirstMoment) *
                           (propagatedSigmaPoints[n] - propagatedFirstMoment);
    }
  }
};

template<typename Scalar>
struct UKFCorrelation<Scalar, 1, 1>
{
  using Parameters = UnscentedTransformParameters<Scalar>;
  using State = GaussianState<Scalar, 1>;
  using Observation = GaussianObservation<Scalar, 1>;
  using StateSigmaPoints = typename State::SigmaPoints;
  using ObservationSigmaPoints = typename Observation::SigmaPoints;

  static void compute(
    const Parameters & parameters,
    const State & state,
    const Observation & propagatedState,
    const StateSigmaPoints & stateSigmaPoints,
    const ObservationSigmaPoints & propagatedSigmaPoints,
    double & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 3);
    assert(propagatedSigmaPoints.size() == 3);
    assert(parameters.covariance_weights.size() == 3);

    const auto & stateFirstMoment = state.first_moment;
    const auto & propagatedFirstMoment = propagatedState.first_moment;
    const auto & covariance_weights = parameters.covariance_weights;

    correlationMatrix = 0;
    for (size_t n = 0; n < 3; ++n) {
      correlationMatrix += covariance_weights[n] * (stateSigmaPoints[n] - stateFirstMoment) *
                           (propagatedSigmaPoints[n] - propagatedFirstMoment);
    }
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__CORRELATION_HPP_
