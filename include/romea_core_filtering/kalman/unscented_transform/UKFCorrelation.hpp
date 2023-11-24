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


#ifndef ROMEA_CORE_FILTERING__KALMAN__UNSCENTED__TRANSFORM__UKFCORRELATION_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__UNSCENTED__TRANSFORM__UKFCORRELATION_HPP_

#include "romea_core_filtering/GaussianState.hpp"
#include "romea_core_filtering/GaussianObservation.hpp"
#include "romea_core_filtering/kalman/unscented_transform/UnscentedTransformParameters.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct UKFCorrelation
{
  static void compute(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianState<Scalar, StateDIM> & state,
    const GaussianObservation<Scalar, ObservationDIM> & propagatedState,
    const typename GaussianState<Scalar, StateDIM>::SigmaPoints & stateSigmaPoints,
    const typename GaussianDistribution<Scalar,
    ObservationDIM>::SigmaPoints & propagatedSigmaPoints,
    Eigen::Matrix<Scalar, StateDIM, ObservationDIM> & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 2 * StateDIM + 1);
    assert(propagatedSigmaPoints.size() == 2 * StateDIM + 1);
    assert(parameters.covarianceWeights.size() == 2 * StateDIM + 1);

    const auto & stateFirstMoment = state.firstMoment;
    const auto & propagatedFirstMoment = propagatedState.firstMoment;
    const auto & covarianceWeights = parameters.covarianceWeights;

    correlationMatrix.setConstant(0);
    for (size_t n = 0; n < 2 * StateDIM + 1; ++n) {
      correlationMatrix += covarianceWeights[n] *
        (stateSigmaPoints[n] - stateFirstMoment) *
        (propagatedSigmaPoints[n] - propagatedFirstMoment).transpose();
    }
  }
};

template<typename Scalar, size_t StateDIM>
struct UKFCorrelation<Scalar, StateDIM, 1>
{
  static void compute(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianState<Scalar, StateDIM> & state,
    const GaussianObservation<Scalar, 1> & propagatedState,
    const typename GaussianState<Scalar, StateDIM>::SigmaPoints & stateSigmaPoints,
    const typename GaussianObservation<Scalar, 1>::SigmaPoints & propagatedSigmaPoints,
    Eigen::Matrix<Scalar, StateDIM, 1> & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 2 * StateDIM + 1);
    assert(propagatedSigmaPoints.size() == 2 * StateDIM + 1);
    assert(parameters.covarianceWeights.size() == 2 * StateDIM + 1);

    const auto & stateFirstMoment = state.firstMoment;
    const auto & propagatedFirstMoment = propagatedState.firstMoment;
    const auto & covarianceWeights = parameters.covarianceWeights;

    correlationMatrix.setConstant(0);
    for (size_t n = 0; n < 2 * StateDIM + 1; ++n) {
      correlationMatrix += covarianceWeights[n] *
        (stateSigmaPoints[n] - stateFirstMoment) *
        (propagatedSigmaPoints[n] - propagatedFirstMoment);
    }
  }
};

template<typename Scalar>
struct UKFCorrelation<Scalar, 1, 1>
{
  static void compute(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianState<Scalar, 1> & state,
    const GaussianObservation<Scalar, 1> & propagatedState,
    const typename GaussianState<Scalar, 1>::SigmaPoints & stateSigmaPoints,
    const typename GaussianObservation<Scalar, 1>::SigmaPoints & propagatedSigmaPoints,
    double & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 3);
    assert(propagatedSigmaPoints.size() == 3);
    assert(parameters.covarianceWeights.size() == 3);

    const auto & stateFirstMoment = state.firstMoment;
    const auto & propagatedFirstMoment = propagatedState.firstMoment;
    const auto & covarianceWeights = parameters.covarianceWeights;

    correlationMatrix = 0;
    for (size_t n = 0; n < 3; ++n) {
      correlationMatrix += covarianceWeights[n] *
        (stateSigmaPoints[n] - stateFirstMoment) *
        (propagatedSigmaPoints[n] - propagatedFirstMoment);
    }
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__UNSCENTED__TRANSFORM__UKFCORRELATION_HPP_
