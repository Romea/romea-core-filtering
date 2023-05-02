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


#ifndef ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMINVERSE_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMINVERSE_HPP_

// std
#include <vector>

// romea
#include "romea_core_filtering/GaussianDistribution.hpp"
#include "romea_core_filtering/kalman/unscented_transform/UnscentedTransformParameters.hpp"

namespace romea
{

template<typename Scalar, size_t DIM>
struct UnscentedTransformInverse
{
  static void toGaussian(
    const UnscentedTransformParameters<Scalar> & parameters,
    const typename GaussianDistribution<Scalar, DIM>::SigmaPoints & sigmaPoints,
    GaussianDistribution<Scalar, DIM> & gaussianDistribution)
  {
    assert(parameters.meanWeights.size() == sigmaPoints.size());

    const auto & meanWeights = parameters.meanWeights;
    const auto & covarianceWeights = parameters.covarianceWeights;
    auto & firstMoment = gaussianDistribution.firstMoment;
    auto & secondMoment = gaussianDistribution.secondMoment;

    firstMoment.setConstant(0);
    for (size_t n = 0; n < sigmaPoints.size(); ++n) {
      firstMoment += meanWeights[n] * sigmaPoints[n];
    }

    secondMoment.setConstant(0);
    for (size_t n = 0; n < sigmaPoints.size(); ++n) {
      secondMoment += covarianceWeights[n] * (sigmaPoints[n] - firstMoment) *
        (sigmaPoints[n] - firstMoment).transpose();
    }
  }
};

template<typename Scalar>
struct UnscentedTransformInverse<Scalar, 1>
{
  static void toGaussian(
    const UnscentedTransformParameters<Scalar> & parameters,
    const typename GaussianDistribution<Scalar, 1>::SigmaPoints & sigmaPoints,
    GaussianDistribution<Scalar, 1> & gaussianDistribution)
  {
    assert(parameters.meanWeights.size() == sigmaPoints.size());

    const auto & meanWeights = parameters.meanWeights;
    const auto & covarianceWeights = parameters.covarianceWeights;
    auto & firstMoment = gaussianDistribution.firstMoment;
    auto & secondMoment = gaussianDistribution.secondMoment;

    firstMoment = 0;
    for (size_t n = 0; n < sigmaPoints.size(); ++n) {
      firstMoment += meanWeights[n] * sigmaPoints[n];
    }

    secondMoment = 0;
    for (size_t n = 0; n < sigmaPoints.size(); ++n) {
      secondMoment += covarianceWeights[n] * std::pow(sigmaPoints[n] - firstMoment, 2);
    }
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMINVERSE_HPP_
