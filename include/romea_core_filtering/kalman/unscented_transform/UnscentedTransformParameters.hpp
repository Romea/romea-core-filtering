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


#ifndef ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMPARAMETERS_HPP
#define ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMPARAMETERS_HPP

// std
#include <vector>

// romea
#include "romea_core_filtering/GaussianDistribution.hpp"


namespace romea
{
namespace core
{


template<typename Scalar>
struct UnscentedTransformParameters
{
  UnscentedTransformParameters(
    const size_t & DIM,
    const double & kappa,
    const double & alpha,
    const double & beta)
  : gamma(0),
    meanWeights(2 * DIM + 1),
    covarianceWeights(2 * DIM + 1)
  {
    double lambda = (alpha * alpha * (DIM + kappa) - DIM);
    gamma = std::sqrt(DIM + lambda);

    meanWeights[0] = lambda / (DIM + lambda);
    covarianceWeights[0] = meanWeights[0] + 1 + alpha * alpha + beta;
    std::fill(std::begin(meanWeights) + 1, std::end(meanWeights), 1 / (2 * (DIM + lambda)));
    std::fill(
      std::begin(covarianceWeights) + 1, std::end(covarianceWeights),
      1 / (2 * (DIM + lambda)));
  }

  double gamma;
  std::vector<Scalar> meanWeights;
  std::vector<Scalar> covarianceWeights;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMPARAMETERS_HPP
