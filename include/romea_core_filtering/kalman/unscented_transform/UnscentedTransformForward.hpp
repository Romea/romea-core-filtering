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


#ifndef ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMFORWARD_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMFORWARD_HPP_

// eigen
#include <Eigen/SVD>

// std
#include <vector>

// romea
#include "romea_core_filtering/GaussianDistribution.hpp"
#include "romea_core_filtering/kalman/unscented_transform/UnscentedTransformParameters.hpp"

namespace romea
{
namespace core
{


template<typename Scalar, size_t DIM>
struct UnscentedTransformFoward
{
  static void toSigmaPoints(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianDistribution<Scalar, DIM> & gaussianDistribution,
    typename GaussianDistribution<Scalar, DIM>::SigmaPoints & sigmaPoints)
  {
    assert(sigmaPoints.size() == parameters.meanWeights.size());

    const Scalar & gamma = parameters.gamma;
    const auto & firstMoment = gaussianDistribution.firstMoment;
    const auto & secondMoment = gaussianDistribution.secondMoment;

    Eigen::JacobiSVD<Eigen::Matrix<Scalar, -1, -1>> svd(
      secondMoment, Eigen::ComputeThinU | Eigen::ComputeThinV);

    auto sqrCovariance = svd.matrixU() * Eigen::Matrix<Scalar, DIM, 1>(
      svd.singularValues().array().sqrt()).asDiagonal() * svd.matrixV().transpose();

    sigmaPoints[0] = firstMoment;
    for (size_t n = 0; n < DIM; ++n) {
      sigmaPoints[n + 1] = firstMoment + gamma * sqrCovariance.col(int(n));
      sigmaPoints[n + 1 + DIM] = firstMoment - gamma * sqrCovariance.col(int(n));
    }
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN_UNSCENTED__TRANSFORM__UNSCENTEDTRANSFORMFORWARD_HPP_
