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

#ifndef ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__FORWARD_HPP_
#define ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__FORWARD_HPP_

// eigen
#include <Eigen/SVD>

// std
#include <vector>

// romea
#include "romea_core_filtering/gaussian/distribution.hpp"
#include "romea_core_filtering/unscented_transform/parameters.hpp"

namespace romea {
namespace core {

template <typename Scalar, size_t DIM>
struct UnscentedTransformForward {
  static void to_sigma_points(
      const UnscentedTransformParameters<Scalar>& parameters,
      const GaussianDistribution<Scalar, DIM>& gaussian_distribution,
      typename GaussianDistribution<Scalar, DIM>::SigmaPoints& sigma_points) {
    assert(sigma_points.size() == parameters.mean_weights.size());

    const Scalar& gamma = parameters.gamma;
    const auto& first_moment = gaussian_distribution.first_moment;
    const auto& second_moment = gaussian_distribution.second_moment;

    Eigen::JacobiSVD<Eigen::Matrix<Scalar, -1, -1>> svd(
        second_moment, Eigen::ComputeThinU | Eigen::ComputeThinV);

    auto sqrCovariance =
        svd.matrixU() *
        Eigen::Matrix<Scalar, DIM, 1>(svd.singularValues().array().sqrt())
            .asDiagonal() *
        svd.matrixV().transpose();

    sigma_points[0] = first_moment;
    for (size_t n = 0; n < DIM; ++n) {
      sigma_points[n + 1] = first_moment + gamma * sqrCovariance.col(int(n));
      sigma_points[n + 1 + DIM] =
          first_moment - gamma * sqrCovariance.col(int(n));
    }
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__FORWARD_HPP_
