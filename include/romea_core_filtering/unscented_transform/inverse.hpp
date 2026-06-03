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

#ifndef ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__INVERSE_HPP_
#define ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__INVERSE_HPP_

// std
#include <vector>

// romea
#include "romea_core_filtering/gaussian/distribution.hpp"
#include "romea_core_filtering/unscented_transform/parameters.hpp"

namespace romea {
namespace core {

template <typename Scalar, size_t DIM>
struct UnscentedTransformInverse {
  static void to_gaussian(
      const UnscentedTransformParameters<Scalar>& parameters,
      const typename GaussianDistribution<Scalar, DIM>::SigmaPoints&
          sigma_points,
      GaussianDistribution<Scalar, DIM>& gaussian_distribution) {
    assert(parameters.mean_weights.size() == sigma_points.size());

    const auto& mean_weights = parameters.mean_weights;
    const auto& covariance_weights = parameters.covariance_weights;
    auto& first_moment = gaussian_distribution.first_moment;
    auto& second_moment = gaussian_distribution.second_moment;

    first_moment.setConstant(0);
    for (size_t n = 0; n < sigma_points.size(); ++n) {
      first_moment += mean_weights[n] * sigma_points[n];
    }

    second_moment.setConstant(0);
    for (size_t n = 0; n < sigma_points.size(); ++n) {
      second_moment += covariance_weights[n] *
                       (sigma_points[n] - first_moment) *
                       (sigma_points[n] - first_moment).transpose();
    }
  }
};

template <typename Scalar>
struct UnscentedTransformInverse<Scalar, 1> {
  static void to_gaussian(
      const UnscentedTransformParameters<Scalar>& parameters,
      const typename GaussianDistribution<Scalar, 1>::SigmaPoints& sigma_points,
      GaussianDistribution<Scalar, 1>& gaussian_distribution) {
    assert(parameters.mean_weights.size() == sigma_points.size());

    const auto& mean_weights = parameters.mean_weights;
    const auto& covariance_weights = parameters.covariance_weights;
    auto& first_moment = gaussian_distribution.first_moment;
    auto& second_moment = gaussian_distribution.second_moment;

    first_moment = 0;
    for (size_t n = 0; n < sigma_points.size(); ++n) {
      first_moment += mean_weights[n] * sigma_points[n];
    }

    second_moment = 0;
    for (size_t n = 0; n < sigma_points.size(); ++n) {
      second_moment +=
          covariance_weights[n] * std::pow(sigma_points[n] - first_moment, 2);
    }
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__UNSCENTED_TRANSFORM__INVERSE_HPP_
