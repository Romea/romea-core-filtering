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

#include <gtest/gtest.h>

#include <vector>

#include "romea_core_filtering/filter/kalman/updater/base/unscented.hpp"

namespace
{

using StateSigmaPoints = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using ObservationSigmaPoints = std::vector<double>;

romea::core::GaussianState<double, 3> make_reference_state()
{
  romea::core::GaussianState<double, 3> state;
  state.X() << 10.0401196, -6.541962718, 5.773763097;
  state.P().row(0) << 0.003713927914, 0.004473035868, 0.003144414499;
  state.P().row(1) << 0.004473035868, 0.009133254681, 0.005490835273;
  state.P().row(2) << 0.003144414499, 0.005490835273, 0.005852381092;
  return state;
}

romea::core::UnscentedTransformParameters<double> make_reference_parameters()
{
  return romea::core::UnscentedTransformParameters<double>(3, 3., 0.75, 2.);
}

StateSigmaPoints make_reference_sigma_points()
{
  StateSigmaPoints sigma_points(7);
  romea::core::UnscentedTransformForward<double, 3>::to_sigma_points(
    make_reference_parameters(), make_reference_state(), sigma_points);
  return sigma_points;
}

ObservationSigmaPoints make_reference_propagated_sigma_points()
{
  return {11.07597392, 11.1579882, 11.10114414, 11.09709313, 10.99435493, 11.05318411, 11.05509837};
}

void expect_vector_near(
  const Eigen::Vector3d & value, const Eigen::Vector3d & expected, const double tolerance)
{
  EXPECT_NEAR(value.x(), expected.x(), tolerance);
  EXPECT_NEAR(value.y(), expected.y(), tolerance);
  EXPECT_NEAR(value.z(), expected.z(), tolerance);
}

void expect_matrix_near(
  const Eigen::Matrix3d & value, const Eigen::Matrix3d & expected, const double tolerance)
{
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      EXPECT_NEAR(value(row, col), expected(row, col), tolerance);
    }
  }
}

}  // namespace

TEST(TestUnscentedTransform, convertsGaussianStateToSigmaPoints)
{
  const auto state = make_reference_state();
  const auto parameters = make_reference_parameters();
  const auto sigma_points = make_reference_sigma_points();

  ASSERT_EQ(sigma_points.size(), 7u);
  expect_vector_near(sigma_points[0], state.X(), 0.01);

  for (size_t n = 1; n <= 3; ++n) {
    expect_vector_near(sigma_points[n] + sigma_points[n + 3], 2. * state.X(), 0.01);
  }

  Eigen::Vector3d reconstructed_mean = Eigen::Vector3d::Zero();
  for (size_t n = 0; n < sigma_points.size(); ++n) {
    reconstructed_mean += parameters.mean_weights[n] * sigma_points[n];
  }
  expect_vector_near(reconstructed_mean, state.X(), 0.01);

  Eigen::Matrix3d reconstructed_covariance = Eigen::Matrix3d::Zero();
  for (size_t n = 0; n < sigma_points.size(); ++n) {
    const auto dX = sigma_points[n] - reconstructed_mean;
    reconstructed_covariance += parameters.covariance_weights[n] * dX * dX.transpose();
  }
  expect_matrix_near(reconstructed_covariance, state.P(), 0.01);
}

TEST(TestUnscentedTransform, reconstructsGaussianObservationFromSigmaPoints)
{
  romea::core::GaussianObservation<double, 1> propagated_state;

  romea::core::UnscentedTransformInverse<double, 1>::to_gaussian(
    make_reference_parameters(), make_reference_propagated_sigma_points(), propagated_state);

  EXPECT_NEAR(propagated_state.Y(), 11.07642123, 0.01);
  EXPECT_NEAR(propagated_state.R(), 0.002285136065, 0.01);
}

TEST(TestUnscentedTransform, computesStateObservationCorrelation)
{
  romea::core::GaussianObservation<double, 1> propagated_state;
  const auto propagated_sigma_points = make_reference_propagated_sigma_points();
  romea::core::UnscentedTransformInverse<double, 1>::to_gaussian(
    make_reference_parameters(), propagated_sigma_points, propagated_state);

  Eigen::Matrix<double, 3, 1> propagation_correlation(3, 1);
  romea::core::UKFCorrelation<double, 3, 1>::compute(
    make_reference_parameters(),
    make_reference_state(),
    propagated_state,
    make_reference_sigma_points(),
    propagated_sigma_points,
    propagation_correlation);

  EXPECT_NEAR(propagation_correlation(0, 0), 0.002835369261, 0.01);
  EXPECT_NEAR(propagation_correlation(1, 0), 0.0027504505, 0.01);
  EXPECT_NEAR(propagation_correlation(2, 0), 0.002030419041, 0.01);
}
