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

#include <cmath>

#include <gtest/gtest.h>

#include "romea_core_filtering/gaussian/input.hpp"
#include "romea_core_filtering/gaussian/observation.hpp"
#include "romea_core_filtering/gaussian/state.hpp"

TEST(TestGaussianState, exposesStateVectorAndCovarianceAccessors) {
  romea::core::GaussianState<double, 3> state;

  EXPECT_TRUE(std::isnan(state.X(0)));
  EXPECT_TRUE(std::isnan(state.X(1)));
  EXPECT_TRUE(std::isnan(state.X(2)));
  EXPECT_DOUBLE_EQ(state.P().sum(), 0.);

  state.X() << 1., 2., 3.;
  state.P().setIdentity();
  state.P(0, 1) = 0.25;

  EXPECT_DOUBLE_EQ(state.X(0), 1.);
  EXPECT_DOUBLE_EQ(state.X(1), 2.);
  EXPECT_DOUBLE_EQ(state.X(2), 3.);
  EXPECT_DOUBLE_EQ(state.P(0, 0), 1.);
  EXPECT_DOUBLE_EQ(state.P(0, 1), 0.25);

  state.reset();

  EXPECT_TRUE(std::isnan(state.X(0)));
  EXPECT_TRUE(std::isnan(state.X(1)));
  EXPECT_TRUE(std::isnan(state.X(2)));
  EXPECT_DOUBLE_EQ(state.P().sum(), 0.);
}

TEST(TestGaussianState, supportsScalarStateSpecialization) {
  romea::core::GaussianState<double, 1> state;

  EXPECT_TRUE(std::isnan(state.X()));
  EXPECT_DOUBLE_EQ(state.P(), 0.);

  state.X() = 4.;
  state.P() = 9.;

  EXPECT_DOUBLE_EQ(state.X(), 4.);
  EXPECT_DOUBLE_EQ(state.P(), 9.);

  state.reset();

  EXPECT_TRUE(std::isnan(state.X()));
  EXPECT_DOUBLE_EQ(state.P(), 0.);
}

TEST(TestGaussianInput, exposesInputVectorAndCovarianceAccessors) {
  romea::core::GaussianInput<double, 2> input;

  input.U() << 0.5, -0.25;
  input.QU().row(0) << 0.1, 0.01;
  input.QU().row(1) << 0.01, 0.2;

  EXPECT_DOUBLE_EQ(input.U(0), 0.5);
  EXPECT_DOUBLE_EQ(input.U(1), -0.25);
  EXPECT_DOUBLE_EQ(input.QU(0, 0), 0.1);
  EXPECT_DOUBLE_EQ(input.QU(1, 1), 0.2);
}

TEST(TestGaussianObservation, exposesObservationVectorAndCovarianceAccessors) {
  romea::core::GaussianObservation<double, 2> observation;

  observation.Y() << 7., 8.;
  observation.R().row(0) << 0.3, 0.02;
  observation.R().row(1) << 0.02, 0.4;

  EXPECT_DOUBLE_EQ(observation.Y(0), 7.);
  EXPECT_DOUBLE_EQ(observation.Y(1), 8.);
  EXPECT_DOUBLE_EQ(observation.R(0, 0), 0.3);
  EXPECT_DOUBLE_EQ(observation.R(1, 1), 0.4);
}
