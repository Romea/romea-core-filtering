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

#include <chrono>
#include <cmath>

#include "romea_core_filtering/filter/particle/filter.hpp"
#include "romea_core_filtering/filter/particle/state.hpp"
#include "romea_core_filtering/filter/particle/updater/algorithm/resampling.hpp"
#include "romea_core_filtering/filter/particle/updater/base/gaussian.hpp"

namespace
{

using Duration = std::chrono::duration<long long int, std::nano>;

enum class ParticleFSMState
{
  INIT,
  RUN
};

class TestParticleFilter
: public romea::core::
    ParticleFilter<romea::core::ParticleFilterState<double, 2>, ParticleFSMState, Duration>
{
public:
  TestParticleFilter(const size_t & state_pool_size, const size_t & number_of_particles)
  : ParticleFilter(state_pool_size, number_of_particles)
  {
  }
};

class TestGaussianParticleUpdater : public romea::core::PFGaussianUpdaterBase<double, 1, 1>
{
public:
  TestGaussianParticleUpdater(
    const size_t & number_of_particles, const double & maximal_mahalanobis_distance)
  : PFGaussianUpdaterBase(number_of_particles, maximal_mahalanobis_distance)
  {
  }

  void set_apriori_observations(const ObservationVector & observations)
  {
    apriori_observations_ = observations;
  }

  bool update(State & state, const Observation & observation)
  {
    return update_state_(state, observation);
  }
};

}  // namespace

TEST(TestParticleState, initializesParticlesAndWeights)
{
  romea::core::ParticleFilterState<double, 2> state(4);

  ASSERT_EQ(state.particles.rows(), 2);
  ASSERT_EQ(state.particles.cols(), 4);
  ASSERT_EQ(state.weights.cols(), 4);

  for (int row = 0; row < state.particles.rows(); ++row) {
    for (int col = 0; col < state.particles.cols(); ++col) {
      EXPECT_TRUE(std::isnan(state.particles(row, col)));
    }
  }

  for (int col = 0; col < state.weights.cols(); ++col) {
    EXPECT_DOUBLE_EQ(state.weights(col), 0.25);
  }
}

TEST(TestParticleState, resetRestoresNanParticlesAndUniformWeights)
{
  romea::core::ParticleFilterState<double, 2> state(3);
  state.particles.setConstant(4.);
  state.weights << 0.1, 0.2, 0.7;

  state.reset();

  for (int row = 0; row < state.particles.rows(); ++row) {
    for (int col = 0; col < state.particles.cols(); ++col) {
      EXPECT_TRUE(std::isnan(state.particles(row, col)));
    }
  }

  for (int col = 0; col < state.weights.cols(); ++col) {
    EXPECT_DOUBLE_EQ(state.weights(col), 1. / 3.);
  }
}

TEST(TestParticleFilter, exposesConfiguredNumberOfParticles)
{
  TestParticleFilter filter(3, 12);

  EXPECT_EQ(filter.get_number_of_particles(), 12u);
}

TEST(TestParticleResampling, normalizesWeightsEvenWhenResamplingIsNotNeeded)
{
  romea::core::ParticleFilterState<double, 1> state(3);
  state.particles << 1., 2., 3.;
  state.weights << 2., 3., 5.;

  romea::core::ParticleFilterResampling<double, 1> resampling(3, 42);
  resampling.resampling(state, romea::core::ParticleFilterResamplingScheme::SYSTEMATIC, 0.);

  EXPECT_NEAR(state.weights.sum(), 1., 1e-12);
  EXPECT_NEAR(state.weights(0), 0.2, 1e-12);
  EXPECT_NEAR(state.weights(1), 0.3, 1e-12);
  EXPECT_NEAR(state.weights(2), 0.5, 1e-12);
}

TEST(TestParticleResampling, throwsOnDegenerateWeights)
{
  romea::core::ParticleFilterState<double, 1> state(3);
  state.weights.setZero();

  romea::core::ParticleFilterResampling<double, 1> resampling(3, 42);

  EXPECT_THROW(
    resampling.resampling(state, romea::core::ParticleFilterResamplingScheme::SYSTEMATIC),
    std::runtime_error);
}

TEST(TestParticleResampling, systematicResamplingProducesUniformWeights)
{
  romea::core::ParticleFilterState<double, 1> state(4);
  state.particles << 10., 20., 30., 40.;
  state.weights << 0.97, 0.01, 0.01, 0.01;

  romea::core::ParticleFilterResampling<double, 1> resampling(4, 1);
  resampling.resampling(state, romea::core::ParticleFilterResamplingScheme::SYSTEMATIC, 1.);

  for (int col = 0; col < state.weights.cols(); ++col) {
    EXPECT_DOUBLE_EQ(state.weights(col), 0.25);
  }
}

TEST(TestPFGaussianUpdaterBase, acceptsCloseGaussianObservationAndResamples)
{
  romea::core::ParticleFilterState<double, 1> state(3);
  state.particles << 1., 2., 3.;
  state.weights << 1. / 3., 1. / 3., 1. / 3.;

  TestGaussianParticleUpdater updater(3, 10.);
  TestGaussianParticleUpdater::ObservationVector apriori_observations(3);
  apriori_observations << 1., 2., 3.;
  updater.set_apriori_observations(apriori_observations);

  TestGaussianParticleUpdater::Observation observation;
  observation.Y() = 2.;
  observation.R() = 1.;

  EXPECT_TRUE(updater.update(state, observation));

  EXPECT_NEAR(state.weights.sum(), 1., 1e-12);
  EXPECT_GT(state.weights(1), state.weights(0));
  EXPECT_GT(state.weights(1), state.weights(2));
  EXPECT_DOUBLE_EQ(state.weights(0), state.weights(2));
}

TEST(TestPFGaussianUpdaterBase, rejectsFarGaussianObservation)
{
  romea::core::ParticleFilterState<double, 1> state(3);
  state.particles << 1., 2., 3.;
  state.weights << 1. / 3., 1. / 3., 1. / 3.;

  TestGaussianParticleUpdater updater(3, 0.1);
  TestGaussianParticleUpdater::ObservationVector apriori_observations(3);
  apriori_observations << 1., 2., 3.;
  updater.set_apriori_observations(apriori_observations);

  TestGaussianParticleUpdater::Observation observation;
  observation.Y() = 100.;
  observation.R() = 1.;

  EXPECT_FALSE(updater.update(state, observation));
}
