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
#include <memory>
#include <utility>

#include "romea_core_filtering/filter/kalman/filter.hpp"
#include "romea_core_filtering/filter/predictor_base.hpp"
#include "romea_core_filtering/gaussian/state.hpp"

namespace
{

using Duration = std::chrono::duration<long long int, std::nano>;
using State = romea::core::GaussianState<double, 1>;

enum class KalmanFSMState
{
  INIT,
  RUN
};

class ScalarPredictor : public romea::core::FilterPredictorBase<State, KalmanFSMState, Duration>
{
public:
  void predict(
    const Duration & previous_duration,
    const KalmanFSMState & previous_fsm_state,
    const State & previous_state,
    const Duration & current_duration,
    KalmanFSMState & current_fsm_state,
    State & current_state) override
  {
    current_state = previous_state;
    current_state.X() += static_cast<double>((current_duration - previous_duration).count());
    current_fsm_state = previous_fsm_state;
  }
};

romea::core::FilterMetaState<State, KalmanFSMState, Duration>::UpdateFunction make_scalar_update(
  const double value, const double variance)
{
  return [value, variance](const Duration &, KalmanFSMState & fsm_state, State & state) {
    state.X() = value;
    state.P() = variance;
    fsm_state = KalmanFSMState::RUN;
  };
}

}  // namespace

TEST(TestKalmanFilter, storesGaussianStatesAndUsesRegisteredPredictor)
{
  romea::core::KalmanFilter<State, KalmanFSMState, Duration> filter(4);
  filter.register_predictor(std::make_unique<ScalarPredictor>());

  filter.process(Duration(10), make_scalar_update(100., 4.));

  State current_state;
  ASSERT_EQ(
    filter.get_state(Duration(15), &current_state),
    romea::core::FilterGetStateStatus::AVAILABLE);

  EXPECT_DOUBLE_EQ(current_state.X(), 105.);
  EXPECT_DOUBLE_EQ(current_state.P(), 4.);
  EXPECT_EQ(filter.get_fsm_state(), KalmanFSMState::RUN);
}
