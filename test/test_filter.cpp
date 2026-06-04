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

#include "romea_core_filtering/filter/filter_base.hpp"
#include "romea_core_filtering/filter/predictor_base.hpp"

namespace
{

using Duration = std::chrono::duration<long long int, std::nano>;

struct TimeState
{
  TimeState() : elapsed_time(Duration::zero()), update_count(0) {}

  Duration elapsed_time;
  size_t update_count;
};

enum class TimeFSMState
{
  INIT = 0,
  RUN
};

class TimePredictor : public romea::core::FilterPredictorBase<TimeState, TimeFSMState, Duration>
{
public:
  void predict(
    const Duration & previous_duration,
    const TimeFSMState & previous_fsm_state,
    const TimeState & previous_state,
    const Duration & current_duration,
    TimeFSMState & current_fsm_state,
    TimeState & current_state) override
  {
    current_state = previous_state;
    current_state.elapsed_time += current_duration - previous_duration;
    current_fsm_state = previous_fsm_state;
  }
};

class TimerFilter : public romea::core::FilterBase<TimeState, TimeFSMState, Duration>
{
public:
  explicit TimerFilter(const size_t & state_pool_size)
  : romea::core::FilterBase<TimeState, TimeFSMState, Duration>(state_pool_size)
  {
    register_predictor(std::make_unique<TimePredictor>());
    for (size_t n = 0; n < state_pool_size; ++n) {
      state_vector_pool_.push_back(std::make_unique<TimeState>());
    }
  }
};

romea::core::FilterMetaState<TimeState, TimeFSMState, Duration>::UpdateFunction make_time_update()
{
  return [](const Duration & duration, TimeFSMState & fsm_state, TimeState & state) {
    state.elapsed_time = duration;
    state.update_count += 1;
    fsm_state = TimeFSMState::RUN;
  };
}

}  // namespace

TEST(TestFilter, processesLongSequenceWithPeriodicDelayedObservations)
{
  TimerFilter filter(20);

  constexpr long long dt = 1000;
  constexpr long long lag = 333;

  for (size_t i = 0; i < 50; ++i) {
    const long long delayed_time = i * dt - static_cast<long long>(i / 10) * lag;
    filter.process(Duration(delayed_time), make_time_update());
  }

  TimeState current_state;
  ASSERT_TRUE(filter.get_current_state(Duration(49000), &current_state));

  EXPECT_EQ(current_state.elapsed_time.count(), 49000);
  EXPECT_EQ(current_state.update_count, 50u);
  EXPECT_EQ(filter.get_fsm_state(), TimeFSMState::RUN);
}
