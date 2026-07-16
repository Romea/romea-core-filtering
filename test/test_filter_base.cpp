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

struct CounterState
{
  CounterState() : value(0), update_count(0) {}

  long long value;
  size_t update_count;
};

enum class CounterFSMState
{
  INIT,
  RUN
};

class CounterPredictor
: public romea::core::FilterPredictorBase<CounterState, CounterFSMState, Duration>
{
public:
  void predict(
    const Duration & previous_duration,
    const CounterFSMState & previous_fsm_state,
    const CounterState & previous_state,
    const Duration & current_duration,
    CounterFSMState & current_fsm_state,
    CounterState & current_state) override
  {
    current_state = previous_state;
    current_state.value += (current_duration - previous_duration).count();
    current_fsm_state = previous_fsm_state;
  }
};

class CounterFilter : public romea::core::FilterBase<CounterState, CounterFSMState, Duration>
{
public:
  explicit CounterFilter(const size_t & state_pool_size)
  : romea::core::FilterBase<CounterState, CounterFSMState, Duration>(state_pool_size)
  {
    register_predictor(std::make_unique<CounterPredictor>());
    for (size_t n = 0; n < state_pool_size; ++n) {
      auto state = std::make_unique<CounterState>();
      state_vector_pool_.push_back(std::move(state));
    }
  }
};

romea::core::FilterMetaState<CounterState, CounterFSMState, Duration>::UpdateFunction
make_update_function(const long long value)
{
  return [value](const Duration & duration, CounterFSMState & fsm_state, CounterState & state) {
    state.value = value + duration.count();
    state.update_count += 1;
    fsm_state = CounterFSMState::RUN;
  };
}

}  // namespace

TEST(TestFilterBase, returnsFalseWhenNoStateHasBeenInserted)
{
  CounterFilter filter(4);
  CounterState current_state;

  EXPECT_EQ(
    filter.get_current_state(Duration(0), &current_state),
    romea::core::FilterGetCurrentStateStatus::UNAVAILABLE);
  EXPECT_EQ(filter.get_fsm_state(), CounterFSMState::INIT);
}

TEST(TestFilterBase, predictsCurrentStateAfterLastObservation)
{
  CounterFilter filter(4);
  EXPECT_EQ(
    filter.process(Duration(10), make_update_function(100)),
    romea::core::FilterProcessStatus::ACCEPTED);

  CounterState current_state;
  ASSERT_EQ(
    filter.get_current_state(Duration(15), &current_state),
    romea::core::FilterGetCurrentStateStatus::AVAILABLE);

  EXPECT_EQ(current_state.value, 115);
  EXPECT_EQ(current_state.update_count, 1u);
  EXPECT_EQ(filter.get_fsm_state(), CounterFSMState::RUN);
}

TEST(TestFilterBase, replayStatesWhenDelayedObservationIsInserted)
{
  CounterFilter filter(5);

  filter.process(Duration(10), make_update_function(100));
  filter.process(Duration(30), make_update_function(300));
  filter.process(Duration(20), make_update_function(200));

  CounterState current_state;
  ASSERT_EQ(
    filter.get_current_state(Duration(35), &current_state),
    romea::core::FilterGetCurrentStateStatus::AVAILABLE);

  EXPECT_EQ(current_state.value, 335);
  EXPECT_EQ(current_state.update_count, 3u);
}

TEST(TestFilterBase, rejectsObservationOlderThanRetainedHistory)
{
  CounterFilter filter(3);

  EXPECT_EQ(
    filter.process(Duration(10), make_update_function(100)),
    romea::core::FilterProcessStatus::ACCEPTED);
  EXPECT_EQ(
    filter.process(Duration(20), make_update_function(200)),
    romea::core::FilterProcessStatus::ACCEPTED);
  EXPECT_EQ(
    filter.process(Duration(30), make_update_function(300)),
    romea::core::FilterProcessStatus::ACCEPTED);
  EXPECT_EQ(
    filter.process(Duration(40), make_update_function(400)),
    romea::core::FilterProcessStatus::ACCEPTED);
  EXPECT_EQ(
    filter.process(Duration(5), make_update_function(500)),
    romea::core::FilterProcessStatus::OUT_OF_HISTORY);

  CounterState current_state;
  ASSERT_EQ(
    filter.get_current_state(Duration(40), &current_state),
    romea::core::FilterGetCurrentStateStatus::AVAILABLE);

  EXPECT_EQ(current_state.value, 440);
  EXPECT_EQ(current_state.update_count, 4u);
}

TEST(TestFilterBase, returnsOutOfHistoryWhenCurrentStateIsOlderThanRetainedHistory)
{
  CounterFilter filter(3);

  filter.process(Duration(10), make_update_function(100));
  filter.process(Duration(20), make_update_function(200));
  filter.process(Duration(30), make_update_function(300));
  filter.process(Duration(40), make_update_function(400));

  CounterState current_state;
  EXPECT_EQ(
    filter.get_current_state(Duration(5), &current_state),
    romea::core::FilterGetCurrentStateStatus::OUT_OF_HISTORY);
}

TEST(TestFilterBase, resetClearsStoredStates)
{
  CounterFilter filter(4);
  filter.process(Duration(10), make_update_function(100));

  CounterState current_state;
  ASSERT_EQ(
    filter.get_current_state(Duration(10), &current_state),
    romea::core::FilterGetCurrentStateStatus::AVAILABLE);

  filter.reset();

  EXPECT_EQ(
    filter.get_current_state(Duration(10), &current_state),
    romea::core::FilterGetCurrentStateStatus::UNAVAILABLE);
  EXPECT_EQ(filter.get_fsm_state(), CounterFSMState::INIT);
}
