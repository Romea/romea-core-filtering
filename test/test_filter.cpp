// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// std
#include <chrono>
#include <memory>
#include <utility>

// romea
#include "romea_core_filtering/Filter.hpp"
#include "romea_core_filtering/FilterPredictor.hpp"
#include "romea_core_filtering/FilterUpdater.hpp"

using Duration = std::chrono::duration<long long int, std::nano>;

//-----------------------------------------------------------------------------
struct  TimeState
{
  TimeState()
  : computedElapsedTime(Duration::zero())
  {
  }

  Duration computedElapsedTime;
};


//-----------------------------------------------------------------------------
struct TimeObservation
{
public:
  explicit TimeObservation(const Duration duration)
  : duration(duration)
  {
  }

  Duration duration;
};

//-----------------------------------------------------------------------------
enum class TimeFSMState
{
  INIT = 0,
  RUN
};

//-----------------------------------------------------------------------------
class TimePredictor : public romea::FilterPredictor<TimeState, TimeFSMState, Duration>
{
public:
  TimePredictor() {}

  void predict(
    const Duration & previousDuration,
    const TimeFSMState & previousFSMState,
    const TimeState & previousState,
    const Duration & currentDuration,
    TimeFSMState & currentFSMState,
    TimeState & currentState)override
  {
    auto dt = currentDuration - previousDuration;
    currentState.computedElapsedTime = previousState.computedElapsedTime + dt;
    currentFSMState = previousFSMState;

    // std::cout << "Prediction previous duration "
    //           << DurationToMicroSecond(previousState.computedElapsedTime) << std::endl;
    // std::cout << "Prediction current duration "
    //           <<  DurationToMicroSecond(currentState.computedElapsedTime) << std::endl;
    // std::cout << "Prediction current fsm state "
    //           << int(currentFSMState) << std::endl;
  }
};

//-----------------------------------------------------------------------------
class TimeUpdater
{
public:
  TimeUpdater()
  {
  }

  void update(
    const Duration & duration,
    const TimeObservation & currentObservation,
    TimeFSMState & currentFsmState,
    TimeState & currentstate)
  {
    if (currentFsmState == TimeFSMState::INIT) {
      currentstate.computedElapsedTime = currentObservation.duration;
      currentFsmState = TimeFSMState::RUN;
    }

    // std::cout << "Update state duration "
    //           <<  DurationToMicroSecond(currentstate.computedElapsedTime) << std::endl;
    // std::cout << "Update observation duration "
    //           <<  DurationToMicroSecond(currentObservation.duration) << std::endl;
    // std::cout << "Update current fsm state "
    //           << int(currentFsmState)  << std::endl;

    EXPECT_EQ(currentstate.computedElapsedTime.count(), currentObservation.duration.count());
    EXPECT_EQ(currentstate.computedElapsedTime.count(), duration.count());
    EXPECT_EQ(currentFsmState, TimeFSMState::RUN);
  }
};

//-----------------------------------------------------------------------------
class TimerFilter : public romea::Filter<TimeState, TimeFSMState, Duration>
{
public:
  explicit TimerFilter(const size_t & statePoolSize)
  : romea::Filter<TimeState, TimeFSMState, Duration>(statePoolSize)
  {
    predictor_ = std::make_unique<TimePredictor>();
    for (size_t n = 0; n < statePoolSize; ++n) {
      auto state = std::make_unique<TimeState>();
      this->stateVectorPool_.push_back(std::move(state));
    }
  }
};

//-----------------------------------------------------------------------------
TEST(TestFilter, testFilterCore)
{
  TimerFilter filter(20);
  TimeUpdater updator;

  unsigned int dt = 1000;
  unsigned int lag = 333;

  Duration duration = Duration::zero();
  TimeObservation observation(duration);

  auto updateFunction = std::bind(
    &TimeUpdater::update,
    std::ref(updator),
    std::placeholders::_1,
    std::move(observation),
    std::placeholders::_2,
    std::placeholders::_3);

  filter.process(duration, std::move(updateFunction));

  for (size_t i = 1; i < 50; i++) {
    unsigned int ilag = i % 10 == 0 ? lag : 0;
    Duration duration = Duration(i * dt - i / 10 * ilag);
//    std::cout <<"\n iteration "<< i<<" "<<ilag<< " "<< duration.count()/1000000000.<< std::endl;

    TimeObservation observation(duration);

    auto updateFunction = std::bind(
      &TimeUpdater::update,
      std::ref(updator),
      std::placeholders::_1,
      std::move(observation),
      std::placeholders::_2,
      std::placeholders::_3);

    filter.process(duration, std::move(updateFunction));
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
