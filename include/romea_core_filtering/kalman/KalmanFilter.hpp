// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_FILTERING__KALMAN__KALMANFILTER_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__KALMANFILTER_HPP_

// std
#include <memory>
#include <utility>

// romea
#include "romea_core_filtering/Filter.hpp"

namespace romea
{

template<class State, class FSMState, class Duration>
class KalmanFilter : public Filter<State, FSMState, Duration>
{
public:
  explicit KalmanFilter(const size_t statePoolSize)
  : Filter<State, FSMState, Duration>(statePoolSize)
  {
    for (size_t n = 0; n < statePoolSize; ++n) {
      std::unique_ptr<State> state(new State());
      this->stateVectorPool_.push_back(std::move(state));
    }
  }

  virtual ~KalmanFilter() = default;
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__KALMANFILTER_HPP_
