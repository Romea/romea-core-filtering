// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


#ifndef ROMEA_CORE_FILTERING__KALMAN__KALMANFILTER_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__KALMANFILTER_HPP_

// std
#include <memory>
#include <utility>

// romea
#include "romea_core_filtering/Filter.hpp"

namespace romea
{
namespace core
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

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__KALMANFILTER_HPP_
