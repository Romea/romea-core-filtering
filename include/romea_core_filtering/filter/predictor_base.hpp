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

#ifndef ROMEA_CORE__FILTERING__FILTERPREDICTOR_HPP_
#define ROMEA_CORE__FILTERING__FILTERPREDICTOR_HPP_

// romea
#include "romea_core_filtering/filter/meta_state.hpp"

namespace romea
{
namespace core
{

template<class State, class FSMState, class Duration>
class FilterPredictorBase
{
public:
  FilterPredictorBase() {}

  virtual ~FilterPredictorBase() = default;

  virtual void predict(
    const Duration & previous_duration,
    const FSMState & previous_fsm_state,
    const State & previous_state_vector,
    const Duration & currentDuration,
    FSMState & current_fsm_State,
    State & current_state) = 0;

  virtual Duration maximal_extrapolation_duration() const { return Duration::max(); }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTERPREDICTOR_HPP_
