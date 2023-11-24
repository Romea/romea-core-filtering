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


#ifndef INCLUDE_ROMEA_CORE__FILTERING__FILTERMETASTATE_HPP_
#define INCLUDE_ROMEA_CORE__FILTERING__FILTERMETASTATE_HPP_

// std
#include <memory>
#include <functional>
#include <utility>

// romea
#include "romea_core_filtering/FilterUpdater.hpp"


namespace romea
{
namespace core
{

template<class State, class FSMState, class Duration>
struct FilterMetaState
{
public:
  using UpdateFunction = std::function<void (const Duration &,
      FSMState &,
      State &)>;

  using PredictFunction = std::function<void (const Duration &,
      const FSMState &,
      const State &,
      const Duration &,
      FSMState &, State &)>;

public:
  FilterMetaState(
    const Duration & duration,
    std::unique_ptr<State> state,
    UpdateFunction && update) :

  duration(duration),
  fsmState(),
  state(std::move(state)),
  update(std::move(update))
  {
  }

  Duration duration;
  FSMState fsmState;
  std::unique_ptr<State> state;
  UpdateFunction update;
};

}  // namespace core
}  // namespace romea

#endif  // INCLUDE_ROMEA_CORE__FILTERING__FILTERMETASTATE_HPP_
