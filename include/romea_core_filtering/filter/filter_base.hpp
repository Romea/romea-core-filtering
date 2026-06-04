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

#ifndef INCLUDE_ROMEA_CORE_FILTERING_FILTER_HPP_
#define INCLUDE_ROMEA_CORE_FILTERING_FILTER_HPP_

// std
#include <cassert>
#include <cstddef>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// romea
// #include "romea_core_filtering/filter/Updater.hpp"
#include "romea_core_filtering/filter/meta_state.hpp"
#include "romea_core_filtering/filter/predictor_base.hpp"
#include "romea_core_filtering/filter/type.hpp"

namespace romea
{
namespace core
{

template<class State, class FSMState, class Duration>
class FilterBase
{
public:
  using State_ = State;
  using Predictor = FilterPredictorBase<State, FSMState, Duration>;
  using PredictorPtr = std::unique_ptr<Predictor>;

  using StatePtr = std::unique_ptr<State>;
  using MetaState = FilterMetaState<State, FSMState, Duration>;
  using UpdateFunction = typename FilterMetaState<State, FSMState, Duration>::UpdateFunction;

public:
  explicit FilterBase(const size_t & poolSize);

  virtual ~FilterBase() = default;

public:
  void register_predictor(PredictorPtr predicter);

  FSMState get_fsm_state() const;

  bool get_current_state(const Duration & duration, State * current_state);

  void process(const Duration & duration, UpdateFunction && update_function);

  void reset();

protected:
  std::deque<MetaState> meta_states_;

  std::vector<StatePtr> state_vector_pool_;

  PredictorPtr predictor_;

  mutable std::mutex mutex_;
};

//-----------------------------------------------------------------------------
template<class State, class FSMState, class Duration>
FilterBase<State, FSMState, Duration>::FilterBase(const size_t & state_pool_size)
: meta_states_(), state_vector_pool_(), predictor_(), mutex_()
{
  state_vector_pool_.reserve(state_pool_size);
}

//-----------------------------------------------------------------------------
template<class State, class FSMState, class Duration>
void FilterBase<State, FSMState, Duration>::register_predictor(PredictorPtr predicter)
{
  predictor_.swap(predicter);
}

//-----------------------------------------------------------------------------
template<class State, class FSMState, class Duration>
FSMState FilterBase<State, FSMState, Duration>::get_fsm_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (meta_states_.empty()) {
    return FSMState();
  } else {
    return meta_states_.back().fsm_state;
  }
}

//-----------------------------------------------------------------------------
template<class State, class FSMState, class Duration>
void FilterBase<State, FSMState, Duration>::reset()
{
  std::cout << " reset ???........................................" << std::endl;
  std::lock_guard<std::mutex> lock(mutex_);

  // Reset stateVectorPool
  for (size_t n = 0; n < meta_states_.size(); ++n) {
    state_vector_pool_[n].swap(meta_states_[n].state);
  }

  // Clear metaStates
  meta_states_.clear();
}

//-----------------------------------------------------------------------------
template<class State, class FSMState, class Duration>
bool FilterBase<State, FSMState, Duration>::get_current_state(
  const Duration & currentDuration, State * current_state)
{
  //  std::cout << " get current state "<< std::endl;
  std::lock_guard<std::mutex> lock(mutex_);

  assert(current_state);

  // If no metaStates have been inserted
  if (meta_states_.empty()) {
    return false;
  }

  // Search the position of required state vector
  auto Ir = meta_states_.rbegin();
  while (currentDuration < (*Ir).duration && Ir != meta_states_.rend()) {
    Ir++;
  }

  // If the date out of range
  if (Ir == meta_states_.rend()) {
    return false;
  }

  // Estimate the current state vector
  auto I = --(Ir.base());

  assert(std::distance(std::begin(meta_states_), I) >= 0);
  assert(std::distance(std::begin(meta_states_), I) < ptrdiff_t(meta_states_.size()));
  assert((*I).state.get());

  const Duration & previous_duration = (*I).duration;
  FSMState & previous_fsm_state = (*I).fsm_state;
  State * previous_state = (*I).state.get();

  FSMState current_fsm_State;
  predictor_->predict(
    previous_duration,
    previous_fsm_state,
    *previous_state,
    currentDuration,
    current_fsm_State,
    *current_state);

  return true;
}

//-----------------------------------------------------------------------------
template<class State, class FSMState, class Duration>
void FilterBase<State, FSMState, Duration>::process(
  const Duration & duration, UpdateFunction && update_function)
{
  std::lock_guard<std::mutex> lock(mutex_);
  assert(!state_vector_pool_.empty());

  auto I = std::begin(meta_states_);
  if (!meta_states_.empty()) {
    // Search where the observation must be inserted
    auto Ir = std::rbegin(meta_states_);
    while (Ir != meta_states_.rend() && duration < (*Ir).duration) {
      Ir++;
    }

    // Discard metaState prior to the first metaState
    if (Ir == meta_states_.rend() || duration < meta_states_[0].duration) {
      std::cout << " Discard metaState because is prior to the first metaState" << std::endl;
      return;
    }

    I = Ir.base();
  }

  // Insert metaState
  //  std::cout << " insert meta state " <<
  //  std::distance(std::begin(meta_states_),I)<< std::endl;
  if (meta_states_.size() < state_vector_pool_.size()) {
    assert(state_vector_pool_[meta_states_.size()] != nullptr);
    auto & state = state_vector_pool_[meta_states_.size()];
    I = meta_states_.emplace(I, duration, std::move(state), std::move(update_function));
  } else {
    assert(meta_states_[0].state != nullptr);
    auto & state = meta_states_[0].state;
    I = meta_states_.emplace(I, duration, std::move(state), std::move(update_function));
    meta_states_.erase(std::begin(meta_states_));
  }

  // Update metaStates from current metaStates to the last metaState
  auto J = I - 1;
  while (I != meta_states_.end()) {
    const Duration & currentDuration = (*I).duration;
    FSMState & current_fsm_State = (*I).fsm_state;
    State * current_state = (*I).state.get();
    assert(current_state != nullptr);

    // Precdiction step
    if (I != std::begin(meta_states_)) {
      const Duration & previous_duration = (*J).duration;
      FSMState & previous_fsm_state = (*J).fsm_state;
      State * previous_state = (*J).state.get();
      assert(previous_state != nullptr);

      predictor_->predict(
        previous_duration,
        previous_fsm_state,
        *previous_state,
        currentDuration,
        current_fsm_State,
        *current_state);

      //      current_fsm_State = previous_fsm_state;
    }

    (*I).update(currentDuration, current_fsm_State, *current_state);

    I++;
    J++;
  }
}

}  // namespace core
}  // namespace romea

#endif  // INCLUDE_ROMEA_CORE_FILTERING_FILTER_HPP_
