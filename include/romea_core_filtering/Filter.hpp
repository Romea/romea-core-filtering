#ifndef INCLUDE_ROMEA_CORE_FILTERING_FILTER_HPP_ 
#define INCLUDE_ROMEA_CORE_FILTERING_FILTER_HPP_ 

// std
#include <functional>
#include <cstddef>
#include <cassert>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <iostream>
#include <utility>

// romea
#include "romea_core_filtering/FilterMetaState.hpp"
#include "romea_core_filtering/FilterPredictor.hpp"
#include "romea_core_filtering/FilterUpdater.hpp"
#include "romea_core_filtering/FilterType.hpp"

namespace romea {

template <class State , class FSMState, class Duration>
class Filter{
public :

  using State_ = State;
  using Predictor = FilterPredictor<State, FSMState, Duration>;
  using PredictorPtr = std::unique_ptr<Predictor>;

  using StatePtr = std::unique_ptr<State>;
  using MetaState  = FilterMetaState<State, FSMState, Duration>;
  using UpdateFunction = typename FilterMetaState<State, FSMState, Duration>::UpdateFunction;

public :

  explicit Filter(const size_t & poolSize);

  virtual ~Filter() = default;

public :

  void registerPredictor(PredictorPtr predicter);

  FSMState getFSMState()const;

  bool getCurrentState(const Duration & duration, State * currentState);

  void process(const Duration & duration, UpdateFunction && updateFunction);

  void reset();

protected:
  std::deque< MetaState > metaStates_;

  std::vector< StatePtr > stateVectorPool_;

  PredictorPtr predictor_;

  mutable std::mutex mutex_;
};

//-----------------------------------------------------------------------------
template <class State, class FSMState, class Duration>
Filter<State, FSMState, Duration>::Filter(const size_t & statePoolSize):
  metaStates_(),
  stateVectorPool_(),
  predictor_(),
  mutex_()
{
  stateVectorPool_.reserve(statePoolSize);
}

//-----------------------------------------------------------------------------
template <class State, class FSMState, class Duration>
void Filter<State, FSMState, Duration>::registerPredictor(PredictorPtr predicter)
{
  predictor_.swap(predicter);
}

//-----------------------------------------------------------------------------
template <class State, class FSMState, class Duration>
FSMState Filter<State, FSMState, Duration>::getFSMState()const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (metaStates_.empty())
  {
    return FSMState();
  } else {
    return  metaStates_.back().fsmState;
  }
}

//-----------------------------------------------------------------------------
template <class State, class FSMState, class Duration>
void Filter<State, FSMState, Duration>::Filter::reset()
{
  std::cout << " reset ???........................................" << std::endl;
  std::lock_guard<std::mutex> lock(mutex_);

  // Reset stateVectorPool
  for (size_t n = 0; n < metaStates_.size(); ++n)
    stateVectorPool_[n].swap(metaStates_[n].state);

  // Clear metaStates
  metaStates_.clear();
}


//-----------------------------------------------------------------------------
template <class State, class FSMState, class Duration>
bool Filter<State, FSMState, Duration>::getCurrentState(Duration const & currentDuration,
                                                        State * currentState)
{
  //  std::cout << " get current state "<< std::endl;
  std::lock_guard<std::mutex> lock(mutex_);

  assert(currentState);

  // If no metaStates have been inserted
  if (metaStates_.empty())
    return false;

  // Search the position of required state vector
  auto Ir = metaStates_.rbegin();
  while (currentDuration < (*Ir).duration && Ir != metaStates_.rend()) Ir++;

  // If the date out of range
  if (Ir == metaStates_.rend())
    return false;

  // Estimate the current state vector
  auto I = --(Ir.base());

  assert(std::distance(std::begin(metaStates_), I) >= 0);
  assert(std::distance(std::begin(metaStates_), I) < ptrdiff_t(metaStates_.size()));
  assert((*I).state.get());

  const Duration & previousDuration = (*I).duration;
  FSMState & previousFSMState = (*I).fsmState;
  State * previousState = (*I).state.get();

  FSMState currentFSMState;
  predictor_->predict(previousDuration,
                      previousFSMState,
                      *previousState,
                      currentDuration,
                      currentFSMState,
                      *currentState);

  return true;
}


//-----------------------------------------------------------------------------
template <class State, class FSMState, class Duration>
void Filter<State, FSMState, Duration>::process(const Duration & duration,
                                                UpdateFunction && updateFunction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  assert(!stateVectorPool_.empty());

  auto I = std::begin(metaStates_);
  if (!metaStates_.empty())
  {
    // Search where the observation must be inserted
    auto Ir = std::rbegin(metaStates_);
    while (Ir != metaStates_.rend() && duration < (*Ir).duration)
    {
      Ir++;
    }

    // Discard metaState prior to the first metaState
    if (Ir == metaStates_.rend() || duration < metaStates_[0].duration)
    {
      std::cout << " Discard metaState because is prior to the first metaState" << std::endl;
      return;
    }

    I = Ir.base();
  }

  // Insert metaState
  //  std::cout << " insert meta state " << std::distance(std::begin(metaStates_),I)<< std::endl;
  if (metaStates_.size() < stateVectorPool_.size()){
    assert(stateVectorPool_[metaStates_.size()] != nullptr);
    auto & state = stateVectorPool_[metaStates_.size()];
    I = metaStates_.emplace(I, duration, std::move(state), std::move(updateFunction));
  }else{
    assert(metaStates_[0].state != nullptr);
    auto & state = metaStates_[0].state;
    I = metaStates_.emplace(I, duration, std::move(state), std::move(updateFunction));
    metaStates_.erase(std::begin(metaStates_));
  }

  // Update metaStates from current metaStates to the last metaState
  auto J = I-1;
  while (I != metaStates_.end()){
    const Duration & currentDuration = (*I).duration;
    FSMState & currentFSMState = (*I).fsmState;
    State * currentState = (*I).state.get();
    assert(currentState != nullptr);

    // Precdiction step
    if (I != std::begin(metaStates_))
    {
      const Duration & previousDuration = (*J).duration;
      FSMState & previousFSMState = (*J).fsmState;
      State * previousState =  (*J).state.get();
      assert(previousState != nullptr);

      predictor_->predict(previousDuration,
                          previousFSMState,
                          *previousState,
                          currentDuration,
                          currentFSMState,
                          *currentState);

//      currentFSMState = previousFSMState;
    }

    (*I).update(currentDuration,
                currentFSMState,
                *currentState);

    I++;
    J++;
  }
}

}  // namespace romea

#endif  // INCLUDE_ROMEA_CORE_FILTERING_FILTER_HPP_
