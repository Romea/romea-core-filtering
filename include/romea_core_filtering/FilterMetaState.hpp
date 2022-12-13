#ifndef INCLUDE_ROMEA_CORE_FILTERING_FILTERMETASTATE_HPP_
#define INCLUDE_ROMEA_CORE_FILTERING_FILTERMETASTATE_HPP_

// std
#include <memory>
#include <functional>
#include <utility>

// romea
#include "romea_core_filtering/FilterUpdater.hpp"


namespace romea {

template <class State , class FSMState, class Duration>
struct FilterMetaState
{
public :

  using UpdateFunction = std::function<void(const Duration &,
                                            FSMState &,
                                            State&)> ;

  using PredictFunction = std::function<void(const Duration &,
                                             const FSMState &,
                                             const State&,
                                             const Duration &,
                                             FSMState &, State &)> ;

public :

  FilterMetaState(const Duration & duration,
                  std::unique_ptr<State> state,
                  UpdateFunction && update):

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

}  // namespace romea

#endif  // INCLUDE_ROMEA_CORE_FILTERING_FILTERMETASTATE_HPP_
