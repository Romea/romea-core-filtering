#ifndef ROMEA_CORE_FILTERING_FILTERPREDICTOR_HPP_
#define ROMEA_CORE_FILTERING_FILTERPREDICTOR_HPP_

// romea
#include "romea_core_filtering/FilterMetaState.hpp"

namespace romea {

template < class State , class FSMState, class Duration>
class FilterPredictor
{
public :

  FilterPredictor(){}

  virtual ~FilterPredictor() = default;

  virtual void predict(const Duration & previousDuration,
                       const FSMState & previousFSMState,
                       const State & previousStateVector,
                       const Duration & currentDuration,
                       FSMState & currentFSMState,
                       State & currentState) = 0;
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_FILTERPREDICTOR_HPP_
