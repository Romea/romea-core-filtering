#ifndef romea_BayesianFilterPredictor_hpp
#define romea_BayesianFilterPredictor_hpp

//romea
#include "FilterMetaState.hpp"

namespace romea {

template < class State , class FSMState, class Duration>
class FilterPredictor
{

public :

  FilterPredictor(){}

  virtual ~FilterPredictor()=default;

  virtual void predict(const Duration & previousDuration,
                       const FSMState & previousFSMState,
                       const State & previousStateVector,
                       const Duration & currentDuration,
                       FSMState & currentFSMState,
                       State & currentState) =0;


};

}//romea

#endif
