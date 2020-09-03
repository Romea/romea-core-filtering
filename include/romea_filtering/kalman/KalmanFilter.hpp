#ifndef romea_KalmanFilter_hpp
#define romea_KalmanFilter_hpp

//romea
#include "../Filter.hpp"

namespace romea {

template < class State , class FSMState, class Duration>
class KalmanFilter : public Filter<State,FSMState,Duration>
{

public :

  KalmanFilter(const size_t statePoolSize):
    Filter<State,FSMState,Duration>(statePoolSize)
  {
    for(size_t n=0;n<statePoolSize;++n)
    {
      std::unique_ptr<State> state(new State());
      this->stateVectorPool_.push_back(std::move(state));
    }
  }

  virtual ~KalmanFilter()=default;


};

}//romea

#endif
