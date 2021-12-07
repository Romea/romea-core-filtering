#ifndef romea_ParticleFilter_hpp
#define romea_ParticleFilter_hpp

//Eigen
#include <Eigen/Core>

//romea
#include "../Filter.hpp"

//std
#include <algorithm>
#include <memory>

namespace romea {

template < class State, class FSMState, class Duration>
class ParticleFilter : public Filter<State,FSMState,Duration>
{

public :

  ParticleFilter(const size_t & statePoolSize,
                 const size_t & numberOfParticles):
    Filter<State,FSMState,Duration>(statePoolSize),
    numberOfParticles_(0)
  {
    for(size_t n=0;n<statePoolSize;++n)
    {
      std::unique_ptr<State> state(new State(numberOfParticles));
      this->stateVectorPool_.push_back(std::move(state));
    }
  }

  virtual ~ParticleFilter()=default;

  virtual size_t getNumberOfParticles() const
  {
    return numberOfParticles_ ;
  }

private :

  size_t numberOfParticles_;

};



}

#endif
