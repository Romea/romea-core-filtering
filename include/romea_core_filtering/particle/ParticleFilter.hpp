// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTER_HPP_
#define ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTER_HPP_

// Eigen
#include <Eigen/Core>

// std
#include <algorithm>
#include <memory>
#include <utility>

// romea
#include "romea_core_filtering/Filter.hpp"


namespace romea
{

template<class State, class FSMState, class Duration>
class ParticleFilter : public Filter<State, FSMState, Duration>
{
public:
  ParticleFilter(
    const size_t & statePoolSize,
    const size_t & numberOfParticles)
  : Filter<State, FSMState, Duration>(statePoolSize),
    numberOfParticles_(0)
  {
    for (size_t n = 0; n < statePoolSize; ++n) {
      std::unique_ptr<State> state(new State(numberOfParticles));
      this->stateVectorPool_.push_back(std::move(state));
    }
  }

  virtual ~ParticleFilter() = default;

  virtual size_t getNumberOfParticles() const
  {
    return numberOfParticles_;
  }

private:
  size_t numberOfParticles_;
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTER_HPP_
