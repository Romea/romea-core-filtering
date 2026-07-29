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

#ifndef ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTER_HPP_
#define ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTER_HPP_

// Eigen
#include <Eigen/Core>

// std
#include <algorithm>
#include <memory>
#include <utility>

// romea
#include "romea_core_filtering/filter/filter_base.hpp"

namespace romea
{
namespace core
{

template<class State, class FSMState, class Duration>
class ParticleFilter : public FilterBase<State, FSMState, Duration>
{
private:
  using Base = FilterBase<State, FSMState, Duration>;

public:
  ParticleFilter(
    const size_t & state_pool_size,
    const size_t & number_of_particles)
  : Base(state_pool_size),
    number_of_particles_(number_of_particles)
  {
    for (size_t n = 0; n < state_pool_size; ++n) {
      std::unique_ptr<State> state(new State(number_of_particles));
      this->state_vector_pool_.push_back(std::move(state));
    }
  }

  ParticleFilter(
    const size_t & state_pool_size,
    const size_t & number_of_particles,
    typename Base::PredictorPtr predictor)
  : ParticleFilter(state_pool_size, number_of_particles)
  {
    this->register_predictor(std::move(predictor));
  }

  virtual ~ParticleFilter() = default;

  virtual size_t get_number_of_particles() const { return number_of_particles_; }

private:
  size_t number_of_particles_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTER_HPP_
