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


#ifndef ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERUPDATERCORE_HPP_
#define ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERUPDATERCORE_HPP_

// eigen
#include <Eigen/Core>

// romea
#include "romea_core_filtering/particle/ParticleFilterState.hpp"
#include "romea_core_filtering/particle/ParticleFilterResampling.hpp"


namespace romea
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
class PFUpdaterCore
{
public:
  using RowMajorVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

public:
  explicit PFUpdaterCore(const std::size_t & numberOfParticles);

  virtual ~PFUpdaterCore() = default;

protected:
  size_t numberOfParticles_;
  ParticleFilterResampling<Scalar, StateDIM> resampling_;
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
PFUpdaterCore<Scalar, StateDIM,
  ObservationDIM>::PFUpdaterCore(const std::size_t & numberOfParticles)
: numberOfParticles_(numberOfParticles),
  resampling_(numberOfParticles)
{
}

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERUPDATERCORE_HPP_
