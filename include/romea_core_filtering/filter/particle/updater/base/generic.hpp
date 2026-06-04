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

#ifndef ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__BASE__GENERIC_HPP_
#define ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__BASE__GENERIC_HPP_

// eigen
#include <Eigen/Core>

// romea
#include "romea_core_filtering/filter/particle/state.hpp"
#include "romea_core_filtering/filter/particle/updater/algorithm/resampling.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
class PFUpdaterBase
{
public:
  using RowMajorVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

public:
  explicit PFUpdaterBase(const std::size_t & number_of_particles);

  virtual ~PFUpdaterBase() = default;

protected:
  size_t number_of_particles_;
  ParticleFilterResampling<Scalar, StateDIM> resampling_;
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
PFUpdaterBase<Scalar, StateDIM, ObservationDIM>::PFUpdaterBase(
  const std::size_t & number_of_particles)
: number_of_particles_(number_of_particles), resampling_(number_of_particles)
{
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__BASE__GENERIC_HPP_
