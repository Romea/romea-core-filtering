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


#ifndef ROMEA_CORE_FILTERING__KALMAN__CORE__KFGAIN_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__CORE__KFGAIN_HPP_

// romea
#include "romea_core_filtering/kalman/core/KFUpdaterTraits.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFGain
{
  static void compute(
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::P & P,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::H & H,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn & QInnInverse,
    typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K & K)
  {
    K = P * H.transpose() * QInnInverse;
  }
};


//-----------------------------------------------------------------------------
template<typename Scalar>
struct KFGain<Scalar, 1, 1>
{
  static void compute(
    const Scalar & P,
    const Scalar & H,
    const Scalar & QInnInverse,
    Scalar & K)
  {
    K = P * H * QInnInverse;
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_KALMAN_CORE_KFGAIN_HPP_
