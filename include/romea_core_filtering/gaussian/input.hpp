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

#ifndef ROMEA_CORE_FILTERING__GAUSSIAN__INPUT_HPP_
#define ROMEA_CORE_FILTERING__GAUSSIAN__INPUT_HPP_

#include "romea_core_filtering/gaussian/distribution.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t DIM>
struct GaussianInput : GaussianDistribution<Scalar, DIM>
{
  GaussianInput() : GaussianDistribution<Scalar, DIM>() {}

  virtual ~GaussianInput() = default;

  typename GaussianDistribution<Scalar, DIM>::FirstMoment & U() { return this->first_moment; }

  const typename GaussianDistribution<Scalar, DIM>::FirstMoment & U() const
  {
    return this->first_moment;
  }

  Scalar & U(const size_t & i) { return this->first_moment(i); }

  const Scalar & U(const size_t & i) const { return this->first_moment(i); }

  typename GaussianDistribution<Scalar, DIM>::SecondMoment & QU() { return this->second_moment; }

  const typename GaussianDistribution<Scalar, DIM>::SecondMoment & QU() const
  {
    return this->second_moment;
  }

  const Scalar & QU(const size_t & i, const size_t & j) const { return this->second_moment(i, j); }

  Scalar & QU(const size_t & i, const size_t & j) { return this->second_moment(i, j); }

  void reset()
  {
    this->first_moment.setConstant(NAN);
    this->second_moment.setZero();
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__GAUSSIAN_INPUT_HPP_
