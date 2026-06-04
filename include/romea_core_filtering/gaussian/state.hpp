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

#ifndef ROMEA_CORE_FILTERING__GAUSSIAN__STATE_HPP_
#define ROMEA_CORE_FILTERING__GAUSSIAN__STATE_HPP_

#include "romea_core_filtering/gaussian/distribution.hpp"

namespace romea
{
namespace core
{

template<typename Scalar, size_t DIM>
struct GaussianState : GaussianDistribution<Scalar, DIM>
{
  GaussianState() : GaussianDistribution<Scalar, DIM>() {}

  virtual ~GaussianState() = default;

  typename GaussianDistribution<Scalar, DIM>::FirstMoment & X() { return this->first_moment; }

  const typename GaussianDistribution<Scalar, DIM>::FirstMoment & X() const
  {
    return this->first_moment;
  }

  Scalar & X(const size_t & i) { return this->first_moment(i); }

  const Scalar & X(const size_t & i) const { return this->first_moment(i); }

  typename GaussianDistribution<Scalar, DIM>::SecondMoment & P() { return this->second_moment; }

  const typename GaussianDistribution<Scalar, DIM>::SecondMoment & P() const
  {
    return this->second_moment;
  }

  const Scalar & P(const size_t & i, const size_t & j) const { return this->second_moment(i, j); }

  Scalar & P(const size_t & i, const size_t & j) { return this->second_moment(i, j); }

  void reset()
  {
    this->first_moment.setConstant(NAN);
    this->second_moment.setZero();
  }
};

template<typename Scalar>
struct GaussianState<Scalar, 1> : GaussianDistribution<Scalar, 1>
{
  GaussianState() : GaussianDistribution<Scalar, 1>() {}

  virtual ~GaussianState() = default;

  typename GaussianDistribution<Scalar, 1>::FirstMoment & X() { return this->first_moment; }

  const typename GaussianDistribution<Scalar, 1>::FirstMoment & X() const
  {
    return this->first_moment;
  }

  Scalar & X(const size_t &) { return this->first_moment; }

  const Scalar & X(const size_t &) const { return this->first_moment; }

  typename GaussianDistribution<Scalar, 1>::SecondMoment & P() { return this->second_moment; }

  const typename GaussianDistribution<Scalar, 1>::SecondMoment & P() const
  {
    return this->second_moment;
  }

  const Scalar & P(const size_t &, const size_t &) const { return this->second_moment; }

  Scalar & P(const size_t &, const size_t &) { return this->second_moment; }

  void reset()
  {
    this->first_moment = NAN;
    this->second_moment = 0;
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__GAUSSIAN__STATE_HPP_
