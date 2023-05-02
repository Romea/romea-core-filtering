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


#ifndef ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERSTATE_HPP_
#define ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERSTATE_HPP_

// Eigen
#include <Eigen/Core>

namespace romea
{

template<typename Scalar, size_t DIM>
struct ParticleFilterState
{
public:
  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic, Eigen::RowMajor> RowMajorVector;
  typedef Eigen::Array<Scalar, DIM, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

public:
  explicit ParticleFilterState(const size_t & numberOfParticles);

  virtual ~ParticleFilterState() = default;

  virtual void reset();

  RowMajorMatrix particles;
  RowMajorVector weights;
};


//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
ParticleFilterState<Scalar, DIM>::ParticleFilterState(const size_t & numberOfParticles)
: particles(RowMajorMatrix::Constant(DIM, numberOfParticles, NAN)),
  weights(RowMajorVector::Constant(numberOfParticles, 1. / numberOfParticles))
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterState<Scalar, DIM>::reset()
{
  particles.setConstant(NAN);
  weights.setConstant(1. / weights.cols());
}

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERSTATE_HPP_
