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


#ifndef ROMEA_CORE_FILTERING__GAUSSIANDISTRIBUTION_HPP_
#define ROMEA_CORE_FILTERING__GAUSSIANDISTRIBUTION_HPP_

// Eigen
#include <Eigen/Core>

// std
#include <vector>

namespace romea
{
namespace core
{

template<typename Scalar, size_t DIM>
struct GaussianDistribution
{
  using FirstMoment = Eigen::Matrix<Scalar, DIM, 1>;
  using SecondMoment = Eigen::Matrix<Scalar, DIM, DIM>;
  using SigmaPoints = std::vector<FirstMoment, Eigen::aligned_allocator<FirstMoment>>;

  GaussianDistribution()
  : firstMoment(FirstMoment::Constant(NAN)),
    secondMoment(SecondMoment::Zero())
  {
  }

  virtual ~GaussianDistribution() = default;

  FirstMoment firstMoment;
  SecondMoment secondMoment;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Scalar, DIM)
};


template<typename Scalar>
struct GaussianDistribution<Scalar, 1>
{
  using FirstMoment = Scalar;
  using SecondMoment = Scalar;
  using SigmaPoints = std::vector<Scalar>;


  GaussianDistribution()
  : firstMoment(NAN),
    secondMoment(0)
  {
  }

  virtual ~GaussianDistribution() = default;

  FirstMoment firstMoment;
  SecondMoment secondMoment;
};


//template <typename Scalar, size_t DIM>
//inline GaussianDistribution<Scalar,DIM> addition(const GaussianDistribution<Scalar,DIM> & distribution1,
//                                                 const GaussianDistribution<Scalar,DIM> & distribution2,
//                                                 GaussianDistribution<Scalar,DIM> & distribution1add2)
//{
//  distribution1add2.firstMoment = distribution1.firstMoment + distribution2.firstMoment;
//  distribution1add2.secondMoment = distribution1.secondMoment + distribution2.secondMoment;

//}


//template <typename Scalar, size_t DIM>
//inline GaussianDistribution<Scalar,DIM> operator+(const GaussianDistribution<Scalar,DIM> & distribution1,
//                                                  const GaussianDistribution<Scalar,DIM> & distribution2)
//{
//  GaussianDistribution<Scalar,DIM> distribution12;
//  addition(distribution1,distribution2,distribution12);
//  return distribution12;
//}


//template <typename Scalar, size_t DIM>
//inline GaussianDistribution<Scalar,DIM> substraction(const GaussianDistribution<Scalar,DIM> & distribution1,
//                                                     const GaussianDistribution<Scalar,DIM> & distribution2,
//                                                     GaussianDistribution<Scalar,DIM> & distribution1add2)
//{
//  distribution1add2.firstMoment = distribution1.firstMoment - distribution2.firstMoment;
//  distribution1add2.secondMoment = distribution1.secondMoment + distribution2.secondMoment;

//}


//template <typename Scalar, size_t DIM>
//inline GaussianDistribution<Scalar,DIM> operator-(const GaussianDistribution<Scalar,DIM> & distribution1,
//                                                  const GaussianDistribution<Scalar,DIM> & distribution2)
//{
//  GaussianDistribution<Scalar,DIM> distribution12;
//  substraction(distribution1,distribution2,distribution12);
//  return distribution12;
//}


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__GAUSSIANDISTRIBUTION_HPP_
