#ifndef ROMEA_CORE_FILTERING_GAUSSIANSTATE_HPP_
#define ROMEA_CORE_FILTERING_GAUSSIANSTATE_HPP_

#include "romea_core_filtering/GaussianDistribution.hpp"

namespace romea {

template<typename Scalar, size_t DIM >
struct GaussianState : GaussianDistribution<Scalar, DIM>
{
  GaussianState():
    GaussianDistribution<Scalar, DIM>()
  {
  }

  virtual ~GaussianState() = default;

  typename GaussianDistribution<Scalar, DIM>::FirstMoment & X()
  {
    return this->firstMoment;
  }

  const typename GaussianDistribution<Scalar, DIM>::FirstMoment & X()const
  {
    return this->firstMoment;
  }

  Scalar & X(const size_t & i)
  {
    return this->firstMoment(i);
  }

  const Scalar & X(const size_t & i)const
  {
    return this->firstMoment(i);
  }

  typename GaussianDistribution<Scalar, DIM>::SecondMoment & P()
  {
    return this->secondMoment;
  }

  const typename GaussianDistribution<Scalar, DIM>::SecondMoment & P()const
  {
    return this->secondMoment;
  }

  const Scalar & P(const size_t & i, const size_t &j)const
  {
    return this->secondMoment(i, j);
  }

  Scalar & P(const size_t & i, const size_t &j)
  {
    return this->secondMoment(i, j);
  }


  void reset()
  {
    this->firstMoment.setConstant(NAN);
    this->secondMoment.setZero();
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_GAUSSIANSTATE_HPP_
