#ifndef ROMEA_CORE_FILTERING_GAUSSIANINPUT_HPP_
#define ROMEA_CORE_FILTERING_GAUSSIANINPUT_HPP_


#include "romea_core_filtering/GaussianDistribution.hpp"

namespace romea {


template<typename Scalar, size_t DIM >
struct GaussianInput : GaussianDistribution<Scalar, DIM>
{
  GaussianInput():
    GaussianDistribution<Scalar, DIM>()
  {
  }

  virtual ~GaussianInput() = default;

  typename GaussianDistribution<Scalar, DIM>::FirstMoment & U()
  {
    return this->firstMoment;
  }

  const typename GaussianDistribution<Scalar, DIM>::FirstMoment & U()const
  {
    return this->firstMoment;
  }

  Scalar & U(const size_t & i)
  {
    return this->firstMoment(i);
  }

  const Scalar & U(const size_t & i)const
  {
    return this->firstMoment(i);
  }

  typename GaussianDistribution<Scalar, DIM>::SecondMoment & QU()
  {
    return this->secondMoment;
  }

  const typename GaussianDistribution<Scalar, DIM>::SecondMoment & QU()const
  {
    return this->secondMoment;
  }

  const Scalar & QU(const size_t & i, const size_t &j)const
  {
    return this->secondMoment(i, j);
  }

  Scalar & QU(const size_t & i, const size_t &j)
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

#endif  // ROMEA_CORE_FILTERING_GAUSSIANINPUT_HPP_
