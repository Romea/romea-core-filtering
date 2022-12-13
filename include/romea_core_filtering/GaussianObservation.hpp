#ifndef ROMEA_CORE_FILTERING_GAUSSIANOBSERVATION_HPP_
#define ROMEA_CORE_FILTERING_GAUSSIANOBSERVATION_HPP_


#include "romea_core_filtering/GaussianDistribution.hpp"


namespace romea {


template<typename Scalar, size_t DIM >
struct GaussianObservation : GaussianDistribution<Scalar, DIM>
{
  GaussianObservation():
    GaussianDistribution<Scalar, DIM>()
  {
  }

  virtual ~GaussianObservation() = default;

  typename GaussianDistribution<Scalar, DIM>::FirstMoment & Y()
  {
    return this->firstMoment;
  }

  const typename GaussianDistribution<Scalar, DIM>::FirstMoment & Y()const
  {
    return this->firstMoment;
  }

  Scalar & Y(const size_t & i)
  {
    return this->firstMoment(i);
  }

  const Scalar & Y(const size_t & i)const
  {
    return this->firstMoment(i);
  }

  typename GaussianDistribution<Scalar, DIM>::SecondMoment & R()
  {
    return this->secondMoment;
  }

  const typename GaussianDistribution<Scalar, DIM>::SecondMoment & R()const
  {
    return this->secondMoment;
  }

  const Scalar & R(const size_t & i, const size_t &j)const
  {
    return this->secondMoment(i, j);
  }

  Scalar & R(const size_t & i, const size_t &j)
  {
    return this->secondMoment(i, j);
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_GAUSSIANOBSERVATION_HPP_
