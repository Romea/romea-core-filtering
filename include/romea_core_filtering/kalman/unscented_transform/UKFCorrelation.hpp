#ifndef ROMEA_CORE_FILTERING_KALMAN_UNSCENTED_TRANSFORM_UKFCORRELATION_HPP_
#define ROMEA_CORE_FILTERING_KALMAN_UNSCENTED_TRANSFORM_UKFCORRELATION_HPP_

#include "romea_core_filtering/GaussianState.hpp"
#include "romea_core_filtering/GaussianObservation.hpp"
#include "romea_core_filtering/kalman/unscented_transform/UnscentedTransformParameters.hpp"

namespace romea {

template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct UKFCorrelation
{
  static void compute(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianState<Scalar, StateDIM> & state,
    const GaussianObservation<Scalar, ObservationDIM> & propagatedState,
    const typename GaussianState<Scalar, StateDIM>::SigmaPoints & stateSigmaPoints,
    const typename GaussianDistribution<Scalar, ObservationDIM>::SigmaPoints & propagatedSigmaPoints,
    Eigen::Matrix<Scalar, StateDIM, ObservationDIM> & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 2*StateDIM+1);
    assert(propagatedSigmaPoints.size() == 2*StateDIM+1);
    assert(parameters.covarianceWeights.size() == 2*StateDIM+1);

    const auto & stateFirstMoment = state.firstMoment;
    const auto & propagatedFirstMoment = propagatedState.firstMoment;
    const auto & covarianceWeights = parameters.covarianceWeights;

    correlationMatrix.setConstant(0);
    for (size_t n = 0; n< 2*StateDIM+1; ++n)
    {
      correlationMatrix+= covarianceWeights[n]*
          (stateSigmaPoints[n]-stateFirstMoment)*
          (propagatedSigmaPoints[n]-propagatedFirstMoment).transpose();
    }
  }
};

template<typename Scalar, size_t StateDIM>
struct UKFCorrelation<Scalar, StateDIM, 1>
{
  static void compute(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianState<Scalar, StateDIM> & state,
    const GaussianObservation<Scalar, 1> & propagatedState,
    const typename GaussianState<Scalar, StateDIM>::SigmaPoints & stateSigmaPoints,
    const typename GaussianObservation<Scalar, 1>::SigmaPoints & propagatedSigmaPoints,
    Eigen::Matrix<Scalar, StateDIM, 1> & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 2*StateDIM+1);
    assert(propagatedSigmaPoints.size() == 2*StateDIM+1);
    assert(parameters.covarianceWeights.size() == 2*StateDIM+1);

    const auto & stateFirstMoment = state.firstMoment;
    const auto & propagatedFirstMoment = propagatedState.firstMoment;
    const auto & covarianceWeights = parameters.covarianceWeights;

    correlationMatrix.setConstant(0);
    for (size_t n = 0;n < 2*StateDIM+1; ++n)
    {
      correlationMatrix+= covarianceWeights[n]*
          (stateSigmaPoints[n]-stateFirstMoment)*
          (propagatedSigmaPoints[n]-propagatedFirstMoment);
    }
  }
};

template<typename Scalar>
struct UKFCorrelation<Scalar, 1, 1>
{
  static void compute(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianState<Scalar, 1> & state,
    const GaussianObservation<Scalar, 1> & propagatedState,
    const typename GaussianState<Scalar, 1>::SigmaPoints & stateSigmaPoints,
    const typename GaussianObservation<Scalar, 1>::SigmaPoints & propagatedSigmaPoints,
    double & correlationMatrix)
  {
    assert(stateSigmaPoints.size() == 3);
    assert(propagatedSigmaPoints.size() == 3);
    assert(parameters.covarianceWeights.size() == 3);

    const auto & stateFirstMoment = state.firstMoment;
    const auto & propagatedFirstMoment = propagatedState.firstMoment;
    const auto & covarianceWeights = parameters.covarianceWeights;

    correlationMatrix = 0;
    for (size_t n = 0; n < 3; ++n)
    {
      correlationMatrix+= covarianceWeights[n]*
          (stateSigmaPoints[n]-stateFirstMoment)*
          (propagatedSigmaPoints[n]-propagatedFirstMoment);
    }
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_KALMAN_UNSCENTED_TRANSFORM_UKFCORRELATION_HPP_

