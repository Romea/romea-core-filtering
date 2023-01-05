// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMFORWARD_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__UNSCENTED_TRANSFORM__UNSCENTEDTRANSFORMFORWARD_HPP_

// eigen
#include <Eigen/SVD>

// std
#include <vector>

// romea
#include "romea_core_filtering/GaussianDistribution.hpp"
#include "romea_core_filtering/kalman/unscented_transform/UnscentedTransformParameters.hpp"

namespace romea
{


template<typename Scalar, size_t DIM>
struct UnscentedTransformFoward
{
  static void toSigmaPoints(
    const UnscentedTransformParameters<Scalar> & parameters,
    const GaussianDistribution<Scalar, DIM> & gaussianDistribution,
    typename GaussianDistribution<Scalar, DIM>::SigmaPoints & sigmaPoints)
  {
    assert(sigmaPoints.size() == parameters.meanWeights.size());

    const Scalar & gamma = parameters.gamma;
    const auto & firstMoment = gaussianDistribution.firstMoment;
    const auto & secondMoment = gaussianDistribution.secondMoment;

    Eigen::JacobiSVD<Eigen::Matrix<Scalar, -1, -1>> svd(
      secondMoment, Eigen::ComputeThinU | Eigen::ComputeThinV);

    auto sqrCovariance = svd.matrixU() * Eigen::Matrix<Scalar, DIM, 1>(
      svd.singularValues().array().sqrt()).asDiagonal() * svd.matrixV().transpose();

    sigmaPoints[0] = firstMoment;
    for (size_t n = 0; n < DIM; ++n) {
      sigmaPoints[n + 1] = firstMoment + gamma * sqrCovariance.col(int(n));
      sigmaPoints[n + 1 + DIM] = firstMoment - gamma * sqrCovariance.col(int(n));
    }
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN_UNSCENTED__TRANSFORM__UNSCENTEDTRANSFORMFORWARD_HPP_
