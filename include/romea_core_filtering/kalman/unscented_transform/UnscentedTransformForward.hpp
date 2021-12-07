#ifndef romea_UnscentedTransformForward_hpp
#define romea_UnscentedTransformForward_hpp

//romea
#include "../../GaussianDistribution.hpp"
#include "UnscentedTransformParameters.hpp"

//std
#include <vector>

//eigen
#include <Eigen/SVD>

namespace romea {


template<typename Scalar, size_t DIM>
struct UnscentedTransformFoward
{

  static void toSigmaPoints(const UnscentedTransformParameters<Scalar> & parameters,
                            const GaussianDistribution<Scalar,DIM> & gaussianDistribution,
                            typename GaussianDistribution<Scalar,DIM>::SigmaPoints & sigmaPoints)
  {
    assert(sigmaPoints.size()==parameters.meanWeights.size());

    const Scalar & gamma = parameters.gamma;
    const auto & firstMoment = gaussianDistribution.firstMoment;
    const auto & secondMoment = gaussianDistribution.secondMoment;

    Eigen::JacobiSVD<Eigen::Matrix<Scalar,-1,-1>> svd(secondMoment,Eigen::ComputeThinU|Eigen::ComputeThinV);
    auto sqrCovariance = svd.matrixU()*Eigen::Matrix<Scalar,DIM,1>(svd.singularValues().array().sqrt()).asDiagonal()*svd.matrixV().transpose();

    sigmaPoints[0]=firstMoment;
    for(size_t n=0;n<DIM;++n){
      sigmaPoints[n+1]= firstMoment+ gamma*sqrCovariance.col(int(n));
      sigmaPoints[n+1+DIM]= firstMoment- gamma*sqrCovariance.col(int(n));
    }
  }
};

}//romea

#endif

