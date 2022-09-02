#ifndef romea_UnscentedTransformParameters_hpp
#define romea_UnscentedTransformParameters_hpp

//romea
#include "../../GaussianDistribution.hpp"

//std
#include <vector>

namespace romea {


template<typename Scalar>
struct UnscentedTransformParameters
{

  UnscentedTransformParameters(const size_t & DIM,
                               const double & kappa,
                               const double & alpha,
                               const double & beta):
    gamma(0),
    meanWeights(2*DIM+1),
    covarianceWeights(2*DIM+1)
  {
    double  lambda=(alpha*alpha*(DIM+kappa)-DIM);
    gamma = std::sqrt(DIM+lambda);

    meanWeights[0]=lambda/(DIM+lambda);
    covarianceWeights[0]=meanWeights[0] + 1+ alpha*alpha+beta;
    std::fill(std::begin(meanWeights)+1,std::end(meanWeights),1/(2*(DIM+lambda)));
    std::fill(std::begin(covarianceWeights)+1,std::end(covarianceWeights),1/(2*(DIM+lambda)));
  }

  double gamma;
  std::vector<Scalar> meanWeights;
  std::vector<Scalar> covarianceWeights;

};


}//romea

#endif
