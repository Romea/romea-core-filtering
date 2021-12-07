//#ifndef romea_ParticleFilterEstimator_hpp
//#define romea_ParticleFilterEstimator_hpp

////Eigen
//#include <Eigen/Core>

//#include "../GaussianDistribution.hpp"

//namespace romea {

//template <typename Scalar,size_t DIM>
//class ParticleFilterEstimator
//{
  
//public :
  
//  using RowMajorMatrix = Eigen::Array<Scalar, Eigen::Dynamic, DIM, Eigen::RowMajor> ;
  
//public:
  
//  ParticleFilterEstimator(const size_t & numberOfParticles);
  
//  GaussianDistribution<Scalar,DIM> compute(const RowMajorMatrix & particles,
//                                           const RowMajorVector & particlesWeight,
//                                           GaussianDistribution<Scalar,DIM> & distribution);
  
//protected:
  
//  RowMajorMatrix meanCenteredParticles_;
  
//};


////-----------------------------------------------------------------------------
//template<typename Scalar,size_t DIM>
//ParticleFilterEstimator<Scalar,DIM>::ParticleFilterEstimator(const size_t & numberOfParticles):
//  meanCenteredParticles_(RowMajorMatrix::Zero(DIM,numberOfParticles))
//{
  
//}


////-----------------------------------------------------------------------------
//template<typename Scalar,size_t DIM> GaussianDistribution<Scalar,DIM>
//ParticleFilterEstimator<Scalar,DIM>::compute(const RowMajorMatrix & particles,
//                                             const RowMajorVector & particlesWeight,
//                                             GaussianDistribution<Scalar,DIM> & distribution)
//{
  
//  auto & estimate = distribution.firstMoment;
//  auto & estimateCovariance = distribution.secondMoment;
//  Scalar particlesWeightSum = particlesWeight.sum();
  
//  for(int i=0;i<dimension;++i){
//    estimate(i) = (particles.row(i)*particlesWeight).sum()/particlesWeightSum;
//    meanCenteredParticles_.row(i) -= particles.row(i)-estimate(i);
//  }
  
//  for(int i=0 ; i<dimension ; ++i){
//    for(int j=i ; j<dimension ; ++j){
//      estimateCovariance(i,j) = estimateCovariance(j,i)= (meanCenteredParticles_.row(i)*
//                                                          meanCenteredParticles_.row(j)*
//                                                          particlesWeight).sum()/particlesWeightSum;
//    }
//  }
//}

//template <typename Scalar>
//class ParticleFilterEstimator<Scalar,1>
//{
  
//public :
  
//  using RowMajorVector = Eigen::Array<Scalar, Eigen::Dynamic, 1, Eigen::RowMajor>;
  
//public:
  
//  ParticleFilterEstimator();
  
//  GaussianDistribution<Scalar,1> compute(const RowMajorVector & particles,
//                                         const RowMajorVector & particlesWeight);
//};


////-----------------------------------------------------------------------------
//template<typename Scalar,size_t DIM>
//ParticleFilterEstimator<Scalar,DIM>::ParticleFilterEstimator(const size_t & numberOfParticles)
//{
  
//}


////-----------------------------------------------------------------------------
//template<typename Scalar> GaussianDistribution<Scalar,1>
//ParticleFilterEstimator<Scalar,1>::compute(const RowMajorVector & particles,
//                                           const RowMajorVector & particlesWeight)
//{
//  GaussianDistribution<Scalar,DIM> & distribution;
//  distribution.firstMoment = (particlesWeight*particles).sum();
//  distribution.secondMoment = (particlesWeight*(particles-distribution.firstMoment).square()).sum();
//}

//}

//#endif
