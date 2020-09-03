#ifndef romea_GaussianDistribution_hpp
#define romea_GaussianDistribution_hpp

//Eigen
#include <Eigen/Core>

//std
#include <vector>

namespace romea {

template<typename Scalar,size_t DIM >
struct GaussianDistribution
{

  using FirstMoment = Eigen::Matrix<Scalar,DIM,1>;
  using SecondMoment = Eigen::Matrix<Scalar,DIM,DIM>;
  using SigmaPoints = std::vector<FirstMoment,Eigen::aligned_allocator<FirstMoment>>;

  GaussianDistribution():
    firstMoment(FirstMoment::Constant(NAN)),
    secondMoment(SecondMoment::Zero())
  {

  }

  virtual ~GaussianDistribution()=default;

  FirstMoment firstMoment;
  SecondMoment secondMoment;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Scalar,DIM)
};


template<typename Scalar>
struct GaussianDistribution<Scalar,1>
{

  using FirstMoment = Scalar;
  using SecondMoment = Scalar;
  using SigmaPoints = std::vector<Scalar>;


  GaussianDistribution():
    firstMoment(NAN),
    secondMoment(0)
  {

  }

  virtual ~GaussianDistribution()=default;

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





}//romea

#endif
