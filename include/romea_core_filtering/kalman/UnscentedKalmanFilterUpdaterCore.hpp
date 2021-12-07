#ifndef romea_UKFUpdaterCore_hpp
#define romea_UKFUpdaterCore_hpp

//romea
#include "core/KFUpdate.hpp"
#include "core/KFInnovation.hpp"
#include "core/KFMahalanobis.hpp"
#include "unscented_transform/UKFCorrelation.hpp"
#include "unscented_transform/UnscentedTransformForward.hpp"
#include "unscented_transform/UnscentedTransformInverse.hpp"

namespace romea {

template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
class UKFUpdaterCore
{

public :

  using State = GaussianState<Scalar,StateDIM>;
  using Observation = GaussianObservation<Scalar,ObservationDIM>;
  using CorrelationMatrix = Eigen::Matrix<Scalar,StateDIM,ObservationDIM> ;


public :

  UKFUpdaterCore(const double & UTKappa,
                 const double & UTAlpha,
                 const double & UTBeta,
                 const double & maximalMahalanobisDistance);

  virtual ~UKFUpdaterCore()=default;

protected :

  bool updateState_(State &state);

  bool updateState_(State &state,const Observation & observation);

  void computeStateSigmaPoints_(const State & state);

protected :

  UnscentedTransformParameters<Scalar> unscentedTransformParameters_;
  typename GaussianDistribution<Scalar,StateDIM>::SigmaPoints stateSigmaPoints_;
  typename GaussianDistribution<Scalar,ObservationDIM>::SigmaPoints propagatedSigmaPoints_;
  GaussianObservation<Scalar,ObservationDIM> propagatedState_;

  typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::Inn Inn_;
  typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::QInn QInn_;
  typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::QInn QInnInverse_;
  typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::K K_;

  Scalar mahalanobisDistance_;
  Scalar maximalMahalanobisDistance_;


};

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
UKFUpdaterCore<Scalar,StateDIM,ObservationDIM>::UKFUpdaterCore(const double & UTKappa,
                                                               const double & UTAlpha,
                                                               const double & UTBeta,
                                                               const double & maximalMahalanobisDistance):
  unscentedTransformParameters_(StateDIM,UTKappa,UTAlpha,UTBeta),
  stateSigmaPoints_(2*StateDIM+1),
  propagatedSigmaPoints_(2*StateDIM+1),
  propagatedState_(),
  Inn_(Zero<typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::Inn>::zero()),
  QInn_(Zero<typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::QInn>::zero()),
  QInnInverse_(Zero<typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::QInn>::zero()),
  K_(Zero<typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::K>::zero()),
  mahalanobisDistance_(std::numeric_limits<Scalar>::max()),
  maximalMahalanobisDistance_(maximalMahalanobisDistance)
{

}


//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM> void
UKFUpdaterCore<Scalar , StateDIM , ObservationDIM>::computeStateSigmaPoints_(const State & state)
{
  UnscentedTransformFoward<Scalar,StateDIM>::toSigmaPoints(unscentedTransformParameters_,
                                                           state,
                                                           stateSigmaPoints_);
}


//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
bool UKFUpdaterCore<Scalar , StateDIM , ObservationDIM>::updateState_(State &state,const Observation & observation)
{
  UnscentedTransformInverse<Scalar,ObservationDIM>::toGaussian(unscentedTransformParameters_,
                                                               propagatedSigmaPoints_,
                                                               propagatedState_);

  this->Inn_= observation.Y() - propagatedState_.Y();
  this->QInn_=observation.R() + propagatedState_.R();
  return updateState_(state);
}


//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
bool UKFUpdaterCore<Scalar , StateDIM , ObservationDIM>::updateState_(State &state)
{

  mahalanobisDistance_=KFMahalanobis<Scalar,ObservationDIM>::compute(Inn_,QInn_,QInnInverse_);
  if(mahalanobisDistance_<maximalMahalanobisDistance_)
  {
    UKFCorrelation<Scalar,StateDIM,ObservationDIM>::compute(unscentedTransformParameters_,
                                                            state,
                                                            propagatedState_,
                                                            stateSigmaPoints_,
                                                            propagatedSigmaPoints_,
                                                            K_);


    K_*=QInnInverse_;
    KFUpdateStateVector<Scalar,StateDIM,ObservationDIM>::compute(state.X(),Inn_,K_);
    KFUpdateStateCovariance<Scalar,StateDIM,ObservationDIM>::compute(state.P(),QInn_,K_);
    return true;
  }
  else
  {
    return false;
  }
}

}

#endif

