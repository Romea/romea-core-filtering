#ifndef romea_KalmanUpdate_hpp
#define romea_KalmanUpdate_hpp

//romea
#include "../../GaussianState.hpp"
#include "KFUpdaterTraits.hpp"

namespace romea {


//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdateStateVector
{
  static void compute(typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::X & X,
                      const typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::Inn & Inn,
                      const typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::K & K)
  {
    X+=K*Inn;
  }
};

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdateStateCovariance
{

  static void compute(typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::P & P,
                      const typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::QInn & QInn,
                      const typename KFUpdaterTraits<Scalar,StateDIM,ObservationDIM>::K & K)
  {
    P-=K*QInn*K.transpose();
  }
};


//-----------------------------------------------------------------------------
template <typename Scalar>
struct KFUpdateStateCovariance<Scalar,1,1>
{
  static void compute(Scalar & P,
                      const Scalar & QInn,
                      const Scalar & K)
  {
    P-=K*QInn*K;
  }
};


}


#endif
