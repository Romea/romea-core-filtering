// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_FILTERING__KALMAN__CORE__KFUPDATE_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__CORE__KFUPDATE_HPP_

// romea
#include "romea_core_filtering/GaussianState.hpp"
#include "romea_core_filtering/kalman/core/KFUpdaterTraits.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdateStateVector
{
  static void compute(
    typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::X & X,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::Inn & Inn,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K & K)
  {
    X += K * Inn;
  }
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdateStateCovariance
{
  static void compute(
    typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::P & P,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn & QInn,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K & K)
  {
    P -= K * QInn * K.transpose();
  }
};


//-----------------------------------------------------------------------------
template<typename Scalar>
struct KFUpdateStateCovariance<Scalar, 1, 1>
{
  static void compute(
    Scalar & P,
    const Scalar & QInn,
    const Scalar & K)
  {
    P -= K * QInn * K;
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__CORE__KFUPDATE_HPP_
