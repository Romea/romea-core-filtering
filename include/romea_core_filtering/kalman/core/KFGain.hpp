#ifndef ROMEA_CORE_FILTERING_KALMAN_CORE_KFGAIN_HPP_
#define ROMEA_CORE_FILTERING_KALMAN_CORE_KFGAIN_HPP_

// romea
#include "romea_core_filtering/kalman/core/KFUpdaterTraits.hpp"

namespace romea {

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFGain
{
  static void compute(
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::P & P,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::H & H,
    const typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::QInn & QInnInverse,
    typename KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>::K & K)
  {
    K = P*H.transpose()*QInnInverse;
  }
};


//-----------------------------------------------------------------------------
template <typename Scalar>
struct KFGain<Scalar, 1, 1>
{
  static void compute(const Scalar & P,
                      const Scalar & H,
                      const Scalar & QInnInverse,
                      Scalar & K)
  {
    K = P*H*QInnInverse;
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_KALMAN_CORE_KFGAIN_HPP_
