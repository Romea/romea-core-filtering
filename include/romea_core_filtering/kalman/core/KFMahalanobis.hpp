#ifndef ROMEA_CORE_FILTERING_KALMAN_CORE_KFMAHALANOBIS_HPP_
#define ROMEA_CORE_FILTERING_KALMAN_CORE_KFMAHALANOBIS_HPP_

// std
#include <limits>


// romea
#include "romea_core_filtering/GaussianDistribution.hpp"
#include "romea_core_filtering/kalman/core/KFUpdaterTraits.hpp"


namespace romea {


//-----------------------------------------------------------------------------
template <typename Scalar, size_t ObservationDIM>
struct KFMahalanobis
{
  static Scalar compute(const Eigen::Matrix<Scalar, ObservationDIM, 1> & Inn,
                        const Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> & QInn,
                        Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM> & QInnInverse)
  {
    // Check if QInn_ is inversible
    if (QInn.determinant() < std::numeric_limits<double>::epsilon())
    {
      throw std::runtime_error("Innovation covariance cannot be inversed");
    }

    QInnInverse = QInn.ldlt().solve(
      Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM>::Identity());

    return std::sqrt((Inn.transpose()*QInnInverse*Inn)(0, 0));
  }
};


//-----------------------------------------------------------------------------
template <typename Scalar>
struct KFMahalanobis<Scalar, 1>
{
  static Scalar compute(const Scalar & Inn, const Scalar & QInn, Scalar & QInnInverse )
  {
    QInnInverse = 1/QInn;
    return std::sqrt(Inn*Inn*QInnInverse);
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_KALMAN_CORE_KFMAHALANOBIS_HPP_ 
