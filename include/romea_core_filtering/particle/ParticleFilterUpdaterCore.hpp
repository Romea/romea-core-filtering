#ifndef ROMEA_CORE_FILTERING_PARTICLE_PARTICLEFILTERUPDATERCORE_HPP_
#define ROMEA_CORE_FILTERING_PARTICLE_PARTICLEFILTERUPDATERCORE_HPP_

// eigen
#include <Eigen/Core>

// romea
#include "romea_core_filtering/particle/ParticleFilterState.hpp"
#include "romea_core_filtering/particle/ParticleFilterResampling.hpp"


namespace romea {

template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
class PFUpdaterCore
{
public :

  using RowMajorVector = Eigen::Array<Scalar, 1, Eigen::Dynamic>;

public :

  explicit PFUpdaterCore(const std::size_t & numberOfParticles);

  virtual ~PFUpdaterCore() = default;

protected :

  size_t numberOfParticles_;
  ParticleFilterResampling<Scalar, StateDIM> resampling_;
};

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
PFUpdaterCore<Scalar, StateDIM, ObservationDIM>::
PFUpdaterCore(const std::size_t &numberOfParticles):
  numberOfParticles_(numberOfParticles),
  resampling_(numberOfParticles)
{
}

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_PARTICLE_PARTICLEFILTERUPDATERCORE_HPP_
