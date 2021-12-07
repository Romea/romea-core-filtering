#ifndef romea_PFUpdaterCore_hpp
#define romea_PFUpdaterCore_hpp

//romea
#include "ParticleFilterState.hpp"
#include "ParticleFilterResampling.hpp"

//eigen
#include <Eigen/Core>

namespace romea {

template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
class PFUpdaterCore
{

public :

  using RowMajorVector = Eigen::Array<Scalar,1,Eigen::Dynamic>;

public :

  PFUpdaterCore(const std::size_t & numberOfParticles);

  virtual ~PFUpdaterCore()=default;

protected :

  size_t numberOfParticles_;
  ParticleFilterResampling<Scalar,StateDIM> resampling_;
};

//-----------------------------------------------------------------------------
template <typename Scalar, size_t StateDIM, size_t ObservationDIM>
PFUpdaterCore<Scalar,StateDIM,ObservationDIM>::PFUpdaterCore(const std::size_t &numberOfParticles):
  numberOfParticles_(numberOfParticles),
  resampling_(numberOfParticles)
{

}

}

#endif
