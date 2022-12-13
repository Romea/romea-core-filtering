#ifndef ROMEA_CORE_FILTERING_PARTICLE_PARTICLEFILTERSTATE_HPP_
#define ROMEA_CORE_FILTERING_PARTICLE_PARTICLEFILTERSTATE_HPP_

// Eigen
#include <Eigen/Core>

namespace romea {

template <typename Scalar, size_t DIM >
struct ParticleFilterState
{
public :

  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic, Eigen::RowMajor> RowMajorVector;
  typedef Eigen::Array<Scalar, DIM, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

public:

  explicit ParticleFilterState(const size_t & numberOfParticles);

  virtual ~ParticleFilterState() = default;

  virtual void reset();

  RowMajorMatrix particles;
  RowMajorVector weights;
};


//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
ParticleFilterState<Scalar, DIM>::ParticleFilterState(const size_t & numberOfParticles):
  particles(RowMajorMatrix::Constant(DIM, numberOfParticles, NAN)),
  weights(RowMajorVector::Constant(numberOfParticles, 1./numberOfParticles))
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterState<Scalar, DIM>::reset()
{
  particles.setConstant(NAN);
  weights.setConstant(1./weights.cols());
}

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING_PARTICLE_PARTICLEFILTERSTATE_HPP_
