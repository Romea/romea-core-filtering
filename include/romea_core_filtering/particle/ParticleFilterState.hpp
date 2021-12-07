#ifndef romea_ParticleFilterState_hpp
#define romea_ParticleFilterState_hpp

//Eigen
#include <Eigen/Core>

namespace romea {

template <typename Scalar, size_t DIM >
struct ParticleFilterState
{

public :

  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic, Eigen::RowMajor> RowMajorVector;
  typedef Eigen::Array<Scalar, DIM, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

public:

  ParticleFilterState(const size_t & numberOfParticles);

  virtual ~ParticleFilterState()=default;

  virtual void reset();

  RowMajorMatrix particles;
  RowMajorVector weights;
};


//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
ParticleFilterState<Scalar,DIM>::ParticleFilterState(const size_t & numberOfParticles):
  particles(RowMajorMatrix::Constant(DIM,numberOfParticles,NAN)),
  weights(RowMajorVector::Constant(numberOfParticles,1./numberOfParticles))
{

}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterState<Scalar,DIM>::reset()
{
  particles.setConstant(NAN);
  weights.setConstant(1./weights.cols());
}



}

#endif
