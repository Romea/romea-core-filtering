// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERRESAMPLING_HPP_
#define ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERRESAMPLING_HPP_

// eigen
#include <Eigen/Core>

// std
#include <limits>
#include <numeric>
#include <random>
#include <iostream>
#include <utility>

// romea
#include "romea_core_filtering/particle/ParticleFilterState.hpp"

namespace romea
{

enum class ParticleFilterResamplingScheme
{
  MUTINOMIAL,
  STRATIFIED,
  SYSTEMATIC,
};

template<class Scalar, size_t DIM>
class ParticleFilterResampling
{
public:
  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic, Eigen::RowMajor> RowMajorVector;
  typedef Eigen::Array<Scalar, DIM, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

public:
  ParticleFilterResampling(
    const size_t & numberOfParticles,
    const long long & rngSeed = std::random_device()());

  void resampling(
    ParticleFilterState<Scalar, DIM> & state,
    const ParticleFilterResamplingScheme & resamplingType,
    const Scalar & numberOfeffectiveSampleThreshold = 1.);

  void setNumberOfEffectiveSampleThreshold(const Scalar & threshold);

protected:
  void normalizeStateWeights_(RowMajorVector & weights);

  void computeStateCumSumWeights_(const RowMajorVector & weights);

  void computeRandomCumSumWeights_(const ParticleFilterResamplingScheme & resamplingScheme);

  void computeMultinomialRandomCumSumWeights_();

  void computeStatifiedRandomCumSumWeights_();

  void computeSystematicRandomCumSumWeights_();

  void resampling_(ParticleFilterState<Scalar, DIM> & state);

protected:
  size_t numberOfParticles_;
  RowMajorVector stateCumSumWeights_;
  RowMajorVector randomCumSumWeights_;
  RowMajorMatrix resampledParticles_;

  std::mt19937_64 rng_;
  std::uniform_real_distribution<Scalar> uniform_distribution;
};

//-----------------------------------------------------------------------------
template<class Scalar, size_t DIM>
ParticleFilterResampling<Scalar, DIM>::ParticleFilterResampling(
  const size_t & numberOfParticles,
  const long long & rngSeed)
: numberOfParticles_(numberOfParticles),
  stateCumSumWeights_(RowMajorVector::Zero(numberOfParticles)),
  randomCumSumWeights_(RowMajorVector::Zero(numberOfParticles)),
  resampledParticles_(RowMajorMatrix::Zero(DIM, numberOfParticles)),
  rng_(rngSeed),
  uniform_distribution(0, 1)
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::resampling(
  ParticleFilterState<Scalar, DIM> & state,
  const ParticleFilterResamplingScheme & resamplingScheme,
  const Scalar & numberOfeffectiveSampleThreshold)
{
  assert(size_t(state.particles.cols()) == numberOfParticles_);

  normalizeStateWeights_(state.weights);
  double numberOfEffectiveSamples = 1. / (state.weights.array().square().sum());
  if (numberOfEffectiveSamples < numberOfParticles_ * numberOfeffectiveSampleThreshold) {
    computeStateCumSumWeights_(state.weights);
    computeRandomCumSumWeights_(resamplingScheme);
    resampling_(state);
  }
}


//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::normalizeStateWeights_(RowMajorVector & weights)
{
  // Check if degenerescence
  double weightsSum = weights.array().sum();
  if (weightsSum < std::numeric_limits<Scalar>::epsilon()) {
    throw std::runtime_error("Particle filter degeneracy");
  }

  weights.array() /= weightsSum;
}


//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::computeStateCumSumWeights_(
  const RowMajorVector & weights)
{
  std::partial_sum(
    weights.data(),
    weights.data() + numberOfParticles_,
    stateCumSumWeights_.data());

  stateCumSumWeights_ /= stateCumSumWeights_(numberOfParticles_ - 1);
}


//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::computeStatifiedRandomCumSumWeights_()
{
  for (size_t n = 0; n < numberOfParticles_; ++n) {
    randomCumSumWeights_(n) = (n + uniform_distribution(rng_)) / numberOfParticles_;
  }
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::computeMultinomialRandomCumSumWeights_()
{
  Scalar cumSumWeight = 0;
  for (size_t n = 0; n < numberOfParticles_; ++n) {
    cumSumWeight -= std::log2(uniform_distribution(rng_));
    randomCumSumWeights_(n) = cumSumWeight;
  }
  randomCumSumWeights_ /= cumSumWeight;
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::computeSystematicRandomCumSumWeights_()
{
  Scalar us = uniform_distribution(rng_);
  for (size_t n = 0; n < numberOfParticles_; ++n) {
    randomCumSumWeights_(n) = (n + us) / numberOfParticles_;
  }
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::computeRandomCumSumWeights_(
  const ParticleFilterResamplingScheme & resamplingScheme)
{
//  //draw random weights
//  randomCumSumWeights_.setRandom();
//  randomCumSumWeights_.array()+=1;

//  //Compute random cumulative sum wieghts
//  std::partial_sum(randomCumSumWeights_.data(),
//                   randomCumSumWeights_.data()+numberOfParticles_,
//                   randomCumSumWeights_.data());

//  randomCumSumWeights_/=randomCumSumWeights_(numberOfParticles_-1);

  switch (resamplingScheme) {
    case ParticleFilterResamplingScheme::MUTINOMIAL:
      computeMultinomialRandomCumSumWeights_();
      break;
    case ParticleFilterResamplingScheme::STRATIFIED:
      computeStatifiedRandomCumSumWeights_();
      break;
    case ParticleFilterResamplingScheme::SYSTEMATIC:
      computeSystematicRandomCumSumWeights_();
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<class Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::resampling_(ParticleFilterState<Scalar, DIM> & state)
{
  std::cout << " resampling" << std::endl;
  std::cout << stateCumSumWeights_(0) << " " <<
    stateCumSumWeights_(numberOfParticles_ - 1) << std::endl;
  std::cout << randomCumSumWeights_(0) << " " <<
    randomCumSumWeights_(numberOfParticles_ - 1) << std::endl;

  size_t j = 0;
  for (size_t i = 0; i < numberOfParticles_; ++i) {
    do {
      if (randomCumSumWeights_(i) < stateCumSumWeights_(j)) {
        // std::cout << i <<" "<<resampledParticles_.cols()<<" "
        //           <<j <<" "<<state.particles.cols()<< std::endl;
        resampledParticles_.col(i) = state.particles.col(j);
        break;
      }
      ++j;
    } while (j < numberOfParticles_);

    if (j == numberOfParticles_) {
      resampledParticles_.col(i) = resampledParticles_.col(0);
    }
  }

  // swap particles and reset weights
  std::swap(state.particles, resampledParticles_);
  state.weights.setConstant(1. / numberOfParticles_);
}

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__PARTICLE__PARTICLEFILTERRESAMPLING_HPP_
