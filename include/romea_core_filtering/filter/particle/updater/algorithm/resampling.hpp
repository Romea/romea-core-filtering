// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__RESAMPLING_HPP_
#define ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__RESAMPLING_HPP_

// eigen
#include <Eigen/Core>

// std
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <utility>

// romea
#include "romea_core_filtering/filter/particle/state.hpp"

namespace romea
{
namespace core
{

enum class ParticleFilterResamplingScheme
{
  MUTINOMIAL,
  STRATIFIED,
  SYSTEMATIC,
};

struct ParticleFilterResamplingStatus
{
  double number_of_effective_samples = 0.;
  bool resampled = false;
};

template<class Scalar, size_t DIM>
class ParticleFilterResampling
{
public:
  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic, Eigen::RowMajor> RowMajorVector;
  typedef Eigen::Array<Scalar, DIM, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

public:
  ParticleFilterResampling(
    const size_t & number_of_particles, const long long & rng_seed = std::random_device()());

  void resampling(
    ParticleFilterState<Scalar, DIM> & state,
    const ParticleFilterResamplingScheme & resampling_type,
    const Scalar & number_of_effective_sample_threshold = 1.);

  const ParticleFilterResamplingStatus & get_status() const;

  double get_number_of_effective_samples() const;

  bool has_resampled() const;

  void set_number_of_effective_sample_threshold(const Scalar & threshold);

protected:
  void normalize_state_weights_(RowMajorVector & weights);

  void compute_state_sum_sum_weights_(const RowMajorVector & weights);

  void compute_random_cum_sum_weights_(const ParticleFilterResamplingScheme & resampling_scheme);

  void compute_multinomial_random_cum_sum_weights_();

  void compute_statified_random_cum_sum_weights_();

  void compute_systematic_random_cum_sum_weights_();

  void resampling_(ParticleFilterState<Scalar, DIM> & state);

protected:
  size_t number_of_particles_;
  RowMajorVector state_cum_sum_weights_;
  RowMajorVector random_cum_sum_weights_;
  RowMajorMatrix resampled_particles_;
  ParticleFilterResamplingStatus status_;

  std::mt19937_64 rng_;
  std::uniform_real_distribution<Scalar> uniform_distribution;
};

//-----------------------------------------------------------------------------
template<class Scalar, size_t DIM>
ParticleFilterResampling<Scalar, DIM>::ParticleFilterResampling(
  const size_t & number_of_particles, const long long & rng_seed)
: number_of_particles_(number_of_particles),
  state_cum_sum_weights_(RowMajorVector::Zero(number_of_particles)),
  random_cum_sum_weights_(RowMajorVector::Zero(number_of_particles)),
  resampled_particles_(RowMajorMatrix::Zero(DIM, number_of_particles)),
  status_(),
  rng_(rng_seed),
  uniform_distribution(0, 1)
{
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::resampling(
  ParticleFilterState<Scalar, DIM> & state,
  const ParticleFilterResamplingScheme & resampling_scheme,
  const Scalar & number_of_effective_sample_threshold)
{
  assert(size_t(state.particles.cols()) == number_of_particles_);

  normalize_state_weights_(state.weights);
  status_.number_of_effective_samples = 1. / (state.weights.array().square().sum());
  status_.resampled =
    status_.number_of_effective_samples <
    number_of_particles_ * number_of_effective_sample_threshold;

  if (status_.resampled) {
    compute_state_sum_sum_weights_(state.weights);
    compute_random_cum_sum_weights_(resampling_scheme);
    resampling_(state);
  }
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
const ParticleFilterResamplingStatus & ParticleFilterResampling<Scalar, DIM>::get_status() const
{
  return status_;
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
double ParticleFilterResampling<Scalar, DIM>::get_number_of_effective_samples() const
{
  return status_.number_of_effective_samples;
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
bool ParticleFilterResampling<Scalar, DIM>::has_resampled() const
{
  return status_.resampled;
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::normalize_state_weights_(RowMajorVector & weights)
{
  // Check if degenerescence
  double weights_sum = weights.array().sum();
  if (weights_sum < std::numeric_limits<Scalar>::epsilon()) {
    throw std::runtime_error("Particle filter degeneracy");
  }

  weights.array() /= weights_sum;
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::compute_state_sum_sum_weights_(
  const RowMajorVector & weights)
{
  std::partial_sum(
    weights.data(), weights.data() + number_of_particles_, state_cum_sum_weights_.data());

  state_cum_sum_weights_ /= state_cum_sum_weights_(number_of_particles_ - 1);
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::compute_statified_random_cum_sum_weights_()
{
  for (size_t n = 0; n < number_of_particles_; ++n) {
    random_cum_sum_weights_(n) = (n + uniform_distribution(rng_)) / number_of_particles_;
  }
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::compute_multinomial_random_cum_sum_weights_()
{
  Scalar cum_sum_weight = 0;
  for (size_t n = 0; n < number_of_particles_; ++n) {
    cum_sum_weight -= std::log2(uniform_distribution(rng_));
    random_cum_sum_weights_(n) = cum_sum_weight;
  }
  random_cum_sum_weights_ /= cum_sum_weight;
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::compute_systematic_random_cum_sum_weights_()
{
  Scalar us = uniform_distribution(rng_);
  for (size_t n = 0; n < number_of_particles_; ++n) {
    random_cum_sum_weights_(n) = (n + us) / number_of_particles_;
  }
}

//-----------------------------------------------------------------------------
template<typename Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::compute_random_cum_sum_weights_(
  const ParticleFilterResamplingScheme & resampling_scheme)
{
  //  //draw random weights
  //  random_cum_sum_weights_.setRandom();
  //  random_cum_sum_weights_.array()+=1;

  //  //Compute random cumulative sum wieghts
  //  std::partial_sum(random_cum_sum_weights_.data(),
  //                   random_cum_sum_weights_.data()+number_of_particles_,
  //                   random_cum_sum_weights_.data());

  //  random_cum_sum_weights_/=random_cum_sum_weights_(number_of_particles_-1);

  switch (resampling_scheme) {
    case ParticleFilterResamplingScheme::MUTINOMIAL:
      compute_multinomial_random_cum_sum_weights_();
      break;
    case ParticleFilterResamplingScheme::STRATIFIED:
      compute_statified_random_cum_sum_weights_();
      break;
    case ParticleFilterResamplingScheme::SYSTEMATIC:
      compute_systematic_random_cum_sum_weights_();
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<class Scalar, size_t DIM>
void ParticleFilterResampling<Scalar, DIM>::resampling_(ParticleFilterState<Scalar, DIM> & state)
{
  size_t j = 0;
  for (size_t i = 0; i < number_of_particles_; ++i) {
    do {
      if (random_cum_sum_weights_(i) < state_cum_sum_weights_(j)) {
        // std::cout << i <<" "<<resampled_particles_.cols()<<" "
        //           <<j <<" "<<state.particles.cols()<< std::endl;
        resampled_particles_.col(i) = state.particles.col(j);
        break;
      }
      ++j;
    } while (j < number_of_particles_);

    if (j == number_of_particles_) {
      resampled_particles_.col(i) = resampled_particles_.col(0);
    }
  }

  // swap particles and reset weights
  std::swap(state.particles, resampled_particles_);
  state.weights.setConstant(1. / number_of_particles_);
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTER__PARTICLE__UPDATER__RESAMPLING_HPP_
