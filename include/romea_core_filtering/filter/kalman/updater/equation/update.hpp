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

#ifndef ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__EQUATION__UPDATE_HPP_
#define ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__EQUATION__UPDATE_HPP_

// romea
#include "romea_core_filtering/filter/kalman/updater/traits.hpp"
#include "romea_core_filtering/gaussian/state.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdateStateVector
{
  using Traits = KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>;
  using X = typename Traits::X;
  using Inn = typename Traits::Inn;
  using K = typename Traits::K;

  static void compute(
    X & state,
    const Inn & innovation,
    const K & gain)
  {
    state += gain * innovation;
  }
};

//-----------------------------------------------------------------------------
template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdateStateCovariance
{
  using Traits = KFUpdaterTraits<Scalar, StateDIM, ObservationDIM>;
  using P = typename Traits::P;
  using QInn = typename Traits::QInn;
  using H = typename Traits::H;
  using K = typename Traits::K;

  static void compute(
    P & covariance,
    const QInn & innovation_covariance,
    const K & gain)
  {
    covariance -= gain * innovation_covariance * gain.transpose();
    covariance = (covariance + covariance.transpose()) / 2.;
  }

  static void compute_joseph(
    P & covariance,
    const QInn & observation_covariance,
    const H & observation_matrix,
    const K & gain)
  {
    const auto I = P::Identity();
    const auto IKH = I - gain * observation_matrix;
    covariance = IKH * covariance * IKH.transpose() +
      gain * observation_covariance * gain.transpose();
    covariance = (covariance + covariance.transpose()) / 2.;
  }
};

//-----------------------------------------------------------------------------
template<typename Scalar>
struct KFUpdateStateCovariance<Scalar, 1, 1>
{
  static void compute(
    Scalar & covariance,
    const Scalar & innovation_covariance,
    const Scalar & gain)
  {
    covariance -= gain * innovation_covariance * gain;
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__FILTER__KALMAN__UPDATER__EQUATION__UPDATE_HPP_
