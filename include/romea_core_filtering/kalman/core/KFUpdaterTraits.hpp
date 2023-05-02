// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


#ifndef ROMEA_CORE_FILTERING__KALMAN__CORE__KFUPDATERTRAITS_HPP_
#define ROMEA_CORE_FILTERING__KALMAN__CORE__KFUPDATERTRAITS_HPP_

#include <Eigen/Core>
#include <type_traits>

namespace romea
{


template<typename T, typename Enable = void>
struct Zero
{
  static T zero() {return 0;}
};

template<typename T>
struct Zero<T, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<T>, T>::value>::type>
{
  static T zero() {return T::Zero();}
};


template<typename Scalar, size_t StateDIM, size_t ObservationDIM>
struct KFUpdaterTraits
{
  using X = Eigen::Matrix<Scalar, StateDIM, 1>;
  using P = Eigen::Matrix<Scalar, StateDIM, StateDIM>;
  using H = Eigen::Matrix<Scalar, ObservationDIM, StateDIM>;
  using K = Eigen::Matrix<Scalar, StateDIM, ObservationDIM>;
  using Inn = Eigen::Matrix<Scalar, ObservationDIM, 1>;
  using QInn = Eigen::Matrix<Scalar, ObservationDIM, ObservationDIM>;
};

template<typename Scalar, size_t StateDIM>
struct KFUpdaterTraits<Scalar, StateDIM, 1>
{
  using X = Eigen::Matrix<Scalar, StateDIM, 1>;
  using P = Eigen::Matrix<Scalar, StateDIM, StateDIM>;
  using H = Eigen::Matrix<Scalar, 1, StateDIM>;
  using K = Eigen::Matrix<Scalar, StateDIM, 1>;
  using Inn = Scalar;
  using QInn = Scalar;
};

template<typename Scalar>
struct KFUpdaterTraits<Scalar, 1, 1>
{
  using X = Scalar;
  using P = Scalar;
  using H = Scalar;
  using K = Scalar;
  using Inn = Scalar;
  using QInn = Scalar;
};

}  // namespace romea

#endif  // ROMEA_CORE_FILTERING__KALMAN__CORE__KFUPDATERTRAITS_HPP_
