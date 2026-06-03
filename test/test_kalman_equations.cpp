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

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include "romea_core_filtering/filter/kalman/updater/equation/gain.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/innovation.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/mahalanobis.hpp"
#include "romea_core_filtering/filter/kalman/updater/equation/update.hpp"

TEST(TestKalmanEquations, computesScalarInnovationCovarianceGainAndUpdate) {
  double P = 4.;
  double H = 0.5;
  double R = 1.;
  double QInn = 0.;

  romea::core::compute_innovation_covariance<double>(P, H, R, QInn);

  EXPECT_DOUBLE_EQ(QInn, 2.);

  double QInnInverse = 0.;
  const double mahalanobis =
      romea::core::KFMahalanobis<double, 1>::compute(2., QInn, QInnInverse);

  EXPECT_DOUBLE_EQ(QInnInverse, 0.5);
  EXPECT_DOUBLE_EQ(mahalanobis, std::sqrt(2.));

  double K = 0.;
  romea::core::KFGain<double, 1, 1>::compute(P, H, QInnInverse, K);

  EXPECT_DOUBLE_EQ(K, 1.);

  double X = 10.;
  romea::core::KFUpdateStateVector<double, 1, 1>::compute(X, 2., K);
  romea::core::KFUpdateStateCovariance<double, 1, 1>::compute(P, QInn, K);

  EXPECT_DOUBLE_EQ(X, 12.);
  EXPECT_DOUBLE_EQ(P, 2.);
}

TEST(TestKalmanEquations, computesMatrixInnovationCovarianceGainAndUpdate) {
  Eigen::Matrix2d P;
  P << 2., 0.5, 0.5, 1.;

  Eigen::Matrix2d H = Eigen::Matrix2d::Identity();

  Eigen::Matrix2d R;
  R << 0.25, 0., 0., 0.5;

  Eigen::Matrix2d QInn;
  romea::core::compute_innovation_covariance<double, 2, 2>(P, H, R, QInn);

  Eigen::Matrix2d expected_QInn = P + R;

  EXPECT_TRUE(QInn.isApprox(expected_QInn, 1e-12));

  Eigen::Matrix2d QInnInverse;
  Eigen::Vector2d Inn;
  Inn << 1.5, -0.5;

  Eigen::Matrix2d expected_QInnInverse = expected_QInn.inverse();
  const double mahalanobis =
      romea::core::KFMahalanobis<double, 2>::compute(Inn, QInn, QInnInverse);

  EXPECT_TRUE(QInnInverse.isApprox(expected_QInnInverse, 1e-12));
  EXPECT_NEAR(mahalanobis,
              std::sqrt((Inn.transpose() * expected_QInnInverse * Inn)(0, 0)),
              1e-12);

  Eigen::Matrix2d K;
  Eigen::Matrix2d expected_K = P * H.transpose() * expected_QInnInverse;
  romea::core::KFGain<double, 2, 2>::compute(P, H, QInnInverse, K);

  EXPECT_TRUE(K.isApprox(expected_K, 1e-12));

  Eigen::Vector2d X;
  X << 10., 20.;
  Eigen::Vector2d expected_X = X + expected_K * Inn;
  Eigen::Matrix2d expected_P = P - expected_K * expected_QInn * expected_K.transpose();

  romea::core::KFUpdateStateVector<double, 2, 2>::compute(X, Inn, K);
  romea::core::KFUpdateStateCovariance<double, 2, 2>::compute(P, QInn, K);

  EXPECT_TRUE(X.isApprox(expected_X, 1e-12));
  EXPECT_TRUE(P.isApprox(expected_P, 1e-12));
}

TEST(TestKalmanEquations, throwsWhenInnovationCovarianceIsSingular) {
  Eigen::Matrix<double, 2, 1> Inn;
  Inn << 1., 2.;

  Eigen::Matrix2d QInn = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d QInnInverse;

  EXPECT_THROW((romea::core::KFMahalanobis<double, 2>::compute(
                   Inn, QInn, QInnInverse)),
               std::runtime_error);
}
