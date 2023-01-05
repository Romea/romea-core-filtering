// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// std
#include <vector>

// romea
#include "romea_core_filtering/kalman/UnscentedKalmanFilterUpdaterCore.hpp"

TEST(testUKF, testUnscentedTransform)
{
  using StateSigmaPoints = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
  using ObservationSigmaPoints = std::vector<double>;

  romea::GaussianState<double, 3> state;
  state.X() << 10.0401196, -6.541962718, 5.773763097;
  state.P().row(0) << 0.003713927914, 0.004473035868, 0.003144414499;
  state.P().row(1) << 0.004473035868, 0.009133254681, 0.005490835273;
  state.P().row(2) << 0.003144414499, 0.005490835273, 0.005852381092;

  const double UNSCENTED_TRANSFORM_KAPPA = 3;
  const double UNSCENTED_TRANSFORM_ALPHA = 0.75;
  const double UNSCENTED_TRANSFORM_BETA = 2;

  romea::UnscentedTransformParameters<double> params(3,
    UNSCENTED_TRANSFORM_KAPPA,
    UNSCENTED_TRANSFORM_ALPHA,
    UNSCENTED_TRANSFORM_BETA);

  StateSigmaPoints stateSigmaPoints(7);
  romea::UnscentedTransformFoward<double, 3>::toSigmaPoints(params, state, stateSigmaPoints);

  EXPECT_NEAR(stateSigmaPoints[0][0], 10.04011967, 0.01);
  EXPECT_NEAR(stateSigmaPoints[0][1], -6.541962718, 0.01);
  EXPECT_NEAR(stateSigmaPoints[0][2], 5.773763097, 0.01);
  EXPECT_NEAR(stateSigmaPoints[1][0], 10.13285102, 0.01);
  EXPECT_NEAR(stateSigmaPoints[1][1], -6.489773659, 0.01);
  EXPECT_NEAR(stateSigmaPoints[1][2], 5.808572647, 0.01);
  EXPECT_NEAR(stateSigmaPoints[2][0], 10.09230873, 0.01);
  EXPECT_NEAR(stateSigmaPoints[2][1], -6.385416757, 0.01);
  EXPECT_NEAR(stateSigmaPoints[2][2], 5.833716403, 0.01);
  EXPECT_NEAR(stateSigmaPoints[3][0], 10.07492922, 0.01);
  EXPECT_NEAR(stateSigmaPoints[3][1], -6.482009413, 0.01);
  EXPECT_NEAR(stateSigmaPoints[3][2], 5.896015634, 0.01);
  EXPECT_NEAR(stateSigmaPoints[4][0], 9.947388315, 0.01);
  EXPECT_NEAR(stateSigmaPoints[4][1], -6.594151777, 0.01);
  EXPECT_NEAR(stateSigmaPoints[4][2], 5.738953548, 0.01);
  EXPECT_NEAR(stateSigmaPoints[5][0], 9.98793061, 0.01);
  EXPECT_NEAR(stateSigmaPoints[5][1], -6.698508679, 0.01);
  EXPECT_NEAR(stateSigmaPoints[5][2], 5.713809792, 0.01);
  EXPECT_NEAR(stateSigmaPoints[6][0], 10.00531012, 0.01);
  EXPECT_NEAR(stateSigmaPoints[6][1], -6.601916023, 0.01);
  EXPECT_NEAR(stateSigmaPoints[6][2], 5.651510561, 0.01);

  ObservationSigmaPoints propagatedSigmaPoints(7);
  propagatedSigmaPoints[0] = 11.07597392;
  propagatedSigmaPoints[1] = 11.1579882;
  propagatedSigmaPoints[2] = 11.10114414;
  propagatedSigmaPoints[3] = 11.09709313;
  propagatedSigmaPoints[4] = 10.99435493;
  propagatedSigmaPoints[5] = 11.05318411;
  propagatedSigmaPoints[6] = 11.05509837;

  romea::GaussianObservation<double, 1> propagatedState;
  romea::UnscentedTransformInverse<double, 1>::
  toGaussian(params, propagatedSigmaPoints, propagatedState);

  EXPECT_NEAR(propagatedState.Y(), 11.07642123, 0.01);
  EXPECT_NEAR(propagatedState.R(), 0.002285136065, 0.01);

  Eigen::Matrix<double, 3, 1> propagationCorrelation(3, 1);
  romea::UKFCorrelation<double, 3, 1>::compute(
    params,
    state,
    propagatedState,
    stateSigmaPoints,
    propagatedSigmaPoints,
    propagationCorrelation);

  EXPECT_NEAR(propagationCorrelation(0, 0), 0.002835369261, 0.01);
  EXPECT_NEAR(propagationCorrelation(1, 0), 0.0027504505, 0.01);
  EXPECT_NEAR(propagationCorrelation(2, 0), 0.002030419041, 0.01);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
