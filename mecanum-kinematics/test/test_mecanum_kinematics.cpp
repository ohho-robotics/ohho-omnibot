#include <cmath>

#include <gtest/gtest.h>

#include "mecanum_drive_ros2/mecanum_kinematics.hpp"

namespace {

// Matches config/mecanum_params.yaml and tests/test_kinematics.py.
constexpr mecanum_drive::RobotGeometry kGeometry{0.04, 0.215, 0.165};
constexpr double kTolerance = 1e-9;
const double kPi = std::acos(-1.0);

void expectWheels(
  const mecanum_drive::WheelVelocities & actual,
  const mecanum_drive::WheelVelocities & expected)
{
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], kTolerance) << "wheel " << i;
  }
}

void expectTwist(const mecanum_drive::BodyTwist & actual, double vx, double vy, double omega)
{
  EXPECT_NEAR(actual.vx, vx, kTolerance);
  EXPECT_NEAR(actual.vy, vy, kTolerance);
  EXPECT_NEAR(actual.omega, omega, kTolerance);
}

TEST(InverseKinematics, PureForwardMotion)
{
  expectWheels(
    mecanum_drive::inverseKinematics(0.5, 0.0, 0.0, kGeometry),
    {12.5, 12.5, 12.5, 12.5});
}

TEST(InverseKinematics, PureLateralMotion)
{
  expectWheels(
    mecanum_drive::inverseKinematics(0.0, 0.3, 0.0, kGeometry),
    {-7.5, 7.5, 7.5, -7.5});
}

TEST(InverseKinematics, PureRotation)
{
  // Half the wheelbase plus half the track is 0.19 m; 0.19 / 0.04 = 4.75.
  expectWheels(
    mecanum_drive::inverseKinematics(0.0, 0.0, 1.0, kGeometry),
    {-4.75, 4.75, -4.75, 4.75});
}

TEST(InverseKinematics, ZeroVelocity)
{
  expectWheels(
    mecanum_drive::inverseKinematics(0.0, 0.0, 0.0, kGeometry),
    {0.0, 0.0, 0.0, 0.0});
}

TEST(InverseKinematics, ForwardSpeedProportionalToVx)
{
  const auto slow = mecanum_drive::inverseKinematics(0.2, 0.0, 0.0, kGeometry);
  const auto fast = mecanum_drive::inverseKinematics(0.4, 0.0, 0.0, kGeometry);
  for (std::size_t i = 0; i < slow.size(); ++i) {
    EXPECT_NEAR(fast[i], 2.0 * slow[i], kTolerance) << "wheel " << i;
  }
}

TEST(ForwardKinematics, PureForwardMotion)
{
  expectTwist(
    mecanum_drive::forwardKinematics({12.5, 12.5, 12.5, 12.5}, kGeometry),
    0.5, 0.0, 0.0);
}

TEST(ForwardKinematics, PureLateralMotion)
{
  expectTwist(
    mecanum_drive::forwardKinematics({-7.5, 7.5, 7.5, -7.5}, kGeometry),
    0.0, 0.3, 0.0);
}

TEST(ForwardKinematics, PureRotation)
{
  expectTwist(
    mecanum_drive::forwardKinematics({-4.75, 4.75, -4.75, 4.75}, kGeometry),
    0.0, 0.0, 1.0);
}

TEST(ForwardKinematics, ZeroWheels)
{
  expectTwist(
    mecanum_drive::forwardKinematics({0.0, 0.0, 0.0, 0.0}, kGeometry),
    0.0, 0.0, 0.0);
}

TEST(ForwardKinematics, CombinedInverseForwardRoundtrip)
{
  const auto wheels = mecanum_drive::inverseKinematics(0.3, -0.1, 0.5, kGeometry);
  expectTwist(mecanum_drive::forwardKinematics(wheels, kGeometry), 0.3, -0.1, 0.5);
}

TEST(IntegratePose, StationaryNoChange)
{
  double x = 1.0, y = 2.0, theta = 0.5;
  mecanum_drive::integratePose(x, y, theta, 0.0, 0.0, 0.0, 0.1);
  EXPECT_NEAR(x, 1.0, kTolerance);
  EXPECT_NEAR(y, 2.0, kTolerance);
  EXPECT_NEAR(theta, 0.5, kTolerance);
}

TEST(IntegratePose, ForwardAlongXAxis)
{
  double x = 0.0, y = 0.0, theta = 0.0;
  mecanum_drive::integratePose(x, y, theta, 1.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(x, 1.0, kTolerance);
  EXPECT_NEAR(y, 0.0, kTolerance);
  EXPECT_NEAR(theta, 0.0, kTolerance);
}

TEST(IntegratePose, LateralMotion)
{
  double x = 0.0, y = 0.0, theta = 0.0;
  mecanum_drive::integratePose(x, y, theta, 0.0, 1.0, 0.0, 1.0);
  EXPECT_NEAR(x, 0.0, kTolerance);
  EXPECT_NEAR(y, 1.0, kTolerance);
  EXPECT_NEAR(theta, 0.0, kTolerance);
}

TEST(IntegratePose, PositiveRotationWrapsToPi)
{
  double x = 0.0, y = 0.0, theta = 3.1;
  mecanum_drive::integratePose(x, y, theta, 0.0, 0.0, 1.0, 1.0);
  EXPECT_GE(theta, -kPi);
  EXPECT_LE(theta, kPi);
  EXPECT_NEAR(theta, 4.1 - 2.0 * kPi, kTolerance);
}

TEST(IntegratePose, NegativeRotationWrapsToPi)
{
  double x = 0.0, y = 0.0, theta = -3.1;
  mecanum_drive::integratePose(x, y, theta, 0.0, 0.0, -1.0, 1.0);
  EXPECT_GE(theta, -kPi);
  EXPECT_LE(theta, kPi);
  EXPECT_NEAR(theta, -4.1 + 2.0 * kPi, kTolerance);
}

TEST(IntegratePose, RotationQuarterTurn)
{
  double x = 0.0, y = 0.0, theta = 0.0;
  mecanum_drive::integratePose(x, y, theta, 0.0, 0.0, kPi / 2.0, 1.0);
  EXPECT_NEAR(theta, kPi / 2.0, kTolerance);
}

TEST(IntegratePose, HeadingAffectsTranslation)
{
  double x = 0.0, y = 0.0, theta = kPi / 2.0;
  mecanum_drive::integratePose(x, y, theta, 1.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(x, 0.0, kTolerance);
  EXPECT_NEAR(y, 1.0, kTolerance);
  EXPECT_NEAR(theta, kPi / 2.0, kTolerance);
}

TEST(IntegratePose, ZeroTimeStep)
{
  double x = 1.0, y = 2.0, theta = 0.5;
  mecanum_drive::integratePose(x, y, theta, 0.3, -0.1, 0.5, 0.0);
  EXPECT_NEAR(x, 1.0, kTolerance);
  EXPECT_NEAR(y, 2.0, kTolerance);
  EXPECT_NEAR(theta, 0.5, kTolerance);
}

}  // namespace
