// Copyright 2025 Luis Batista
//
// Licensed under the BSD License; see the package.xml for details.

#include <gtest/gtest.h>

#include <cmath>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "WindModel.hh"

using gz::math::Quaterniond;
using gz::math::Vector3d;
using kingfisher_gazebo::AngleOfAttack;
using kingfisher_gazebo::LowPass;
using kingfisher_gazebo::SailForce;
using kingfisher_gazebo::SailTable;

namespace
{
constexpr double kRho = 1.225;
constexpr double kArea = 0.14;

// Rows of the NACA 0018, Re 80k table in kingfisher_gazebo_plugins.xacro
SailTable Naca0018()
{
  return SailTable(
    {0, 8, 9, 12, 14, 30, 45, 90, 135, 180},
    {0, 0.6589, 0.6606, 0.4408, 0.2256, 0.855, 1.05, 0.09, -0.93, 0},
    {0.0162, 0.0242, 0.0264, 0.08, 0.158, 0.57, 1.075, 1.8, 1.085, 0.025});
}

double Rad(double _deg)
{
  return _deg * GZ_PI / 180.0;
}

// Apparent wind of speed _s coming from _fromDeg, counter-clockwise from +x
Vector3d WindFrom(double _fromDeg, double _s)
{
  return {-_s * std::cos(Rad(_fromDeg)), -_s * std::sin(Rad(_fromDeg)), 0};
}

// Force in base_link of the wind _wBase with the sail turned by _sailDeg
Vector3d BaseForce(const Vector3d &_wBase, double _sailDeg,
                   const SailTable &_table)
{
  const Quaterniond sail(0, 0, Rad(_sailDeg));
  const Vector3d f = SailForce(sail.RotateVectorReverse(_wBase), _table,
                               kArea, kRho);
  return sail.RotateVector(f);
}
}  // namespace

TEST(SailTable, InterpolatesAtAndBetweenNodes)
{
  const auto t = Naca0018();
  EXPECT_NEAR(t.Cl(Rad(9)), 0.6606, 1e-9);
  EXPECT_NEAR(t.Cd(Rad(45)), 1.075, 1e-9);
  EXPECT_NEAR(t.Cl(Rad(13)), (0.4408 + 0.2256) / 2, 1e-9);
  EXPECT_NEAR(t.Cd(Rad(112.5)), (1.8 + 1.085) / 2, 1e-9);
  EXPECT_NEAR(t.Cl(Rad(180)), 0.0, 1e-9);
}

TEST(SailTable, MirrorsNegativeAngles)
{
  const auto t = Naca0018();
  EXPECT_NEAR(t.Cl(Rad(-45)), -1.05, 1e-9);
  EXPECT_NEAR(t.Cd(Rad(-45)), 1.075, 1e-9);
  EXPECT_NEAR(t.Cl(Rad(-135)), 0.93, 1e-9);
}

TEST(SailTable, RejectsBadTables)
{
  EXPECT_THROW(SailTable({0, 90}, {0}, {0, 1}), std::invalid_argument);
  EXPECT_THROW(SailTable({0, 90}, {0, 1}, {0, 1}), std::invalid_argument);
  EXPECT_THROW(SailTable({0, 120, 90, 180}, {0, 0, 0, 0}, {0, 0, 0, 0}),
               std::invalid_argument);
}

TEST(SailForce, AngleOfAttack)
{
  EXPECT_NEAR(AngleOfAttack(WindFrom(0, 3)), 0.0, 1e-12);
  EXPECT_NEAR(AngleOfAttack(WindFrom(30, 3)), Rad(30), 1e-12);
  EXPECT_NEAR(AngleOfAttack(WindFrom(-30, 3)), Rad(-30), 1e-12);
}

TEST(SailForce, NoseIntoTheWindIsPureDragAft)
{
  const auto t = Naca0018();
  const Vector3d f = SailForce(WindFrom(0, 5), t, kArea, kRho);
  const double qa = 0.5 * kRho * 25 * kArea;
  EXPECT_NEAR(f.X(), -qa * 0.0162, 1e-9);
  EXPECT_NEAR(f.Y(), 0.0, 1e-12);
  EXPECT_EQ(f.Z(), 0.0);
}

TEST(SailForce, CalmIsZero)
{
  EXPECT_EQ(SailForce(Vector3d::Zero, Naca0018(), kArea, kRho),
            Vector3d::Zero);
}

TEST(SailForce, StarboardBowLiftsToPortAndForward)
{
  const auto t = Naca0018();
  // C_L / C_D = 27 > cot 8 deg: the net force has a forward component
  Vector3d f = SailForce(WindFrom(-8, 5), t, kArea, kRho);
  EXPECT_GT(f.Y(), 0.0);
  EXPECT_GT(f.X(), 0.0);
  // C_L / C_D = 1.5 < cot 30 deg: lift still to port, but drag wins
  f = SailForce(WindFrom(-30, 5), t, kArea, kRho);
  EXPECT_GT(f.Y(), 0.0);
  EXPECT_LT(f.X(), 0.0);
}

TEST(SailForce, BeamWindFromPort)
{
  const auto t = Naca0018();
  const double s = 5.0;
  const double qa = 0.5 * kRho * s * s * kArea;
  const Vector3d fromPort(0, -s, 0);

  // Sail at +45 deg: lift is forward, drag is leeway to starboard
  Vector3d f = BaseForce(fromPort, 45, t);
  EXPECT_NEAR(f.X(), qa * 1.05, 1e-9);
  EXPECT_NEAR(f.Y(), -qa * 1.075, 1e-9);

  // Sail at -45 deg (alpha 135): pushed backward
  f = BaseForce(fromPort, -45, t);
  EXPECT_NEAR(f.X(), qa * -0.93, 1e-9);
  EXPECT_NEAR(f.Y(), -qa * 1.085, 1e-9);

  // Sail at 0 (alpha 90): mostly drag to starboard
  f = SailForce(fromPort, t, kArea, kRho);
  EXPECT_NEAR(f.X(), qa * 0.09, 1e-9);
  EXPECT_NEAR(f.Y(), -qa * 1.8, 1e-9);
}

TEST(SailForce, MirrorSymmetry)
{
  const auto t = Naca0018();
  for (double from : {5.0, 20.0, 60.0, 100.0, 150.0, 175.0})
  {
    const Vector3d w = WindFrom(from, 4);
    const Vector3d mirrored(w.X(), -w.Y(), 0);
    const Vector3d f = SailForce(w, t, kArea, kRho);
    const Vector3d g = SailForce(mirrored, t, kArea, kRho);
    EXPECT_NEAR(g.X(), f.X(), 1e-9) << from;
    EXPECT_NEAR(g.Y(), -f.Y(), 1e-9) << from;
  }
}

TEST(SailForce, IgnoresVerticalWind)
{
  const auto t = Naca0018();
  EXPECT_EQ(SailForce(Vector3d(-3, 1, 2), t, kArea, kRho),
            SailForce(Vector3d(-3, 1, 0), t, kArea, kRho));
}

TEST(LowPass, StepResponseReaches63PercentAtTau)
{
  const double tau = 1.0;
  const double dt = 0.001;
  LowPass filter(tau);
  filter.Update(Vector3d::Zero, dt);
  Vector3d y;
  for (int i = 0; i < 1000; ++i)
    y = filter.Update(Vector3d(1, -2, 0), dt);
  EXPECT_NEAR(y.X(), 1.0 - std::exp(-1.0), 1e-3);
  EXPECT_NEAR(y.Y(), -2.0 * (1.0 - std::exp(-1.0)), 2e-3);
}

TEST(LowPass, StartsFromTheFirstSample)
{
  LowPass filter(1.0);
  EXPECT_EQ(filter.Update(Vector3d(-1.5, 0, 0), 0.001), Vector3d(-1.5, 0, 0));
}

TEST(LowPass, PassThroughWithoutTimeConstant)
{
  LowPass filter(0.0);
  filter.Update(Vector3d(1, 1, 0), 0.001);
  EXPECT_EQ(filter.Update(Vector3d(3, -4, 0), 0.001), Vector3d(3, -4, 0));
}
