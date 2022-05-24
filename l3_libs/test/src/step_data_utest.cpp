#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_libs/test_macros.h>
#include <l3_libs/types/step_data.h>
#include <l3_libs/types/floating_base.h>

using namespace l3;

// Test for checking functionality of l3::FloatingBase
TEST(StepData, FloatingBase_Equality)
{
  EXPECT_TRUE(FloatingBase() == FloatingBase());
  EXPECT_TRUE(FloatingBase(0.1, 0.2, 0.3, 0.4, 0.5, 0.6) == FloatingBase(0.1, 0.2, 0.3, 0.4, 0.5, 0.6));
  EXPECT_FALSE(FloatingBase(1.0, 0.0, 0.0) == FloatingBase());
  EXPECT_FALSE(FloatingBase(0.0, 1.0, 0.0) == FloatingBase());
  EXPECT_FALSE(FloatingBase(0.0, 0.0, 1.0) == FloatingBase());
  EXPECT_FALSE(FloatingBase(0.0, 0.0, 0.0, 1.0, 0.0, 0.0) == FloatingBase());
  EXPECT_FALSE(FloatingBase(0.0, 0.0, 0.0, 0.0, 1.0, 0.0) == FloatingBase());
  EXPECT_FALSE(FloatingBase(0.0, 0.0, 0.0, 0.0, 0.0, 1.0) == FloatingBase());
}

// Test for checking functionality of l3::FloatingBase
TEST(StepData, FloatingBase)
{
  FloatingBase floating_base;

  // check clean initialization
  EXPECT_DOUBLE_EQ(0.0, floating_base.x());
  EXPECT_DOUBLE_EQ(0.0, floating_base.y());
  EXPECT_DOUBLE_EQ(0.0, floating_base.z());
  EXPECT_ANGLE_EQ(0.0, floating_base.roll());
  EXPECT_ANGLE_EQ(0.0, floating_base.pitch());
  EXPECT_ANGLE_EQ(0.0, floating_base.yaw());

  // check setters and getters
  floating_base.setX(1.0 * M_PI);
  floating_base.setY(2.0 * M_PI);
  floating_base.setZ(3.0 * M_PI);
  EXPECT_DOUBLE_EQ(1.0 * M_PI, floating_base.x());
  EXPECT_DOUBLE_EQ(2.0 * M_PI, floating_base.y());
  EXPECT_DOUBLE_EQ(3.0 * M_PI, floating_base.z());

  floating_base.setRoll(0.1 * M_PI);
  floating_base.setPitch(0.2 * M_PI);
  floating_base.setYaw(0.3 * M_PI);
  EXPECT_ANGLE_EQ(0.1 * M_PI, floating_base.roll());
  EXPECT_ANGLE_EQ(0.2 * M_PI, floating_base.pitch());
  EXPECT_ANGLE_EQ(0.3 * M_PI, floating_base.yaw());

  floating_base.setXYZ(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(1.0, floating_base.x());
  EXPECT_DOUBLE_EQ(2.0, floating_base.y());
  EXPECT_DOUBLE_EQ(3.0, floating_base.z());

  floating_base.setRPY(0.2 * M_PI, 0.3 * M_PI, 0.1 * M_PI);
  double roll, pitch, yaw;
  floating_base.getRPY(roll, pitch, yaw);
  EXPECT_ANGLE_EQ(0.2 * M_PI, roll);
  EXPECT_ANGLE_EQ(0.3 * M_PI, pitch);
  EXPECT_ANGLE_EQ(0.1 * M_PI, yaw);

  // projection test
  EXPECT_POSE_EQ(Pose(2.0, 3.0, 0.0, 0.0, 0.0, 0.3 * M_PI), FloatingBase(2.0, 3.0, 1.0, 3.1, -1.3, 0.3 * M_PI).project2D());

  // basic translation tests
  EXPECT_POSE_EQ(Pose(), FloatingBase::getDelta(FloatingBase(), FloatingBase()));
  EXPECT_POSE_EQ(Pose(1.0, 2.0, 3.0), FloatingBase::getDelta(FloatingBase(), FloatingBase(1.0, 2.0, 3.0)));
  EXPECT_POSE_EQ(Pose(-1.0, -2.0, -3.0), FloatingBase::getDelta(FloatingBase(1.0, 2.0, 3.0), FloatingBase()));
  EXPECT_POSE_EQ(Pose(4.0, -6.0, -1.0), FloatingBase::getDelta(FloatingBase(1.0, 2.0, 3.0), FloatingBase(5.0, -4.0, 2.0)));

  // basic rotation tests
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.1, 0.2, -0.3), FloatingBase::getDelta(FloatingBase(), FloatingBase(0.0, 0.0, 0.0, 0.1, 0.2, -0.3)));
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.3), FloatingBase::getDelta(FloatingBase(0.0, 0.0, 0.0, 0.0, 0.0, -0.3), FloatingBase()));

  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, -0.23), FloatingBase::getDelta2D(FloatingBase(), FloatingBase(0.0, 0.0, 0.0, 0.1, 0.34, -0.23)));
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.23), FloatingBase::getDelta2D(FloatingBase(0.0, 0.0, 0.0, 0.23, 0.6, -0.23), FloatingBase()));

  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, -0.1), FloatingBase::getDelta2D(FloatingBase(0.0, 0.0, 0.0, 0.1, 0.2, -0.3), FloatingBase(0.0, 0.0, 0.0, 0.2, 0.3, -0.4)));

  // complex tests
  double yaw_1 = 0.3;
  EXPECT_POSE_EQ(Pose(0.3 * cos(-yaw_1) - 0.05 * sin(-yaw_1), 0.3 * sin(-yaw_1) + 0.05 * cos(-yaw_1), 0.0, 0.0, 0.0, 0.35),
                 FloatingBase::getDelta2D(FloatingBase(1.0, 2.30, 1.2, 0.30, 0.12, yaw_1), FloatingBase(1.3, 2.35, 1.3, 0.32, -0.14, 0.65)));
}

// Test for checking functionality of l3::Foothold
TEST(StepData, Foothold_Equality)
{
  EXPECT_TRUE(Foothold() == Foothold());
  EXPECT_TRUE(Foothold(0.1, 0.2, 0.3, 0.4, 0.5, 0.6) == Foothold(0.1, 0.2, 0.3, 0.4, 0.5, 0.6));
  EXPECT_FALSE(Foothold(1.0, 0.0, 0.0) == Foothold());
  EXPECT_FALSE(Foothold(0.0, 1.0, 0.0) == Foothold());
  EXPECT_FALSE(Foothold(0.0, 0.0, 1.0) == Foothold());
  EXPECT_FALSE(Foothold(0.0, 0.0, 0.0, 1.0, 0.0, 0.0) == Foothold());
  EXPECT_FALSE(Foothold(0.0, 0.0, 0.0, 0.0, 1.0, 0.0) == Foothold());
  EXPECT_FALSE(Foothold(0.0, 0.0, 0.0, 0.0, 0.0, 1.0) == Foothold());
}

// Test for checking functionality of l3::Foothold
TEST(StepData, Foothold)
{
  Foothold foothold;

  // check clean initialization
  EXPECT_DOUBLE_EQ(0.0, foothold.x());
  EXPECT_DOUBLE_EQ(0.0, foothold.y());
  EXPECT_DOUBLE_EQ(0.0, foothold.z());
  EXPECT_ANGLE_EQ(0.0, foothold.roll());
  EXPECT_ANGLE_EQ(0.0, foothold.pitch());
  EXPECT_ANGLE_EQ(0.0, foothold.yaw());

  // check setters and getters
  foothold.setX(1.0 * M_PI);
  foothold.setY(2.0 * M_PI);
  foothold.setZ(3.0 * M_PI);
  EXPECT_DOUBLE_EQ(1.0 * M_PI, foothold.x());
  EXPECT_DOUBLE_EQ(2.0 * M_PI, foothold.y());
  EXPECT_DOUBLE_EQ(3.0 * M_PI, foothold.z());

  foothold.setRoll(0.1 * M_PI);
  foothold.setPitch(0.2 * M_PI);
  foothold.setYaw(0.3 * M_PI);
  EXPECT_ANGLE_EQ(0.1 * M_PI, foothold.roll());
  EXPECT_ANGLE_EQ(0.2 * M_PI, foothold.pitch());
  EXPECT_ANGLE_EQ(0.3 * M_PI, foothold.yaw());

  foothold.setXYZ(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(1.0, foothold.x());
  EXPECT_DOUBLE_EQ(2.0, foothold.y());
  EXPECT_DOUBLE_EQ(3.0, foothold.z());

  foothold.setRPY(0.2 * M_PI, 0.3 * M_PI, 0.1 * M_PI);
  double roll, pitch, yaw;
  foothold.getRPY(roll, pitch, yaw);
  EXPECT_ANGLE_EQ(0.2 * M_PI, roll);
  EXPECT_ANGLE_EQ(0.3 * M_PI, pitch);
  EXPECT_ANGLE_EQ(0.1 * M_PI, yaw);

  // projection test
  EXPECT_POSE_EQ(Pose(2.0, 3.0, 0.0, 0.0, 0.0, 0.3 * M_PI), Foothold(2.0, 3.0, 1.0, 3.1, -1.3, 0.3 * M_PI).project2D());

  // basic translation tests
  EXPECT_POSE_EQ(Pose(), Foothold::getDelta(Foothold(), Foothold()));
  EXPECT_POSE_EQ(Pose(1.0, 2.0, 3.0), Foothold::getDelta(Foothold(), Foothold(1.0, 2.0, 3.0)));
  EXPECT_POSE_EQ(Pose(-1.0, -2.0, -3.0), Foothold::getDelta(Foothold(1.0, 2.0, 3.0), Foothold()));
  EXPECT_POSE_EQ(Pose(4.0, -6.0, -1.0), Foothold::getDelta(Foothold(1.0, 2.0, 3.0), Foothold(5.0, -4.0, 2.0)));

  // basic rotation tests
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.1, 0.2, -0.3), Foothold::getDelta(Foothold(), Foothold(0.0, 0.0, 0.0, 0.1, 0.2, -0.3)));
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.3), Foothold::getDelta(Foothold(0.0, 0.0, 0.0, 0.0, 0.0, -0.3), Foothold()));

  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, -0.23), Foothold::getDelta2D(Foothold(), Foothold(0.0, 0.0, 0.0, 0.1, 0.34, -0.23)));
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.23), Foothold::getDelta2D(Foothold(0.0, 0.0, 0.0, 0.23, 0.6, -0.23), Foothold()));

  EXPECT_POSE_EQ(Pose(0.0, 0.0, 0.0, 0.0, 0.0, -0.1), Foothold::getDelta2D(Foothold(0.0, 0.0, 0.0, 0.1, 0.2, -0.3), Foothold(0.0, 0.0, 0.0, 0.2, 0.3, -0.4)));

  // complex tests
  double yaw_1 = 0.3;
  EXPECT_POSE_EQ(Pose(0.3 * cos(-yaw_1) - 0.05 * sin(-yaw_1), 0.3 * sin(-yaw_1) + 0.05 * cos(-yaw_1), 0.0, 0.0, 0.0, 0.35),
                 Foothold::getDelta2D(Foothold(1.0, 2.30, 1.2, 0.30, 0.12, yaw_1), Foothold(1.3, 2.35, 1.3, 0.32, -0.14, 0.65)));
}
