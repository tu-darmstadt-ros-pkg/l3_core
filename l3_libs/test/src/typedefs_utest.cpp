#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_libs/test_macros.h>
#include <l3_libs/types/types.h>

using namespace l3;

TEST(Typedefs, Vector3)
{
  Vector3 p;

  // check clean initialization
  EXPECT_DOUBLE_EQ(0.0, p.x());
  EXPECT_DOUBLE_EQ(0.0, p.y());
  EXPECT_DOUBLE_EQ(0.0, p.z());

  // check setters and getters
  p.x() = 1.0 * M_PI;
  p.y() = 2.0 * M_PI;
  p.z() = 3.0 * M_PI;
  EXPECT_DOUBLE_EQ(1.0 * M_PI, p.x());
  EXPECT_DOUBLE_EQ(2.0 * M_PI, p.y());
  EXPECT_DOUBLE_EQ(3.0 * M_PI, p.z());

  p.setX(2.1 * M_PI);
  p.setY(3.2 * M_PI);
  p.setZ(4.3 * M_PI);
  EXPECT_DOUBLE_EQ(2.1 * M_PI, p.x());
  EXPECT_DOUBLE_EQ(3.2 * M_PI, p.y());
  EXPECT_DOUBLE_EQ(4.3 * M_PI, p.z());
}

TEST(Typedefs, Vector4)
{
  Vector4 p;

  // check clean initialization
  EXPECT_DOUBLE_EQ(0.0, p.x());
  EXPECT_DOUBLE_EQ(0.0, p.y());
  EXPECT_DOUBLE_EQ(0.0, p.z());
  EXPECT_DOUBLE_EQ(0.0, p.w());

  // check setters and getters
  p.x() = 1.0 * M_PI;
  p.y() = 2.0 * M_PI;
  p.z() = 3.0 * M_PI;
  p.w() = 4.0 * M_PI;
  EXPECT_DOUBLE_EQ(1.0 * M_PI, p.x());
  EXPECT_DOUBLE_EQ(2.0 * M_PI, p.y());
  EXPECT_DOUBLE_EQ(3.0 * M_PI, p.z());
  EXPECT_DOUBLE_EQ(4.0 * M_PI, p.w());

  p.setX(2.1 * M_PI);
  p.setY(3.2 * M_PI);
  p.setZ(4.3 * M_PI);
  p.setW(5.4 * M_PI);
  EXPECT_DOUBLE_EQ(2.1 * M_PI, p.x());
  EXPECT_DOUBLE_EQ(3.2 * M_PI, p.y());
  EXPECT_DOUBLE_EQ(4.3 * M_PI, p.z());
  EXPECT_DOUBLE_EQ(5.4 * M_PI, p.w());
}

TEST(Typedefs, Quaternion)
{
  Quaternion q;

  // check clean initialization
  EXPECT_DOUBLE_EQ(1.0, q.w());
  EXPECT_DOUBLE_EQ(0.0, q.x());
  EXPECT_DOUBLE_EQ(0.0, q.y());
  EXPECT_DOUBLE_EQ(0.0, q.z());

  // check setters and getters
  q.setW(1.0 * M_PI);
  q.setX(2.0 * M_PI);
  q.setY(3.0 * M_PI);
  q.setZ(4.0 * M_PI);
  EXPECT_DOUBLE_EQ(1.0 * M_PI, q.w());
  EXPECT_DOUBLE_EQ(2.0 * M_PI, q.x());
  EXPECT_DOUBLE_EQ(3.0 * M_PI, q.y());
  EXPECT_DOUBLE_EQ(4.0 * M_PI, q.z());

  q.setWXYZ(1.0, 2.0, 3.0, 4.0);
  EXPECT_DOUBLE_EQ(1.0, q.w());
  EXPECT_DOUBLE_EQ(2.0, q.x());
  EXPECT_DOUBLE_EQ(3.0, q.y());
  EXPECT_DOUBLE_EQ(4.0, q.z());

  q.setRPY(0.1 * M_PI, 0.2 * M_PI, 0.3 * M_PI);
  EXPECT_ANGLE_EQ(0.1 * M_PI, q.roll());
  EXPECT_ANGLE_EQ(0.2 * M_PI, q.pitch());
  EXPECT_ANGLE_EQ(0.3 * M_PI, q.yaw());

  q.setRPY(0.2 * M_PI, 0.3 * M_PI, 0.1 * M_PI);
  double roll, pitch, yaw;
  q.getRPY(roll, pitch, yaw);
  EXPECT_ANGLE_EQ(0.2 * M_PI, roll);
  EXPECT_ANGLE_EQ(0.3 * M_PI, pitch);
  EXPECT_ANGLE_EQ(0.1 * M_PI, yaw);

  q.setW(1.0 * M_PI);
  q.setX(2.0 * M_PI);
  q.setY(3.0 * M_PI);
  q.setZ(4.0 * M_PI);
  q.normalize();
  Rotation rotation = q.getRotation();
  Quaternion q_res(rotation);
  EXPECT_DOUBLE_EQ(q.x(), q_res.x());
  EXPECT_DOUBLE_EQ(q.y(), q_res.y());
  EXPECT_DOUBLE_EQ(q.z(), q_res.z());
  EXPECT_DOUBLE_EQ(q.w(), q_res.w());
  Rotation rotation_res = q_res.getRotation();
  EXPECT_DOUBLE_EQ(rotation(0, 0), rotation_res(0, 0));
  EXPECT_DOUBLE_EQ(rotation(0, 1), rotation_res(0, 1));
  EXPECT_DOUBLE_EQ(rotation(0, 2), rotation_res(0, 2));
  EXPECT_DOUBLE_EQ(rotation(1, 0), rotation_res(1, 0));
  EXPECT_DOUBLE_EQ(rotation(1, 1), rotation_res(1, 1));
  EXPECT_DOUBLE_EQ(rotation(1, 2), rotation_res(1, 2));
  EXPECT_DOUBLE_EQ(rotation(2, 0), rotation_res(2, 0));
  EXPECT_DOUBLE_EQ(rotation(2, 1), rotation_res(2, 1));
  EXPECT_DOUBLE_EQ(rotation(2, 2), rotation_res(2, 2));
}

// Test for checking functionality of l3::Pose
TEST(Typedefs, Pose)
{
  Pose p;

  // check clean initialization
  EXPECT_DOUBLE_EQ(0.0, p.x());
  EXPECT_DOUBLE_EQ(0.0, p.y());
  EXPECT_DOUBLE_EQ(0.0, p.z());
  EXPECT_ANGLE_EQ(0.0, p.roll());
  EXPECT_ANGLE_EQ(0.0, p.pitch());
  EXPECT_ANGLE_EQ(0.0, p.yaw());

  // check setters and getters
  p.setX(1.0 * M_PI);
  p.setY(2.0 * M_PI);
  p.setZ(3.0 * M_PI);
  EXPECT_DOUBLE_EQ(1.0 * M_PI, p.x());
  EXPECT_DOUBLE_EQ(2.0 * M_PI, p.y());
  EXPECT_DOUBLE_EQ(3.0 * M_PI, p.z());

  p.setRoll(0.1 * M_PI);
  p.setPitch(0.2 * M_PI);
  p.setYaw(0.3 * M_PI);
  EXPECT_ANGLE_EQ(0.1 * M_PI, p.roll());
  EXPECT_ANGLE_EQ(0.2 * M_PI, p.pitch());
  EXPECT_ANGLE_EQ(0.3 * M_PI, p.yaw());

  p.setXYZ(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(1.0, p.x());
  EXPECT_DOUBLE_EQ(2.0, p.y());
  EXPECT_DOUBLE_EQ(3.0, p.z());

  p.setRPY(0.2 * M_PI, 0.3 * M_PI, 0.1 * M_PI);
  double roll, pitch, yaw;
  p.getRPY(roll, pitch, yaw);
  EXPECT_ANGLE_EQ(0.2 * M_PI, roll);
  EXPECT_ANGLE_EQ(0.3 * M_PI, pitch);
  EXPECT_ANGLE_EQ(0.1 * M_PI, yaw);

  // check quaternion operation
  Quaternion q;
  q.setRPY(0.1 * M_PI, -0.2 * M_PI, 0.3 * M_PI);
  p = Pose(Position(), q);
  p.getRPY(roll, pitch, yaw);
  EXPECT_ANGLE_EQ(0.1 * M_PI, roll);
  EXPECT_ANGLE_EQ(-0.2 * M_PI, pitch);
  EXPECT_ANGLE_EQ(0.3 * M_PI, yaw);

  // check consistency using quaternions
  p.setQuaternion(p.getQuaternion());
  p.getRPY(roll, pitch, yaw);
  EXPECT_ANGLE_EQ(0.1 * M_PI, roll);
  EXPECT_ANGLE_EQ(-0.2 * M_PI, pitch);
  EXPECT_ANGLE_EQ(0.3 * M_PI, yaw);

  // check functions
  EXPECT_DOUBLE_EQ(0.0, Pose().squaredNorm());
  EXPECT_DOUBLE_EQ(0.0, Pose().norm());
  EXPECT_DOUBLE_EQ(29.0, Pose(2.0, 3.0, 4.0).squaredNorm());
  EXPECT_DOUBLE_EQ(sqrt(29.0), Pose(2.0, 3.0, 4.0).norm());

  EXPECT_POSE_EQ(Pose(2.0, 3.0, 0.0, 0.0, 0.0, 0.3 * M_PI), Pose(2.0, 3.0, 1.0, 3.1, -1.3, 0.3 * M_PI).project2D());
}

// Test for checking functionality of transformations
TEST(Typedefs, Transformations)
{
  Pose p_zero;
  Pose p_one(1.0, 1.0, 1.0);

  /* |  1  0  0 -2 |
   * |  0  1  0 -3 |
   * |  0  0  1  0 |
   * |  0  0  0  1 | */
  Transform t1 = Transform(-2.0, -3.0, 0.0);
  EXPECT_POSE_EQ(t1, t1 * p_zero);
  EXPECT_POSE_EQ(Pose(-1.0, -2.0, 1.0), t1 * p_one);
  EXPECT_POSE_EQ(Transform(2.0, 3.0, 0.0), t1.inverse());

  /* | -1  0  0  1 |
   * |  0  1  0 -1 |
   * |  0  0 -1  2 |
   * |  0  0  0  1 | */
  Transform t2 = Transform(1.0, -1.0, 2.0, 0.0, M_PI, 0.0);
  EXPECT_POSE_EQ(t2, t2 * p_zero);
  EXPECT_POSE_EQ(Pose(0.0, 0.0, 1.0, 0.0, M_PI, 0.0), t2 * p_one);
  EXPECT_POSE_EQ(Transform(1.0, 1.0, 2.0, 0.0, -M_PI, 0.0), t2.inverse());

  /// @TODO: Add more test with more complex rotation

  // check calculation of transforms
  EXPECT_POSE_EQ(t1, Transform::getTransform(t1, Transform()));
  EXPECT_POSE_EQ(t1.inverse(), Transform::getTransform(Transform(), t1));

  EXPECT_POSE_EQ(t2, Transform::getTransform(t2, Transform()));
  EXPECT_POSE_EQ(t2.inverse(), Transform::getTransform(Transform(), t2));

  EXPECT_POSE_EQ(Transform(3.0, -2.0, 2.0, 0.0, M_PI, 0.0), Transform::getTransform(t1, t2));

  EXPECT_VECTOR_EQ(Vector3(-2.0, -3.0, 0.0), t1 * Vector3());
  EXPECT_VECTOR_EQ(Vector3(-1.0, -2.0, 1.0), t1 * Vector3(1.0, 1.0, 1.0));
  EXPECT_VECTOR_EQ(Vector3(0.0, -4.0, 3.0), t1 * Vector3(2.0, -1.0, 3.0));

  EXPECT_VECTOR_EQ(Vector3(1.0, -1.0, 2.0), t2 * Vector3());
  EXPECT_VECTOR_EQ(Vector3(0.0, 0.0, 1.0), t2 * Vector3(1.0, 1.0, 1.0));
  EXPECT_VECTOR_EQ(Vector3(-1.0, -2.0, -1.0), t2 * Vector3(2.0, -1.0, 3.0));
}

// Test for checking Array2D
TEST(Array2D, Basics)
{
  Array2D<int> array;
  EXPECT_TRUE(array.empty());

  array.init(10, 5);
  array.at(0, 0) = 9;
  array.at(9, 4) = 1;
  array.at(2, 3) = 42;

  EXPECT_FALSE(array.empty());
  EXPECT_EQ(array.size(), 50);
  EXPECT_EQ(array.at(0, 0), 9);
  EXPECT_EQ(array.at(9, 4), 1);
  EXPECT_EQ(array.at(2, 3), 42);
}
