//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_LIBS_EIGEN_TYPES_H__
#define L3_LIBS_EIGEN_TYPES_H__

#include <eigen3/Eigen/Eigen>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <l3_libs/macros.h>

namespace l3
{
/*
 * Typedefs based on Eigen library
 */

typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix3d Rotation;

// forward declaration
struct Transform;

struct Quaternion : public Eigen::Quaterniond
{
  Quaternion()
    : Quaternion(1.0, 0.0, 0.0, 0.0)
  {}

  Quaternion(double w, double x, double y, double z) { setWXYZ(w, x, y, z); }

  Quaternion(const Rotation& rotation)
    : Eigen::Quaterniond(rotation)
  {}

  Quaternion(const Eigen::Quaterniond& other)
    : Eigen::Quaterniond(other)
  {}

  template <typename T>
  inline Quaternion& operator=(const T& other)
  {
    Eigen::Quaterniond::operator=(other);
    return *this;
  }

  inline Vector3 operator*(const Vector3& other) const { return Vector3(Eigen::Quaterniond::operator*(other)); }

  template <typename T>
  inline Quaternion operator*(const T& other) const
  {
    return Quaternion(Eigen::Quaterniond::operator*(other));
  }

  template <typename T>
  inline Quaternion& operator*=(const T& other)
  {
    Eigen::Quaterniond::operator*=(other);
    return *this;
  }

  inline void setW(double w) { this->w() = w; }
  inline void setX(double x) { this->x() = x; }
  inline void setY(double y) { this->y() = y; }
  inline void setZ(double z) { this->z() = z; }

  inline void setWXYZ(double w, double x, double y, double z)
  {
    this->w() = w;
    this->x() = x;
    this->y() = y;
    this->z() = z;
  }

  inline Rotation getRotation() const { return toRotationMatrix(); }

  /**
   * @brief Returns corresponding angle of this quaternion in range of [-pi, pi].
   * Taken from https://en.wikipedia.org/wiki/Quaternion
   * @return corresponding angle
   */
  inline void getRPY(double& roll, double& pitch, double& yaw) const
  {
    roll = this->roll();
    pitch = this->pitch();
    yaw = this->yaw();
  }

  inline double roll() const
  {
    Rotation matrix = getRotation();
    return atan2(matrix(2, 1), matrix(2, 2));
  }

  inline double pitch() const
  {
    Rotation matrix = getRotation();
    return asin(-matrix(2, 0));
  }

  inline double yaw() const
  {
    Rotation matrix = getRotation();
    return atan2(matrix(1, 0), matrix(0, 0));
  }

  inline void setRoll(double roll) { setRPY(roll, pitch(), yaw()); }
  inline void setPitch(double pitch) { setRPY(roll(), pitch, yaw()); }
  inline void setYaw(double yaw) { setRPY(roll(), pitch(), yaw); }

  void setRPY(double roll, double pitch, double yaw)
  {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    setW(cy * cp * cr + sy * sp * sr);
    setX(cy * cp * sr - sy * sp * cr);
    setY(sy * cp * sr + cy * sp * cr);
    setZ(sy * cp * cr - cy * sp * sr);
  }
};

struct Vector2 : public Eigen::Vector2d
{
  Vector2()
    : Eigen::Vector2d(0.0, 0.0)
  {}
  Vector2(const Vector2& other)
    : Eigen::Vector2d(other)
  {}
  Vector2(const Eigen::Vector2d& other)
    : Eigen::Vector2d(other)
  {}
  Vector2(double x, double y)
    : Eigen::Vector2d(x, y)
  {}

  template <typename T>
  inline Vector2 operator+(const T& other) const
  {
    return Vector2(Eigen::Vector2d::operator+(other));
  }

  template <typename T>
  inline Vector2 operator-(const T& other) const
  {
    return Vector2(Eigen::Vector2d::operator-(other));
  }

  template <typename T>
  inline Vector2 operator-() const
  {
    return Vector2(Eigen::Vector2d::operator-());
  }

  template <typename T>
  inline Vector2 operator*(const T& other) const
  {
    return Vector2(Eigen::Vector2d::operator*(other));
  }

  template <typename T>
  inline Vector2 operator/(const T& other) const
  {
    return Vector2(Eigen::Vector2d::operator/(other));
  }

  inline void setX(double x) { this->x() = x; }
  inline void setY(double y) { this->y() = y; }

  std::string toString() const
  {
    std::stringstream ss;
    ss << std::setprecision(2);
    ss << "[" << x() << ", " << y() << "]";
    return ss.str();
  }
};

inline std::ostream& operator<<(std::ostream& stream, const Vector2& vec) { return stream << vec.toString(); }

typedef Vector2 Position2D;

struct Vector3 : public Eigen::Vector3d
{
  Vector3()
    : Eigen::Vector3d(0.0, 0.0, 0.0)
  {}
  Vector3(const Vector3& other)
    : Eigen::Vector3d(other)
  {}
  Vector3(const Eigen::Vector3d& other)
    : Eigen::Vector3d(other)
  {}
  Vector3(double x, double y, double z)
    : Eigen::Vector3d(x, y, z)
  {}

  template <typename T>
  inline Vector3 operator+(const T& other) const
  {
    return Vector3(Eigen::Vector3d::operator+(other));
  }

  template <typename T>
  inline Vector3 operator-(const T& other) const
  {
    return Vector3(Eigen::Vector3d::operator-(other));
  }

  template <typename T>
  inline Vector3 operator-() const
  {
    return Vector3(Eigen::Vector3d::operator-());
  }

  template <typename T>
  inline Vector3 operator*(const T& other) const
  {
    return Vector3(Eigen::Vector3d::operator*(other));
  }

  template <typename T>
  inline Vector3 operator/(const T& other) const
  {
    return Vector3(Eigen::Vector3d::operator/(other));
  }

  inline void setX(double x) { this->x() = x; }
  inline void setY(double y) { this->y() = y; }
  inline void setZ(double z) { this->z() = z; }

  std::string toString() const
  {
    std::stringstream ss;
    ss << std::setprecision(2);
    ss << "[" << x() << ", " << y() << ", " << z() << "]";
    return ss.str();
  }
};

inline std::ostream& operator<<(std::ostream& stream, const Vector3& vec) { return stream << vec.toString(); }

typedef Vector3 Point;
typedef Vector3 Position;
typedef std::vector<Vector3> Vector3Array;
typedef std::vector<Point> PointArray;
typedef Vector3Array ReferenceZMPArray;

struct Vector4 : public Eigen::Vector4d
{
  Vector4()
    : Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)
  {}
  Vector4(const Vector4& other)
    : Eigen::Vector4d(other)
  {}
  Vector4(const Eigen::Vector4d& other)
    : Eigen::Vector4d(other)
  {}
  Vector4(double x, double y, double z, double w)
    : Eigen::Vector4d(x, y, z, w)
  {}

  template <typename T>
  inline Vector4 operator+(const T& other) const
  {
    return Vector4(Eigen::Vector4d::operator+(other));
  }

  template <typename T>
  inline Vector4 operator-() const
  {
    return Vector4(Eigen::Vector4d::operator-());
  }

  template <typename T>
  inline Vector4 operator-(const T& other) const
  {
    return Vector4(Eigen::Vector4d::operator-(other));
  }

  template <typename T>
  inline Vector4 operator*(const T& other) const
  {
    return Vector4(Eigen::Vector4d::operator*(other));
  }

  template <typename T>
  inline Vector4 operator/(const T& other) const
  {
    return Vector4(Eigen::Vector4d::operator/(other));
  }

  inline void setX(double x) { this->x() = x; }
  inline void setY(double y) { this->y() = y; }
  inline void setZ(double z) { this->z() = z; }
  inline void setW(double w) { this->w() = w; }
};

typedef std::vector<Vector4> Vector4Array;

struct Pose2D : public Eigen::Vector3d
{
  Pose2D()
    : Eigen::Vector3d(0.0, 0.0, 0.0)
  {}
  Pose2D(const Pose2D& other)
    : Eigen::Vector3d(other)
  {}
  Pose2D(const Eigen::Vector3d& other)
    : Eigen::Vector3d(other)
  {}
  Pose2D(double x, double y, double yaw)
    : Eigen::Vector3d(x, y, yaw)
  {}

  inline double yaw() const { return coeff(2); }
  inline double& yaw() { return coeffRef(2); }

  inline void setX(double x) { this->x() = x; }
  inline void setY(double y) { this->y() = y; }
  inline void setYaw(double yaw) { this->z() = yaw; }
};

/**
 * @brief The Pose struct represents a 4x4 transformation (pose). The data
 * structure is composed by a linear and translation part:
 * | linear translation |
 * | 0 0 0      1       |
 */
struct Pose : public Eigen::Affine3d
{
  Pose()
    : Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  {}

  Pose(double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
  {
    setXYZ(x, y, z);
    setRPY(roll, pitch, yaw);
  }

  Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
  {
    setPosition(pos);
    setQuaternion(rot);
  }

  Pose(const Eigen::Affine3d& other)
    : Eigen::Affine3d(other)
  {}

  template <typename T>
  inline Pose& operator=(const T& other)
  {
    Eigen::Affine3d::operator=(other);
    return *this;
  }

  inline Vector3 operator*(const Vector3& other) const { return Vector3(Eigen::Affine3d::operator*(other)); }

  template <typename T>
  inline Pose operator*(const T& other) const
  {
    return Pose(Eigen::Affine3d::operator*(other));
  }

  template <typename T>
  inline Pose& operator*=(const T& other)
  {
    Eigen::Affine3d::operator*=(other);
    return *this;
  }

  inline const MatrixType& getMatrix() const { return m_matrix; }

  /**
   * @brief Returns a writable expression of the translation vector of the transformation
   * @returns writable expression of the translation vector
   */
  inline TranslationPart position() { return translation(); }

  inline Point getPosition() const { return Point(translation()); }

  inline void setPosition(const Position& position) { this->position() = position; }

  inline double x() const { return getPosition().x(); }
  inline double y() const { return getPosition().y(); }
  inline double z() const { return getPosition().z(); }

  inline void setX(double x) { position()(0) = x; }
  inline void setY(double y) { position()(1) = y; }
  inline void setZ(double z) { position()(2) = z; }

  inline void setXYZ(double x, double y, double z)
  {
    position()(0) = x;
    position()(1) = y;
    position()(2) = z;
  }

  /**
   * @brief Returns corresponding angle of this pose in range of [-pi, pi].
   * Taken from https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/common/impl/eigen.hpp#L585
   * @return corresponding angle
   */
  inline double roll() const { return atan2(m_matrix(2, 1), m_matrix(2, 2)); }
  inline double pitch() const { return asin(-m_matrix(2, 0)); }
  inline double yaw() const { return atan2(m_matrix(1, 0), m_matrix(0, 0)); }

  inline Rotation getRotation() const { return rotation(); }

  inline Quaternion getQuaternion() const { return Quaternion(rotation()); }

  inline void setQuaternion(const Quaternion& q) { setQuaternion(static_cast<Eigen::Quaterniond>(q)); }
  inline void setQuaternion(const Eigen::Quaterniond& q) { m_matrix.block<3, 3>(0, 0) = q.toRotationMatrix(); }

  inline void getRPY(double& roll, double& pitch, double& yaw) const
  {
    //    Vector3 ea = rotation().eulerAngles(2, 1, 0);
    //    roll = ea[2];
    //    pitch = ea[1];
    //    yaw = ea[0];
    roll = this->roll();
    pitch = this->pitch();
    yaw = this->yaw();
  }

  inline void setRoll(double roll) { setRPY(roll, pitch(), yaw()); }
  inline void setPitch(double pitch) { setRPY(roll(), pitch, yaw()); }
  inline void setYaw(double yaw) { setRPY(roll(), pitch(), yaw); }

  void setRPY(double roll, double pitch, double yaw)
  {
    // clang-format off
    // taken from https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/common/impl/eigen.hpp#L608
    double A = cos(yaw),  B = sin(yaw),  C  = cos(pitch), D  = sin(pitch),
           E = cos(roll), F = sin(roll), DE = D*E,        DF = D*F;

    m_matrix(0, 0) = A*C;  m_matrix(0, 1) = A*DF - B*E;  m_matrix(0, 2) = B*F + A*DE;  /*m_matrix(0, 3) = x();*/
    m_matrix(1, 0) = B*C;  m_matrix(1, 1) = A*E + B*DF;  m_matrix(1, 2) = B*DE - A*F;  /*m_matrix(1, 3) = y();*/
    m_matrix(2, 0) = -D;   m_matrix(2, 1) = C*F;         m_matrix(2, 2) = C*E;         /*m_matrix(2, 3) = z();*/
    m_matrix(3, 0) = 0;    m_matrix(3, 1) = 0;           m_matrix(3, 2) = 0;           m_matrix(3, 3) = 1;
    // clang-format on
  }

  inline Pose inverse() const { return Eigen::Affine3d::inverse(); }

  /**
   * @brief Computes squared norm of translation part
   * @return Squared norm of translation part
   */
  inline double squaredNorm() const { return m_matrix.block<3, 1>(0, 3).squaredNorm(); }

  /**
   * @brief Computes norm of translation part
   * @return Norm of translation part
   */
  inline double norm() const { return m_matrix.block<3, 1>(0, 3).norm(); }

  /**
   * @brief Projects current Pose down to 2D space (x-y-yaw).
   * @return Projected pose
   */
  inline Pose project2D() const { return project2D(*this); }
  static inline Pose project2D(const Pose& pose) { return Pose(pose.x(), pose.y(), 0.0, 0.0, 0.0, pose.yaw()); }

  std::string toString() const
  {
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::stringstream ss;
    ss << matrix().format(CleanFmt);
    return ss.str();
  }
};

inline std::ostream& operator<<(std::ostream& stream, const Pose& pose) { return stream << pose.toString(); }

typedef std::vector<Pose> PoseArray;

/**
 * @brief Structure describing homogeneous transformation.
 * Note: A homogeneous transformation T describes the translation and rotation of
 * local a point p_l in system T relative to the world system, such as p_w = T * p_l
 */
struct Transform : public Pose
{
  Transform()
    : Pose()
  {}

  Transform(double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
    : Pose(x, y, z, roll, pitch, yaw)
  {}

  Transform(const Eigen::Affine3d& other)
    : Pose(other)
  {}

  Transform(const Pose& other)
    : Pose(other)
  {}

  Transform(const tf::Transform& transform) { tf::transformTFToEigen(transform, *this); }

  template <typename T>
  inline Transform& operator=(const T& other)
  {
    Pose::operator=(other);
    return *this;
  }

  inline Vector3 operator*(const Vector3& other) const { return Vector3(Eigen::Affine3d::operator*(other)); }

  template <typename T>
  inline Transform operator*(const T& other) const
  {
    return Transform(Eigen::Affine3d::operator*(other));
  }

  template <typename T>
  inline Transform& operator*=(const T& other)
  {
    Eigen::Affine3d::operator*=(other);
    return *this;
  }

  inline Transform inverse() const { return Pose::inverse(); }

  /**
   * @brief Computes transform from current (T1) to target (T2) frame.
   * The resulting transformation (T) transforms any point given in T1 into T2,
   * such that p_2 = T^2_1 * p_1.
   * Note: A l3::Pose P represents implicitly a transformation T^w_p from the pose's local
   * into world frame (p->w). Hence, any point is transformed into world frame via: p_w = P * p_p
   * The transformation from pose P1 into pose P2 yields:
   * (P2^w_p2)⁻¹ * P1^w_p1 = P2^p2_w * P1^w_p1 = T^p2_p1
   */
  template <typename T1, typename T2>
  inline static Transform getTransform(const T1& current, const T2& target)
  {
    return target.inverse() * current;
  }
};

inline std::ostream& operator<<(std::ostream& stream, const Transform& transform) { return stream << transform.toString(); }

// assertion to ensure that alle classes are implemented to support movable operation
L3_STATIC_ASSERT_MOVEABLE(Quaternion)
L3_STATIC_ASSERT_MOVEABLE(Vector2)
L3_STATIC_ASSERT_MOVEABLE(Vector3)
L3_STATIC_ASSERT_MOVEABLE(Vector4)
L3_STATIC_ASSERT_MOVEABLE(Point)
L3_STATIC_ASSERT_MOVEABLE(Position2D)
L3_STATIC_ASSERT_MOVEABLE(Position)
L3_STATIC_ASSERT_MOVEABLE(Pose2D)
L3_STATIC_ASSERT_MOVEABLE(Pose)
L3_STATIC_ASSERT_MOVEABLE(Transform)
}  // namespace l3

#endif
