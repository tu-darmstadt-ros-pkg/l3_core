//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_BASE_LINK_H__
#define L3_BASE_LINK_H__

#include <angles/angles.h>

#include <l3_libs/types/memory.h>
#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/variant_data.h>

namespace l3
{
typedef unsigned int LinkIndex;
typedef std::vector<LinkIndex> LinkIndexArray;
typedef std::set<LinkIndex> LinkIndexSet;
typedef std::vector<LinkIndexArray> MultiLinkIndexArray;

/**
 * @brief Generic structure for describing links
 */
template <typename LinkType>
struct BaseLink
{
  // typedefs
  typedef SharedPtr<BaseLink> Ptr;
  typedef SharedPtr<const BaseLink> ConstPtr;

  BaseLink()
    : idx(0)
    , pose_(Pose())
    , roll_(0.0)
    , pitch_(0.0)
    , yaw_(0.0)
  {}

  BaseLink(LinkIndex idx, const Pose& pose, const std_msgs::Header& header = std_msgs::Header(), const VariantDataSet& data = VariantDataSet())
    : header(header)
    , idx(idx)
    , data(data)
  {
    setPose(pose);
  }

  BaseLink(LinkIndex idx, double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, const std_msgs::Header& header = std_msgs::Header(),
           const VariantDataSet& data = VariantDataSet())
    : header(header)
    , idx(idx)
    , data(data)
  {
    setPose(x, y, z, roll, pitch, yaw);
  }

  inline BaseLink(double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, const std_msgs::Header& header = std_msgs::Header(),
                  const VariantDataSet& data = VariantDataSet())
    : BaseLink(0, x, y, z, roll, pitch, yaw, header, data)
  {}

  bool operator==(const BaseLink& other) const
  {
    if (idx != other.idx)
      return false;
    if (!DOUBLE_EQ(x(), other.x()))
      return false;
    if (!DOUBLE_EQ(y(), other.y()))
      return false;
    if (!DOUBLE_EQ(z(), other.z()))
      return false;
    if (!DOUBLE_EQ(roll(), other.roll()))
      return false;
    if (!DOUBLE_EQ(pitch(), other.pitch()))
      return false;
    if (!DOUBLE_EQ(yaw(), other.yaw()))
      return false;

    return true;
  }

  inline bool operator!=(const BaseLink& other) const { return !this->operator==(other); }

  static ByteStream& serialize(ByteStream& stream, const VariantData& in)
  {
    const LinkType& link = in.value<LinkType>();

    stream << link.header.stamp;
    stream << link.header.frame_id;
    stream << link.idx;
    stream << link.pose();
    stream << link.data;

    return stream;
  }

  static ByteStream& deserialize(ByteStream& stream, VariantData& out)
  {
    LinkType link;

    stream >> link.header.stamp;
    stream >> link.header.frame_id;
    stream >> link.idx;
    Pose pose;
    stream >> pose;
    link.setPose(pose);
    stream >> link.data;

    out = link;

    return stream;
  }

  std::string toString() const
  {
    std::stringstream s;
    s << std::setprecision(3) << std::fixed;
    s << "(" << idx << ") ";
    s << x() << "/" << y() << "/" << z() << " - ";
    s << pitch() << "/" << roll() << "/" << yaw();
    return s.str();
  }

  inline const Pose& pose() const { return pose_; }

  void setPose(const Pose& p)
  {
    pose_ = p;
    pose_.getRPY(roll_, pitch_, yaw_);
  }

  void setPose(Pose&& p)
  {
    pose_ = std::move(p);
    pose_.getRPY(roll_, pitch_, yaw_);
  }

  void setPose(double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
  {
    pose_.setXYZ(x, y, z);
    setRPY(roll, pitch, yaw);
  }

  inline double x() const { return pose_.x(); }
  inline double y() const { return pose_.y(); }
  inline double z() const { return pose_.z(); }

  inline void setX(double x) { pose_.setX(x); }
  inline void setY(double y) { pose_.setY(y); }
  inline void setZ(double z) { pose_.setZ(z); }

  inline void setXYZ(double x, double y, double z) { pose_.setXYZ(x, y, z); }

  inline double roll() const { return roll_; }
  inline double pitch() const { return pitch_; }
  inline double yaw() const { return yaw_; }

  inline void getRPY(double& roll, double& pitch, double& yaw) const
  {
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
  }

  inline void setRoll(double roll) { setRPY(roll, pitch_, yaw_); }
  inline void setPitch(double pitch) { setRPY(roll_, pitch, yaw_); }
  inline void setYaw(double yaw) { setRPY(roll_, pitch_, yaw); }

  void setRPY(double roll, double pitch, double yaw)
  {
    pose_.setRPY(roll, pitch, yaw);
    roll_ = angles::normalize_angle(roll);
    pitch_ = angles::normalize_angle(pitch);
    yaw_ = angles::normalize_angle(yaw);
  }

  /**
   * @brief Projects the link's pose down to 2D space (x-y-yaw).
   * @return Projected pose
   */
  inline Pose project2D() const { return project2D(*this); }
  static inline Pose project2D(const BaseLink& base_link) { return Pose(base_link.x(), base_link.y(), 0.0, 0.0, 0.0, base_link.yaw()); }

  /**
   * @brief Calculates delta step between two links such that target = origin + dstep
   * @param origin reference link
   * @param target target link
   * @return delta between both links
   */
  static inline Transform getDelta(const BaseLink& origin, const BaseLink& target)
  {
    if (origin == target)
      return Transform();
    else
      return Transform::getTransform(target.pose(), origin.pose());
  }

  /**
   * @brief Calculates 2D projected (x-y-yaw) delta step between two links such that target = origin + dstep
   * @param origin reference link
   * @param target target link
   * @return delta between both links
   */
  static inline Transform getDelta2D(const BaseLink& origin, const BaseLink& target)
  {
    if (origin == target)
      return Transform();
    else
      return Transform::getTransform(target.project2D(), origin.project2D());
  }

  std_msgs::Header header;
  LinkIndex idx;        // index number of foot (e.g. bipedal robot: left = 0; right = 1); defintion depends on user

  VariantDataSet data;  // may contain user specific data

protected:
  Pose pose_;    // pose of link
  double roll_;  // caching roll, pitch and yaw in order to prevent excessive usage of atan2 function
  double pitch_;
  double yaw_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace l3

#endif
