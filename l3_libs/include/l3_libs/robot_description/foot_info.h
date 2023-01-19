//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOT_INFO_H__
#define L3_FOOT_INFO_H__

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/foothold.h>

#include <l3_msgs/FootInfo.h>

namespace l3
{
struct FootInfo
{
  enum Shape
  {
    CUBOID = l3_msgs::FootInfo::CUBOID,
    SPHERICAL = l3_msgs::FootInfo::SPHERICAL
  };

  FootInfo();
  FootInfo(const XmlRpc::XmlRpcValue& params);
  FootInfo(const l3_msgs::FootInfo& msg);

  void fromMsg(const l3_msgs::FootInfo& msg);

  void toMsg(l3_msgs::FootInfo& msg) const;

  inline l3_msgs::FootInfo toMsg() const
  {
    l3_msgs::FootInfo msg;
    toMsg(msg);
    return msg;
  }

  FootIndex idx;
  std::string name;

  // foot properties
  Shape shape;
  Vector3 size;
  std::string link;
  Transform link_to_sole_offset;  // position (offset) of center of sole given in robot's "foot" link (foot frame -> sole frame transform)

  Pose neutral_stance;

  bool indirect;  // Indicates the foothold represents the pose of a inner link such as the base instead of a foot.
                  // Indirect links do not require to maintain contact with the environment and thus will not be created by default during state generation.

  // visualization related parameters
  std_msgs::ColorRGBA color;
  std::string mesh_resource;
  Pose mesh_offset;
  Vector3 mesh_scale;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::pair<const FootIndex, FootInfo> FootInfoPair;
typedef std::map<FootIndex, FootInfo> FootInfoMap;

bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, FootInfo::Shape& val);

L3_STATIC_ASSERT_MOVEABLE(FootInfo)
}  // namespace l3

#endif
