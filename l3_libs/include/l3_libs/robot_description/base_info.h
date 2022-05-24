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

#ifndef L3_BASE_INFO_H__
#define L3_BASE_INFO_H__

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/floating_base.h>

#include <l3_msgs/BaseInfo.h>

namespace l3
{
struct BaseInfo
{
  static constexpr BaseIndex MAIN_BODY_IDX = 0;

  enum Shape
  {
    CUBOID = l3_msgs::BaseInfo::CUBOID,
    SPHERICAL = l3_msgs::BaseInfo::SPHERICAL
  };

  BaseInfo();
  BaseInfo(const XmlRpc::XmlRpcValue& params);
  BaseInfo(const l3_msgs::BaseInfo& msg);

  void fromMsg(const l3_msgs::BaseInfo& msg);

  void toMsg(l3_msgs::BaseInfo& msg) const;

  inline l3_msgs::BaseInfo toMsg() const
  {
    l3_msgs::BaseInfo msg;
    toMsg(msg);
    return msg;
  }

  BaseIndex idx;
  std::string name;

  Shape shape;
  Vector3 size;
  std::string link;
  Transform link_to_center_offset;  // position (offset) of center of bounding box given in robot's "base" link (base frame -> box center frame transform)

  // visualization related parameters
  std_msgs::ColorRGBA color;
  std::string mesh_resource;
  Pose mesh_offset;
  Vector3 mesh_scale;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::pair<const BaseIndex, BaseInfo> BaseInfoPair;
typedef std::map<BaseIndex, BaseInfo> BaseInfoMap;

bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, BaseInfo::Shape& val);

L3_STATIC_ASSERT_MOVEABLE(BaseInfo)
}  // namespace l3

#endif
