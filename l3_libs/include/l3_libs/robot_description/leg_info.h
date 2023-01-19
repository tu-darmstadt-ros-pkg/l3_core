//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_LEG_INFO_H__
#define L3_LEG_INFO_H__

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>

#include <l3_libs/types/typedefs.h>

#include <l3_msgs/LegInfo.h>

namespace l3
{
typedef unsigned int LegIndex;

struct LegInfo
{
  LegInfo();
  LegInfo(const XmlRpc::XmlRpcValue& params);
  LegInfo(const l3_msgs::LegInfo& msg);

  void fromMsg(const l3_msgs::LegInfo& msg);

  void toMsg(l3_msgs::LegInfo& msg) const;

  inline l3_msgs::LegInfo toMsg() const
  {
    l3_msgs::LegInfo msg;
    toMsg(msg);
    return msg;
  }

  LegIndex idx;  // Note: Assuming leg idx = foot idx
  std::string name;
  std::string root_link, tip_link;
  std::vector<std::string> joints;
};

typedef std::pair<const LegIndex, LegInfo> LegInfoPair;
typedef std::map<LegIndex, LegInfo> LegInfoMap;
typedef std::pair<const LegIndex, Pose> LegPosePair;
typedef std::map<LegIndex, Pose> LegPoseMap;
typedef std::map<LegIndex, WrenchData> LegFTMap;
typedef std::pair<LegIndex, WrenchData> LegFTPair;

L3_STATIC_ASSERT_MOVEABLE(LegInfo)
}  // namespace l3

#endif
