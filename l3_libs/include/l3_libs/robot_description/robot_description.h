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

#ifndef L3_ROBOT_DESCRIPTION_H__
#define L3_ROBOT_DESCRIPTION_H__

#include <ros/ros.h>

#include <l3_msgs/RobotDescription.h>

#include <l3_libs/robot_description/base_info.h>
#include <l3_libs/robot_description/foot_info.h>
#include <l3_libs/robot_description/leg_info.h>

namespace l3
{
class RobotDescription
{
public:
  // typedefs
  typedef SharedPtr<RobotDescription> Ptr;
  typedef SharedPtr<const RobotDescription> ConstPtr;

  RobotDescription(const XmlRpc::XmlRpcValue& params);
  RobotDescription(const l3_msgs::RobotDescription& msg);
  RobotDescription(ros::NodeHandle& nh, const std::string& topic = "robot_description");

  void fromMsg(const l3_msgs::RobotDescription& msg);

  void toMsg(l3_msgs::RobotDescription& msg) const;

  inline l3_msgs::RobotDescription toMsg() const
  {
    l3_msgs::RobotDescription msg;
    toMsg(msg);
    return msg;
  }

  inline const XmlRpc::XmlRpcValue& getParams() const { return params_; }

  bool isValid() const;

  inline bool hasFootInfo(const FootIndex& foot_idx) const { return foot_info_map_.find(foot_idx) != foot_info_map_.end(); }

  bool getFootInfo(const FootIndex& foot_idx, FootInfo& foot_info) const;
  FootInfo getFootInfo(const FootIndex& foot_idx) const;

  inline bool hasBaseInfo(const BaseIndex& base_idx) const { return base_info_map_.find(base_idx) != base_info_map_.end(); }

  bool getBaseInfo(const BaseIndex& base_idx, BaseInfo& base_info) const;
  BaseInfo getBaseInfo(const BaseIndex& base_idx) const;

  inline bool hasLegInfo(const LegIndex& leg_idx) const { return leg_info_map_.find(leg_idx) != leg_info_map_.end(); }

  bool getLegInfo(const LegIndex& leg_idx, LegInfo& leg_info) const;
  LegInfo getLegInfo(const LegIndex& leg_idx) const;

  inline const FootInfoMap& getFootInfoMap() const { return foot_info_map_; }
  inline const BaseInfoMap& getBaseInfoMap() const { return base_info_map_; }
  inline const LegInfoMap& getLegInfoMap() const { return leg_info_map_; }

  inline const std::vector<FootIndex>& footIdxList() const { return foot_idx_; }
  inline const std::vector<BaseIndex>& baseIdxList() const { return base_idx_; }
  inline const std::vector<LegIndex>& legIdxList() const { return leg_idx_; }

  inline const std::string& footId(const FootIndex& foot_idx) const { return foot_ids_[foot_idx_map_.at(foot_idx)]; }
  inline const std::vector<std::string>& footIdList() const { return foot_ids_; }
  inline const std::string& baseId(const BaseIndex& base_idx) const { return base_ids_[base_idx_map_.at(base_idx)]; }
  inline const std::vector<std::string>& baseIdList() const { return base_ids_; }
  inline const std::string& legId(const LegIndex& leg_idx) const { return leg_ids_[foot_idx_map_.at(leg_idx)]; }
  inline const std::vector<std::string>& legIdList() const { return leg_ids_; }

  /**
   * @brief Returns for each known FootIndex a unique id in range of [0, #Feet]. This mapping can be used to efficiently
   * perform lookups with foot indeces in simple array data structures (e.g. FootholdArray).
   * @param foot_idx Foot Index as specified by robot model
   * @return Unique id in range of [0, #Feet].
   */
  size_t getFootIdx(const FootIndex& foot_idx) const;

  /**
   * @brief Returns for each known BaseIndex a unique id in range of [0, #Base]. This mapping can be used to efficiently
   * perform lookups with base indeces in simple array data structures (e.g. FloatingBaseArray).
   * @param base_idx Base Index as specified by robot model
   * @return Unique id in range of [0, #Base].
   */
  size_t getBaseIdx(const BaseIndex& base_idx) const;

  /**
   * @brief Returns for each known LegIndex a unique id in range of [0, #Legs]. This mapping can be used to efficiently
   * perform lookups with foot indeces in simple array data structures.
   * @param leg_idx Leg Index as specified by robot model
   * @return Unique id in range of [0, #Legs].
   */
  size_t getLegIdx(const LegIndex& leg_idx) const;

  /**
   * @brief Generates neutral stance on given robot pose.
   * @param center Robot's center from which the neutral stance is derived
   * @return Neutral stance as array of footholds
   */
  FootholdPtrArray getNeutralStance(const Pose& center = Pose()) const;

  /**
   * @brief Generates neutral stance aligned to the given foothold which is part of the neutral stance pose.
   * @param foothold Foothold from which the neutral stance is derived
   * @return Neutral stance as array of footholds
   */
  inline FootholdPtrArray getNeutralStance(const Foothold& foothold) const { return getNeutralStance(foothold.pose() * getFootInfo(foothold.idx).neutral_stance.inverse()); }

  inline bool isIndirectFoot(const FootIndex& foot_idx) const { return indirect_foot_idx_.find(foot_idx) != indirect_foot_idx_.end(); }
  inline const FootIndexSet& getIndirectFootIdx() const { return indirect_foot_idx_; }

protected:
  void addFootInfo(const FootInfo& foot_info);
  void addBaseInfo(const BaseInfo& base_info);
  void addLegInfo(const LegInfo& leg_info);

  void parseFeet(XmlRpc::XmlRpcValue& feet);
  void parseBases(XmlRpc::XmlRpcValue& bases);
  void parseLegs(XmlRpc::XmlRpcValue& legs);

  void generateIdxMaps();

  bool checkConsistency() const;

  FootInfoMap foot_info_map_;  // mapping of foot index to detailed foot information
  BaseInfoMap base_info_map_;  // data about robot upper body base
  LegInfoMap leg_info_map_;    // mapping of leg index to detailed leg information (leg infos are optional for each foot)

  std::vector<FootIndex> foot_idx_;             // list of available foot indeces
  std::vector<std::string> foot_ids_;           // list of foot names
  std::map<FootIndex, size_t> foot_idx_map_;    // mapping of foot index to internal position in foot_idx_ and foot_ids_ vector

  std::vector<BaseIndex> base_idx_;             // list of available base indeces
  std::vector<std::string> base_ids_;           // list of base names
  std::map<BaseIndex, size_t> base_idx_map_;    // mapping of base index to internal position in base_idx_ and base_ids_ vector

  std::vector<LegIndex> leg_idx_;           // list of available leg indeces
  std::vector<std::string> leg_ids_;        // list of leg names
  std::map<FootIndex, size_t> leg_idx_map_; // mapping of leg index to internal position in leg_idx_ and leg_ids_ vector

  FootholdArray neutral_stance_;  // footholds for neutral stance given in robot's center

  FootIndexSet indirect_foot_idx_;

  XmlRpc::XmlRpcValue params_;  // parameter structure from which this config has been parsed from; saved in order to support user-defined parameters
};

L3_STATIC_ASSERT_MOVEABLE(RobotDescription)
}  // namespace l3

#endif
