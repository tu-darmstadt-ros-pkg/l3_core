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

#ifndef L3_KINEMATICS_PLUGIN_H__
#define L3_KINEMATICS_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_pluginlib/plugin.h>

#include <l3_libs/types/types.h>
#include <l3_libs/robot_description/robot_description.h>

#include <l3_math/math.h>

namespace l3
{
/**
 * Notes:
 * root link: Link at which the legs are attached to
 * base link: base link of robot
 */

class KinematicsPlugin : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<KinematicsPlugin> Ptr;
  typedef l3::SharedPtr<const KinematicsPlugin> ConstPtr;

  KinematicsPlugin(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool isUnique() const final { return true; }

  virtual void setRobotDescription(RobotDescription::ConstPtr robot_description);

  const std::string& getRootLink() const { return root_link_; }

  virtual bool getMassOfChain(const std::string& start_link_id, double& mass) const { return false; }
  virtual bool getMassOfLink(const std::string& link_id, double& mass) const { return false; }
  virtual bool calcTotalMass(double& result) const { return false; }

  virtual bool calcCenterOfMass(const std::string& frame_link, Vector3& com) const { return false; }

  /**
   * @brief Calculates geometric feet center based on the given foothold configuration.
   * If not overloaded the naive implementation is used.
   * @param footholds foothold configuration
   * @return geometric feet center
   */
  virtual Pose calcFeetCenter(const FootholdArray& footholds) const { return l3::calcFeetCenter(footholds); }
  virtual Pose calcFeetCenter(const FootholdConstPtrArray& footholds) const { return l3::calcFeetCenter(footholds); }
  /// If upper variant is overloaded, one should use following code snippet for sake of simplicity (calling overloaded calcFeetCenter)
  // Pose calcFeetCenter(const FootholdConstPtrArray& footholds) const override
  //{
  //  FootholdArray temp;
  //  for (Foothold::ConstPtr f : footholds)
  //    temp.push_back(*f);
  //  return calcFeetCenter(temp);
  //}

  /**
   * @brief Determines the static transform from the geometric feet center to the root link based on neutral stance.
   * @param feet_center feet center from which the base leveling (roll, pitch) is determined
   * @return static transfrom from geometric feet center to root link
   */
  virtual Transform calcStaticFeetCenterToRoot() const;
  virtual Transform calcStaticFeetCenterToRoot(const Pose& feet_center) const;

  /**
   * @brief Determines the transform from the feet center of the given robot configuration
   * to the root link.
   * @param feet_center feet center from which the base leveling (roll, pitch) is determined
   * @param footholds foothold configuration
   * @return transform from given feet center to root link
   */
  virtual Transform calcFeetCenterToRoot(const Pose& feet_center, const FootholdArray& footholds) const;
  virtual Transform calcFeetCenterToRoot(const Pose& feet_center, const FootholdConstPtrArray& footholds) const;
  /// If upper variant is overloaded, one should use following code snippet for sake of simplicity (calling overloaded calcFeetCenterToRoot)
  // Transform calcFeetCenterToRoot(const Pose& feet_center, const FootholdConstPtrArray& footholds) const override
  //{
  //  FootholdArray temp;
  //  for (Foothold::ConstPtr f : footholds)
  //    temp.push_back(*f);
  //  return calcFeetCenterToRoot(temp);
  //}

  /**
   * @brief Determines the static transform from the geometric feet center to the robot base based on neutral stance.
   * @param feet_center feet center from which the base leveling (roll, pitch) is determined
   * @return static transfrom from geometric feet center to base link
   */
  virtual Transform calcStaticFeetCenterToBase() const;
  virtual Transform calcStaticFeetCenterToBase(const Pose& feet_center) const;

  /**
   * @brief Determines the transform from the geometric feet center of the given robot configuration
   * to the robot base. Useful for use with calcLegIK(...). The given feet center is only used to determine
   * base leveling (roll, pitch).
   * @param feet_center feet center from which the base leveling (roll, pitch) is determined
   * @param footholds current foothold configuration
   * @return transfrom from geometric feet center to base link
   */
  virtual Transform calcFeetCenterToBase(const Pose& feet_center, const FootholdArray& footholds) const;
  virtual Transform calcFeetCenterToBase(const Pose& feet_center, const FootholdConstPtrArray& footholds) const;

  virtual Transform calcBaseToRoot() const;

  /**
   * @brief Returns the base pose based on given feet center and the static transforms.
   * @param feet_center feet center from which the base pose should be determined
   * @return base pose calculated with the static transforms
   */
  inline Pose calcStaticBasePose(const Pose& feet_center) const { return feet_center * calcStaticFeetCenterToBase(feet_center); }

  /**
   * @brief Returns the base pose based on given feet center and the dynamically determined transforms.
   * @param feet_center feet center from which the base pose should be determined
   * @param footholds current foothold configuration
   * @return base pose calculated with the dynamically determined transforms
   */
  inline Pose calcBasePose(const Pose& feet_center, const FootholdArray& footholds) const { return feet_center * calcFeetCenterToBase(feet_center, footholds); }
  inline Pose calcBasePose(const Pose& feet_center, const FootholdConstPtrArray& footholds) const { return feet_center * calcFeetCenterToBase(feet_center, footholds); }

  virtual bool calcStaticTransformForChain(const std::string& root_link, const std::string& tip_link, Transform& transform) const { return false; }

  virtual bool calcForwardKinematicsForChain(const std::string& root_link, const std::string& tip_link, const std::vector<double>& q, Pose& result) const { return false; }

  /**
   * @brief Calculates inverse kinematics
   * @param root_link
   * @param tip_link
   * @param goal target pose of tip link relative to root link
   * @param curr_q initial q
   * @param q list of joint configurations in the range [-PI; PI]. Joints are given in the same order as occuring from root to tip link.
   * @return true if valid solution was found
   */
  virtual bool calcInverseKinematicsForChain(const std::string& root_link, const std::string& tip_link, const Pose& goal, const std::vector<double>& curr_q,
                                             std::vector<double>& q) const
  {
    return false;
  }
  virtual bool calcInverseKinematicsForChain(const std::string& root_link, const std::string& tip_link, const Pose& goal, std::vector<double>& q) const { return false; }

  virtual bool calcInverseKinematicsWholeBody(const FootholdArray& footholds, std::map<std::string, double>& result) const { return false; }
  virtual bool calcInverseKinematicsWholeBody(const Pose& base_pose, const FootholdArray& footholds, std::map<std::string, double>& result) const { return false; }

  /**
   * @brief Calculates the inverse kinematics for a given leg.
   * @param base_pose Pose of the robot's base link
   * @param feet_center Pose of the geometrical feet center
   * @param foothold Target foothold location (given in planning/sole frame)
   * @param cur_q (optional) Current joint states which can improve finding a valid solution
   * @param q [out] Calculated inverse kinematics solution if found
   * @return True if a valid inverse kinematics solution was found
   */
  virtual bool calcLegIK(const Pose& base_pose, const Foothold& foothold, const std::vector<double>& cur_q, std::vector<double>& q) const;
  inline bool calcLegIK(const Pose& base_pose, const Foothold& foothold, std::vector<double>& q) const { return calcLegIK(base_pose, foothold, std::vector<double>(), q); }
  inline bool calcLegIK(const Foothold& foothold, const std::vector<double>& cur_q, std::vector<double>& q) const
  {
    return calcLegIK(calcStaticFeetCenterToBase(), foothold, cur_q, q);
  }
  inline bool calcLegIK(const Foothold& foothold, std::vector<double>& q) const { return calcLegIK(calcStaticFeetCenterToBase(), foothold, std::vector<double>(), q); }

  /**
   * @brief Computes the inverse kinematics solution for the neutral stance.
   * @param base_pose Pose of the robot's base link
   * @param leg_joint_states [out] Resulting joint states
   * @return True if a valid inverse kinematics solution was found
   */
  bool calcNeutralStanceIK(const Pose& base_pose, std::map<LegIndex, std::vector<double>>& leg_joint_states) const;
  inline bool calcNeutralStanceIK(std::map<LegIndex, std::vector<double>>& leg_joint_states) const { return calcNeutralStanceIK(calcStaticFeetCenterToBase(), leg_joint_states); }

protected:
  RobotDescription::ConstPtr robot_description_;

  std::string root_link_;  // root link of chain

  bool leveled_base_;  // indicates if base is leveled (base center is over feet center)

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace l3

#endif
