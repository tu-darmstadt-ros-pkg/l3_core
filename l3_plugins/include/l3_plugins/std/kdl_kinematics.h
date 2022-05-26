//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf TU Darmstadt
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

#ifndef L3_KDL_KINEMATICS_H__
#define L3_KDL_KINEMATICS_H__

#include <ros/ros.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <l3_plugins/base/kinematics_plugin.h>

namespace l3
{
class KdlKinematics : public KinematicsPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<KdlKinematics> Ptr;
  typedef l3::SharedPtr<const KdlKinematics> ConstPtr;

  using ChainFkSolverPosPtr = SharedPtr<KDL::ChainFkSolverPos>;
  using ChainIkSolverPosPtr = SharedPtr<KDL::ChainIkSolverPos>;
  using ChainIkSolverVelPtr = SharedPtr<KDL::ChainIkSolverVel>;

  KdlKinematics(const std::string& name = "kdl_kinematics");

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;
  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool getMassOfChain(const std::string& start_link_id, double& mass) const override;
  bool getMassOfLink(const std::string& link_id, double& mass) const override;
  bool calcTotalMass(double& result) const override;

  bool calcCenterOfMass(const std::string& frame_link, Vector3& com) const override;

  Transform calcStaticFeetCenterToRoot() const override { return center_to_root_; }

  Transform calcFeetCenterToRoot(const Pose& feet_center, const FootholdArray& footholds) const override;
  Transform calcFeetCenterToRoot(const Pose& feet_center, const FootholdConstPtrArray& footholds) const override;

  bool calcStaticTransformForChain(const std::string& root_link, const std::string& tip_link, Transform& transform) const override;

  bool calcForwardKinematicsForChain(const std::string& root_link, const std::string& tip_link, const std::vector<double>& q, Pose& result) const override;

  bool calcInverseKinematicsForChain(const std::string& root_link, const std::string& tip_link, const Pose& goal, const std::vector<double>& curr_q, std::vector<double>& q) const override;
  bool calcInverseKinematicsForChain(const std::string& root_link, const std::string& tip_link, const Pose& goal, std::vector<double>& q) const override;

  bool calcInverseKinematicsWholeBody(const Pose& body_pose, const FootholdArray& footholds, std::map<std::string, double>& result) const override;
  bool calcInverseKinematicsWholeBody(const FootholdArray& footholds, std::map<std::string, double>& result) const override;

private:
  double calcTotalMassRecursive(const KDL::TreeElement& root) const;
  bool calcTransformToLink(const std::string& root_link, const std::string& tip_link, KDL::Frame& transform) const;

  std::map<std::string, KDL::TreeElement> all_links_;
  KDL::Tree tree_;

  // parameters
  Transform center_to_root_;  // transformation from geometric center of feet to root link
  bool ignore_foot_orientation_;
};
}  // namespace l3

#endif
