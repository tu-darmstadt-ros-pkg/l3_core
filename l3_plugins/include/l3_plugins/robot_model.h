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

#ifndef L3_ROBOT_MODEL_H__
#define L3_ROBOT_MODEL_H__

#include <ros/ros.h>

#include <boost/noncopyable.hpp>

#include <vigir_pluginlib/plugin_manager.h>

#include <l3_msgs/RobotModel.h>

#include <l3_libs/types/types.h>
#include <l3_libs/singleton.h>
#include <l3_libs/robot_description/robot_description.h>

#include <l3_plugins/base/kinematics_plugin.h>
#include <l3_plugins/base/dynamics_plugin.h>
#include <l3_plugins/base/gait_generator_plugin.h>

namespace l3
{
class RobotModel : public Singleton<RobotModel>
{
public:
  RobotModel();

  /**
   * @brief Initialize robot model
   */
  static bool initialize(const XmlRpc::XmlRpcValue& params = XmlRpc::XmlRpcValue());
  static bool initialize(ros::NodeHandle& nh, const std::string& topic = "robot_model");

  void fromMsg(const l3_msgs::RobotModel& msg);

  void toMsg(l3_msgs::RobotModel& msg) const;

  inline l3_msgs::RobotModel toMsg() const
  {
    l3_msgs::RobotModel msg;
    toMsg(msg);
    return msg;
  }

  /**
   * @brief Fetches all plugins from plugin manager for efficient access using getters.
   * In case of reloading plugins, just recall this method.
   * @return true when all required plugins has been fetched successfully
   */
  static bool loadPlugins();

  static bool loadKinematicsPlugin();
  static bool loadDynamicsPlugin();
  static bool loadGaitGeneratorPlugin();

  static void printPluginSummary();

  /** Note: Using this pointers is not thread-safe. When new model is published (should not happen!) while these pointers are used, unexpected results may occur. */
  static RobotDescription::ConstPtr description() { return instance().robot_description_; }

  static KinematicsPlugin::Ptr kinematics() { return instance().kinematics_; }
  static DynamicsPlugin::Ptr dynamics() { return instance().dynamics_; }

  static GaitGeneratorPlugin::Ptr gaitGenerator() { return instance().gait_generator_; }

  /**
   * @brief Generates neutral stance on given robot pose.
   * @param center Robot's center from which the neutral stance is derived
   * @return Neutral stance as array of footholds
   */
  static inline FootholdPtrArray getNeutralStance(const Pose& center = Pose()) { return instance().robot_description_->getNeutralStance(center); }

  /**
   * @brief Calculates robot's feet center based on the given foothold configuration.
   * The request is forwarded to the kinematics plugin if available, otherwise the
   * naive implementation is used.
   * @param footholds foothold configuration
   * @return robot's base pose
   */
  static Pose calcFeetCenter(const FootholdArray& footholds);
  static Pose calcFeetCenter(const FootholdConstPtrArray& footholds);

protected:
  bool parseConfig(XmlRpc::XmlRpcValue& params);

  // mutex to ensure thread safeness
  mutable Mutex mutex_;

  bool is_initialized_;

  // robot configuration
  RobotDescription::ConstPtr robot_description_;

  // plugins
  KinematicsPlugin::Ptr kinematics_;
  DynamicsPlugin::Ptr dynamics_;

  GaitGeneratorPlugin::Ptr gait_generator_;
};
}  // namespace l3

#endif
