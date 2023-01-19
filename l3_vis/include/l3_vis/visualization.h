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

#ifndef L3_VIS_VISUALIZATION_H__
#define L3_VIS_VISUALIZATION_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <l3_libs/types/types.h>
#include <l3_libs/robot_description/robot_description.h>

namespace l3
{
std_msgs::ColorRGBA createColorMsg(double r, double g, double b, double a = 1.0);

/**
 * @brief Generates lerp color in range of start and goal based on given value [0;1].
 * @param start color for minimal value (0)
 * @param end color for maximal value (1)
 * @param value value to interpolate within; must be in range [0;1]
 * @return lerped color
 */
std_msgs::ColorRGBA getColorScale(const std_msgs::ColorRGBA& start, const std_msgs::ColorRGBA& end, double value);

visualization_msgs::Marker createResetMarker(const std::string& ns = "");

/**
 * @brief Updates a marker array. All markers will be overwritten. If the update has less markers than the old array,
 * all trailing markers will be marked as deleted.
 * Note: All markers must be given with a strictly ordered id starting with 0.
 * @param markers Input marker array to be updated
 * @param update The update marker array (must point to a different address as the original markers!)
 */
void updateMarkerArray(visualization_msgs::MarkerArray& markers, const visualization_msgs::MarkerArray& update);

/**
 * @brief Appends a marker array to another.
 * @param markers Marker array to append to
 * @param update Marker array which should be appended
 * @param up_conting Overwrites marker ids of update marker array in a up-counting fashion starting with the input marker array size
 */
void appendMarkerArray(visualization_msgs::MarkerArray& markers, const visualization_msgs::MarkerArray& update, bool up_counting = false);

void removeDeletedMarkers(visualization_msgs::MarkerArray& markers);

visualization_msgs::Marker footToFootMarker(const l3_msgs::Foothold& foot, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns = "foot");

inline visualization_msgs::Marker footToFootMarker(const l3_msgs::Foothold& foot, const RobotDescription& robot_description, const std::string& ns = "foot")
{
  return footToFootMarker(foot, robot_description, robot_description.getFootInfo(foot.idx).color, ns);
}

visualization_msgs::Marker baseToBaseMarker(const l3_msgs::FloatingBase& base, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns = "floating_base");

inline visualization_msgs::Marker baseToBaseMarker(const l3_msgs::FloatingBase& base, const RobotDescription& robot_description, const std::string& ns = "floating_base")
{
  BaseInfo base_info = robot_description.getBaseInfo(BaseInfo::MAIN_BODY_IDX);
  return baseToBaseMarker(base, robot_description, base_info.color, ns);
}

visualization_msgs::MarkerArray feetToFootMarkerArray(const l3_msgs::FootholdArray& feet, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns = "foot");
visualization_msgs::MarkerArray feetToFootMarkerArray(const l3_msgs::FootholdArray& feet, const RobotDescription& robot_description, const std::string& ns = "foot");

inline visualization_msgs::Marker footStepDataToFootMarker(const l3_msgs::FootStepData& step, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns = "foot")

{
  return footToFootMarker(step.target, robot_description, color, ns);
}

inline visualization_msgs::Marker footStepDataToFootMarker(const l3_msgs::FootStepData& step, const RobotDescription& robot_description, const std::string& ns = "foot")
{
  return footToFootMarker(step.target, robot_description, ns);
}

visualization_msgs::MarkerArray stepToFootMarkerArray(const l3_msgs::Step& step, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns = "step");
visualization_msgs::MarkerArray stepToFootMarkerArray(const l3_msgs::Step& step, const RobotDescription& robot_description, const std::string& ns = "step");

visualization_msgs::MarkerArray stepPlanToFootMarkerArray(const l3_msgs::StepArray& steps, const RobotDescription& robot_description, bool add_step_index = true, const std::string& ns = "step_plan");

visualization_msgs::Marker feetToUpperBodyMarker(const l3_msgs::FootholdArray& feet, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, bool flat = false, const std::string& ns = "upper_body");

visualization_msgs::MarkerArray stepPlanToUpperBodyMarkerArray(const l3_msgs::FootholdArray& start, const l3_msgs::StepArray& steps, const RobotDescription& robot_description, bool add_step_index = true, const std::string& ns = "upper_body");

}  // namespace l3

#endif
