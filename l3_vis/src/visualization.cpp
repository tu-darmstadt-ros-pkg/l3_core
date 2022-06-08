#include <l3_vis/visualization.h>

#include <l3_libs/conversions/l3_msg_conversions.h>
#include <l3_libs/helper.h>

#include <l3_math/math.h>

namespace l3
{
std_msgs::ColorRGBA createColorMsg(double r, double g, double b, double a)
{
  std_msgs::ColorRGBA result;
  result.r = static_cast<float>(r);
  result.g = static_cast<float>(g);
  result.b = static_cast<float>(b);
  result.a = static_cast<float>(a);
  return result;
}

std_msgs::ColorRGBA getColorScale(const std_msgs::ColorRGBA& start, const std_msgs::ColorRGBA& end, double value)
{
  // For second half of color range move towards RED
  if (value <= 0.0)
    return start;
  else if (value >= 1.0)
    return end;

  std_msgs::ColorRGBA result;
  result.r = lerp(start.r, end.r, 1.0, value);
  result.g = lerp(start.g, end.g, 1.0, value);
  result.b = lerp(start.b, end.b, 1.0, value);
  result.a = lerp(start.a, end.a, 1.0, value);

  return result;
}

visualization_msgs::Marker createResetMarker(const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

void updateMarkerArray(visualization_msgs::MarkerArray& markers, const visualization_msgs::MarkerArray& update)
{
  // resize marker array
  if (markers.markers.size() < update.markers.size())
    markers.markers.resize(update.markers.size());

  // overwrite old markers
  for (size_t i = 0; i < update.markers.size(); i++)
    markers.markers[i] = update.markers[i];

  // set needless markers to be deleted
  for (size_t i = update.markers.size(); i < markers.markers.size(); i++)
    markers.markers[i].action = visualization_msgs::Marker::DELETE;
}

void appendMarkerArray(visualization_msgs::MarkerArray& markers, const visualization_msgs::MarkerArray& update, bool up_counting)
{
  if (up_counting)
  {
    for (visualization_msgs::Marker marker : update.markers)
    {
      marker.id = static_cast<int>(markers.markers.size());
      markers.markers.push_back(marker);
    }
  }
  else
    markers.markers.insert(markers.markers.end(), update.markers.begin(), update.markers.end());
}

void removeDeletedMarkers(visualization_msgs::MarkerArray& markers)
{
  for (std::vector<visualization_msgs::Marker>::iterator itr = markers.markers.begin(); itr != markers.markers.end();)
  {
    if (itr->action == visualization_msgs::Marker::DELETE)
      itr = markers.markers.erase(itr);
    else
      itr++;
  }
}

visualization_msgs::Marker footToFootMarker(const l3_msgs::Foothold& foot, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;

  marker.header = foot.header;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;

  FootInfo foot_info = robot_description.getFootInfo(foot.idx);

  marker.color = color;

  Pose pose;
  poseMsgToL3(foot.pose, pose);

  if (!foot_info.mesh_resource.empty())
  {
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = foot_info.mesh_resource;

    pose = pose * foot_info.mesh_offset;
    poseL3ToMsg(pose, marker.pose);

    vectorL3ToMsg(foot_info.mesh_scale, marker.scale);
  }
  else if (foot_info.shape == FootInfo::Shape::CUBOID)
  {
    marker.type = visualization_msgs::Marker::CUBE;

    Pose offset(0.0, 0.0, 0.5 * foot_info.size.z());
    pose = pose * offset * foot_info.mesh_offset;
    poseL3ToMsg(pose, marker.pose);

    vectorL3ToMsg(foot_info.size, marker.scale);
  }
  else if (foot_info.shape == FootInfo::Shape::SPHERICAL)
  {
    marker.type = visualization_msgs::Marker::SPHERE;

    Pose offset(0.0, 0.0, 0.5 * foot_info.size.z());
    pose = pose * offset * foot_info.mesh_offset;
    poseL3ToMsg(pose, marker.pose);

    vectorL3ToMsg(foot_info.size, marker.scale);
  }
  else
  {
    ROS_ERROR("Unknown foot shape type '%u'!", foot_info.shape);
  }

  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker baseToBaseMarker(const l3_msgs::FloatingBase& base, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;

  marker.header = base.header;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;

  BaseInfo base_info = robot_description.getBaseInfo(BaseInfo::MAIN_BODY_IDX);

  marker.color = color;

  Pose pose;
  poseMsgToL3(base.pose, pose);

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  pose = pose * base_info.mesh_offset;
  poseL3ToMsg(pose, marker.pose);

  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::MarkerArray feetToFootMarkerArray(const l3_msgs::FootholdArray& feet, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color,
                                                      const std::string& ns)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;

  for (const l3_msgs::Foothold& foot : feet)
  {
    marker = footToFootMarker(foot, robot_description, color, ns);
    marker.id = static_cast<int>(markers.markers.size());
    markers.markers.push_back(marker);
  }

  return markers;
}

visualization_msgs::MarkerArray feetToFootMarkerArray(const l3_msgs::FootholdArray& feet, const RobotDescription& robot_description, const std::string& ns)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;

  for (const l3_msgs::Foothold& foot : feet)
  {
    marker = footToFootMarker(foot, robot_description, ns);
    marker.id = static_cast<int>(markers.markers.size());
    markers.markers.push_back(marker);
  }

  return markers;
}

visualization_msgs::MarkerArray stepToFootMarkerArray(const l3_msgs::Step& step, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;

  for (const l3_msgs::StepData& step_data : step.step_data)
  {
    marker = stepDataToFootMarker(step_data, robot_description, color, ns);
    marker.id = static_cast<int>(markers.markers.size());
    markers.markers.push_back(marker);
  }

  return markers;
}

visualization_msgs::MarkerArray stepToFootMarkerArray(const l3_msgs::Step& step, const RobotDescription& robot_description, const std::string& ns)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;

  for (const l3_msgs::StepData& step_data : step.step_data)
  {
    marker = stepDataToFootMarker(step_data, robot_description, ns);
    marker.id = static_cast<int>(markers.markers.size());
    markers.markers.push_back(marker);
  }

  return markers;
}

visualization_msgs::MarkerArray stepPlanToFootMarkerArray(const l3_msgs::StepArray& steps, const RobotDescription& robot_description, bool add_step_index, const std::string& ns)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;

  for (const l3_msgs::Step& step : steps)
  {
    l3_msgs::FootholdArray footholds;

    // check if step data is available for visualization, otherwise use supporting legs
    if (step.step_data.empty())
    {
      footholds = step.support;
    }
    else
    {
      for (const l3_msgs::StepData& step_data : step.step_data)
        footholds.push_back(step_data.target);
    }

    // handle footholds
    for (const l3_msgs::Foothold& fh : footholds)
    {
      if (robot_description.isIndirectFoot(fh.idx))
        continue;

      // transform
      marker = footToFootMarker(fh, robot_description, ns + "_footholds");
      marker.id = static_cast<int>(markers.markers.size());
      markers.markers.push_back(marker);

      // add text
      if (add_step_index)
      {
        FootInfo foot_info = robot_description.getFootInfo(fh.idx);

        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = boost::lexical_cast<std::string>(step.idx);
        marker.mesh_resource.clear();

        vectorL3ToMsg(foot_info.size, marker.scale);
        marker.scale.x = 0.0;
        marker.scale.y = 0.0;
        if (marker.type != visualization_msgs::Marker::SPHERE)
          marker.scale.z *= 2;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.7f;

        marker.id = static_cast<int>(markers.markers.size());
        markers.markers.push_back(marker);
      }
    }

    l3_msgs::FloatingBaseArray floating_bases;

    // check if moving base is available for visualization, otherwise use resting base
    if (step.moving_bases.empty())
    {
      floating_bases = step.resting_bases;
    }
    else
    {
      for (const l3_msgs::BaseStepData& base_step_data : step.moving_bases)
        floating_bases.push_back(base_step_data.target);
    }

    // handle floating bases
    for (const l3_msgs::FloatingBase& fb : floating_bases)
    {
      // transform
      marker = baseToBaseMarker(fb, robot_description, ns + "_floating_base");
      marker.id = static_cast<int>(markers.markers.size());
      markers.markers.push_back(marker);
    }
  }

  return markers;
}

visualization_msgs::Marker feetToUpperBodyMarker(const l3_msgs::FootholdArray& feet, const RobotDescription& robot_description, const std_msgs::ColorRGBA& color, bool flat,
                                                 const std::string& ns)
{
  visualization_msgs::Marker marker;

  if (feet.empty())
    return marker;

  marker.header = feet.front().header;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  // approximate base position
  FootholdArray feet_l3;
  footholdArrayMsgToL3(feet, feet_l3);
  /// @todo Use RobotModel here?
  Pose pose = calcFeetCenter(feet_l3);
  poseL3ToMsg(pose, marker.pose);

  // determine shift of polygon based on orientation
  BaseInfo base_info = robot_description.getBaseInfo(BaseInfo::MAIN_BODY_IDX);
  Pose shifted = pose * base_info.mesh_offset;

  marker.pose.position.x = shifted.x();
  marker.pose.position.y = shifted.y();
  marker.pose.position.z = shifted.z();

  // finalize marker
  const Vector3& base_size = base_info.size;

  marker.pose.position.z += flat ? 0.01 : 0.5 * base_size.z();
  vectorL3ToMsg(base_size, marker.scale);
  if (flat)
    marker.scale.z = 0.02;
  marker.color = color;

  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::MarkerArray stepPlanToUpperBodyMarkerArray(const l3_msgs::FootholdArray& start, const l3_msgs::StepArray& steps, const RobotDescription& robot_description,
                                                               bool add_step_index, const std::string& ns)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;

  BaseInfo base_info = robot_description.getBaseInfo(BaseInfo::MAIN_BODY_IDX);
  const std_msgs::ColorRGBA& color = base_info.color;

  // init robot state
  std::map<FootIndex, l3_msgs::Foothold> feet_map;
  for (const l3_msgs::Foothold& f : start)
    feet_map[f.idx] = f;

  for (const l3_msgs::Step& step : steps)
  {
    // update robot state
    for (const l3_msgs::StepData& step_data : step.step_data)
      feet_map[step_data.target.idx] = step_data.target;

    // extract current footholds
    l3_msgs::FootholdArray feet;
    for (const std::pair<FootIndex, l3_msgs::Foothold>& p : feet_map)
      feet.push_back(p.second);

    // transform
    /// @todo Check if floating base exists and use it instead
    marker = feetToUpperBodyMarker(feet, robot_description, color, false, ns);
    marker.id = static_cast<int>(markers.markers.size());
    markers.markers.push_back(marker);

    // add text
    if (add_step_index)
    {
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = boost::lexical_cast<std::string>(step.idx);

      marker.scale.x = 0.0;
      marker.scale.y = 0.0;
      marker.scale.z *= 0.1;

      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.7f;

      marker.id = static_cast<int>(markers.markers.size());
      markers.markers.push_back(marker);
    }
  }

  return markers;
}
}  // namespace l3
