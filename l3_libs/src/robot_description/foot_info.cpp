#include <l3_libs/robot_description/foot_info.h>

#include <ros/package.h>

#include <l3_libs/yaml_parser.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3
{
FootInfo::FootInfo()
  : idx(-1)
  , name("INVALID")
  , shape(Shape::CUBOID)
  , link("NO_LINK_SPECIFIED")
  , mesh_scale(1.0, 1.0, 1.0)
{
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 0.6;
}

FootInfo::FootInfo(const XmlRpc::XmlRpcValue& params)
  : FootInfo()
{
  if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("FootInfo", "[FootInfo] Foot parameter has wrong format.");
    return;
  }

  XmlRpc::XmlRpcValue p = params;

  if (!getYamlValue(p, "idx", idx))
    return;

  if (!getYamlValue(p, "name", name))
    return;

  if (!getYamlValue(p, "shape", shape))
    return;

  if (!getYamlValue(p, "size", size))
    return;

  if (!getYamlValue(p, "link", link))
    return;

  if (!getYamlValue(p, "link_to_sole_offset", link_to_sole_offset))
    return;

  if (p.hasMember("indirect"))
    getYamlValue(p, "indirect", indirect);
  else
  {
    indirect = false;

    if (!getYamlValue(p, "neutral_stance", neutral_stance))
      return;
  }

  // parse vis section (optional)
  XmlRpc::XmlRpcValue vis = p["vis"];
  if (vis.valid())
  {
    if (vis.hasMember("color"))
      getYamlValue(vis, "color", color);

    if (vis.hasMember("mesh_resource"))
      getYamlValue(vis, "mesh_resource", mesh_resource);

    if (vis.hasMember("mesh_offset"))
      getYamlValue(vis, "mesh_offset", mesh_offset);

    if (vis.hasMember("mesh_scale"))
      getYamlValue(vis, "mesh_scale", mesh_scale);
  }
}

FootInfo::FootInfo(const l3_msgs::FootInfo& msg) { fromMsg(msg); }

void FootInfo::fromMsg(const l3_msgs::FootInfo& msg)
{
  idx = msg.idx;
  shape = static_cast<FootInfo::Shape>(msg.shape);
  name = msg.name;
  vectorMsgToL3(msg.size, size);
  link = msg.link;
  poseMsgToL3(msg.link_to_sole_offset, link_to_sole_offset);

  poseMsgToL3(msg.neutral_stance, neutral_stance);

  indirect = msg.indirect;

  color = msg.color;
  mesh_resource = msg.mesh_resource;
  poseMsgToL3(msg.mesh_offset, mesh_offset);
  vectorMsgToL3(msg.mesh_scale, mesh_scale);
}

void FootInfo::toMsg(l3_msgs::FootInfo& msg) const
{
  msg.idx = idx;
  msg.shape = shape;
  msg.name = name;
  vectorL3ToMsg(size, msg.size);
  msg.link = link;
  poseL3ToMsg(link_to_sole_offset, msg.link_to_sole_offset);

  poseL3ToMsg(neutral_stance, msg.neutral_stance);

  msg.indirect = indirect;

  msg.color = color;
  msg.mesh_resource = mesh_resource;
  poseL3ToMsg(mesh_offset, msg.mesh_offset);
  vectorL3ToMsg(mesh_scale, msg.mesh_scale);
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, FootInfo::Shape& val)
{
  XmlRpc::XmlRpcValue& shape = p[key];

  if (!shape.valid())
  {
    ROS_ERROR("Parameter '%s' does not exist!", key.c_str());
    return false;
  }

  // read from array
  if (shape.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    int val_int = static_cast<int>(shape);

    if (val_int != 0 || val_int != 1)
    {
      ROS_ERROR("Parameter '%s' declares unknown Shape type '%i'!", key.c_str(), val_int);
      return false;
    }

    val = static_cast<FootInfo::Shape>(val_int);
    return true;
  }
  // read from struct
  else if (shape.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    std::string val_string = static_cast<std::string>(shape);
    std::transform(val_string.begin(), val_string.end(), val_string.begin(), ::tolower);

    if (val_string == "cuboid")
    {
      val = FootInfo::CUBOID;
      return true;
    }
    else if (val_string == "spherical")
    {
      val = FootInfo::SPHERICAL;
      return true;
    }
    else
    {
      ROS_ERROR("Parameter '%s' declares unknown Shape type '%s'!", key.c_str(), val_string.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("Parameter '%s' is from type '%s' which cannot be converted to l3::FootInfo::Shape.", key.c_str(), toString(shape.getType()).c_str());
    return false;
  }

  return false;
}
}  // namespace l3
