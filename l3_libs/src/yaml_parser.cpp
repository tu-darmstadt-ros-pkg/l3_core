#include <l3_libs/yaml_parser.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3
{
bool getYamlValue(XmlRpc::XmlRpcValue& p, Vector3& val)
{
  if (!p.valid())
  {
    ROS_ERROR("Parameter does not exist!");
    return false;
  }

  // read from array
  if (p.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (p.size() != 3)
    {
      ROS_ERROR("Parameter must have length of 3!");
      return false;
    }

    for (size_t i = 0; i < 3; i++)
    {
      if (p[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        ROS_ERROR("Value at %lu is from type '%s' (should be Double)!", i, toString(p[i].getType()).c_str());
        return false;
      }
    }

    val = Vector3(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
    return true;
  }
  // read from struct
  else if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    double x, y, z;

    if (!getYamlValue(p, "x", x))
      return false;
    if (!getYamlValue(p, "y", y))
      return false;
    if (!getYamlValue(p, "z", z))
      return false;

    val = Vector3(x, y, z);
    return true;
  }
  else
  {
    ROS_ERROR("Parameter is from type '%s' which cannot be converted to l3::Vector3.", toString(p.getType()).c_str());
    return false;
  }

  return false;
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, Pose& val)
{
  if (!p.valid())
  {
    ROS_ERROR("Parameter does not exist!");
    return false;
  }

  // read from array
  if (p.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (p.size() != 6)
    {
      ROS_ERROR("Parameter must have length of 6!");
      return false;
    }

    for (size_t i = 0; i < 6; i++)
    {
      if (p[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        ROS_ERROR("Value at %lu is from type '%s' (should be Double)!", i, toString(p[i].getType()).c_str());
        return false;
      }
    }

    val = Pose(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]), static_cast<double>(p[3]), static_cast<double>(p[4]), static_cast<double>(p[5]));
    return true;
  }
  // read from struct
  else if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    double x, y, z, roll, pitch, yaw;

    if (!getYamlValue(p, "x", x))
      return false;
    if (!getYamlValue(p, "y", y))
      return false;
    if (!getYamlValue(p, "z", z))
      return false;
    if (!getYamlValue(p, "roll", roll))
      return false;
    if (!getYamlValue(p, "pitch", pitch))
      return false;
    if (!getYamlValue(p, "yaw", yaw))
      return false;

    val = Pose(x, y, z, roll, pitch, yaw);
    return true;
  }
  else
  {
    ROS_ERROR("Parameter is from type '%s' which cannot be converted to l3::Pose.", toString(p.getType()).c_str());
    return false;
  }

  return false;
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, FootIndexArray& val)
{
  val.clear();

  std::vector<int> array;
  if (!getYamlValue(p, array))
    return false;

  for (int i : array)
    val.push_back(static_cast<FootIndex>(i));

  return true;
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, MultiFootIndexArray& val)
{
  if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Parameter has wrong type! Expected '%s' but got '%s'.", toString(XmlRpc::XmlRpcValue::TypeArray).c_str(), toString(p.getType()).c_str());
    return false;
  }

  val.clear();
  for (int i = 0; i < p.size(); i++)
  {
    FootIndexArray foot_idx;
    if (!getYamlValue(p[i], foot_idx))
      return false;
    val.push_back(foot_idx);
  }

  return true;
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, std_msgs::ColorRGBA& val)
{
  if (!p.valid())
  {
    ROS_ERROR("Parameter does not exist!");
    return false;
  }

  // read from array
  if (p.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (p.size() != 4)
    {
      ROS_ERROR("Parameter must have length of 4!");
      return false;
    }

    for (size_t i = 0; i < 4; i++)
    {
      if (p[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        ROS_ERROR("Value at %lu is from type '%s' (should be '%s')!", i, toString(p[i].getType()).c_str(), toString(XmlRpc::XmlRpcValue::TypeDouble).c_str());
        return false;
      }
    }

    val.r = static_cast<double>(p[0]);
    val.g = static_cast<double>(p[1]);
    val.b = static_cast<double>(p[2]);
    val.a = static_cast<double>(p[3]);

    return true;
  }
  // read from struct
  else if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (!getYamlValue(p, "r", val.r))
      return false;
    if (!getYamlValue(p, "g", val.g))
      return false;
    if (!getYamlValue(p, "b", val.b))
      return false;
    if (!getYamlValue(p, "a", val.a))
      return false;

    return true;
  }
  else
  {
    ROS_ERROR("Parameter is from type '%s' which cannot be converted to std_msgs::ColorRGBA.", toString(p.getType()).c_str());
    return false;
  }

  return false;
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, geometry_msgs::Vector3& val)
{
  Vector3 val_l3;
  if (getYamlValue(p, val_l3))
  {
    vectorL3ToMsg(val_l3, val);
    return true;
  }
  else
    return false;
}

bool getYamlValue(XmlRpc::XmlRpcValue& p, geometry_msgs::Twist& val)
{
  if (!p.valid())
  {
    ROS_ERROR("Parameter does not exist!");
    return false;
  }

  // read from array
  if (p.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (p.size() != 6)
    {
      ROS_ERROR("Parameter must have length of 6!");
      return false;
    }

    for (size_t i = 0; i < 6; i++)
    {
      if (p[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        ROS_ERROR("Value at %lu is from type '%s' (should be '%s')!", i, toString(p[i].getType()).c_str(), toString(XmlRpc::XmlRpcValue::TypeDouble).c_str());
        return false;
      }
    }

    val.linear.x = static_cast<double>(p[0]);
    val.linear.y = static_cast<double>(p[1]);
    val.linear.z = static_cast<double>(p[2]);
    val.angular.x = static_cast<double>(p[3]);
    val.angular.y = static_cast<double>(p[4]);
    val.angular.z = static_cast<double>(p[5]);

    return true;
  }
  else
  {
    ROS_ERROR("Parameter is from type '%s' which cannot be converted to geometry_msgs::Twist.", toString(p.getType()).c_str());
    return false;
  }

  return false;
}
}  // namespace l3
