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

#ifndef L3_YAML_PARSER_H__
#define L3_YAML_PARSER_H__

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/ColorRGBA.h>

#include <vigir_generic_params/generic_params_msgs.h>

#include <l3_libs/helper.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/foothold.h>

namespace l3
{
template <typename T>
bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, const XmlRpc::XmlRpcValue::Type& type, T& val)
{
  if (!p.valid())
    return false;

  if (!p.hasMember(key))
  {
    ROS_ERROR("Parameter '%s' does not exist!", key.c_str());
    return false;
  }

  return getYamlValue(p[key], type, val);
}

template <typename T>
bool getYamlValue(XmlRpc::XmlRpcValue& p, const XmlRpc::XmlRpcValue::Type& type, T& val)
{
  if (!p.valid())
    return false;

  if (p.getType() != type)
  {
    ROS_ERROR("Parameter has wrong type! Expected '%s' but got '%s'.", toString(type).c_str(), toString(p.getType()).c_str());
    return false;
  }

  val = static_cast<T>(p);
  return true;
}

template <typename T>
bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, const XmlRpc::XmlRpcValue::Type& type, std::vector<T>& val)
{
  if (!p.valid())
    return false;

  if (!p.hasMember(key))
  {
    ROS_ERROR("Parameter '%s' does not exist!", key.c_str());
    return false;
  }

  if (p[key].getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Parameter '%s' has wrong type! Expected '%s' but got '%s'.", key.c_str(), toString(XmlRpc::XmlRpcValue::TypeArray).c_str(), toString(p[key].getType()).c_str());
    return false;
  }

  return getYamlValue(p[key], type, val);
}

template <typename T>
bool getYamlValue(XmlRpc::XmlRpcValue& p, const XmlRpc::XmlRpcValue::Type& type, std::vector<T>& val)
{
  if (!p.valid())
    return false;

  // read from array
  if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Parameter has wrong type! Expected '%s' but got '%s'.", toString(XmlRpc::XmlRpcValue::TypeArray).c_str(), toString(p.getType()).c_str());
    return false;
  }

  val.clear();

  for (int i = 0; i < p.size(); i++)
  {
    if (p[i].getType() != type)
    {
      ROS_ERROR("Parameter at index %i has wrong type! Expected '%s' but got '%s'.", i, toString(XmlRpc::XmlRpcValue::TypeArray).c_str(), toString(p[i].getType()).c_str());
      return false;
    }
    val.push_back(static_cast<T>(p[i]));
  }

  return true;
}

// std types
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, bool& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeBoolean, val); }
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, int& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeInt, val); }
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, float& val)
{
  double val_d;
  bool result = getYamlValue(p, XmlRpc::XmlRpcValue::TypeDouble, val_d);
  val = val_d;
  return result;
}
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, double& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeDouble, val); }
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, std::string& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeString, val); }

inline bool getYamlValue(XmlRpc::XmlRpcValue& p, std::vector<bool>& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeBoolean, val); }
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, std::vector<int>& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeInt, val); }
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, std::vector<double>& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeDouble, val); }
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, std::vector<std::string>& val) { return getYamlValue(p, XmlRpc::XmlRpcValue::TypeString, val); }

// l3 types
bool getYamlValue(XmlRpc::XmlRpcValue& p, Vector3& val);
bool getYamlValue(XmlRpc::XmlRpcValue& p, Pose& val);
inline bool getYamlValue(XmlRpc::XmlRpcValue& p, Transform& val) { return getYamlValue(p, static_cast<Pose&>(val)); }

inline bool getYamlValue(XmlRpc::XmlRpcValue& p, FootIndex& val) { return getYamlValue(p, (int&)val); }

bool getYamlValue(XmlRpc::XmlRpcValue& p, FootIndexArray& val);
bool getYamlValue(XmlRpc::XmlRpcValue& p, MultiFootIndexArray& val);

// ros msg types
bool getYamlValue(XmlRpc::XmlRpcValue& p, std_msgs::ColorRGBA& val);
bool getYamlValue(XmlRpc::XmlRpcValue& p, geometry_msgs::Vector3& val);
bool getYamlValue(XmlRpc::XmlRpcValue& p, geometry_msgs::Twist& val);

// templates in order to prevent code duplication (otherwise specialized version getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, T& val)
// would be required for each type
template <typename T>
bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, T& val)
{
  if (!p.valid())
    return false;

  if (!p.hasMember(key))
  {
    ROS_ERROR("Parameter '%s' does not exist!", key.c_str());
    return false;
  }

  return getYamlValue(p[key], val);
}

template <typename T>
bool getYamlValue(XmlRpc::XmlRpcValue& p, const std::string& key, std::vector<T>& val)
{
  if (!p.valid())
    return false;

  if (!p.hasMember(key))
  {
    ROS_ERROR("Parameter '%s' does not exist!", key.c_str());
    return false;
  }

  if (p[key].getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Parameter '%s' has wrong type! Expected '%s' but got '%s'.", key.c_str(), toString(XmlRpc::XmlRpcValue::TypeArray).c_str(), toString(p[key].getType()).c_str());
    return false;
  }

  return getYamlValue(p[key], val);
}
}  // namespace l3

#endif
