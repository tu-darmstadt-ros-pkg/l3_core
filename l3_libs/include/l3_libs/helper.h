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

#ifndef L3_LIBS_HELPER_H__
#define L3_LIBS_HELPER_H__

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <vigir_generic_params/generic_params_msgs.h>

#include <l3_libs/types/memory.h>

namespace l3
{
/**
 * ActionClient and ActionServer wrapper
 */
template <typename ActionSpec>
class SimpleActionClient : public actionlib::SimpleActionClient<ActionSpec>
{
public:
  // typedefs
  typedef SharedPtr<SimpleActionClient> Ptr;
  typedef SharedPtr<const SimpleActionClient> ConstPtr;

  SimpleActionClient(ros::NodeHandle nh, std::string name, bool spin_thread = true)
    : actionlib::SimpleActionClient<ActionSpec>(nh, name, spin_thread)
  {}

  static Ptr create(ros::NodeHandle nh, std::string name, bool spin_thread = true) { return Ptr(new SimpleActionClient<ActionSpec>(nh, name, spin_thread)); }
};

template <typename ActionSpec>
class SimpleActionServer : public actionlib::SimpleActionServer<ActionSpec>
{
public:
  // typedefs
  typedef SharedPtr<SimpleActionServer<ActionSpec>> Ptr;
  typedef SharedPtr<const SimpleActionServer<ActionSpec>> ConstPtr;
  typedef boost::function<void()> ExecuteCallback;
  typedef boost::function<void()> PreemptCallback;

  SimpleActionServer(ros::NodeHandle nh, std::string name, bool auto_start)
    : actionlib::SimpleActionServer<ActionSpec>(nh, name, auto_start)
  {}

  void preempt() { actionlib::SimpleActionServer<ActionSpec>::setPreempted(); }

  static Ptr create(ros::NodeHandle nh, std::string name, bool auto_start, ExecuteCallback execute_cb, PreemptCallback preempt_cb = PreemptCallback())
  {
    Ptr as(new SimpleActionServer<ActionSpec>(nh, name, false));

    as->registerGoalCallback(execute_cb);

    if (!preempt_cb.empty())
      preempt_cb = boost::bind(&SimpleActionServer::preempt, as.get());
    as->registerPreemptCallback(preempt_cb);

    if (auto_start)
      as->start();

    return as;
  }
};

/**
 * Aliases for string conversions
 */
template <class T, typename std::enable_if<std::is_arithmetic<T>::value>::type* = nullptr>
std::string toString(T val)
{
  return std::to_string(val);
}

/**
 * Helper to extract single values from XmlRpc structs.
 */
std::string toString(const XmlRpc::XmlRpcValue::Type& type);

inline std::string& strip(std::string& s, const char c) { return vigir_generic_params::strip(s, c); }
inline std::string strip_const(const std::string& s, const char c) { return vigir_generic_params::strip_const(s, c); }

/**
 * Generic helpers
 */

// conversion of
template <typename Map, typename Container>
Map footholdArrayToMap(const Container& array)
{
  Map map;
  for (auto& f : array)
    map.emplace(f.idx, f);
  return map;
}

/**
 * @brief Extracts key from a map container into a list
 * @param Container type of output list
 * @param Map type of input map
 * @param map input map
 * @return list of keys extracted from the map
 */
template <typename Container, typename Map>
Container keysAsArray(const Map& map)
{
  Container container;
  if (std::is_member_function_pointer<decltype(&Container::reserve)>::value)
    container.reserve(map.size());
  for (const auto& p : map)
    container.push_back(p.first);
  return container;
}

/**
 * @brief Extracts values from a map container into a list
 * @param Container type of output list
 * @param Map type of input map
 * @param map input map
 * @return list of values extracted from the map
 */
template <typename Container, typename Map>
Container valuesAsArray(const Map& map)
{
  Container container;
  if (std::is_member_function_pointer<decltype(&Container::reserve)>::value)
    container.reserve(map.size());
  for (const auto& p : map)
    container.push_back(p.second);
  return container;
}
}  // namespace l3

#endif
