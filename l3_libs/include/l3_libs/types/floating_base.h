//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FLOATING_BASE_H__
#define L3_FLOATING_BASE_H__

#include <l3_msgs/FloatingBase.h>

#include <l3_libs/types/base_link.h>
#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/variant_data.h>

namespace l3
{
typedef LinkIndex BaseIndex;
typedef std::vector<BaseIndex> BaseIndexArray;
typedef std::set<BaseIndex> BaseIndexSet;
typedef std::vector<BaseIndexArray> MultiBaseIndexArray;

/**
 * @brief Structure for describing floating bases
 */
struct FloatingBase : public BaseLink<FloatingBase>
{
  // typedefs
  typedef SharedPtr<FloatingBase> Ptr;
  typedef SharedPtr<const FloatingBase> ConstPtr;

  FloatingBase();
  FloatingBase(const BaseIndex& idx, const Pose& pose, const std_msgs::Header& header = std_msgs::Header(), const VariantDataSet& data = VariantDataSet());
  FloatingBase(const BaseIndex& idx, double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, const std_msgs::Header& header = std_msgs::Header(),
           const VariantDataSet& data = VariantDataSet());
  inline FloatingBase(double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, const std_msgs::Header& header = std_msgs::Header(),
                  const VariantDataSet& data = VariantDataSet())
    : FloatingBase(0, x, y, z, roll, pitch, yaw, header, data)
  {}
  FloatingBase(const l3_msgs::FloatingBase& msg);

  void fromMsg(const l3_msgs::FloatingBase& msg);

  void toMsg(l3_msgs::FloatingBase& msg) const;
  l3_msgs::FloatingBase toMsg() const;

  inline FloatingBase transform(const Transform& transform) const { return FloatingBase(idx, transform * pose_, header, data); }
  inline const FloatingBase& transform(const Transform& transform)
  {
    setPose(transform * pose_);
    return *this;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator<<(std::ostream& stream, const FloatingBase& floating_base) { return stream << floating_base.toString(); }

typedef std::vector<FloatingBase> FloatingBaseArray;
typedef std::vector<FloatingBase::Ptr> FloatingBasePtrArray;
typedef std::vector<FloatingBase::ConstPtr> FloatingBaseConstPtrArray;

typedef std::pair<const BaseIndex, FloatingBase> FloatingBasePair;
typedef std::pair<const BaseIndex, FloatingBase::Ptr> FloatingBasePtrPair;
typedef std::pair<const BaseIndex, FloatingBase::ConstPtr> FloatingBaseConstPtrPair;

typedef std::map<BaseIndex, FloatingBase> FloatingBaseMap;
typedef std::map<BaseIndex, FloatingBase::Ptr> FloatingBasePtrMap;
typedef std::map<BaseIndex, FloatingBase::ConstPtr> FloatingBaseConstPtrMap;

L3_STATIC_ASSERT_MOVEABLE(BaseIndex)
L3_STATIC_ASSERT_MOVEABLE(FloatingBase)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::FloatingBase> FloatingBaseArray;
}

#endif
