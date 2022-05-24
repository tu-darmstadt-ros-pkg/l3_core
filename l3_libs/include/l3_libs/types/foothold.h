//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTHOLD_H__
#define L3_FOOTHOLD_H__

#include <l3_msgs/Foothold.h>

#include <l3_libs/types/base_link.h>
#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/variant_data.h>

namespace l3
{
typedef LinkIndex FootIndex;
typedef std::vector<FootIndex> FootIndexArray;
typedef std::set<FootIndex> FootIndexSet;
typedef std::vector<FootIndexArray> MultiFootIndexArray;

/**
 * @brief Structure for describing footholds
 */
struct Foothold : public BaseLink<Foothold>
{
  // typedefs
  typedef SharedPtr<Foothold> Ptr;
  typedef SharedPtr<const Foothold> ConstPtr;

  Foothold();
  Foothold(FootIndex idx, const Pose& pose, const std_msgs::Header& header = std_msgs::Header(), const VariantDataSet& data = VariantDataSet());
  Foothold(FootIndex idx, double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, const std_msgs::Header& header = std_msgs::Header(),
           const VariantDataSet& data = VariantDataSet());
  inline Foothold(double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, const std_msgs::Header& header = std_msgs::Header(),
                  const VariantDataSet& data = VariantDataSet())
    : Foothold(0, x, y, z, roll, pitch, yaw, header, data)
  {}
  Foothold(const l3_msgs::Foothold& msg);

  void fromMsg(const l3_msgs::Foothold& msg);

  void toMsg(l3_msgs::Foothold& msg) const;
  l3_msgs::Foothold toMsg() const;  

  inline Foothold transform(const Transform& transform) const { return Foothold(idx, transform * pose_, header, data); }
  inline const Foothold& transform(const Transform& transform)
  {
    setPose(transform * pose_);
    return *this;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator<<(std::ostream& stream, const Foothold& foothold) { return stream << foothold.toString(); }

typedef std::vector<Foothold> FootholdArray;
typedef std::vector<Foothold::Ptr> FootholdPtrArray;
typedef std::vector<Foothold::ConstPtr> FootholdConstPtrArray;

typedef std::pair<const FootIndex, Foothold> FootholdPair;
typedef std::pair<const FootIndex, Foothold::Ptr> FootholdPtrPair;
typedef std::pair<const FootIndex, Foothold::ConstPtr> FootholdConstPtrPair;

typedef std::map<FootIndex, Foothold> FootholdMap;
typedef std::map<FootIndex, Foothold::Ptr> FootholdPtrMap;
typedef std::map<FootIndex, Foothold::ConstPtr> FootholdConstPtrMap;

L3_STATIC_ASSERT_MOVEABLE(FootIndex)
L3_STATIC_ASSERT_MOVEABLE(Foothold)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::Foothold> FootholdArray;
}

#endif
