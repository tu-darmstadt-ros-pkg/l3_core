#include <l3_libs/types/base_step_data.h>

namespace l3
{
BaseStepData::BaseStepData()
  : origin(new FloatingBase())
  , target(new FloatingBase())
  , dx(0.0)
  , dy(0.0)
  , dz(0.0)
  , droll(0.0)
  , dpitch(0.0)
  , dyaw(0.0)
  , step_duration(0.0)
{}

BaseStepData::BaseStepData(const FloatingBase::ConstPtr origin, const FloatingBase::ConstPtr target, const Transform delta, double step_duration)
  : origin(origin)
  , target(target)
  , dx(delta.x())
  , dy(delta.y())
  , dz(delta.z())
  , droll(delta.roll())
  , dpitch(delta.pitch())
  , dyaw(delta.yaw())
  , step_duration(step_duration)
{
  ROS_ASSERT(origin);
  ROS_ASSERT(target);
  if (origin->idx != target->idx)
    ROS_ERROR("[BaseStepData] Origin and Target BaseIndex mismatch! Fix it immediately!");
}

void BaseStepData::fromMsg(const l3_msgs::BaseStepData& msg)
{
  origin.reset(new FloatingBase(msg.origin));
  target.reset(new FloatingBase(msg.target));

  if (origin->idx != target->idx)
    ROS_ERROR("[BaseStepData] fromMsg: Origin and Target BaseIndex mismatch (%i <-> %i)! Fix it immediately!", origin->idx, target->idx);

  dx = msg.dx;
  dy = msg.dy;
  dz = msg.dz;
  droll = msg.droll;
  dpitch = msg.dpitch;
  dyaw = msg.dyaw;

  step_duration = msg.step_duration;

  variantDataSetMsgToL3(msg.data, data);
}

void BaseStepData::toMsg(l3_msgs::BaseStepData& msg) const
{
  if (origin)
    origin->toMsg(msg.origin);
  if (target)
    target->toMsg(msg.target);

  msg.dx = dx;
  msg.dy = dy;
  msg.dz = dz;
  msg.droll = droll;
  msg.dpitch = dpitch;
  msg.dyaw = dyaw;

  msg.step_duration = step_duration;

  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::BaseStepData BaseStepData::toMsg() const
{
  l3_msgs::BaseStepData msg;
  toMsg(msg);
  return msg;
}

std::string BaseStepData::toString() const
{
  std::stringstream s;
  s << std::setprecision(2) << std::fixed;
  s << *origin << "   ->   " << *target;
  return s.str();
}
}  // namespace l3
