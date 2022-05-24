#include <l3_libs/types/step_data.h>

namespace l3
{
StepData::StepData()
  : origin(new Foothold())
  , target(new Foothold())
  , dx(0.0)
  , dy(0.0)
  , dz(0.0)
  , droll(0.0)
  , dpitch(0.0)
  , dyaw(0.0)
  , sway_duration(0.0)
  , step_duration(0.0)
  , swing_height(0.0)
{}

StepData::StepData(const Foothold::ConstPtr origin, const Foothold::ConstPtr target, const Transform delta, double sway_duration, double step_duration, double swing_height)
  : origin(origin)
  , target(target)
  , dx(delta.x())
  , dy(delta.y())
  , dz(delta.z())
  , droll(delta.roll())
  , dpitch(delta.pitch())
  , dyaw(delta.yaw())
  , sway_duration(sway_duration)
  , step_duration(step_duration)
  , swing_height(swing_height)
{
  ROS_ASSERT(origin);
  ROS_ASSERT(target);
  if (origin->idx != target->idx)
    ROS_ERROR("[StepData] Origin and Target FootIndex mismatch! Fix it immediately!");
}

void StepData::fromMsg(const l3_msgs::StepData& msg)
{
  origin.reset(new Foothold(msg.origin));
  target.reset(new Foothold(msg.target));

  if (origin->idx != target->idx)
    ROS_ERROR("[StepData] fromMsg: Origin and Target FootIndex mismatch (%i <-> %i)! Fix it immediately!", origin->idx, target->idx);

  dx = msg.dx;
  dy = msg.dy;
  dz = msg.dz;
  droll = msg.droll;
  dpitch = msg.dpitch;
  dyaw = msg.dyaw;

  sway_duration = msg.sway_duration;
  step_duration = msg.step_duration;
  swing_height = msg.swing_height;

  variantDataSetMsgToL3(msg.data, data);
}

void StepData::toMsg(l3_msgs::StepData& msg) const
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

  msg.sway_duration = sway_duration;
  msg.step_duration = step_duration;
  msg.swing_height = swing_height;

  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::StepData StepData::toMsg() const
{
  l3_msgs::StepData msg;
  toMsg(msg);
  return msg;
}

std::string StepData::toString() const
{
  std::stringstream s;
  s << std::setprecision(2) << std::fixed;
  s << *origin << "   ->   " << *target;
  return s.str();
}
}  // namespace l3
