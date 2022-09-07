#include <l3_libs/types/step.h>

namespace l3
{
void Step::fromMsg(const l3_msgs::Step& msg)
{
  clear();

  for (const l3_msgs::FootStepData& s : msg.foot_steps)
    foot_step_.updateMovingLink(s.origin.idx, makeShared<FootStepData>(s));

  for (const l3_msgs::Foothold& fh : msg.support_feet)
    foot_step_.updateNonMovingLink(fh.idx, makeShared<Foothold>(fh));

  for (const l3_msgs::BaseStepData& s : msg.moving_bases)
    base_step_.updateMovingLink(s.origin.idx, makeShared<BaseStepData>(s));

  for (const l3_msgs::FloatingBase& fb : msg.resting_bases)
    base_step_.updateNonMovingLink(fb.idx, makeShared<FloatingBase>(fb));

  setStepIndex(msg.idx);

  variantDataSetMsgToL3(msg.data, data);
}

void Step::toMsg(l3_msgs::Step& msg) const
{
  msg.foot_steps.clear();
  for (const FootStep::MovingDataPair& p : foot_step_.getMovingLinks())
    msg.foot_steps.push_back(p.second->toMsg());

  msg.support_feet.clear();
  for (const FootStep::NonMovingDataPair& p : foot_step_.getNonMovingLinks())
    msg.support_feet.push_back(p.second->toMsg());

  msg.moving_bases.clear();
  for (const BaseStep::MovingDataPair& p : base_step_.getMovingLinks())
    msg.moving_bases.push_back(p.second->toMsg());

  msg.resting_bases.clear();
  for (const BaseStep::NonMovingDataPair& p : base_step_.getNonMovingLinks())
    msg.resting_bases.push_back(p.second->toMsg());

  msg.idx = getStepIndex();

  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::Step Step::toMsg() const
{
  l3_msgs::Step msg;
  toMsg(msg);
  return msg;
}

void Step::clear()
{
  foot_step_.clear();
  base_step_.clear();
}

Step& Step::transform(const Transform& transform, const std_msgs::Header& header)
{
  // transform moving footholds
  for (FootStep::MovingDataPair& p : foot_step_.getMovingLinks())
  {
    Foothold fh = p.second->origin->transform(transform);
    if (!header.frame_id.empty())
      fh.header = header;
    p.second->origin = makeShared<Foothold>(fh);

    fh = p.second->target->transform(transform);
    if (!header.frame_id.empty())
      fh.header = header;
    p.second->target = makeShared<Foothold>(fh);
  }

  // transform non-moving footholds
  for (FootStep::NonMovingDataPair& p : foot_step_.getNonMovingLinks())
  {
    Foothold fh = p.second->transform(transform);
    if (!header.frame_id.empty())
      fh.header = header;
    p.second = makeShared<Foothold>(fh);
  }

  // transform moving floating bases
  for (BaseStep::MovingDataPair& p : base_step_.getMovingLinks())
  {
    FloatingBase fb = p.second->origin->transform(transform);
    if (!header.frame_id.empty())
      fb.header = header;
    p.second->origin = makeShared<FloatingBase>(fb);

    fb = p.second->target->transform(transform);
    if (!header.frame_id.empty())
      fb.header = header;
    p.second->target = makeShared<FloatingBase>(fb);
  }

  // transform non-moving floating bases
  for (BaseStep::NonMovingDataPair& p : base_step_.getNonMovingLinks())
  {
    FloatingBase fb = p.second->transform(transform);
    if (!header.frame_id.empty())
      fb.header = header;
    p.second = makeShared<FloatingBase>(fb);
  }

  return *this;
}

FootholdConstPtrArray Step::getAllFootholds() const
{
  FootholdConstPtrArray footholds;

  for (const FootStep::MovingDataPair& p : foot_step_.getMovingLinks())
    footholds.push_back(p.second->target);

  for (const FootStep::NonMovingDataPair& p : foot_step_.getNonMovingLinks())
    footholds.push_back(p.second);

  return footholds;
}

Foothold::ConstPtr Step::getFoothold(const FootIndex& foot_idx) const
{
  FootStepData::ConstPtr step_data = foot_step_.getMovingLink(foot_idx);
  if (step_data)
    return step_data->target;

  return foot_step_.getNonMovingLink(foot_idx);
}

FloatingBaseConstPtrArray Step::getAllFloatingBases() const
{
  FloatingBaseConstPtrArray floating_bases;

  for (const BaseStep::MovingDataPair& p : base_step_.getMovingLinks())
    floating_bases.push_back(p.second->target);

  for (const BaseStep::NonMovingDataPair& p : base_step_.getNonMovingLinks())
    floating_bases.push_back(p.second);

  return floating_bases;
}

FloatingBase::ConstPtr Step::getFloatingBase(const BaseIndex& base_idx) const
{
  BaseStepData::ConstPtr base_step_data = base_step_.getMovingLink(base_idx);
  if (base_step_data)
    return base_step_data->target;

  return base_step_.getNonMovingLink(base_idx);
}
}  // namespace l3
