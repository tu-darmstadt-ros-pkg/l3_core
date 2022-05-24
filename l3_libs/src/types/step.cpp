#include <l3_libs/types/step.h>

namespace l3
{
void Step::fromMsg(const l3_msgs::Step& msg)
{
  clear();

  for (const l3_msgs::StepData& s : msg.step_data)
    step_data_map_[s.origin.idx].reset(new StepData(s));

  for (const l3_msgs::Foothold& fh : msg.support)
    support_[fh.idx].reset(new Foothold(fh));

  for (const l3_msgs::BaseStepData& s : msg.moving_bases)
    moving_bases_map_[s.origin.idx].reset(new BaseStepData(s));

  for (const l3_msgs::FloatingBase& fb : msg.resting_bases)
    resting_bases_map_[fb.idx].reset(new FloatingBase(fb));

  idx_ = msg.idx;

  variantDataSetMsgToL3(msg.data, data);
}

void Step::toMsg(l3_msgs::Step& msg) const
{
  msg.step_data.clear();
  for (const StepDataPair& p : step_data_map_)
    msg.step_data.push_back(p.second->toMsg());

  msg.support.clear();
  for (const FootholdConstPtrPair& p : support_)
    msg.support.push_back(p.second->toMsg());

  msg.moving_bases.clear();
  for (const BaseStepDataPair& p : moving_bases_map_)
    msg.moving_bases.push_back(p.second->toMsg());

  msg.resting_bases.clear();
  for (const FloatingBaseConstPtrPair& p : resting_bases_map_)
    msg.resting_bases.push_back(p.second->toMsg());

  msg.idx = idx_;

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
  BaseStep<StepData::Ptr>::clear();
  support_.clear();
  moving_bases_map_.clear();
  resting_bases_map_.clear();
}

Step& Step::transform(const Transform& transform, const std_msgs::Header& header)
{
  // transform step data
  for (StepDataPair& p : step_data_map_)
  {
    Foothold fh = p.second->origin->transform(transform);
    if (!header.frame_id.empty())
      fh.header = header;
    p.second->origin.reset(new Foothold(fh));

    fh = p.second->target->transform(transform);
    if (!header.frame_id.empty())
      fh.header = header;
    p.second->target.reset(new Foothold(fh));
  }

  // transform support footholds
  for (FootholdConstPtrPair p : support_)
  {
    Foothold fh = p.second->transform(transform);
    if (!header.frame_id.empty())
      fh.header = header;
    support_[p.first].reset(new Foothold(fh));
    // p.second.reset(new Foothold(f));
  }

  // transform moving floating bases
  for (BaseStepDataPair& p : moving_bases_map_)
  {
    FloatingBase fb = p.second->origin->transform(transform);
    if (!header.frame_id.empty())
      fb.header = header;
    p.second->origin.reset(new FloatingBase(fb));

    fb = p.second->target->transform(transform);
    if (!header.frame_id.empty())
      fb.header = header;
    p.second->target.reset(new FloatingBase(fb));
  }

  // transform resting floating bases
  for (FloatingBaseConstPtrPair p : resting_bases_map_)
  {
    FloatingBase fb = p.second->transform(transform);
    if (!header.frame_id.empty())
      fb.header = header;
    resting_bases_map_[p.first].reset(new FloatingBase(fb));
  }

  return *this;
}

FootholdConstPtrArray Step::getFootholds() const
{
  FootholdConstPtrArray footholds;

  for (const StepDataPair& p : step_data_map_)
    footholds.push_back(p.second->target);

  for (const FootholdConstPtrPair& p : getSupportMap())
    footholds.push_back(p.second);

  return footholds;
}

Foothold::ConstPtr Step::getFoothold(const FootIndex& foot_idx) const
{
  StepData::ConstPtr step_data = getStepData(foot_idx);
  if (step_data)
    return step_data->target;

  return getSupport(foot_idx);
}

FloatingBaseConstPtrArray Step::getFloatingBases() const
{
  FloatingBaseConstPtrArray floating_bases;

  for (const BaseStepDataPair& p : moving_bases_map_)
    floating_bases.push_back(p.second->target);

  for (const FloatingBaseConstPtrPair& p : getRestingFloatingBaseMap())
    floating_bases.push_back(p.second);

  return floating_bases;
}

FloatingBase::ConstPtr Step::getFloatingBase(const BaseIndex& base_idx) const
{
  BaseStepData::ConstPtr base_step_data = getMovingFloatingBase(base_idx);
  if (base_step_data)
    return base_step_data->target;

  return getRestingFloatingBase(base_idx);
}
}  // namespace l3
