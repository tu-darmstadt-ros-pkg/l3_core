#include "l3_math/polynomial_approach_trajectory.h"

namespace l3
{
PolynomialApproachTrajectory::PolynomialApproachTrajectory(double init_time, double init_pose, double final_time, double final_pose)
{
  changeTrajectory(init_time, init_pose, final_time, final_pose);
}

PolynomialApproachTrajectory::PolynomialApproachTrajectory() { resetTrajectory(); }

PolynomialApproachTrajectory::~PolynomialApproachTrajectory() {}

bool PolynomialApproachTrajectory::changeTrajectory(double init_time, double init_pose, double final_time, double final_pose)
{
  if (final_time < init_time)
    return false;

  init_time_ = init_time;
  init_pose_ = init_pose;

  final_time_ = final_time;
  final_pose_ = final_pose;

  initCoefficientValues();
  return true;
}

void PolynomialApproachTrajectory::resetTrajectory() { changeTrajectory(0.0, 0.0, 0.0, 0.0); }

double PolynomialApproachTrajectory::getPosition(double time)
{
  updateCurrentValues(time);
  return cur_pose_;
}

double PolynomialApproachTrajectory::getVelocity(double time)
{
  updateCurrentValues(time);
  return cur_vel_;
}

double PolynomialApproachTrajectory::getAcceleration(double time)
{
  updateCurrentValues(time);
  return cur_acc_;
}

void PolynomialApproachTrajectory::updateCurrentValues(double time)
{
  if (time > final_time_ || time < init_time_)
  {
    cur_pose_ = 0;
    cur_vel_ = 0;
    cur_acc_ = 0;
  }
  else
  {
    double delta_time = time - final_time_;

    cur_pose_ = coeff_a_ * powDI(delta_time, 2) + coeff_b_;
    cur_vel_ = 2.0 * coeff_a_ * powDI(delta_time, 1);
    cur_acc_ = 2.0 * coeff_a_;
  }
}

void PolynomialApproachTrajectory::initCoefficientValues()
{
  double delta_pose = init_pose_ - final_pose_;
  double delta_time = init_time_ - final_time_;

  coeff_a_ = delta_pose / powDI(delta_time, 2);
  coeff_b_ = final_pose_;
}
}  // namespace l3
