#include "l3_math/fifth_order_polynomial_trajectory.h"

namespace l3
{
FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory(double init_time, double init_pose, double init_vel, double init_acc, double final_time, double final_pose,
                                                               double final_vel, double final_acc)
{
  position_coeff_.resize(6, 1);
  velocity_coeff_.resize(6, 1);
  acceleration_coeff_.resize(6, 1);
  time_variable_.resize(1, 6);

  position_coeff_.fill(0);
  velocity_coeff_.fill(0);
  acceleration_coeff_.fill(0);

  if (final_time > init_time)
  {
    cur_time_ = init_time_ = init_time;
    cur_pose_ = init_pose_ = init_pose;
    cur_vel_ = init_vel_ = init_vel;
    cur_acc_ = init_acc_ = init_acc;

    final_time_ = final_time;
    final_pose_ = final_pose;
    final_vel_ = final_vel;
    final_acc_ = final_acc;

    initCoefficientValues();
  }
}

FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory()
{
  init_time_ = init_pose_ = init_vel_ = init_acc_ = 0.0;
  cur_time_ = cur_pose_ = cur_vel_ = cur_acc_ = 0.0;
  final_time_ = final_pose_ = final_vel_ = final_acc_ = 0.0;
  start_pose_ = start_vel_ = start_acc_ = 0.0;
  end_pose_ = end_vel_ = end_acc_ = 0.0;

  time_variable_.resize(1, 6);
  position_coeff_.resize(6, 1);
  velocity_coeff_.resize(6, 1);
  acceleration_coeff_.resize(6, 1);

  time_variable_.fill(0);
  position_coeff_.fill(0);
  velocity_coeff_.fill(0);
  acceleration_coeff_.fill(0);
}

FifthOrderPolynomialTrajectory::~FifthOrderPolynomialTrajectory() {}

bool FifthOrderPolynomialTrajectory::changeTrajectory(double final_pose, double final_vel, double final_acc)
{
  final_pose_ = final_pose;
  final_vel_ = final_vel;
  final_acc_ = final_acc;

  initCoefficientValues();

  return true;
}

bool FifthOrderPolynomialTrajectory::changeTrajectory(double final_time, double final_pose, double final_vel, double final_acc)
{
  if (final_time < init_time_)
    return false;

  final_time_ = final_time;
  return changeTrajectory(final_pose, final_vel, final_acc);
}

bool FifthOrderPolynomialTrajectory::changeTrajectory(double init_time, double init_pose, double init_vel, double init_acc, double final_time, double final_pose, double final_vel,
                                                      double final_acc)
{
  if (final_time < init_time)
    return false;

  init_time_ = init_time;
  init_pose_ = init_pose;
  init_vel_ = init_vel;
  init_acc_ = init_acc;

  final_time_ = final_time;

  return changeTrajectory(final_pose, final_vel, final_acc);
}

double FifthOrderPolynomialTrajectory::getPosition(double time)
{
  updateCurrentValues(time);
  return cur_pose_;
}

double FifthOrderPolynomialTrajectory::getVelocity(double time)
{
  updateCurrentValues(time);
  return cur_vel_;
}

double FifthOrderPolynomialTrajectory::getAcceleration(double time)
{
  updateCurrentValues(time);
  return cur_acc_;
}

double FifthOrderPolynomialTrajectory::getPosition() { return cur_pose_; }

double FifthOrderPolynomialTrajectory::getVelocity() { return cur_vel_; }

double FifthOrderPolynomialTrajectory::getAcceleration() { return cur_acc_; }

void FifthOrderPolynomialTrajectory::setTime(double time) { updateCurrentValues(time); }

void FifthOrderPolynomialTrajectory::updateCurrentValues(double time)
{
  if (time > final_time_)
  {
    setToEndValues();
  }
  else if (time <= init_time_)
  {
    setToStartValues();
  }
  else
  {
    cur_time_ = time;
    time_variable_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
    cur_pose_ = (time_variable_ * position_coeff_).coeff(0, 0);
    cur_vel_ = (time_variable_ * velocity_coeff_).coeff(0, 0);
    cur_acc_ = (time_variable_ * acceleration_coeff_).coeff(0, 0);
  }
}

void FifthOrderPolynomialTrajectory::initCoefficientValues()
{
  Eigen::MatrixXd time;
  Eigen::MatrixXd cond;

  time.resize(6, 6);
  time << powDI(init_time_, 5), powDI(init_time_, 4), powDI(init_time_, 3), powDI(init_time_, 2), init_time_, 1.0, 5.0 * powDI(init_time_, 4), 4.0 * powDI(init_time_, 3),
      3.0 * powDI(init_time_, 2), 2.0 * init_time_, 1.0, 0.0, 20.0 * powDI(init_time_, 3), 12.0 * powDI(init_time_, 2), 6.0 * init_time_, 2.0, 0.0, 0.0, powDI(final_time_, 5),
      powDI(final_time_, 4), powDI(final_time_, 3), powDI(final_time_, 2), final_time_, 1.0, 5.0 * powDI(final_time_, 4), 4.0 * powDI(final_time_, 3), 3.0 * powDI(final_time_, 2),
      2.0 * final_time_, 1.0, 0.0, 20.0 * powDI(final_time_, 3), 12.0 * powDI(final_time_, 2), 6.0 * final_time_, 2.0, 0.0, 0.0;

  cond.resize(6, 1);
  cond << init_pose_, init_vel_, init_acc_, final_pose_, final_vel_, final_acc_;

  position_coeff_ = time.inverse() * cond;
  velocity_coeff_ << 0.0, 5.0 * position_coeff_.coeff(0, 0), 4.0 * position_coeff_.coeff(1, 0), 3.0 * position_coeff_.coeff(2, 0), 2.0 * position_coeff_.coeff(3, 0),
      1.0 * position_coeff_.coeff(4, 0);
  acceleration_coeff_ << 0.0, 0.0, 20.0 * position_coeff_.coeff(0, 0), 12.0 * position_coeff_.coeff(1, 0), 6.0 * position_coeff_.coeff(2, 0), 2.0 * position_coeff_.coeff(3, 0);
}

void FifthOrderPolynomialTrajectory::setToStartValues()
{
  cur_time_ = init_time_;
  cur_pose_ = start_pose_;
  cur_vel_ = start_vel_;
  cur_acc_ = start_acc_;
}

void FifthOrderPolynomialTrajectory::setToEndValues()
{
  cur_time_ = final_time_;
  cur_pose_ = end_pose_;
  cur_vel_ = end_vel_;
  cur_acc_ = end_acc_;
}
}  // namespace l3
