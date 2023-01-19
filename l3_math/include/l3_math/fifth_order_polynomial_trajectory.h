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

#include <eigen3/Eigen/Eigen>

#include "l3_math/math.h"

namespace l3
{
class FifthOrderPolynomialTrajectory
{
public:
  FifthOrderPolynomialTrajectory(double init_time, double init_pose, double init_vel, double init_acc, double final_time, double final_pose, double final_vel, double final_acc);
  FifthOrderPolynomialTrajectory(double init_time, double init_vel, double init_acc, double final_time, double final_pose, double final_vel, double final_acc);
  FifthOrderPolynomialTrajectory();
  ~FifthOrderPolynomialTrajectory();

  bool changeTrajectory(double final_pose, double final_vel, double final_acc);
  bool changeTrajectory(double final_time, double final_pose, double final_vel, double final_acc);
  bool changeTrajectory(double init_time, double init_pose, double init_vel, double init_acc, double final_time, double final_pose, double final_vel, double final_acc);

  double getPosition(double time);
  double getVelocity(double time);
  double getAcceleration(double time);

  void setTime(double time);
  double getPosition();
  double getVelocity();
  double getAcceleration();

  inline void setEndValues(double pose, double vel, double acc)
  {
    end_pose_ = pose;
    end_vel_ = vel;
    end_acc_ = acc;
  }
  inline void setStartValues(double pose, double vel, double acc)
  {
    start_pose_ = pose;
    start_vel_ = vel;
    start_acc_ = acc;
  }

  inline Eigen::MatrixXd getPositionCoeff() { return position_coeff_; }
  inline Eigen::MatrixXd getVelocityCoeff() { return velocity_coeff_; }
  inline Eigen::MatrixXd getAccelerationCoeff() { return acceleration_coeff_; }

private:
  void setToStartValues();
  void setToEndValues();
  void initCoefficientValues();
  void updateCurrentValues(double time);

  double init_time_, init_pose_, init_vel_, init_acc_;
  double cur_time_, cur_pose_, cur_vel_, cur_acc_;
  double final_time_, final_pose_, final_vel_, final_acc_;

  double start_pose_, start_vel_, start_acc_;
  double end_pose_, end_vel_, end_acc_;

  Eigen::MatrixXd position_coeff_;
  Eigen::MatrixXd velocity_coeff_;
  Eigen::MatrixXd acceleration_coeff_;
  Eigen::MatrixXd time_variable_;
};
}  // namespace l3
