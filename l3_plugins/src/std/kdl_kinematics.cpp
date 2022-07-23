#include <l3_plugins/std/kdl_kinematics.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <l3_libs/conversions/l3_kdl_conversions.h>
#include <l3_libs/yaml_parser.h>

#include <l3_math/angles.h>

namespace l3
{
KdlKinematics::KdlKinematics(const std::string& name)
  : KinematicsPlugin(name)
{}

bool KdlKinematics::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!KinematicsPlugin::loadParams(params))
    return false;

  getParam("ignore_foot_orientation", ignore_foot_orientation_, false, true);

  return true;
}

bool KdlKinematics::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!KinematicsPlugin::initialize(params))
    return false;

  if (!kdl_parser::treeFromParam(param("robot_description", std::string()), tree_))
  {
    ROS_ERROR("[KDLKinematicsPlugin] initialize: Failed to construct KDL tree of robot description");
    return false;
  }

  all_links_ = tree_.getSegments();

  if (root_link_.empty())
    root_link_ = tree_.getRootSegment()->first;

  return true;
}

bool KdlKinematics::getMassOfLink(const std::string& link_id, double& mass) const
{
  try
  {
    mass = all_links_.at(link_id).segment.getInertia().getMass();
    return true;
  }
  catch (...)
  {
    ROS_ERROR("[KDLKinematicsPlugin] getMassOfLink: No Link with given link_id found: '%s'", link_id.c_str());
    mass = 0.0;
    return false;
  }
}

bool KdlKinematics::getMassOfChain(const std::string& start_link_id, double& mass) const
{
  try
  {
    mass = calcTotalMassRecursive(all_links_.at(start_link_id));
    return true;
  }
  catch (...)
  {
    ROS_ERROR("[KDLKinematicsPlugin] getMassOfChain: No Link with given link_id found: '%s'", start_link_id.c_str());
    mass = 0.0;
    return false;
  }
}

bool KdlKinematics::calcTotalMass(double& mass) const
{
  mass = 0.0;
  try
  {
    mass = calcTotalMassRecursive(tree_.getRootSegment()->second);
    return true;
  }
  catch (...)
  {
    ROS_ERROR("[KDLKinematicsPlugin] calcTotalMass: No root link found");
    return false;
  }
}

bool KdlKinematics::calcCenterOfMass(const std::string& frame_link, Vector3& com) const
{
  KDL::Vector sumOfCOM = KDL::Vector(0, 0, 0);
  for (const std::pair<std::string, KDL::TreeElement>& te : all_links_)
  {
    KDL::Frame transform;
    if (calcTransformToLink(frame_link, te.second.segment.getName(), transform))
      sumOfCOM += transform * te.second.segment.getInertia().getCOG();
    else
    {
      ROS_ERROR("[KDLKinematicsPlugin] calcCenterOfMass: Could not calculate center of mass related to '%s'", frame_link.c_str());
      return false;
    }
  }

  com.x() = sumOfCOM.x() / all_links_.size();
  com.y() = sumOfCOM.y() / all_links_.size();
  com.z() = sumOfCOM.z() / all_links_.size();

  return true;
}

bool KdlKinematics::calcStaticTransformForChain(const std::string& root_link, const std::string& tip_link, Transform& transform) const
{
  KDL::Frame frame;
  if (calcTransformToLink(root_link, tip_link, frame))
  {
    transformKdlToL3(frame, transform);
    return true;
  }
  return false;
}

bool KdlKinematics::calcForwardKinematicsForChain(const std::string& root_link, const std::string& tip_link, const std::vector<double>& q, Pose& result) const
{
  KDL::Chain chain;
  if (!tree_.getChain(root_link, tip_link, chain))
  {
    ROS_ERROR("[KDLKinematicsPlugin] calcForwardKinematicsForChain: No chain found between '%s' and '%s'.", root_link.c_str(), tip_link.c_str());
    return false;
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::JntArray kdl_q(chain.getNrOfJoints());
  for (size_t i = 0; i < chain.getNrOfJoints(); i++)
    kdl_q(i) = q[i];

  KDL::Frame f_result;
  int status = fk_solver.JntToCart(kdl_q, f_result);
  if (status >= 0)
  {
    tf::transformKDLToEigen(f_result, result);
    return true;
  }
  else
  {
    ROS_ERROR("[KDLKinematicsPlugin] calcForwardKinematicsForChain: No Solution found in forward Kinematics.");
    return false;
  }
}

bool KdlKinematics::calcInverseKinematicsForChain(const std::string& root_link, const std::string& tip_link, const Pose& goal, const std::vector<double>& curr_q,
                                                  std::vector<double>& q) const
{
  q.clear();

  int max_iterations = 100;
  double eps = 1e-6;

  KDL::Chain chain;

  if (!tree_.getChain(root_link, tip_link, chain))
  {
    ROS_ERROR("[KDLKinematicsPlugin] calcInverseKinematicsForChain: No chain found between '%s' and '%s'.", root_link.c_str(), tip_link.c_str());
    return false;
  }

  int nr_of_joints = chain.getNrOfJoints();

  // loosening orientation constraints
  if (ignore_foot_orientation_)
  {
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0))));
  }

  // instantiate IK solver
  ChainIkSolverVelPtr ik_solver_vel = makeShared<KDL::ChainIkSolverVel_pinv>(chain);
  ChainFkSolverPosPtr fk_solver = makeShared<KDL::ChainFkSolverPos_recursive>(chain);
  ChainIkSolverPosPtr ik_solver_pos = makeShared<KDL::ChainIkSolverPos_NR>(chain, *fk_solver, *ik_solver_vel, max_iterations, eps);

  KDL::JntArray kdl_q(chain.getNrOfJoints());
  KDL::JntArray kdl_q_init(chain.getNrOfJoints());

  // copy start state
  for (size_t i = 0; i < curr_q.size(); i++)
    kdl_q_init(i) = curr_q[i];

  for (size_t i = curr_q.size(); i < chain.getNrOfJoints(); i++)
    kdl_q_init(i) = 0.0;

  // try to solve IK
  KDL::Frame f_dest;
  l3::poseL3ToKdl(goal, f_dest);

  if (ik_solver_pos->CartToJnt(kdl_q_init, f_dest, kdl_q) < KDL::SolverI::E_NOERROR)
  {
    ROS_WARN_THROTTLE(1.0, "[KDLKinematicsPlugin] calcInverseKinematicsForChain: No inverse solution found between '%s' and '%s'", root_link.c_str(), tip_link.c_str());
    return false;
  }

  // write back result
  for (int i = 0; i < nr_of_joints; i++)
    q.push_back(l3::normalizeAngle(kdl_q(i)));

  return true;
}

bool KdlKinematics::calcInverseKinematicsForChain(const std::string& root_link, const std::string& tip_link, const Pose& goal, std::vector<double>& result) const
{
  KDL::Chain chain;

  if (!tree_.getChain(root_link, tip_link, chain))
  {
    ROS_ERROR("[KDLKinematicsPlugin] calcInverseKinematicsForChain: No chain found between '%s' and '%s'.", root_link.c_str(), tip_link.c_str());
    return false;
  }

  std::vector<double> q_init;
  for (size_t i = 0; i < chain.getNrOfJoints(); i++)
    q_init.push_back(0.0);

  return calcInverseKinematicsForChain(root_link, tip_link, goal, q_init, result);
}

bool KdlKinematics::calcInverseKinematicsWholeBody(const Pose& body_pose, const FootholdArray& footholds, std::map<std::string, double>& result) const
{
  ROS_ERROR("[KdlKinematics] calcInverseKinematicsWholeBody: Not implemented yet!");
  return false;
}

bool KdlKinematics::calcInverseKinematicsWholeBody(const FootholdArray& footholds, std::map<std::string, double>& result) const
{
  ROS_ERROR("[KdlKinematics] calcInverseKinematicsWholeBody: Not implemented yet!");
  return false;
}

double KdlKinematics::calcTotalMassRecursive(const KDL::TreeElement& root) const
{
  double mass = root.segment.getInertia().getMass();
  if (root.children.empty())
    return mass;
  else
  {
    for (const KDL::SegmentMap::const_iterator& child : root.children)
    {
      mass = mass + calcTotalMassRecursive(child->second);
    }
    return mass;
  }
}

bool KdlKinematics::calcTransformToLink(const std::string& root_link, const std::string& tip_link, KDL::Frame& transform) const
{
  KDL::Chain chain;

  if (!tree_.getChain(root_link, tip_link, chain))
  {
    ROS_ERROR("[KDLKinematicsPlugin] calcTransformToLink: No chain found between '%s' and '%s'", root_link.c_str(), tip_link.c_str());
    return false;
  }

  for (const KDL::Segment& seg : chain.segments)
    transform = transform * seg.getFrameToTip();

  return true;
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::KdlKinematics, l3::KinematicsPlugin)
