#include <l3_libs/conversions/l3_tf_conversions.h>

namespace l3
{
void pointArrayTFToL3(const std::vector<tf::Point>& tf, l3::PointArray& p)
{
  p.clear();
  l3::Point l3_p;
  for (const tf::Point& tf_p : tf)
  {
    pointTFToL3(tf_p, l3_p);
    p.push_back(l3_p);
  }
}

void pointArrayL3ToTF(const l3::PointArray& p, std::vector<tf::Point>& tf)
{
  tf.clear();
  tf::Point tf_p;
  for (const l3::Point& l3_p : p)
  {
    pointL3ToTF(l3_p, tf_p);
    tf.push_back(tf_p);
  }
}

void poseArrayTFToL3(const std::vector<tf::Pose>& tf, l3::PoseArray& p)
{
  p.clear();
  l3::Pose l3_p;
  for (const tf::Pose& tf_p : tf)
  {
    poseTFToL3(tf_p, l3_p);
    p.push_back(l3_p);
  }
}

void poseArrayL3ToTF(const l3::PoseArray& p, std::vector<tf::Pose>& tf)
{
  tf.clear();
  tf::Pose tf_p;
  for (const l3::Pose& l3_p : p)
  {
    poseL3ToTF(l3_p, tf_p);
    tf.push_back(tf_p);
  }
}

void vectorArrayTFToL3(const std::vector<tf::Vector3>& tf, l3::Vector3Array& v)
{
  v.clear();
  l3::Vector3 l3_v;
  for (const tf::Vector3& tf_p : tf)
  {
    vectorTFToL3(tf_p, l3_v);
    v.push_back(l3_v);
  }
}

void vectorArrayL3ToTF(const l3::Vector3Array& v, std::vector<tf::Vector3>& tf)
{
  tf.clear();
  tf::Vector3 tf_p;
  for (const l3::Vector3& l3_v : v)
  {
    vectorL3ToTF(l3_v, tf_p);
    tf.push_back(tf_p);
  }
}
}  // namespace l3
