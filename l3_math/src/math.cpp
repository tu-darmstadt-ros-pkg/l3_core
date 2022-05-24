#include <l3_math/math.h>

namespace l3
{
double powDI(double a, int b) { return (b == 0 ? 1 : (b > 0 ? a * powDI(a, b - 1) : 1 / powDI(a, -b))); }

inline double wsin(double magnitude, double magnitude_shift, double sigmoid_distortion_gain, double time)
{
  double amplitude = magnitude / (2.0 * M_PI);
  return magnitude_shift + amplitude * (time - sigmoid_distortion_gain * sin(time));
}

int sign(double x){
  if(x < 0) return -1;
  else return 1;
}

double wsigmoid(double time, double period, double time_shift, double magnitude, double magnitude_shift, double sigmoid_ratio, double distortion_ratio)
{
  double value = magnitude_shift;
  double sigmoid_distortion_gain = 1.0;
  double t = 0.0;

  if ((sigmoid_ratio >= 1.0) && (sigmoid_ratio < 2.0))
  {
    if (time >= time_shift + period * (2 - sigmoid_ratio))
    {
      value = magnitude_shift + magnitude;
    }
    else
    {
      t = 2.0 * M_PI * (time - time_shift) / (period * (2 - sigmoid_ratio));
      sigmoid_distortion_gain = distortion_ratio + (1 - distortion_ratio) * (time - (time_shift + period * (1 - sigmoid_ratio))) / (period * (2 - sigmoid_ratio));
      value = wsin(magnitude, magnitude_shift, sigmoid_distortion_gain, t);
    }
  }
  else if ((sigmoid_ratio >= 0.0) && (sigmoid_ratio < 1.0))
  {
    if (time <= time_shift + period * (1 - sigmoid_ratio))
    {
      value = magnitude_shift;
    }
    else
    {
      t = 2.0 * M_PI * (time - time_shift - period * (1 - sigmoid_ratio)) / (period * sigmoid_ratio);
      sigmoid_distortion_gain = distortion_ratio + (1 - distortion_ratio) * (time - (time_shift + period * (1 - sigmoid_ratio))) / (period * sigmoid_ratio);
      value = wsin(magnitude, magnitude_shift, sigmoid_distortion_gain, t);
    }
  }
  else if ((sigmoid_ratio >= 2.0) && (sigmoid_ratio < 3.0))
  {
    double nsigmoid_ratio = sigmoid_ratio - 2.0;
    if (time <= time_shift + period * (1.0 - nsigmoid_ratio) * 0.5)
    {
      value = magnitude_shift;
    }
    else if (time >= time_shift + period * (1.0 + nsigmoid_ratio) * 0.5)
    {
      value = magnitude + magnitude_shift;
    }
    else
    {
      t = 2.0 * M_PI * (time - (time_shift + period * (1.0 - nsigmoid_ratio) * 0.5)) / (period * nsigmoid_ratio);
      sigmoid_distortion_gain = distortion_ratio + (1.0 - distortion_ratio) * (time - (time_shift + period * (1.0 - nsigmoid_ratio) * 0.5)) / (period * nsigmoid_ratio);
      value = wsin(magnitude, magnitude_shift, sigmoid_distortion_gain, t);
    }
  }
  else
  {
    value = magnitude_shift;
  }

  return value;
}

Rotation getRotationX(double angle)
{
  Rotation rotation(3, 3);

  rotation << 1.0, 0.0, 0.0, 0.0, cos(angle), -sin(angle), 0.0, sin(angle), cos(angle);

  return rotation;
}

Rotation getRotationY(double angle)
{
  Rotation rotation(3, 3);

  rotation << cos(angle), 0.0, sin(angle), 0.0, 1.0, 0.0, -sin(angle), 0.0, cos(angle);

  return rotation;
}

Rotation getRotationZ(double angle)
{
  Rotation rotation(3, 3);

  rotation << cos(angle), -sin(angle), 0.0, sin(angle), cos(angle), 0.0, 0.0, 0.0, 1.0;

  return rotation;
}

double calcHeightOnPlane(const l3::Point& point, const l3::Point& p, const l3::Vector3& n) { return (n.x() * point.x() + n.y() * point.y() - p.dot(n)) / (-n.z()); }

bool isPointOnPlane(const l3::Point& point, const l3::Point& p, const l3::Vector3& n, double eps)
{
  return abs(n.x() * point.x() + n.y() * point.y() + n.z() * point.z() - p.dot(n)) <= eps;
}

bool isPointInTriangle(const l3::Point& point, const l3::Point& p1, const l3::Point& p2, const l3::Point& p3)
{
  double A = (1.0 / 2.0) * (-p2.y() * p3.x() + p1.y() * (-p2.x() + p3.x()) + p1.x() * (p2.y() - p3.y()) + p2.x() * p3.y());
  double sign = A < 0.0 ? -1.0 : 1.0;
  double s = (p1.y() * p3.x() - p1.x() * p3.y() + (p3.y() - p1.y()) * point.x() + (p1.x() - p3.x()) * point.y()) * sign;
  double t = (p1.x() * p2.y() - p1.y() * p2.x() + (p1.y() - p2.y()) * point.x() + (p2.x() - p1.x()) * point.y()) * sign;
  return s >= 0 && t >= 0 && (s + t) <= 2 * A * sign;
}

bool isPointWithinPolygon(int x, int y, const std::vector<std::pair<int, int>>& points)
{
  /// Algorithm based on even-odd rule.
  /// Raycasting is performed from left to right (positiv direction along x-axis)

  int counter = 0;

  // loop through all edges of the polygon
  for (unsigned int i = 0; i < points.size(); i++)
  {
    unsigned int i_plus = (i + 1) % points.size();

    // check if on point
    if (points[i].first == x && points[i].second == y)
    {
      return true;
    }
    // catch special case of horizontal line
    else if (points[i].second == points[i_plus].second)
    {
      if ((points[i].first <= x) != (points[i_plus].first <= x))
      {
        // points on a horizontal line are considered as "inside"
        if (points[i].second == y)
          return true;
      }
    }
    // perform original even-odd counting
    else if ((points[i].second <= y) != (points[i_plus].second <= y))
    {
      float vt = static_cast<float>(points[i_plus].first - points[i].first) / static_cast<float>(points[i_plus].second - points[i].second);
      float t = points[i].first + vt * static_cast<float>(y - points[i].second);

      // points on a line are considered as "inside"
      if (std::abs(static_cast<float>(x) - t) < 0.001)
        return true;

      if (x < t)
        counter++;
    }
  }

  return counter & 1;
}

bool isPointWithinPolygon(double x, double y, const l3::PointArray& points)
{
  /// Algorithm based on even-odd rule.
  /// Raycasting is performed from left to right (positiv direction along x-axis)

  int counter = 0;

  // loop through all edges of the polygon
  for (unsigned int i = 0; i < points.size(); i++)
  {
    unsigned int i_plus = (i + 1) % points.size();

    // check if on point
    if (DOUBLE_EQ(points[i].x(), x) && DOUBLE_EQ(points[i].y(), y))
    {
      return true;
    }
    // catch special case of horizontal line
    else if (DOUBLE_EQ(points[i].y(), points[i_plus].y()))
    {
      if ((points[i].x() <= x) != (points[i_plus].x() <= x))
      {
        // points on a horizontal line are considered as "inside"
        if (DOUBLE_EQ(points[i].y(), y))
          return true;
      }
    }
    // perform original even-odd counting
    else if ((points[i].y() <= y) != (points[i_plus].y() <= y))
    {
      float vt = static_cast<float>(points[i_plus].x() - points[i].x()) / static_cast<float>(points[i_plus].y() - points[i].y());
      float t = points[i].x() + vt * static_cast<float>(y - points[i].y());

      // points on a line are considered as "inside"
      if (std::abs(static_cast<float>(x) - t) < 0.001)
        return true;

      if (x < t)
        counter++;
    }
  }

  return counter & 1;
}

Pose calcFeetCenter(const FootholdArray& footholds)
{
  ROS_ASSERT(!footholds.empty());

  double x, y, z, roll, pitch, yaw;
  x = y = z = roll = pitch = yaw = 0.0;

  const Foothold& ref_foot = footholds.front();

  for (const Foothold& f : footholds)
  {
    x += f.x() - ref_foot.x();
    y += f.y() - ref_foot.y();
    z += f.z() - ref_foot.z();
    roll += f.roll() - ref_foot.roll();
    pitch += f.pitch() - ref_foot.pitch();
    yaw += f.yaw() - ref_foot.yaw();
  }

  double num = static_cast<double>(footholds.size());
  return Pose(ref_foot.x() + x / num, ref_foot.y() + y / num, ref_foot.z() + z / num, ref_foot.roll() + roll / num, ref_foot.pitch() + pitch / num, ref_foot.yaw() + yaw / num);
}

Pose calcFeetCenter(const FootholdConstPtrArray& footholds)
{
  ROS_ASSERT(!footholds.empty());

  double x, y, z, roll, pitch, yaw;
  x = y = z = roll = pitch = yaw = 0.0;

  Foothold::ConstPtr ref_foot = footholds.front();

  for (Foothold::ConstPtr f : footholds)
  {
    x += f->x() - ref_foot->x();
    y += f->y() - ref_foot->y();
    z += f->z() - ref_foot->z();
    roll += f->roll() - ref_foot->roll();
    pitch += f->pitch() - ref_foot->pitch();
    yaw += f->yaw() - ref_foot->yaw();
  }

  double num = static_cast<double>(footholds.size());
  return Pose(ref_foot->x() + x / num, ref_foot->y() + y / num, ref_foot->z() + z / num, ref_foot->roll() + roll / num, ref_foot->pitch() + pitch / num,
              ref_foot->yaw() + yaw / num);
}
}  // namespace l3
