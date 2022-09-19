//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_MATH_MATH_H__
#define L3_MATH_MATH_H__

#include <cmath>

#include <l3_libs/types/types.h>

#include <l3_math/angles.h>

namespace l3
{
constexpr double pround(double x, double prec) { return ::round(x / prec) * prec; }

constexpr double pceil(double x, double prec) { return ::ceil(x / prec) * prec; }

constexpr double pfloor(double x, double prec) { return ::floor(x / prec) * prec; }

inline double clamp(double min, double max, double val) { return std::min(std::max(min, val), max); }

double powDI(double a, int b);

double wsin(double magnitude, double magnitude_shift, double sigmoid_distortion_gain, double time);

double wsigmoid(double time, double period, double time_shift, double magnitude, double magnitude_shift, double sigmoid_ratio, double distortion_ratio);

int sign(double x);

Rotation getRotationX(double angle);

Rotation getRotationY(double angle);

Rotation getRotationZ(double angle);

constexpr double norm_sq(double x, double y)
{
  // note: do *not* use pow() to square!
  return x * x + y * y;
}

constexpr double norm_sq(double x, double y, double z)
{
  // note: do *not* use pow() to square!
  return x * x + y * y + z * z;
}

constexpr double norm(double x, double y) { return sqrt(norm_sq(x, y)); }
constexpr double norm(double x, double y, double z) { return sqrt(norm_sq(x, y, z)); }

/**
 * @brief Calculates point on linear interpolation between [min; max] based in given (scaling) value in [0; range]
 * @param min Minimal out value
 * @param max Maximal out value
 * @param range Range of scaling value
 * @param value Point on interpolating slope which must be in [0; range]
 * @return
 */
constexpr double lerp(double min, double max, double range, double value) { return min + (((max - min) / range) * value); }

inline double slerp(double min, double max, double range, double value)
{
  double delta = shortestAngularDistance(min, max);
  return lerp(min, min + delta, range, value);
}

/**
 * @brief Calculates point on linear interpolation between [min; max] based in given (scaling) value in [0; 1]
 * @param min Minimal out value
 * @param max Maximal out value
 * @param value Point on interpolating slope which must be in [0; 1]
 * @return
 */
constexpr double lerp(double min, double max, double value) { return lerp(min, max, 1.0, value); }

inline double slerp(double min, double max, double value) { return slerp(min, max, 1.0, value); }

/**
 * @brief Interpolates between to given poses
 * @param from Initial pose
 * @param to Target pose
 * @param t Fraction [0; 1] to interpolate in between
 * @return Interpolate pose
 */
Position lerp(const Position& from, const Position& to, double t);
Pose lerp(const Pose& from, const Pose& to, double t);

/**
 * @brief Computes vector's angle spanned by two points.
 */
template <typename T>
inline double calcHeading(const T& from, const T& to)
{
  return calcOrientation(to - from);
}

inline double calcHeading(const Pose& from, const Pose& to) { return calcOrientation(to.getPosition() - from.getPosition()); }

template <typename T>
inline T calcTriangleNormal(const T& p1, const T& p2, const T& p3)
{
  return (p2 - p1).cross(p3 - p1);
}

/**
 * @brief Computes curve parameters based on an assumed driven path (dx, dy) on a circle.
 * @param dx Delta displacement in x direction
 * @param dy Delta displacement in y direction, must be != 0.0
 * @param angle [out] Heading of vehicle , assumed using ackermann based steering
 * @param radius [out] Curve radius
 * @return True, when parameters could be computed. Otherwise false.
 */
bool computeCircle(double dx, double dy, double& angle, double& radius);

/**
 * @brief Computes arc angle based on an assumed driven path (dx, dy) on a circle.
 * @param dx Delta displacement in x direction
 * @param dy Delta displacement in y direction
 * @return arc angle, 0 when y = 0.0
 */
double computeCircleAngle(double dx, double dy);

/**
 * @brief Determines z value on a plane defined by a point-normal pair for a given input point (x, y)
 * @param point Input point (x, y) to check
 * @param p point on plane
 * @param n normal of plane
 * @return z value projected on plane for given input point
 */
double calcHeightOnPlane(const l3::Point& point, const l3::Point& p, const l3::Vector3& n);

/**
 * @brief Checks if a given input point (x, y) is part of a specified ellipse.
 * @param point Input point (x, y) to check
 * @param center Center point (x, y) of the ellipsoid
 * @param size Size of prinicipal axis (x, y) of the ellipsoid
 * @param cos_theta Orientation theta of the ellipsoid given as cos(theta)
 * @param sin_theta Orientation theta of the ellipsoid given as sin(theta)
 * @return
 */
bool isPointInEllipse(const l3::Point& point, const l3::Point& center, const l3::Point& size, double cos_theta, double sin_theta);

/**
 * @brief Checks if a given input point (x, y) is part of a specified ellipse.
 * @param point Input point (x, y) to check
 * @param center Center point (x, y) of the ellipsoid
 * @param size Size of prinicipal axis (x, y) of the ellipsoid
 * @param theta Orientation of the ellipsoid given
 * @return
 */
inline bool isPointInEllipse(const l3::Point& point, const l3::Point& center, const l3::Point& size, double theta)
{
  return isPointInEllipse(point, center, size, cos(theta), sin(theta));
}

/**
 * @brief Determines input point (x, y, z) is on a plane defined by a point-normal pair
 * @param point Input point (x, y, z) to check
 * @param p point on plane
 * @param n normal of plane
 * @param eps max error in z to consider the point on the plane
 * @return returns true if point is on plane
 */
bool isPointOnPlane(const l3::Point& point, const l3::Point& p, const l3::Vector3& n, double eps = DBL_PREC);

/**
 * @brief Check if a point is in a triangle
 * @param point Point to check
 * @param p1 First corner
 * @param p2 Second corner
 * @param p3 Third corner
 * @return True if point is in the triangle
 */
bool isPointInTriangle(const l3::Point& point, const l3::Point& p1, const l3::Point& p2, const l3::Point& p3);

/**
 * @brief Crossing number method to determine whether a point lies within a
 * polygon or not (2D case only!)
 * @param points (x,y)-points defining the polygon.
 *
 * Check http://geomalgorithms.com/a03-_inclusion.html for further details.
 */
bool isPointWithinPolygon(int x, int y, const std::vector<std::pair<int, int>>& points);
bool isPointWithinPolygon(double x, double y, const l3::PointArray& points);

/**
 * @brief Determines geometrical center of robot based on given state/footholds.
 * @param state List of footholds from which the geometric center should be determined
 * @return Geometric center of robot
 */
Pose calcFeetCenter(const FootholdArray& footholds);
Pose calcFeetCenter(const FootholdConstPtrArray& footholds);
}  // namespace l3

#endif
