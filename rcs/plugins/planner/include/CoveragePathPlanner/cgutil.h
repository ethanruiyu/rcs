/**
 * @file torres_etal_2016.hpp
 * @brief Utility for computational geometry
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

#ifndef INCLUDED_cgutil_hpp_
#define INCLUDED_cgutil_hpp_

// c++ libraries
#include <algorithm>
#include <cmath>
#include <list>
#include <stack>
#include <vector>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>



using PointVector = std::vector<cv::Point>;
using LineSegment = std::array<cv::Point, 2>;
using LineSegmentVector = std::vector<LineSegment>;



/**
 * @brief Calculates signed area of given triangle
 * @param p1 The origin of vector \f$ \vec{p_1p_2} \f$ and \f$ \vec{p_1p_3} \f$
 * @param p2 The end point of vector \f$ \vec{p_1p_2} \f$
 * @param p3 The end point of vector \f$ \vec{p_1p_3} \f$
 * @return Signed area of given triangle
 *
 * @details
 * Signed area of triangle \f$ S(p1, p2, p3) \f$ is
 * half of the outer product of vector \f$ \vec{p_1p_2} \f$ and \f$ \vec{p_1p_3}
 * \f$.\n
 * \f[ S(p_1, p_2, p_3) = \frac{1}{2} \vec{p_1p_2} \times \vec{p_1p_3}\f] \n
 * And that can be written as follows,\n
 *   \f{eqnarray*}{
 *       S(p_1, p_2, p_3) & = & \frac{1}{2} \left|
 *           \begin{array}{ccc}
 *               p_1.x & p_1.y & 1 \\
 *               p_2.x & p_2.y & 1 \\
 *               p_3.x & p_3.y & 1
 *           \end{array}
 *       \right| \\
 *           & = & p_1.x(p_2.y - p_3.y) - p_1.y(p_2.x - p_3.x) - (p_2.x\times
 * p_3.y - p_2.y\times p_3.x)
 *   \f}
 */
inline double calculateSignedArea(const cv::Point& p1, const cv::Point& p2,
                                  const cv::Point& p3)
{
  return p1.x * (p2.y - p3.y) - p1.y * (p2.x - p3.x) + (p2.x * p3.y - p2.y * p3.x);
}

/**
 * @brief Check equality of two points
 * @param p1
 * @param p2
 * @return bool
 * @details See https://stackoverflow.com/questions/4010240/comparing-doubles
 */
bool operator==(const cv::Point& p1, const cv::Point& p2);

/**
 * @brief Check equality of two points
 * @param p1
 * @param p2
 * @return bool
 */
bool operator!=(const cv::Point& p1, const cv::Point& p2);

/**
 * @brief Generate Vector of line segment from given PointVector
 * @param vec
 * @param isClosed Set true if the first point and the last are connected
 * @return LineSegmentVector Vector of line segment a.k.a. std::vector<std::array<geometry_msgs::Point, 2>>
 */
LineSegmentVector generateEdgeVector(const PointVector& vec, bool isClosed);

/**
 * @brief Calculates distance between given two points
 * @param p1
 * @param p2
 * @return double Distance between two points
 */
inline double calculateDistance(const cv::Point& p1, const cv::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

/**
 * @brief Calculates angle between segment p1p2 and p1p3
 * @param p1 A vertex which is the origin of segment p1p2 and p1p3
 * @param p2 The other point of segment p1p2
 * @param p3 The other point of segment p1p3
 * @return Angle between segment p1p2 and p1p3 in radian [0, pi)
 */
double calculateVertexAngle(const cv::Point& p1, const cv::Point& p2,
                            const cv::Point& p3);

/**
 * @brief Calculates angle between segment p1p2 and horizontal line
 * @param p1 A vertex which is the origin of segment p1p2
 * @param p2 The other vertex of segment p1p2
 * @return Vertex angle of p1 in radian [-pi, pi]
 */
double calculateHorizontalAngle(const cv::Point& p1, const cv::Point& p2);

/**
 * @brief Calculates distance between given edge and vertex
 * @param edge An edge of given polygon
 * @param vertex A vertex of given polygon
 * @return double Distance between edge and vertex
 */
double calculateDistance(const LineSegment& edge, const cv::Point& vertex);

/**
 * @brief Returns convex hull of given points
 * @param points A set of points in the plane
 * @return Convex hull of given points
 *
 * This function is based on graham scan algorithm
 */
PointVector computeConvexHull(PointVector points);

/**
 * @brief Checks if given polygon is convex or not
 * @param points Points consisting of polygon is to be checked
 * @return True if given polygon is convex, false if it's not convex
 */
inline bool isConvex(PointVector points)
{
  return computeConvexHull(points).size() == points.size();
}

/**
 * @brief Checks if given edges intersect each other
 * @param edge1 An edge
 * @param edge2 An edge
 * @return True if two edges intersect
 */
/**bool hasIntersection(const LineSegment& edge1, const LineSegment& edge2)
{
  geometry_msgs::Point p1, p2, p3, p4;

  try
  {
    p1 = edge1.at(0);
    p2 = edge1.at(1);
    p3 = edge2.at(0);
    p4 = edge2.at(1);
  }
  catch (std::out_of_range& ex)
  {
    //ROS_ERROR("%s", ex.what());
    return false;
  }

  bool condA = ((p1.x - p2.x) * (p3.y - p1.y) + (p1.y - p2.y) * (p1.x - p3.x)) *
                   ((p1.x - p2.x) * (p4.y - p1.y) + (p1.y - p2.y) * (p1.x - p4.x)) <
               0;
  bool condB = ((p3.x - p4.x) * (p1.y - p3.y) + (p3.y - p4.y) * (p3.x - p1.x)) *
                   ((p3.x - p4.x) * (p2.y - p3.y) + (p3.y - p4.y) * (p3.x - p2.x)) <
               0;

  if (condA and condB)
  {
    return true;
  }
  else
  {
    return false;
  }
}**/

double mult(cv::Point a, cv::Point b, cv::Point c);

bool hasIntersection(LineSegment edge1, LineSegment edge2);

/**
 * @brief Checks if given vectors of edges have at least one intersection
 * @param vec1 Vector of line segments
 * @param vec2 Vector of line segments
 * @return True if given two vectors of edges have at least one intersection
 */
bool hasIntersection(const LineSegmentVector& vec1, const LineSegmentVector& vec2);

/**
 * @brief Find the location where given edges intersect each other
 * @param edge1
 * @param edge2
 * @return geometry_msgs::Point Point of intersection
 * @details See http://mf-atelier.sakura.ne.jp/mf-atelier/modules/tips/program/algorithm/a1.html
 */
cv::Point localizeIntersection(const LineSegment& edge1, const LineSegment& edge2);

/**
 * @brief Rotate points by given angle around the origin
 * @param points Points to be rotated
 * @param angle_rad Rotation angle in radian
 * @return PointVector Rotated points
 */
PointVector rotatePoints(const PointVector& points, double angle_rad);

/**
 * @brief Decompose given polygon
 * @param polygon Polygon to be decomposed
 * @return std::vector<PointVector> Decomposed polygons
 * @details
 * This function uses CGAL::optimal_convex_partition_2 in order to perform optimal polygon decomposition.
 * Note that this function has O(n^4) time complexity and O(n^3) space complexity.
 * Use approx_convex_partition_2 instead if the number of vertices are big because its time complexity is O(n).
 * But apptox_convex_partition_2 generates more polygons.
 * For detail, see https://doc.cgal.org/latest/Partition_2/index.html
 */
/**std::vector<PointVector> decomposePolygon(const PointVector& polygon)
{
  std::vector<PointVector> decomposedPolygons;

  // generate Polygon of CGAL from PointVector
  Polygon_2 cgalPolygon;
  for (const auto& vertex : polygon)
  {
    cgalPolygon.push_back(Point_2(vertex.x, vertex.y));
  }

  Polygon_list partialCGALPolygons;
  Traits partitionTraits;

  // note that this function has O(n^4) time complexity and O(n^3) space complexity
  // use approx_convex_partition_2 instead if the number of vertices are big because its time complexity is O(n)
  // but apptox_convex_partition_2 generates more polygons
  CGAL::optimal_convex_partition_2(cgalPolygon.vertices_begin(), cgalPolygon.vertices_end(),
                                   std::back_inserter(partialCGALPolygons), partitionTraits);

  // generate std::vector<PointVector> from polygon of CGAL
  for (const auto& partialCGALPolygon : partialCGALPolygons)
  {
    PointVector partialPolygon;
    for (auto itr = partialCGALPolygon.vertices_begin(); itr != partialCGALPolygon.vertices_end(); ++itr)
    {
      geometry_msgs::Point pt;
      pt.x = itr->x();
      pt.y = itr->y();
      partialPolygon.push_back(pt);
    }
    decomposedPolygons.push_back(partialPolygon);
  }

  return decomposedPolygons;
}**/

#endif