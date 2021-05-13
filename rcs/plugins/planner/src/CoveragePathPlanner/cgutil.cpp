//
// Created by tuqirui on 21-1-12.
//

#include "CoveragePathPlanner/cgutil.h"


bool operator==(const cv::Point& p1, const cv::Point& p2)
{
    bool x =
            p1.x == p2.x || std::abs(p1.x - p2.x) < std::abs(std::min(p1.x, p2.x)) * std::numeric_limits<double>::epsilon();
    bool y =
            p1.y == p2.y || std::abs(p1.y - p2.y) < std::abs(std::min(p1.y, p2.y)) * std::numeric_limits<double>::epsilon();
    //bool z =
    //p1.z == p2.z || std::abs(p1.z - p2.z) < std::abs(std::min(p1.z, p2.z)) * std::numeric_limits<double>::epsilon();

    return x and y;
}

bool operator!=(const cv::Point& p1, const cv::Point& p2)
{
    return !(p1 == p2);
}


LineSegmentVector generateEdgeVector(const PointVector& vec, bool isClosed)
{
    LineSegmentVector edgeVector;

    // break if vector is empty
    if (vec.empty() == true)
    {
        return edgeVector;
    }

    for (int i = 0; i < vec.size(); ++i)
    {
        LineSegment edge;

        edge.at(0) = vec.at(i);

        if (i < vec.size() - 1)
        {
            // except for the last vertex
            edge.at(1) = vec.at(i + 1);
        }
        else
        {
            // for the last vertex
            edge.at(1) = vec.at(0);
            if (not isClosed)
            {
                break;
            }
        }

        edgeVector.push_back(edge);
    }
    return edgeVector;
}


double calculateVertexAngle(const cv::Point& p1, const cv::Point& p2,
                            const cv::Point& p3)
{
    // Length of edges composed of vertices
    // e1: (p1, p2)
    // e2: (p2, p3)
    // e3: (p3, p1)
    double lenE1, lenE2, lenE3;
    lenE1 = calculateDistance(p2, p1);
    lenE2 = calculateDistance(p2, p3);
    lenE3 = calculateDistance(p3, p1);

    // Cosine of angle  between segment p1p2 and p1p3 (Law of cosines)
    double cosineP1;
    cosineP1 = (std::pow(lenE1, 2) + std::pow(lenE3, 2) - std::pow(lenE2, 2)) / (2 * lenE1 * lenE3);

    // vertex angle is 0.0 if lenE1 or lenE3 is zero
    // that means p1 and p2 or p1 and p3 is the same point
    if (std::isnan(cosineP1) != 0)
    {
        return 0.0;
    }

    return std::acos(cosineP1);
}


double calculateHorizontalAngle(const cv::Point& p1, const cv::Point& p2)
{
    cv::Point p3;
    p3.x = p1.x + 1.0;
    p3.y = p1.y;
    return p1.y > p2.y ? -calculateVertexAngle(p1, p2, p3) : calculateVertexAngle(p1, p2, p3);
}


double calculateDistance(const LineSegment& edge, const cv::Point& vertex)
{
    // Vertices of triangle
    cv::Point pointA, pointB;
    pointA = edge.front();
    pointB = edge.back();

    // Calculate length of each edge
    // Edge A: An edge facing vertex A
    // Edge B: An edge facing vertex B
    double lenEdgeA, lenEdgeB;
    lenEdgeA = calculateDistance(pointB, vertex);
    lenEdgeB = calculateDistance(vertex, pointA);

    // Vertex angles
    // alpha: vertex angle of pointA
    // beta: vertex angle of pointB
    double alpha, beta;
    alpha = calculateVertexAngle(pointA, pointB, vertex);
    beta = calculateVertexAngle(pointB, pointA, vertex);

    double distance = alpha < M_PI_2 ? std::sin(alpha) * lenEdgeB : std::sin(beta) * lenEdgeA;

    return distance;
}


PointVector computeConvexHull(PointVector points)
{
    PointVector convexHull;

    if (points.empty() or points.size() < 3)
    {
        return convexHull;
    }

    // remove points that have same coodinate with other points
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        if (points.at(i).x == points.at(i + 1).x and points.at(i).y == points.at(i + 1).y)
        {
            points.erase(points.begin() + i);
        }
    }

    if (points.size() < 3)
    {
        return convexHull;
    }

    // sort by vertex's y coordinate in an ascending order
    std::stable_sort(points.begin(), points.end(),
                     [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });

    // point with minimum y coordinate
    cv::Point pointMinY = points.front();
    points.erase(points.begin());

    // sort by an angle between a segment composed of pointMinY and pj (in a set
    // of points) and horizontal line
    // in an ascending order
    //double a1 = calculateHorizontalAngle(pointMinY, points[2]);
    double a2 = calculateHorizontalAngle(pointMinY, points[3]);
    std::stable_sort(points.begin(), points.end(),
                     [&](const cv::Point& p1, const cv::Point& p2) {return calculateHorizontalAngle(pointMinY, p1) < calculateHorizontalAngle(pointMinY, p2);});

    // add pointMinY in convex hull
    convexHull.push_back(pointMinY);

    // add the point with minimum angle
    convexHull.push_back(points.front());
    points.erase(points.begin());

    for (const auto& point : points)
    {
        for (std::size_t i = convexHull.size() - 1; i > 1; --i)
        {
            if (calculateSignedArea(convexHull.at(i - 1), convexHull.at(i), point) >= 0)
            {
                break;
            }

            convexHull.pop_back();
        }
        convexHull.push_back(point);
    }

    /** geometry_msgs::Point origin;
     origin.x = convexHull[0].x;
     origin.y = convexHull[0].y;
     std::stable_sort(convexHull.begin(), convexHull.end(),
                      [&](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
                        return calculateHorizontalAngle(origin, p1) < calculateHorizontalAngle(origin, p2);
                      });**/

    return convexHull;
}



double mult(cv::Point a, cv::Point b, cv::Point c)
{
    return (a.x-c.x)*(b.y-c.y)-(b.x-c.x)*(a.y-c.y);
}

bool hasIntersection(LineSegment edge1, LineSegment edge2)
{
    cv::Point aa, bb, cc, dd;
    aa = edge1[0];
    bb = edge1[1];
    cc = edge2[0];
    dd = edge2[1];
    if ( std::max(aa.x, bb.x) < std::min(cc.x, dd.x) )
    {
        return false;
    }
    if ( std::max(aa.y, bb.y) < std::min(cc.y, dd.y) )
    {
        return false;
    }
    if ( std::max(cc.x, dd.x) < std::min(aa.x, bb.x) )
    {
        return false;
    }
    if ( std::max(cc.y, dd.y) < std::min(aa.y, bb.y) )
    {
        return false;
    }
    if ( mult(cc, bb, aa)*mult(bb, dd, aa)<0 )
    {
        return false;
    }
    if ( mult(aa, dd, cc)*mult(dd, bb, cc)<0 )
    {
        return false;
    }
    return true;


}


bool hasIntersection(const LineSegmentVector& vec1, const LineSegmentVector& vec2)
{
    for (const auto& segment1 : vec1)
    {
        for (const auto& segment2 : vec2)
        {
            if (hasIntersection(segment1, segment2) == true)
            {
                return true;
            }
        }
    }

    return false;
}


cv::Point localizeIntersection(const LineSegment& edge1, const LineSegment& edge2)
{
    cv::Point p1, p2, p3, p4;

    try
    {
        p1 = edge1.at(0);
        p2 = edge1.at(1);
        p3 = edge2.at(0);
        p4 = edge2.at(1);
    }
    catch (const std::out_of_range& ex)
    {
        //ROS_ERROR("%s", ex.what());
    }

    double xi, eta, delta;
    xi = (p4.y - p3.y) * (p4.x - p1.x) - (p4.x - p3.x) * (p4.y - p1.y);
    eta = -(p2.y - p1.y) * (p4.x - p1.x) + (p2.x - p1.x) * (p4.y - p1.y);
    delta = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);

    double lambda, mu;
    lambda = xi / delta;
    mu = eta / delta;

    cv::Point intersection;

    intersection.x = p1.x + lambda * (p2.x - p1.x);
    intersection.y = p1.y + lambda * (p2.y - p1.y);

    return intersection;
}


PointVector rotatePoints(const PointVector& points, double angle_rad)
{
    std::array<double, 4> rotationMatrix;
    rotationMatrix.at(0) = std::cos(angle_rad);
    rotationMatrix.at(1) = -std::sin(angle_rad);
    rotationMatrix.at(2) = std::sin(angle_rad);
    rotationMatrix.at(3) = std::cos(angle_rad);

    PointVector rotatedPoints;

    for (const auto& point : points)
    {
        cv::Point pt;
        pt.x = rotationMatrix.at(0) * point.x + rotationMatrix.at(1) * point.y;
        pt.y = rotationMatrix.at(2) * point.x + rotationMatrix.at(3) * point.y;
        rotatedPoints.push_back(pt);
    }
    return rotatedPoints;
}

