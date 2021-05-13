//
// Created by tuqirui on 21-1-12.
//

#include "CoveragePathPlanner/torres_etal.h"

Direction identifyOptimalSweepDir(const PointVector& polygon)
{
    Direction sweepDirection;

//  PointVector convexHull = polygon;
    PointVector convexHull = computeConvexHull(polygon);

    // Edges of polygon
    LineSegmentVector edges;

    // Make a list of edges of polygon
    for (std::size_t i = 0; i < convexHull.size(); ++i)
    {
        LineSegment ar;

        ar.at(0) = convexHull.at(i);

        // if vertex is the last one,
        // that vertex makes an edge whose end is the first vertex
        if (i == convexHull.size() - 1)
        {
            ar.at(1) = convexHull.at(0);
        }
        else
        {
            ar.at(1) = convexHull.at(i + 1);
        }
        edges.push_back(ar);
    }

//  double optimalDistance = 0;
//
//  // Calculate line sweep direction
//  // Algorithm 1 in Torres et al, 2016
//  for (const auto& edge : edges | boost::adaptors::indexed())
//  {
//    double edgeMaxDistance = 0;
//    cv::Point opposedVertex;
//
//    for (const cv::Point& vertex : convexHull)
//    {
//      // calculateDistance() function returns distance
//      // between given edge and vertex
//      double distance = calculateDistance(edge.value(), vertex);
//
//      if (distance > edgeMaxDistance)
//      {
//        edgeMaxDistance = distance;
//        opposedVertex = vertex;
//      }
//    }
//
//    if ((edgeMaxDistance < optimalDistance) or edge.index() == 0)
//    {
//      optimalDistance = edgeMaxDistance;
//      sweepDirection.baseEdge = edge.value();
//      sweepDirection.opposedVertex = opposedVertex;
//    }
//  }
    double longest_edge = 0;
    for (const auto& edge : edges)
    {
        double edge_l = calculateDistance(edge[0], edge[1]);
        if(edge_l > longest_edge)
        {
            longest_edge = edge_l;
            double edgeMaxDistance = 0;
            for (const cv::Point& vertex : convexHull)
            {
                // calculateDistance() function returns distance
                // between given edge and vertex
                double distance = calculateDistance(edge, vertex);

                if (distance > edgeMaxDistance)
                {
                    edgeMaxDistance = distance;
                    sweepDirection.baseEdge = edge;
                    sweepDirection.opposedVertex = vertex;
                }
            }
        }
    }


    return sweepDirection;
}


bool find_egde_point_left(cv::Point& p, cv::Point& p1, LineSegmentVector rotatedEdges, double padding)
{

    float l = 0;
    float l2 = 0;
    while (l2 <= 0.8*padding)
    {
        float l_min = DBL_MAX;//MAXFLOAT
        p.x += 2;
        for(auto edge : rotatedEdges)
        {
//            l = getDist_P2L(p, edge[0], edge[1]);
            l = cal_point_to_line(p, edge[0], edge[1]);
            if(l < l_min)
            {
                l_min = l;
                l2 = l;
            }
        }
        if(p.x > p1.x)
        {
            return false;
        }
    }
    return true;
}

bool find_egde_point_right(cv::Point& p, cv::Point& p1, LineSegmentVector rotatedEdges, double padding)
{
    float l = 0;
    float l2 = 0;
    while (l2 <= 0.8*padding)
    {
        float l_min = DBL_MAX;
        p.x -= 2;
        for(auto edge : rotatedEdges)
        {
//            l = getDist_P2L(p, edge[0], edge[1]);
            l = cal_point_to_line(p, edge[0], edge[1]);
            if(l < l_min)
            {
                l_min = l;
                l2 = l;
            }
        }
        if(p.x < p1.x)
        {
            return false;
        }
    }
    return true;
}

PointVector reshapePath(const PointVector& path, double padding, LineSegmentVector rotatedEdges)
{
    PointVector zigzagPath;

    // reshape every traverse
    for (int i = 0; i < std::round(path.size() / 2); ++i)
//    int i=24;
    {
        // even-numbered traverse
        if (i % 2 == 0)
        {
            try
            {
                // in case that the first point of the traverse is located on LEFT side
                if (path.at(2 * i).x < path.at(2 * i + 1).x)
                {
                    cv::Point p1 = path.at(2 * i);
                    cv::Point p2 = path.at(2 * i + 1);

                    bool a =  find_egde_point_left(p1, p2, rotatedEdges, padding);
                    bool b = find_egde_point_right(p2, p1, rotatedEdges, padding);
                    if(find_egde_point_left(p1,p2, rotatedEdges, padding) &&
                       find_egde_point_right(p2,p1, rotatedEdges, padding))
                    {
                        zigzagPath.push_back(p1);
                        zigzagPath.push_back(p2);
                    }
//          if(find_egde_point_left(p1, p2, rotatedEdges, padding))
//          {
//              zigzagPath.push_back(p1);
//          }
//          if(find_egde_point_right(p2, p1, rotatedEdges, padding))
//          {
//
//              zigzagPath.push_back(p2);
//          }

//

                    // add padding
//          p1.x += padding;
//          p2.x -= padding;

                    // be careful with the order of points
//          zigzagPath.push_back(p1);
//          zigzagPath.push_back(p2);
                }
                    // in case that the first point of the traverse is located on RIGHT side
                else
                {
                    cv::Point p1 = path.at(2 * i);
                    cv::Point p2 = path.at(2 * i + 1);

                    // add padding
//          p1.x -= padding;
//          p2.x += padding;
                    bool a =  find_egde_point_left(p2, p1, rotatedEdges, padding);
                    bool b = find_egde_point_right(p1, p2, rotatedEdges, padding);
                    if(find_egde_point_left(p2, p1, rotatedEdges, padding)&&
                       find_egde_point_right(p1, p2, rotatedEdges, padding))
                    {
                        zigzagPath.push_back(p2);
                        zigzagPath.push_back(p1);
                    }

//          if(find_egde_point_left(p2, p1, rotatedEdges, padding))
//          {
//              zigzagPath.push_back(p2);
//          }
//          if(find_egde_point_right(p1, p2, rotatedEdges, padding))
//          {
//
//            zigzagPath.push_back(p1);
//          }

                    // be careful with the order of points
//          zigzagPath.push_back(p2);
//          zigzagPath.push_back(p1);
                }
            }
                // in case that the traverse has only one vertex
            catch (std::out_of_range& ex)
            {
                cv::Point p = path.at(2 * i);
                if (isClockWise(path))
                {
                    // the first vertex of even-numbered traverse of clockwise path is located on RIGHT side of polygon
                    p.x += padding;
                    zigzagPath.push_back(p);
                }
                else
                {
                    // the first vertex of even-numbered traverse of counterclockwise path is located on LEFT side of polygon
                    p.x -= padding;
                    zigzagPath.push_back(p);
                }
//        ROS_ERROR("%s", ex.what());
            }
        }
            // odd-numbered traverse
        else
        {
            try
            {
                // in case that the first point of the traverse is located on RIGHT side
                if (path.at(2 * i).x > path.at(2 * i + 1).x)
                {
                    cv::Point p1 = path.at(2 * i);
                    cv::Point p2 = path.at(2 * i + 1);

                    // add padding
//          p1.x -= padding;
//          p2.x += padding;
                    bool a =  find_egde_point_left(p2, p1, rotatedEdges, padding);
                    bool b = find_egde_point_right(p1, p2, rotatedEdges, padding);
                    if(find_egde_point_left(p2, p1, rotatedEdges, padding)&&
                       find_egde_point_right(p1, p2, rotatedEdges, padding))
                    {
                        zigzagPath.push_back(p1);
                        zigzagPath.push_back(p2);
                    }

//            if(find_egde_point_right(p1, p2, rotatedEdges, padding))
//            {
//
//                zigzagPath.push_back(p1);
//            }
//          if(find_egde_point_left(p2, p1, rotatedEdges, padding))
//            {
//                zigzagPath.push_back(p2);
//            }


                    // be careful with the order of points
//          zigzagPath.push_back(p1);
//          zigzagPath.push_back(p2);
                }
                    // in case that the first point of the traverse is located on LEFT side
                else
                {
                    cv::Point p1 = path.at(2 * i);
                    cv::Point p2 = path.at(2 * i + 1);

                    // add padding
//          p1.x += padding;
//          p2.x -= padding;
                    bool a =  find_egde_point_left(p1, p2, rotatedEdges, padding);
                    bool b = find_egde_point_right(p2, p1, rotatedEdges, padding);
                    if(find_egde_point_left(p1, p2, rotatedEdges, padding)&&
                       find_egde_point_right(p2, p1, rotatedEdges, padding))
                    {
                        zigzagPath.push_back(p2);
                        zigzagPath.push_back(p1);
                    }

//            if(find_egde_point_right(p2, p1, rotatedEdges, padding))
//            {
//
//                zigzagPath.push_back(p2);
//            }
//          if(find_egde_point_left(p1, p2, rotatedEdges, padding))
//            {
//                zigzagPath.push_back(p1);
//            }


                    // be careful with the order of points
//          zigzagPath.push_back(p2);
//          zigzagPath.push_back(p1);
                }
            }
                // in case that the traverse has only one vertex
            catch (std::out_of_range& ex)
            {
                cv::Point p = path.at(2 * i);
                if (isClockWise(path))
                {
                    // the first vertex of odd-numbered traverse of clockwise path is located on LEFT side of polygon
                    p.x -= padding;
                    zigzagPath.push_back(p);
                }
                else
                {
                    // the first vertex of odd-numbered traverse of clockwise path is located on RIGHT side of polygon
                    p.x += padding;
                    zigzagPath.push_back(p);
                }
                //       ROS_ERROR("%s", ex.what());
            }
        }
    }
    return zigzagPath;
}

void check_intersection(PointVector polygon, PointVector path, PointVector& path2, cv::Mat PRM_map)
{
    //如果点不在多边形内，删除该点
    for(int i=0; i<path.size(); i++)
    {
        if(PRM_map.at<uchar>(path[i]) != 0)
        {
            path.erase(path.begin()+i);
        }
    }

    LineSegmentVector sweeplineVector = generateEdgeVector(path, false);
    LineSegmentVector edgeVector = generateEdgeVector(polygon, true);
    LineSegmentVector sweeplineVector2;
    bool is_has_intersection;
    for(int i=0; i<sweeplineVector.size(); i++)
    {
        is_has_intersection = false;
        for (const auto& edge : edgeVector)
        {
            if(hasIntersection(sweeplineVector[i], edge))
            {
                is_has_intersection = true;
                break;
            }
        }
        if(!is_has_intersection)
        {
            sweeplineVector2.push_back(sweeplineVector[i]);
        }
        else
        {
//            std::cout << "path has intersections with edge, start prm path plan" << i << std::endl;
            if(i<sweeplineVector.size()-1)
            {
                std::cout << "===========path has intersections with edge, start prm path plan========" << i << std::endl;
                std::vector<int> point_x, point_y;
                point_x = {sweeplineVector[i-1][0].x, sweeplineVector[i-1][1].x, sweeplineVector[i+1][0].x, sweeplineVector[i+1][1].x};
                point_y = {sweeplineVector[i-1][0].y, sweeplineVector[i-1][1].y, sweeplineVector[i+1][0].y, sweeplineVector[i+1][1].y};
                auto p_x_min = std::min_element(point_x.begin(), point_x.end());
                auto p_x_max = std::max_element(point_x.begin(), point_x.end());
                auto p_y_min = std::min_element(point_y.begin(), point_y.end());
                auto p_y_max = std::max_element(point_y.begin(), point_y.end());

                PRMPlanner prm_planner(30, 0, *(p_x_min), *(p_x_max), *(p_y_min), *(p_y_max));//*(p_x_min), *(p_x_max), *(p_y_min), *(p_y_max)
                prm_planner.initialize(PRM_map);
                prm_planner.plan(sweeplineVector[i][0].x, sweeplineVector[i][0].y, sweeplineVector[i][1].x, sweeplineVector[i][1].y);
                std::vector<TGlobalOrd> prm_path = prm_planner.getpath();
                std::vector<cv::Point> prm_path_cv;
                cv::Point prm_point_cv;
                for(auto point : prm_path)
                {
                    prm_point_cv.x = point.x;
                    prm_point_cv.y = point.y;
                    prm_path_cv.push_back(prm_point_cv);
                }
                LineSegmentVector prm_line_vector = generateEdgeVector(prm_path_cv, false);
                for(auto prm_line : prm_line_vector)
                {
                    sweeplineVector2.push_back(prm_line);
                }
            }
            else
                continue;

        }
    }

    for(int i=0; i<sweeplineVector2.size(); i++)
    {
        path2.push_back(sweeplineVector2[i][0]);
    }
    path2.push_back(*((sweeplineVector2.end()-1)->end()-1));

}


bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           const Direction& sweepDirection, PointVector& path, cv::Mat PRM_map)
{
    // Unable to make polygon with less than 3 points
    if (polygon.size() < 3)
    {
        return false;
    }

    // TODO: Change to configurable
    const double padding = std::floor(footprintWidth/3); //(footprintWidth/2.0 * 1.3)

    // rotate input polygon so that baseEdge become horizontal
    double rotationAngle = calculateHorizontalAngle(sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back());
    PointVector rotatedPolygon = rotatePoints(polygon, -rotationAngle);

    // find x coordinate of most left and most right point
    //double minX = INT_MAX, maxX = INT_MIN;
    double minX (0), maxX (0);
    for (const auto& vertex : rotatedPolygon)
    {
        if (vertex.x < minX)
        {
            minX = vertex.x;
        }
        else if (vertex.x > maxX)
        {
            maxX = vertex.x;
        }
    }

    double stepWidth = footprintWidth * (1 - horizontalOverwrap);

    // calculate sweep direction of rotated polygon
    PointVector dir{ sweepDirection.opposedVertex, sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back() };
    dir = rotatePoints(dir, -rotationAngle);
    Direction rotatedDir;
    rotatedDir.opposedVertex = dir.at(0);
    rotatedDir.baseEdge.front() = dir.at(1);
    rotatedDir.baseEdge.back() = dir.at(2);

    int stepNum = std::ceil((calculateDistance(rotatedDir.baseEdge, rotatedDir.opposedVertex) ) / stepWidth);

    LineSegmentVector sweepLines;

    // generate list of sweep lines which is horizontal against the base edge
    for (int i = 0; i < stepNum; ++i)
    {
        LineSegment ar;
        cv::Point p1, p2;
        p1.x = minX;
        p1.y = rotatedDir.baseEdge.at(0).y + (i * stepWidth) + padding;
        p2.x = maxX;
        p2.y = rotatedDir.baseEdge.at(1).y + (i * stepWidth) + padding;
        //std::cout << minX << "  " << maxX  << std::endl;

        ar.at(0) = p1;
        ar.at(1) = p2;
        sweepLines.push_back(ar);
    }
    if(sweepLines.empty())
        return false;

    if(std::abs(rotatedDir.opposedVertex.y - ((sweepLines.end()-1)->end()-1)->y) >= padding && polygon.size() >= 4)
    {
        LineSegment ab;
        cv::Point p3, p4;
        p3.x = minX;
        p3.y = rotatedDir.opposedVertex.y - padding;
        p4.x = maxX;
        p4.y = rotatedDir.opposedVertex.y - padding;
        ab.at(0) = p3;
        ab.at(1) = p4;
        sweepLines.push_back(ab);
    }
    if(!sweepLines.empty() && std::abs(rotatedDir.opposedVertex.y - ((sweepLines.end()-1)->end()-1)->y) <= padding && polygon.size() >= 4)
    {
        (sweepLines.end()-1)->begin()->x = minX;
        (sweepLines.end()-1)->begin()->y = rotatedDir.opposedVertex.y - padding;
        ((sweepLines.end()-1)->end()-1)->x = maxX;
        ((sweepLines.end()-1)->end()-1)->y = rotatedDir.opposedVertex.y - padding;
    }


    // Localize intersections of sweeplines and edges of rotated polygon
    LineSegmentVector rotatedEdges = generateEdgeVector(rotatedPolygon, true);

    PointVector intersections;

    for (const auto& sweepLine : sweepLines)
    {
        int intersectionCount = 0;
        for (const auto& edge : rotatedEdges)
        {
            bool is_same_point = false;
            if(edge[0].y == sweepLine[0].y && edge[1].y == sweepLine[1].y)
                continue;
            if (hasIntersection(sweepLine, edge))
            {
                cv::Point intersection = localizeIntersection(edge, sweepLine);
                for(auto point : intersections)
                {
                    if(intersection.x == point.x &&  intersection.y == point.y) //判断交点是否是已存在的点
                    {
//                        std::cout << "intersections has same point" << std::endl;
                        is_same_point = true;
                    }
                }
                if(!is_same_point)
                {
                    intersections.push_back(intersection);
                    ++intersectionCount;
                }

            }

            // sweep line in optimal path does not have more than 2 intersections
            if (intersectionCount > 3) //扫描线与多边形交点超过3个
            {
//                std::cout << " intersectionCount >= 3 "<< std::endl;
                return false;
            }
        }
        if (intersectionCount == 1) //如果扫描线与多边形交点只有一个， 移除该交点 判断下一条扫描线
        {
            intersections.pop_back();
            continue;
        }
        if (intersectionCount == 3) //如果扫描线与多边形交点有3个， 移除中间交点
        {
            std::vector<cv::Point> intersection_point_list;
            for(int i=0; i<3; i++)
            {
                intersection_point_list.push_back(*(intersections.end()-1));
                intersections.pop_back();
            }
            //find min point
            int x_max = INT_MAX;
            cv::Point point_min;
            for(auto point1 : intersection_point_list)
            {
                if(point1.x < x_max)
                {
                    x_max = point1.x;
                    point_min = point1;
                }
            }
            //find max point
            int x_min = INT_MIN;
            cv::Point point_max;
            for(auto point2 : intersection_point_list)
            {
                if(point2.x > x_min)
                {
                    x_min = point2.x;
                    point_max = point2;
                }
            }
            intersections.push_back(point_min);
            intersections.push_back(point_max);
            continue;
        }

        if(intersections.size() != 0)
        {
            if(std::abs(intersections[intersections.size()-1].x - intersections[intersections.size()-2].x) < stepWidth/2)
            {
//            intersections.pop_back();
                intersections.pop_back();

            }
        }

    }

    // sort points by y coordinate in ascending order
    std::stable_sort(intersections.begin(), intersections.end(),
                     [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });

    PointVector rotatedPath = reshapePath(intersections, padding, rotatedEdges);


    PointVector path_uncheck;
    path_uncheck = rotatePoints(rotatedPath, rotationAngle);

    path = path_uncheck;

    return true;
}


bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           PointVector& path, cv::Mat PRM_map)
{
    Direction sweepDirection = identifyOptimalSweepDir(polygon);  //所有顶点到所有边中 最短距离的顶点和边
    return computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, sweepDirection, path, PRM_map);
}


double calculatePathLength(const PointVector& path)
{
    if (path.size() < 2)
    {
        return 0;
    }

    double pathLength = 0;
    for (int i = 0; i < path.size() - 1; ++i)
    {
        pathLength += calculateDistance(path.at(i), path.at(i + 1));
    }
    return pathLength;
}


PointVector computeCCWPath(PointVector path)
{
    for (int i = 0; i < std::round(path.size() / 2); ++i)
    {
        // swap the first point and the last point in each sweep line
        cv::Point tmp = path.at(2 * i);

        path.at(2 * i) = path.at(2 * i + 1);
        try
        {
            path.at(2 * i + 1) = tmp;
        }
        catch (std::out_of_range& ex)
        {
//      ROS_ERROR("%s", ex.what());
        }
    }
    return path;
}


PointVector computeOppositePath(const PointVector& path)
{
    PointVector oppositePath;

    // inversely iterate given points
    for (int i = path.size() - 1; i >= 0; --i)
    {
        oppositePath.push_back(path.at(i));
    }

    return oppositePath;
}


PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const cv::Point& start, const cv::Point& end)
{
    // The naming of the following variable follows torres et al. 2016
    std::unordered_map<int, std::unordered_map<std::string, cv::Point>> coverageAlternatives;
    std::unordered_map<std::string, cv::Point> a1, a2, a3, a4;

    PointVector pathCW = isClockWise(path) ? path : computeCCWPath(path);
    PointVector pathCCW = isClockWise(path) ? computeCCWPath(path) : path;

    // a1: clockwise current path
    a1["SP"] = pathCW.front();
    a1["EP"] = pathCW.back();

    // a2: counterclockwise current path
    a2["SP"] = pathCCW.front();
    a2["EP"] = pathCCW.back();

    // a3: clockwise opposite path
    a3["SP"] = pathCW.back();
    a3["EP"] = pathCW.front();

    // a4: counterclockwise opposite path
    a4["SP"] = pathCCW.back();
    a4["EP"] = pathCCW.front();

    coverageAlternatives[1] = a1;
    coverageAlternatives[2] = a2;
    coverageAlternatives[3] = a3;
    coverageAlternatives[4] = a4;

//  bool hasIntersectionCW = hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(pathCW, false));
//  bool hasIntersectionCCW = hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(pathCCW, false));

    double minDistance;
    int optimalPath;

    // check which coverage alternative has the shortest path
    for (const auto& coverage : coverageAlternatives | boost::adaptors::indexed()) //按map中键值遍历
    {
        // skip calculating length if the path has intersections
//    if ((hasIntersectionCW and coverage.index() % 2 == 0) or (hasIntersectionCCW and coverage.index() % 2 != 0))
//    {
//      continue;
//    }

        // only length of transition need to be considered because the length of coverage is almost same
        double distance = calculateDistance(coverage.value().second.at("SP"), start);
        //calculateDistance(end, coverage.value().second.at("EP"));

        if (distance < minDistance or coverage.index() == 0)
        {
            minDistance = distance;
            optimalPath = coverage.value().first;
        }
    }

    switch (optimalPath)
    {
        case 1:
        {
            return pathCW;
        }
        case 2:
        {
            return pathCCW;
        }
        case 3:
        {
            return computeOppositePath(pathCW);
        }
        default:
        {
            return computeOppositePath(pathCCW);
        }
    }
}


PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const cv::Point& start)
{
    return identifyOptimalAlternative(polygon, path, start, start);
}


bool findSecondOptimalPath(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           std::vector<PointVector>& path_list,cv::Mat PRM_map)
{
    std::vector<Direction> sweepDirections;
    PointVector convexHull = computeConvexHull(polygon);
//    PointVector convexHull = polygon;
    LineSegmentVector edges = generateEdgeVector(convexHull, true);

    // compute optimal sweep directions for each edge
    for (const auto& edge : edges)
    {
        double maxDistance = 0;
        Direction direction;
        direction.baseEdge = edge;
        for (const auto& vertex : convexHull)
        {
            double distance = calculateDistance(edge, vertex);

            // optimal sweep direction for a edge is the direction with the largest distance
            if (distance > maxDistance)
            {
                maxDistance = distance;
                direction.opposedVertex = vertex;
            }
        }
        sweepDirections.push_back(direction);
    }

    // compute second optimal path which has the shortest coverage path
    double pathLength = 0;
    PointVector tempPath;
//  for (const auto& sweepDirection : sweepDirections)
    int p_size = INT_MAX;
    for (int i=0; i< sweepDirections.size(); i++)
    {
        PointVector p;

        // isValidPath is true if computed coverage does not have intersection
        bool isValidPath = computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, sweepDirections[i], p, PRM_map);

        // second optimal path is the shortest path without intersection
        if (isValidPath && p.size() !=0)  //calculatePathLength(tempPath) < pathLength
        {
            path_list.push_back(p);
//      tempPath = p;
////      pathLength = calculatePathLength(tempPath);
//        p_size = p.size();
        }
    }

    if (path_list.empty())  //tempPath.size() <= 1
    {
        return false;
    }
    else
        return true;
}


bool isAdjacent(const PointVector& polygon1, const PointVector& polygon2)
{
    for (const auto& vertex1 : polygon1)
    {
        for (const auto& vertex2 : polygon2)
        {
            // consider that two polygons are adjacent if they have at least one point in common
            if (vertex1 == vertex2)
            {
                return true;
            }
        }
    }
    return false;
}

