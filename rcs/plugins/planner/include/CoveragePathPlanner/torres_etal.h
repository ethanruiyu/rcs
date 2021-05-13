/**
 * @file torres_etal_2016.hpp
 * @brief Header file for torres_etal_2016.cpp
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

#ifndef INCLUDED_torres_etal_
#define INCLUDED_torres_etal_

// cgutil
#include <CoveragePathPlanner/cgutil.h>
#include <CoveragePathPlanner/set_start_end.h>
#include <CoveragePathPlanner/prm_planner/prm_planner.h>

// cpp standard libraries
#include <array>
#include <string>
#include <unordered_map>

// Boost
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/reversed.hpp>



/**
 * @struct Direction
 * @brief Storage for line sweep direction
 *
 * Sweep direction is from edge to vertex
 */
struct Direction
{
  LineSegment baseEdge;
  cv::Point opposedVertex;
};

/**
 * @brief Checks if given path is clockwise (the first turn made to the left) or not
 * @param path
 * @return bool True if path is clockwise
 * @details the definition of "clockwise" is based on Fig.8 in Torres et al. 2016
 */
inline bool isClockWise(const PointVector& path)
{
  return path.at(0).x < path.at(1).x ? true : false;
}

/**
 * @brief Calculates line sweep direction for given polygon
 * @param polygon Line sweep direction is calculated on this region
 * @return direction Struct containing edge and vertex
 */
Direction identifyOptimalSweepDir(const PointVector& polygon);

/**
 * @brief Reshape given path
 * @param path The sweep lines of path should be horizontal about x axis
 * @param padding
 * @return PointVector
 * @details Reshape given path so that generated path becomes the sequence of "C" shapes and add padding
 */

bool find_egde_point_left(cv::Point& p, cv::Point& p1, LineSegmentVector rotatedEdges, double padding);

bool find_egde_point_right(cv::Point& p, cv::Point& p1, LineSegmentVector rotatedEdges, double padding);

PointVector reshapePath(const PointVector& path, double padding, LineSegmentVector rotatedEdges);

void check_intersection(PointVector polygon, PointVector path, PointVector& path2, cv::Mat PRM_map);

/**
 * @brief Compute coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param sweepDirection
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           const Direction& sweepDirection, PointVector& path, cv::Mat PRM_map);

/**
 * @brief Compute coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           PointVector& path, cv::Mat PRM_map);

/**
 * @brief Calculates length of path
 * @param path
 * @return double Length of path
 */
double calculatePathLength(const PointVector& path);

/**
 * @brief Return counter clock wise-ed path of given path
 * @param path Clockwise path
 * @return PointVector Counter clock wise version of given path
 */
PointVector computeCCWPath(PointVector path);

/**
 * @brief Return opposite path of given path
 * @param path
 * @return PointVector Path with points of reversed order of given path
 */
PointVector computeOppositePath(const PointVector& path);

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param polygon
 * @param path
 * @param start Start point
 * @param end End point
 * @return PointVector Optimal path that minimizes the length of path
 * @details The naming of the following variable follows torres et al. 2016
 */
PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const cv::Point& start, const cv::Point& end);

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param polygon
 * @param path
 * @param start Start point
 * @return PointVector Optimal path that minimizes the length of path
 * @details The naming of the following variable follows torres et al. 2016
 */
PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const cv::Point& start);

/**
 * @brief Find second optimal path
 * @param polygon
 * @param footprintWidth
 * @param horizontalOverwrap
 * @param path
 * @return bool True if second optimal path exists
 */
bool findSecondOptimalPath(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           std::vector<PointVector>& path_list,cv::Mat PRM_map);

/**
 * @brief Check if given two polygons are adjacent
 * @param polygon1
 * @param polygon2
 * @return True if given two polygons are adjacent
 */
bool isAdjacent(const PointVector& polygon1, const PointVector& polygon2);


#endif
