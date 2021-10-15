//
// Created by tuqirui on 21-4-16.
//

#ifndef QUADTREE_PATH_PLANNER_QUADTREE_ASTAR_PLANNER_H
#define QUADTREE_PATH_PLANNER_QUADTREE_ASTAR_PLANNER_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "planner.h"

class quadtree_astar_planner {
public:
    quadtree_astar_planner(int quadtree_depth, std::string map_path);
    void init(); //override
    void expansionMap(cv::Mat& map_origin, cv::Mat& square_map);
    void setInitPose(cv::Point& init_point);// override
    void setGoal(cv::Point& init_point);
    void addLeafList(QuadTreeNode root_node);
    void visualizeQuadtreeMap(cv::Mat& map, int idx);
    void creatPath(cv::Mat& path_opt_map, cv::Point start_point, cv::Point goal_point);
    void visualizePath(cv::Mat& map);
    void updateQuadtree(QuadTreeNode& root_node);
    void updateEnvQuadtree(cv::Mat map_change);
    int replan(cv::Point& start_point, cv::Point& goal_point);
    int buildEnvQuadtree(cv::Mat map_origin);
    int plan(cv::Point& start_point, cv::Point& goal_point);
    std::vector<cv::Point> optimisePath(cv::Mat &cspace, std::vector<cv::Point> path);
    bool canConnect(cv::Mat &cspace, cv::Point start, cv::Point end);
    bool inMap(cv::Point p);
    bool isAccessible(cv::Mat &cspace, cv::Point p);
    std::vector<cv::Point>  getPath(); // override

private:

    int quadtree_depth_;
    std::string map_path_;

    cv::Mat map_origin_;
    cv::Mat map_change_;


    cv::Point init_point_;
    cv::Point goal_point_;

    Environment env_;
//    Planner planner_;

    bool is_get_map_;
    bool is_get_init_point_;
    bool is_get_goal_;

    std::vector<QuadTreeNode> leaf_list;
    std::vector<cv::Point> path_cv_;
    std::vector<cv::Point> path_cv_opt_;
};


#endif //QUADTREE_PATH_PLANNER_QUADTREE_ASTAR_PLANNER_H
