//
// Created by zhihui on 2020/11/30.
//

#ifndef GRASS_PATH_PLANNER_WELTPATHPLANNER_H
#define GRASS_PATH_PLANNER_WELTPATHPLANNER_H


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include "CoveragePathPlanner/prm_planner/prm_planner.h"

class WeltPathPlanner
{
public:
    WeltPathPlanner(cv::Mat dilate_map, cv::Point start_point);
    double cal_distance(cv::Point point1, cv::Point point2);
    bool is_in_close_list(std::vector<std::vector<cv::Point>> close_list, std::vector<cv::Point> polygon);
    void AroundPolygonPath(std::vector<cv::Point> polygon, int polygon_start_idex, std::vector<cv::Point>& polygon_path);
    void PathSort(std::vector<std::vector<cv::Point>>  welt_path, cv::Point init_point, std::vector<std::vector<cv::Point>>& sort_path);
    void find_xy_area(std::vector<cv::Point> polygon, std::vector<double>& xy_area);
    void dilate_map(cv::Mat coverage_map, cv::Mat& dilate_image, float robot_size);
    bool Plan(bool start_is_current_point);
    std::vector<cv::Point> GetPath();
    std::vector<cv::Point> GetPoint();
    void VisualizePath(cv::Mat image_gray, std::string map_path);
    std::vector<std::vector<cv::Point>> welt_path_;
    std::vector<std::vector<cv::Point>> welt_path_filte_;
    std::vector<cv::Point> welt_path_visualize_;

private:
    cv::Mat dilate_map_;
    cv::Point start_point_;

};

#endif //GRASS_PATH_PLANNER_WELTPATHPLANNER_H
