//
// Created by tuqirui on 21-1-11.
//

#ifndef GRASS_PATH_PLANNER_COVERAGEPATHPLANNERCORE_H
#define GRASS_PATH_PLANNER_COVERAGEPATHPLANNERCORE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include "prefix_errno.h"
#include "CoveragePathPlanner/path_planner.h"

class GlobalPathPlanner
{
public:
    GlobalPathPlanner(){};
    virtual void init() = 0;
    virtual void setMap(cv::Mat& global_map) = 0;
    virtual void setInitPose(cv::Point& init_point) = 0;
//    virtual void setGoal() const = 0;
    virtual int plan() = 0;
    virtual std::vector<cv::Point> getPath() = 0;
};

class CoveragePathPlannerCore : public GlobalPathPlanner
{
public:
    CoveragePathPlannerCore(float coverage_wide, float coincidence_rate, std::string map_path);
    void init() override ;
    void setMap (cv::Mat& global_map) override;
    void setInitPose(cv::Point& init_point) override;
    void setGoal(std::vector<cv::Point>& goal_area);
    void initCoverageMap(std::vector<cv::Point> polygon, cv::Mat& coverage_map);
    bool isExistVisualWall();
    void dilateMap(cv::Mat coverage_map, cv::Mat& dilate_image, float robot_size);
    bool filterPath();
    int plan() override;
    std::vector<cv::Point> getPath() override;
    std::vector<std::vector<cv::Point>> getCoverageDividePath();
    float getCoverageRatio();

private:
    cv::Mat global_map_;
    cv::Mat coverage_map_;
    cv::Point init_point_;
    float coverage_wide_;
    float coincidence_rate_;
    std::string map_path_;
    float coverage_ratio_;

    std::vector<std::vector<cv::Point>> coverage_path_;
    std::vector<cv::Point> coverage_path_filter_;
    std::vector<std::vector<cv::Point>> coverage_divide_path_;

    bool is_get_map_;
    bool is_get_init_point_;
    bool is_get_goal_;
};

class DiffGlobalPathPlannerCore : public GlobalPathPlanner
{
public:
    DiffGlobalPathPlannerCore(float coverage_wide, std::string map_path);
    void init() override ;
    void setMap(cv::Mat& global_map) override;
    void setInitPose(cv::Point& init_point) override;
    void setGoal(cv::Point& init_point);
    void dilateMap(cv::Mat coverage_map, cv::Mat& dilate_image, float robot_size);
    int plan() override;
    std::vector<cv::Point>  getPath() override;

private:
    float robot_size_;
    std::string map_path_;

    cv::Mat global_map_;
    cv::Point init_point_;
    cv::Point goal_point_;

    bool is_get_map_;
    bool is_get_init_point_;
    bool is_get_goal_;

    std::vector<cv::Point> path_cv_;
};

#endif //GRASS_PATH_PLANNER_COVERAGEPATHPLANNERCORE_H
