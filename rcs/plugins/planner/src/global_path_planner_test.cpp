#include <iostream>
#include "GlobalPathPlannerCore.h"



int main() {

    std::string map_path = "/home/tuqirui/workspace/coverage-path-planner-c/plan_map/coverage_map";
    std::string map_path2 = "/home/tuqirui/workspace/coverage-path-planner-c/plan_map/diff_global_map";


    cv::Point init_point_test = {2550, 1000}; //2550, 1000
    cv::Point goal_point_test = {2600, 1100};
    std::vector<cv::Point> coverage_area_test = {{100, 100}, {100, 500}, {500, 500}, {500, 100}};
    std::vector<cv::Point> path_test;
    float grass_wide = 17;
    float coincidence_rate = 0.4;


    cv::Mat virtual_walls_map_gray, virtual_walls_map, coverage_map;
    virtual_walls_map = cv::imread(map_path+"/virtual_walls_map.png");
    if(virtual_walls_map.empty())
    {
        std::cout << "map is empty" << std::endl;
        return 0;
    }
    cvtColor(virtual_walls_map, virtual_walls_map_gray,cv::COLOR_BGR2GRAY);
    threshold(virtual_walls_map_gray,virtual_walls_map_gray,130,255,cv::THRESH_BINARY);

    //coveragePathPlanner
    CoveragePathPlannerCore coveragePathPlanner(grass_wide, coincidence_rate, map_path);
    coveragePathPlanner.setMap(virtual_walls_map_gray);
    coveragePathPlanner.setInitPose(init_point_test);
    coveragePathPlanner.setGoal(coverage_area_test);
    coveragePathPlanner.plan();
    path_test = coveragePathPlanner.getPath();

    //diffGlobalPathPlanner
//    DiffGlobalPathPlannerCore diffGlobalPathPlanner(grass_wide, map_path2);
//    diffGlobalPathPlanner.setMap(virtual_walls_map_gray);
//    diffGlobalPathPlanner.setInitPose(init_point_test);
//    diffGlobalPathPlanner.setGoal(goal_point_test);
//    diffGlobalPathPlanner.plan();
//    path_test = diffGlobalPathPlanner.getPath();

    return 0;
}