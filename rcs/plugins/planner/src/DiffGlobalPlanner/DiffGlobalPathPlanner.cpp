//
// Created by tuqirui on 21-1-11.
//

#include "GlobalPathPlannerCore.h"

DiffGlobalPathPlannerCore::DiffGlobalPathPlannerCore(float robot_size, std::string map_path)
{
    robot_size_= robot_size;
    map_path_ = map_path;
    init();
}

void DiffGlobalPathPlannerCore::init()
{
    is_get_map_ = false;
    is_get_init_point_ = false;
    is_get_goal_ = false;
}

void DiffGlobalPathPlannerCore::setMap(cv::Mat& global_map)
{
    global_map_ = global_map;
    is_get_map_ = true;
}

void DiffGlobalPathPlannerCore::setInitPose(cv::Point& init_point)
{
    init_point_ = init_point;
    is_get_init_point_ = true;

}

void DiffGlobalPathPlannerCore::setGoal(cv::Point& goal_point)
{
    goal_point_ = goal_point;
    is_get_goal_ = true;
}

void DiffGlobalPathPlannerCore::dilateMap(cv::Mat global_map, cv::Mat& dilate_image, float robot_size)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat coverage_map_rgb;
    cv::cvtColor(global_map, coverage_map_rgb, cv::COLOR_GRAY2BGR);
    cv::findContours(global_map.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    cv::drawContours(coverage_map_rgb, contours,-1,(255,0, 255), 8);
    for(const auto contour : contours)
    {
        for(const auto& point:contour)
        {
            cv::Point point1, point2;
            point1 = { point.x + int(robot_size)/2, point.y + int(robot_size)/2};
            point2 = { point.x - int(robot_size)/2, point.y - int(robot_size)/2};
            cv::rectangle(global_map, point1, point2, cv::Scalar(0), -1);
        }
    }
    dilate_image = global_map;
//    cv::imshow("countors1", coverage_map_rgb);
//    cv::imshow("countors", dilate_image);
//    cv::waitKey(0);
}

int DiffGlobalPathPlannerCore::plan()
{
    cv::Mat visual_map = global_map_.clone();
    cv::circle(visual_map, init_point_, 3, cv::Scalar(200));
    cv::circle(visual_map, goal_point_, 3, cv::Scalar(200));
    cv::imwrite(map_path_+"/diff_global_planner_map.png", visual_map);


    cv::Mat plan_map;
    dilateMap(global_map_.clone(), plan_map, robot_size_);
    cv::bitwise_not(plan_map, plan_map);
    cv::imwrite(map_path_+"/plan_map.png", plan_map);
    if(plan_map.at<uchar>(init_point_) != 0)
    {
        std::cout << "init point is not safe" << std::endl;
        return DIFF_GLOBAL_ERROR_INIT_POINT_IS_NOT_SAFE;
    }
    if(plan_map.at<uchar>(goal_point_) != 0)
    {
        std::cout << "goal point is not safe" << std::endl;
        return DIFF_GLOBAL_ERROR_GOAL_POINT_IS_NOT_SAFE;
    }
    //astar path plan
    std::vector<cv::Point> astar_path;
    normal_astar_planner::NormalAstarPlannerCore astar;
    astar.setStart(init_point_);
    astar.setGoal(goal_point_);
    astar.setMap(plan_map);
    if(!astar.makePlan(astar_path))
    {
        std::cout << "failed astar plan" << std::endl;
        return DIFF_GLOBAL_ERROR_ASTAR_PATH_FAIL_TO_PLAN;
    }
    std::cout << "success astar plan" << std::endl;

    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(astar_path);
    PRMPlanner prm_planner1(30); //grass_wide
    prm_planner1.initialize(plan_map);
    if(!prm_planner1.plan_with_astar(init_point_.x, init_point_.y, goal_point_.x, goal_point_.y, astar_path))
    {
        std::cout << "failed prm plan" << std::endl;
        return DIFF_GLOBAL_ERROR_PRM_PATH_FAIL_TO_PLAN;
    }
    std::vector<TGlobalOrd> prm_path = prm_planner1.getpath();
    cv::Point prm_point_cv;
    for(auto point1 : prm_path) {
        prm_point_cv.x = point1.x;
        prm_point_cv.y = point1.y;
        path_cv_.push_back(prm_point_cv);
    }
    std::cout << "success prm plan" << std::endl;

    cv::Mat img_rgb;
    cv::cvtColor(global_map_, img_rgb, cv::COLOR_GRAY2BGR);
    for(int i=0; i<path_cv_.size()-1; i++)
    {
        cv::line(img_rgb, path_cv_[i], path_cv_[i+1], cv::Scalar(0, 0, 255));
    }
    cv::imwrite(map_path_+"/path_map.png", img_rgb);
    return 0;
}

std::vector<cv::Point>  DiffGlobalPathPlannerCore::getPath()
{
    return path_cv_;
}