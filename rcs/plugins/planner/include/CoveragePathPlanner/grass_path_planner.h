//
// Created by tuqirui on 20-7-14.
//

#ifndef BCD_GRASS_PATH_PLANNER_H
#define BCD_GRASS_PATH_PLANNER_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <CoveragePathPlanner/path_planner.h>
//#include "torres_etal_2016.hpp"
//#include "cgutil.h"
#include <CoveragePathPlanner/prm_planner/prm_planner.h>
#include <CoveragePathPlanner/set_start_end.h>
#include <CoveragePathPlanner/map_rotate.h>
#include <CoveragePathPlanner/prefix_errno.h>
#include <CoveragePathPlanner/WeltPathPlanner.h>
#include <CoveragePathPlanner/CPP_BCD.h>


struct point
{
    point(int x1,int y1)
    {
        x = x1;
        y = y1;
    }
    int x;
    int y;
};

struct position
{
    position(int x1,int y1,int z1,int p11,int p21,int p31,int p41)
    {
        x = x1;
        y = y1;
        z = z1;
        p1 = p11;
        p2 = p21;
        p3 = p31;
        p4 = p41;
    }
    double x, y, z, p1, p2, p3, p4;
};

void postion_to_cv_point(position init_position, cv::Point& point_cv)
{
    point_cv.x = init_position.x;
    point_cv.y = init_position.y;
}

void point_to_cv_point(point point, cv::Point& point_cv)
{
    point_cv.x = point.x;
    point_cv.y = point.y;
}

void vector_to_cv_vector(std::vector<point> point_list, std::vector<cv::Point>& point_cv_list)
{
    cv::Point point_cv;
    for(auto point1 : point_list)
    {
        point_cv.x = point1.x;
        point_cv.y = point1.y;
        point_cv_list.push_back(point_cv);
    }

}

void two_vector_to_cv_vector(std::vector<std::vector<point>> visual_wall, std::vector<std::vector<cv::Point>>& visual_wall_cv)
{
    std::vector<cv::Point> point_list_cv;
    for(auto point_list : visual_wall)
    {
        vector_to_cv_vector(point_list, point_list_cv);
        visual_wall_cv.push_back(point_list_cv);
        point_list_cv.clear();
    }
}

bool set_virtual_walls_in_map(cv::Mat map, cv::Mat& prm_map, std::vector<std::vector<cv::Point>> visual_wall)
{
    std::cout << "Set virtual walls in map" << std::endl;
    //prm_map = map.clone();
    cv::Mat prm_map1(map.rows, map.cols, CV_8UC1, Scalar(255,0,0));
    prm_map = prm_map1;
    for(int i = 0; i < prm_map.cols; i++)
    {
        for(int j = 0; j < prm_map.rows; j++)
        {
            for(auto vw1 : visual_wall)
            {
                if(Point_In_Polygon_2D(i, j, vw1))
                {
                    prm_map.at<u_int8_t>(j,i) = 127;
                }
            }
        }
    }
    return true;
}

bool setSweepingAreasInMap(cv::Mat map, cv::Mat& coverage_map, std::vector<cv::Point> coverage_area)
{
    std::cout << "Set sweeping areas in map" << std::endl;
    coverage_map = map.clone();

    for(int i = 0; i < coverage_map.cols; i++)
    {
        for(int j = 0; j < coverage_map.rows; j++)
        {
            if(!Point_In_Polygon_2D(i, j, coverage_area))
            {
                coverage_map.at<u_int8_t>(j,i) = 0;
            }
        }
    }
    return true;
}

bool point_to_position(std::vector<cv::Point> point_path, std::vector<position>& position_path)
{
    position position1(0,0,0,0,0,0,0);
    for(int i=0; i<point_path.size()-1; i++)
    {
        position1.x = point_path[i].x;
        position1.y = point_path[i].y;
        position1.z = 0.0;
        double yaw = atan2((point_path[i+1].y - point_path[i].y), (point_path[i+1].x - point_path[i].x));
        position1.p1 = 0.0;
        position1.p2 = 0.0;
        position1.p3 = sin((180*yaw)/2);
        position1.p4 = cos((180*yaw)/2);
        position_path.push_back(position1);
    }
    position1.x = point_path[point_path.size()-1].x;
    position1.y = point_path[point_path.size()-1].y;
    position1.z = 0.0;
    position1.p1 = 0.0;
    position1.p2 = 0.0;
    position1.p3 = position_path[position_path.size()-1].p3;
    position1.p4 = position_path[position_path.size()-1].p4;
    position_path.push_back(position1);
    return true;
}

bool TGlobalOrd_to_position(std::vector<TGlobalOrd> point_list, std::vector<position>& position_path)
{
    position position1(0,0,0,0,0,0,0);
    for(int i=0; i<point_list.size()-1; i++)
    {
        position1.x = point_list[i].x;
        position1.y = point_list[i].y;
        position1.z = 0.0;
        double yaw = atan2((point_list[i+1].y - point_list[i].y), (point_list[i+1].x - point_list[i].x));
        position1.p1 = 0.0;
        position1.p2 = 0.0;
        position1.p3 = sin((180*yaw)/2);
        position1.p4 = cos((180*yaw)/2);
        position_path.push_back(position1);
    }
    position1.x = point_list[point_list.size()-1].x;
    position1.y = point_list[point_list.size()-1].y;
    position1.z = 0.0;
    position1.p1 = 0.0;
    position1.p2 = 0.0;
    position1.p3 = position_path[position_path.size()-1].p3;
    position1.p4 = position_path[position_path.size()-1].p4;
    position_path.push_back(position1);
    return true;
}

void visualize_prm_path_list(cv::Mat prm_map, std::vector<position> point_list, std::vector<std::vector<position>> path_list, std::string map_path)
{
    cv::Mat prm_map_rgb;
    cvtColor(prm_map, prm_map_rgb, COLOR_GRAY2BGR);
    std::vector<cv::Point> path_list_cv;

    cv::Point point_cv;
    for(auto point : point_list)
    {
        point_cv.x = point.x;
        point_cv.y = point.y;
        cv::circle(prm_map_rgb,point_cv,10,CV_RGB(0,0,255),8);
    }
    for(int i=0; i<path_list.size(); i++)
    {
        for(int j=0; j<path_list[i].size(); j++)
        {
            point_cv.x = path_list[i][j].x;
            point_cv.y = path_list[i][j].y;
            path_list_cv.push_back(point_cv);
        }
    }
    visualize_path(path_list_cv,prm_map_rgb, map_path);
//    cv::imshow("prm_map_rgb", prm_map_rgb);
//    cv::waitKey(0);
}









void find_xy_area(std::vector<cv::Point> polygon, std::vector<double>& xy_area)
{
    std::vector<double> polygon_x, polygon_y;
    for(auto point : polygon)
    {
        polygon_x.push_back(point.x);
        polygon_y.push_back(point.y);
    }
    auto p_x_min = std::min_element(polygon_x.begin(), polygon_x.end());
    xy_area.push_back(*(p_x_min));
    auto p_x_max = std::max_element(polygon_x.begin(), polygon_x.end());
    xy_area.push_back(*(p_x_max));
    auto p_y_min = std::min_element(polygon_y.begin(), polygon_y.end());
    xy_area.push_back(*(p_y_min));
    auto p_y_max = std::max_element(polygon_y.begin(), polygon_y.end());
    xy_area.push_back(*(p_y_max));
}


//int set_visual_walls(std::string map_path, std::vector<std::vector<point>> visual_wall, float resolution)
//{
//    //类型转换
//    cv::Point init_point_cv;
//    std::vector<cv::Point> coverage_area_cv;
//    std::vector<std::vector<cv::Point>> visual_wall_cv;
//    two_vector_to_cv_vector(visual_wall, visual_wall_cv);
//
//    //读地图
//    cv::Mat img= imread(map_path+"/map.png");
//    if(img.empty())
//    {
//        std::cout << "map is empty" << std::endl;
//        return ERROR_MAP_IS_EMPTY;
//    }
//    cv::Mat img_gray, virtual_walls_map;
//    cvtColor(img,img_gray,CV_BGR2GRAY);//三通道的图转化为单通道的灰度图
//    //cv::imshow("img_gray", img_gray);
//
//    //设置虚拟墙
//    if(!set_virtual_walls_in_map(img_gray, virtual_walls_map, visual_wall_cv))
//    {
//        return 2;
//    }
////    cv::imshow("virtual_walls_map", virtual_walls_map);
//    cv::imwrite(map_path+"/virtual_walls_map.png", virtual_walls_map);
////    cv::waitKey(0);
//
//    return 0;
//}

//int point_to_point_path_planner(position init_point, std::string map_path, std::vector<position> point_list, std::vector<std::vector<position>>& path_list, double robot_size)
//{
//    //read map
//    cv::Mat img, img_gray, prm_map;
//    img = cv::imread(map_path+"/virtual_walls_map.png");
//    if(img.empty())
//    {
//        std::cout << "map is empty" << std::endl;
//        return ERROR_MAP_IS_EMPTY;
//    }
//    cvtColor(img, img_gray, COLOR_BGR2GRAY);
//    threshold(img_gray,img_gray,130,255,THRESH_BINARY);
//
//    cv::Mat prm_dilate_image;
//    dilate_map(img_gray, prm_dilate_image, robot_size);
//    cv::bitwise_not(prm_dilate_image, prm_dilate_image);
////    cv::namedWindow("prm_dilate_image", 0);
////    cv::imshow("prm_dilate_image", prm_dilate_image);
////    cv::waitKey(0);
//    cv::imwrite(map_path+"/prm_dilate_image.png", prm_dilate_image);
//
//    //prm path planning
//    PRMPlanner prm_planner(30, 0, std::min(init_point.x, point_list[0].x)-100, std::max(init_point.x, point_list[0].x)+100,
//                         std::min(init_point.y, point_list[0].y)-100, std::max(init_point.y, point_list[0].y)+100); //0, img.cols, 0, img.rows
//    prm_planner.initialize(prm_dilate_image);
//    std::vector<TGlobalOrd> prm_path_TGlobalOrd;
//    std::vector<position> prm_path;
//    for(int i=0; i<point_list.size(); i++)
//    {
//        if(i == 0)
//        {
//            prm_planner.plan(init_point.x, init_point.y, point_list[0].x, point_list[0].y);
//            prm_path_TGlobalOrd = prm_planner.getpath();
//            if(!prm_path_TGlobalOrd.empty())
//            {
//                TGlobalOrd_to_position(prm_path_TGlobalOrd, prm_path);
//                path_list.push_back(prm_path);
//                prm_path.clear();
//            }
//            else{
//                return ERROR_PRM_FAIL_TO_PLAN;
//            }
//
//        }
//        else
//        {
//            prm_planner.plan(point_list[i-1].x, point_list[i-1].y, point_list[i].x, point_list[i].y);
//            prm_path_TGlobalOrd = prm_planner.getpath();
//            if(!prm_path_TGlobalOrd.empty())
//            {
//                TGlobalOrd_to_position(prm_path_TGlobalOrd, prm_path);
//                path_list.push_back(prm_path);
//                prm_path.clear();
//            }
//            else{
//                return ERROR_PRM_FAIL_TO_PLAN;
//            }
//
//        }
//    }
//    visualize_prm_path_list(img_gray, point_list, path_list, map_path);
//    return 0;
//}

#endif //BCD_GRASS_PATH_PLANNER_H
