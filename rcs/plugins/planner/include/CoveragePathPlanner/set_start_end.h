//
// Created by tuqirui on 20-7-10.
//

#ifndef BCD_SET_START_END_H
#define BCD_SET_START_END_H

#include<iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#define EPSILON 0.000001


//二维double矢量
struct  Vec2d
{
    double x, y;

    Vec2d()
    {
        x = 0.0;
        y = 0.0;
    }
    Vec2d(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
    void Set(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
};

//判断点在线段上
bool IsPointOnLine(double px0, double py0, double px1, double py1, double px2, double py2);


//判断两线段相交
bool IsIntersect(double px1, double py1, double px2, double py2, double px3, double py3, double px4, double py4);


//判断点在多边形内
bool Point_In_Polygon_2D(double x, double y, std::vector<cv::Point> POL);

//点到直线的距离
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);


double cal_distance(cv::Point point1, cv::Point point2);


float cal_point_to_line(cv::Point pointP, cv::Point pointA, cv::Point pointB);


//点在多边形的区域位置
void find_point_position(std::vector<std::vector<cv::Point>> subregion_vertex_list, std::vector<std::vector<cv::Point>> path_cv,
                            cv::Point point, int& subregion_idx, int& point_idx);


//获取起点到终点的路径
bool find_path_with_start_end(std::vector<std::vector<cv::Point>> path_cv, std::vector<std::vector<cv::Point>> subregion_vertex_list,
        cv::Point path_start, cv::Point path_end, std::vector<std::vector<cv::Point>>& path_se);


#endif //BCD_SET_START_END_H
