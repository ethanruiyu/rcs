//
// Created by zhihui on 20-10-10.
//

#ifndef GRASS_PATH_PLANNER_MAP_ROTATE_H
#define GRASS_PATH_PLANNER_MAP_ROTATE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

typedef  cv::Vec<float, 4>  Vec4f;

float cal_rat_angle(cv::Mat map, std::string map_path);

void rotate_map(cv::Mat coverage_origin, cv::Mat& coverage_rat, float angle);


void rotate_arbitrarily_angle(cv::Mat &src,cv::Mat &dst,float angle);


std::vector<cv::Point> path_transform(std::vector<cv::Point> path, cv::Mat1b map_transform, cv::Mat1b map_original, double angle);


#endif //GRASS_PATH_PLANNER_MAP_ROTATE_H
