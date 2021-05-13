//
// Created by tuqirui on 20-7-8.
//

#ifndef BCD_COVERAGE_PATH_PLANNER_H
#define BCD_COVERAGE_PATH_PLANNER_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
//#include <ros/ros.h>
#include "CoveragePathPlanner/bcd.h"
//#include <coverage_path_planner.h>
#include "map_rotate.h"
#include "WeltPathPlanner.h"
#include "CoveragePathPlanner/torres_etal.h"
#include "CoveragePathPlanner/cgutil.h"
#include "CoveragePathPlanner/prm_planner/prm_planner.h"
#include "CoveragePathPlanner/ConcavePolygon.h"
#include "CoveragePathPlanner/normal_astar_planner/normal_astar_planner_core.h"

Mat find_subregion(Mat& map, int cost);

bool find_vertex(Mat subregion, std::vector<cv::Point>& contours_vertex, bool display);

int max_x(std::vector<cv::Point> subregion_vertex);

int max_y(std::vector<cv::Point> subregion_vertex);

void visualize_decompose_area(cv::Mat subregion, std::vector<std::vector<cv::Point>> subregion_vertex_list, std::string map_path);

void polygon_cv_to_verts(std::vector<cv::Point> polygon, std::vector<cxd::Vertex>& verts);

void two_vector_verts_to_cv(std::vector<std::vector<cxd::Vertex>>& verts, std::vector<std::vector<cv::Point>>& polygon);

bool BowPathPlanner(std::vector<cv::Point> polygon, float sweep_wide, float coincidence_rate,
                    cv::Mat dilate_image_coverage_prm, cv::Point start_point, std::vector<cv::Point>& path);

//bool coverage_path_planning_delaunay(cv::Mat img_b, cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv,
//                                     float grass_wide, float coincidence_rate, cv::Point start, std::string map_path);

int find_nearist_area(cv::Point start, std::vector<std::vector<cv::Point>> subregion_vertex_list);

bool coverage_path_planning_bcd(cv::Mat img_b, cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv, float grass_wide,
                                float coincidence_rate, cv::Point start, std::string map_path);

bool coverage_path_planning_concave(cv::Mat img_b, cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv, float grass_wide,
                                    float coincidence_rate, cv::Point start, std::string map_path);

void visualize_prm_path(std::vector<TGlobalOrd> path, Mat map);

bool prm_path_planning(cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv, float grass_wide, std::vector<double> xy_area,
                        std::vector<cv::Point> welt_path2);

void path_to_vector_path(std::vector<std::vector<cv::Point>> path_cv, std::vector<cv::Point>& vector_path);

bool to_coverage_start_path(cv::Mat virtual_walls_map, cv::Point init_point_cv, std::vector<std::vector<cv::Point>>& path_se,
                            std::vector<cv::Point>& prm_path_cv, std::string map_path);

void visualize_path(std::vector<cv::Point> path, cv::Mat map, std::string map_path);

int cal_point_num(cv::Mat map);

double  cal_coverage_ratio(std::vector<cv::Point> path, cv::Mat map, float robot_size, std::string map_path);







#endif //BCD_COVERAGE_PATH_PLANNER_H
