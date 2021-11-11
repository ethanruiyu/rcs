//
// Created by tuqirui on 21-10-22.
//
#ifndef OSQP_DEMO_PICECWISE_JERK_PATH_OPTIMIZER_H
#define OSQP_DEMO_PICECWISE_JERK_PATH_OPTIMIZER_H

#include <iostream>

#include <Eigen/Eigen>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include "osqp_problem.h"

using PathBoundary = std::vector<std::pair<double, double> >;

class PathBoundaryInfo {
public:
    inline void set_start_s(double start_s) { start_s_ = start_s; }

    inline void set_delta_s(double delta_s) { delta_s_ = delta_s; }

    inline void set_boundary(const PathBoundary &boundary) { boundary_ = boundary; }

    inline double delta_s() { return delta_s_; }

    inline const PathBoundary &boundary() { return boundary_; }

    PathBoundary boundary_;

private:
    double start_s_;
    double delta_s_;
};

class picecwiseJerkPathOptimizer {
public:
    picecwiseJerkPathOptimizer(cv::Mat plan_map, double path_interval, double resolution);
    ~picecwiseJerkPathOptimizer();
    void setOriginPath(std::vector<cv::Point> path_origin);

    std::vector<Eigen::Vector2d> cvToEigenVector(std::vector<cv::Point> line_cv);
    std::vector<cv::Point> EigenToCvVector(std::vector<Eigen::Vector2d> line);

    std::vector<Eigen::Vector2d> discretizePath(const std::vector<Eigen::Vector2d> &path, int pt_num);
    void smoothPath(std::vector<Eigen::Vector2d> path_in, std::vector<Eigen::Vector2d> &path_out);

    bool findPathBoundry(std::vector<Eigen::Vector2d> &path_origin, cv::Mat plan_map,
                         std::vector<int> &ceil_boundry_pixel, std::vector<int> &floor_boundry_pixel);
    void build_path_boundary(PathBoundaryInfo &bound_info,std::vector<double> ceil_boundry, std::vector<double> floor_boundry,
                             double delta_s, double start_s);
    void frenetPath0ptimize(std::vector<double> ceil_boundry, std::vector<double> floor_boundry, double delta_s, std::vector<double> &opt_frenet_path);
    void frenetToXY(std::vector<cv::Point> path_origin, std::vector<double> frenet_path, std::vector<cv::Point>& path_output);
    void frenetToXY(std::vector<cv::Point> path_origin, std::vector<int> frenet_path, std::vector<cv::Point>& path_output);
    void pixelToActual(std::vector<int> pixel_path, std::vector<double>& actual_path);
    void actualToPixel(std::vector<double> actual_path, std::vector<double>& pixel_path);

    void pathOptimize();

    std::vector<cv::Point> getPath();
    void visualizeFrenetPath();
    void visualize();

private:
    int path_interval_pixel_;
    double path_interval_actual_;
    double resolution_;
    std::vector<cv::Point> path_origin_;
    std::vector<cv::Point> final_path_;
    cv::Mat plan_map_;

    std::vector<cv::Point> pretreatment_path_;

    std::vector<double> final_frenet_path_pixel_;
    std::vector<int> ceil_boundry_pixel_ ;
    std::vector<int> floor_boundry_pixel_ ;

    std::vector<double> final_frenet_path_actual_;
    std::vector<double> ceil_boundry_actual_ ;
    std::vector<double> floor_boundry_actual_ ;
};


#endif //OSQP_DEMO_PICECWISE_JERK_PATH_OPTIMIZER_H
