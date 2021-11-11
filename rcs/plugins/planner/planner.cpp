#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv/cv.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "planner.h"
#include "jps_planner.h"
#include "picecwise_jerk_path_optimizer.h"

std::vector<cv::Point> jpsPlanner(const cv::Point &start, const cv::Point &end, cv::Mat &plan_map)
{
    auto t1 = clock();
    std::vector<cv::Point> resPoints;
    JPSCalc calcjps = JPSCalc();
    calcjps.setMap(plan_map);
    calcjps.setStart(start);
    calcjps.setGoal(end);
    if (!calcjps.GetPath(resPoints))
    { 
        std::cout << "failed to plan" << std::endl;
        return resPoints;
    }

    std::cout << " resPoints size =  " << resPoints.size() << std::endl;
    double t = (double)(clock() - t1) / CLOCKS_PER_SEC;
    std::cout << " jps plan take " << t << " s " << std::endl;
    return resPoints;
}


void dilateMap(cv::Mat src, cv::Mat &dst, int size)
{
    int s = size + 1;
    cv::Mat structureElement = getStructuringElement(cv::MORPH_RECT, cv::Size(s, s), cv::Point(-1, -1)); //创建结构元
    dilate(src, dst, structureElement, cv::Point(-1, -1), 1);
}

int add(int i, int j, cv::Point p)
{
    std::cout << p.x << std::endl;
    std::cout << p.y << std::endl;
    return i + j;
}

std::vector<cv::Point> plan(std::string imgPath, cv::Point start, cv::Point end, float dilate)
{
    std::cout << start.x << ", " << start.y << std::endl;
    std::cout << end.x << ", " << end.y << std::endl;
    std::cout << dilate << std::endl;
    
    cv::Mat tmp, map_rgb;
    cv::Mat map1 = cv::imread(imgPath);
    cv::cvtColor(map1, tmp, cv::COLOR_BGR2GRAY);
    cv::threshold(tmp, tmp, 130, 255, cv::THRESH_BINARY_INV);
    dilateMap(tmp, tmp, dilate);
    cv::imwrite("delate.png", tmp);
    std::vector<cv::Point> rescontours = jpsPlanner(start, end, tmp);

    for (auto i = 0; i < rescontours.size(); i++) {
        std::cout << rescontours[i].x << ", " << rescontours[i].y << std::endl;
    }
    double resolution = 0.05;
    double path_interval = 0.4;
    
    if (rescontours.empty()) {
        std::vector<cv::Point> res;
        return res;
    }

    double t1 = clock();
    picecwiseJerkPathOptimizer picecwiseJerkPathOptimizer(tmp, path_interval, resolution);
    picecwiseJerkPathOptimizer.setOriginPath(rescontours);
    picecwiseJerkPathOptimizer.pathOptimize();
    double t = (double) (clock() - t1) / CLOCKS_PER_SEC;
    std::cout << " path opt take " << t << " s " << std::endl;
//    picecwiseJerkPathOptimizer.visualizeFrenetPath();
    // picecwiseJerkPathOptimizer.visualize();
    std::vector<cv::Point> final_path = picecwiseJerkPathOptimizer.getPath();
    return final_path;
}

PYBIND11_MODULE(planner_module, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");
    m.def("plan", &plan, py::return_value_policy::reference);
}