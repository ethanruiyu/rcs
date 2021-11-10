#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv/cv.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "planner.h"
#include "jps_planner.h"
#include "normal_astar_planner.h"
#include "quadtree_astar_planner/quadtree_astar_planner.h"

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

std::vector<cv::Point> quadtreeAstarPlanner(cv::Point& start, cv::Point& end, cv::Mat& plan_map, cv::Mat map_rgb)
{
    auto t1 = clock();
    std::vector<cv::Point> astar_path;

    quadtree_astar_planner planner1(10, "./../");
    planner1.buildEnvQuadtree(plan_map);
    auto t2 = clock();
    double t_build = (double)(t2 - t1)/CLOCKS_PER_SEC;
    int path_return = planner1.plan(start, end);
    if(path_return != 0)
    {
        std::cout << "failed astar plan in Quadtree" << std::endl;
        return astar_path;
    }
    std::cout << "success astar plan in Quadtree" << std::endl;
    planner1.visualizePath(map_rgb);
    astar_path = planner1.getPath();

    double t_plan = (double)(clock() - t2)/CLOCKS_PER_SEC;
    std::cout << " quadtree A star build tree take " << t_build << " s " << std::endl;
    std::cout << " quadtree A star plan take " << t_plan << " s " << std::endl;

    return astar_path;

}

std::vector<cv::Point> normalAstarPlanner(const cv::Point& start, const cv::Point& end, cv::Mat& plan_map)
{
    auto t1 = clock();
    std::vector<cv::Point> astar_path;
    NormalAstarPlanner astar;
    astar.setMap(plan_map);
    astar.setStart(start);
    astar.setGoal(end);
    if(!astar.makePlan(astar_path))
    {
        std::cout << "failed to plan" << std::endl;
        return astar_path;
    }

    double t = (double)(clock() - t1)/CLOCKS_PER_SEC;
    std::cout << " A star plan take " << t << " s " << std::endl;
    return astar_path;
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
    cv::Mat tmp, map_rgb;
    cv::Mat map1 = cv::imread(imgPath);
    cv::cvtColor(map1, tmp, cv::COLOR_BGR2GRAY);
    cv::threshold(tmp, tmp, 130, 255, cv::THRESH_BINARY_INV);
    dilateMap(tmp, tmp, dilate);
    std::vector<cv::Point> res = jpsPlanner(start, end, tmp);
    return res;
}

PYBIND11_MODULE(planner_module, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");
    m.def("plan", &plan, py::return_value_policy::reference);
}