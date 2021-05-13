//
// Created by tuqirui on 21-1-12.
//

#include "GlobalPathPlannerCore.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

namespace py = pybind11;

struct PyPoint
{
    PyPoint(float x1,float y1)
    {
        x = x1;
        y = y1;
    }
    float x;
    float y;
};

struct PyPose
{
    PyPose(float x1,float y1,float z1,float p11,float p21,float p31,float p41)
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

PYBIND11_MAKE_OPAQUE(std::vector<std::vector<PyPose>,  std::allocator<std::vector<PyPose>>>);

using PathVector = std::vector<std::vector<PyPose>,  std::allocator<std::vector<PyPose>>>;

void postion_to_cv_point(PyPose init_position, cv::Point& point_cv)
{
    point_cv.x = init_position.x;
    point_cv.y = init_position.y;
}

void point_to_cv_point(PyPoint point, cv::Point& point_cv)
{
    point_cv.x = point.x;
    point_cv.y = point.y;
}

void vector_to_cv_vector(std::vector<PyPoint> point_list, std::vector<cv::Point>& point_cv_list)
{
    cv::Point point_cv;
    for(auto point1 : point_list)
    {
        point_cv.x = point1.x;
        point_cv.y = point1.y;
        point_cv_list.push_back(point_cv);
    }

}

void two_vector_to_cv_vector(std::vector<std::vector<PyPoint>> visual_wall, std::vector<std::vector<cv::Point>>& visual_wall_cv)
{
    std::vector<cv::Point> point_list_cv;
    for(auto point_list : visual_wall)
    {
        vector_to_cv_vector(point_list, point_list_cv);
        visual_wall_cv.push_back(point_list_cv);
        point_list_cv.clear();
    }
}

bool set_virtual_walls_in_map(cv::Mat map, cv::Mat& img_rat, std::vector<std::vector<cv::Point>> visual_walls)
{
    std::cout << "Set virtual walls in map" << std::endl;
    //prm_map = map.clone();
//    cv::Mat prm_map1(map.rows, map.cols, CV_8UC1, Scalar(255,0,0));
//    prm_map = prm_map1;
//    for(int i = 0; i < prm_map.cols; i++)
//    {
//        for(int j = 0; j < prm_map.rows; j++)
//        {
//            for(auto vw1 : visual_wall)
//            {
//                if(Point_In_Polygon_2D(i, j, vw1))
//                {
//                    prm_map.at<u_int8_t>(j,i) = 127;
//                }
//            }
//        }
//    }

    img_rat = cv::Mat(map.rows, map.cols, CV_8UC1, cv::Scalar(255));
    for(int i=0; i<visual_walls.size(); i++)
    {
        int npt1;
        npt1 = visual_walls[i].size();
        cv::Point coverage_area[1][npt1];
        for(int j=0; j<visual_walls[i].size(); j++)
        {
            coverage_area[0][j] = visual_walls[i][j];
        }
        const cv::Point* ppt[1]={coverage_area[0]};
        int npt[]={npt1};
        cv::fillPoly(img_rat, ppt, npt, 1, cv::Scalar(127), 0);
    }
    return true;
}

cv::Point mapTocvFrame(PyPose pose, cv::Mat map, float origin_x, float origin_y, float resolution)
{
    cv::Point point_cv;
    float height = map.rows;
    point_cv.x = int((pose.x - origin_x) / resolution);
    point_cv.y = int(height - (pose.y- origin_y) / resolution - 1);
    return point_cv;
}

cv::Point mapTocvFrame(PyPoint point, cv::Mat map, float origin_x, float origin_y, float resolution)
{
    cv::Point point_cv;
    float height = map.rows;
    point_cv.x = int((point.x - origin_x) / resolution);
    point_cv.y = int(height - (point.y- origin_y) / resolution - 1);
    return point_cv;
}

PyPose cvToMapFrame(cv::Point point_cv, cv::Mat map, float origin_x, float origin_y, float resolution)
{
    PyPose pose_map(0, 0, 0, 0, 0, 0,0);
    float height = map.rows;
    pose_map.x = point_cv.x * resolution + origin_x;
    pose_map.y = (height - point_cv.y - 1) * resolution + origin_y;
    return pose_map;
}

double getYawFromPoses(PyPose &p1, PyPose &p2)
{
    double delta_y, delta_x;
    delta_x = p2.x - p1.x;
    delta_y = p2.y - p1.y;
    double yaw = std::atan2(delta_y, delta_x);
    return yaw;
}

void getOrientationFromPoses(PyPose &p1, PyPose &p2)
{
    double yaw = getYawFromPoses(p1, p2);
    p1.p1 = 0;
    p1.p2 = 0;
    p1.p3 = std::sin(0.5 * yaw);
    p1.p4 = std::cos(0.5 * yaw);
}

int coverage_path_planner(PyPose init_pose, std::vector<PyPoint> coverage_area, std::string map_path,float origin_x,float origin_y,float resolution,
                            float grass_wide,float coincidence_rate, PyPose path_start, PyPose path_end,
                          std::vector<std::vector<PyPose>>& coverage_path,  double coverage_ratio)
{
    cv::Mat virtual_walls_map_gray, virtual_walls_map;
    virtual_walls_map = cv::imread(map_path+"/virtual_walls_map.png");
    if(virtual_walls_map.empty())
    {
        std::cout << "map is empty" << std::endl;
        return 1;
    }
    cvtColor(virtual_walls_map, virtual_walls_map_gray,COLOR_BGR2GRAY);
    threshold(virtual_walls_map_gray,virtual_walls_map_gray,130,255,THRESH_BINARY);

    cv::Point init_pose_cv = mapTocvFrame(init_pose, virtual_walls_map_gray, origin_x, origin_y, resolution);
    std::vector<cv::Point> coverage_area_cv;
    for(auto area_point : coverage_area)
    {
        cv::Point area_point_cv = mapTocvFrame(area_point, virtual_walls_map_gray, origin_x, origin_y, resolution);
        coverage_area_cv.push_back(area_point_cv);
    }

    CoveragePathPlannerCore coveragePathPlanner(grass_wide, coincidence_rate, map_path);
    coveragePathPlanner.setMap(virtual_walls_map_gray);
    coveragePathPlanner.setInitPose(init_pose_cv);
    coveragePathPlanner.setGoal(coverage_area_cv);
    int s = coveragePathPlanner.plan();
    if(s != 0)
        return s;
    std::vector<std::vector<cv::Point>> path = coveragePathPlanner.getCoverageDividePath();

    if(path.empty())
    {
        std::cout << "path is empty" << std::endl;
        return 1;
    }

    for(auto path_1 : path)
    {
        std::vector<PyPose> path_1_pypose;
        for(auto point_cv : path_1)
        {
            PyPose map_pose = cvToMapFrame(point_cv, virtual_walls_map_gray, origin_x, origin_y, resolution);
            path_1_pypose.push_back(map_pose);
        }
        coverage_path.push_back(path_1_pypose);
    }

    for(int i=0; i<coverage_path.size()-1; i++)
    {
        for(int j=0; j<coverage_path[i].size()-1; j++)
        {
            getOrientationFromPoses(coverage_path[i][j], coverage_path[i][j+1]);
        }
        getOrientationFromPoses(*(coverage_path[i].end()-1), *(coverage_path[i].end()-2));
    }

    return 0;
}


int point_to_point_path_planner(PyPose init_pose, std::vector<PyPose> goal_pose_list, std::string map_path,float origin_x,float origin_y,float resolution,
                                 float robot_size, std::vector<std::vector<PyPose>>& global_path_list)
{
    cv::Mat virtual_walls_map_gray, virtual_walls_map;
    virtual_walls_map = cv::imread(map_path+"/virtual_walls_map.png");
    if(virtual_walls_map.empty())
    {
        std::cout << "map is empty" << std::endl;
        return 1;
    }
    cvtColor(virtual_walls_map, virtual_walls_map_gray,COLOR_BGR2GRAY);
    threshold(virtual_walls_map_gray,virtual_walls_map_gray,130,255,THRESH_BINARY);

    cv::Point init_pose_cv = mapTocvFrame(init_pose, virtual_walls_map_gray, origin_x, origin_y, resolution);
    std::vector<cv::Point> goal_list_cv;
    for(auto goal_pose : goal_pose_list)
    {
        cv::Point area_point_cv = mapTocvFrame(goal_pose, virtual_walls_map_gray, origin_x, origin_y, resolution);
        goal_list_cv.push_back(area_point_cv);
    }

    std::vector<std::vector<cv::Point>> global_path_list_cv;
    for(int i=0; i<goal_list_cv.size(); i++)
    {
        if(i == 0)
        {
            DiffGlobalPathPlannerCore diffGlobalPathPlanner(robot_size, map_path);
            diffGlobalPathPlanner.setMap(virtual_walls_map_gray);
            diffGlobalPathPlanner.setInitPose(init_pose_cv);
            diffGlobalPathPlanner.setGoal(goal_list_cv[0]);
            int s = diffGlobalPathPlanner.plan();
            if(s != 0)
                return s;
            std::vector<cv::Point> global_path = diffGlobalPathPlanner.getPath();
            global_path_list_cv.push_back(global_path);
        }
        else
        {
            DiffGlobalPathPlannerCore diffGlobalPathPlanner(robot_size, map_path);
            diffGlobalPathPlanner.setMap(virtual_walls_map_gray);
            diffGlobalPathPlanner.setInitPose(goal_list_cv[i-1]);
            diffGlobalPathPlanner.setGoal(goal_list_cv[i]);
            int s = diffGlobalPathPlanner.plan();
            if(s != 0)
                return s;
            std::vector<cv::Point> global_path = diffGlobalPathPlanner.getPath();
            global_path_list_cv.push_back(global_path);
        }
    }

    for(int i=0; i<global_path_list_cv.size(); i++)
    {
         std::vector<PyPose> map_path_1;
         for(int j=0; j<global_path_list_cv[i].size(); j++)
         {
             PyPose map_pose = cvToMapFrame(global_path_list_cv[i][j], virtual_walls_map_gray, origin_x, origin_y, resolution);
             map_path_1.push_back(map_pose);
         }
         global_path_list.push_back(map_path_1);
    }

    for(int i=0; i<global_path_list.size(); i++)
    {
        for(int j=0; j<global_path_list[i].size()-1; j++)
        {
            getOrientationFromPoses(global_path_list[i][j], global_path_list[i][j+1]);
        }
        global_path_list[i].back().p1 = goal_pose_list[i].p1;
        global_path_list[i].back().p2 = goal_pose_list[i].p2;
        global_path_list[i].back().p3 = goal_pose_list[i].p3;
        global_path_list[i].back().p4 = goal_pose_list[i].p4;
    }
    return 0;
}

void set_visual_walls(std::string map_path, std::vector<std::vector<PyPoint>> visual_wall, float origin_x,float origin_y,float resolution)
{
    //读地图
    cv::Mat img= imread(map_path+"/map.png");
    if(img.empty())
    {
        std::cout << "map is empty" << std::endl;
        return ;
    }
    cv::Mat img_gray, virtual_walls_map;
    cvtColor(img,img_gray,CV_BGR2GRAY);//三通道的图转化为单通道的灰度图
    //cv::imshow("img_gray", img_gray);

    //类型转换
    std::vector<std::vector<cv::Point>> visual_walls_cv;
    for(int i=0; i< visual_wall.size(); i++)
    {
        std::vector<cv::Point> visual_wall_cv;
        for(int j=0; j<visual_wall[i].size(); j++)
        {
            cv::Point visual_wall_point =  mapTocvFrame(visual_wall[i][j], img, origin_x, origin_y, resolution);
            visual_wall_cv.push_back(visual_wall_point);
        }
        visual_walls_cv.push_back(visual_wall_cv);
    }

    //设置虚拟墙
    if(!set_virtual_walls_in_map(img_gray, virtual_walls_map, visual_walls_cv))
    {
        return ;
    }
//    cv::imshow("virtual_walls_map", virtual_walls_map);
    cv::imwrite(map_path+"/virtual_walls_map.png", virtual_walls_map);
//    cv::waitKey(0);

    return ;
}


//int main() {
//
//    PyPose init_point(550, 550, 0, 0, 0, 0, 0); //1500, 3000
//    PyPose end_point1(500, 500, 0, 0, 0, 0, 0); //726, 1700
//    PyPose end_point2(430, 280, 0, 0, 0, 0, 0);
//    PyPose end_point3(500, 800, 0, 0, 0, 0, 0);
//    PyPose end_point4(600, 450, 0, 0, 0, 0, 0);
//    std::vector<PyPose> goal_pose_list = {end_point1, end_point2, end_point3, end_point4};
////    std::vector<PyPose> point_list = {end_point1};
//
////    point coverage_area_point1(15.08, -0.69);
////    point coverage_area_point2(10.64, -4.16);
////    point coverage_area_point3(13.74, -7.4);
////    point coverage_area_point4(18.61, -7.5);
////    point coverage_area_point5(19.31, -1.77);
////    point coverage_area_point6(200, 50);
//
//
//    PyPoint coverage_area_point1(500, 500);
//    PyPoint coverage_area_point2(500, 600);
//    PyPoint coverage_area_point3(600, 600);
//    PyPoint coverage_area_point4(600, 500);
////    point coverage_area_point5(300, 80);
//    std::vector<PyPoint> coverage_area;
//    coverage_area = {coverage_area_point1, coverage_area_point2, coverage_area_point3, coverage_area_point4};
//
//    PyPoint visual_wall_point11(200, 200);
//    PyPoint visual_wall_point12(200, 300);
//    PyPoint visual_wall_point13(300, 300);
//    PyPoint visual_wall_point14(300, 250);
//
//    PyPoint visual_wall_point21(150, 150);
//    PyPoint visual_wall_point22(150, 180);
//    PyPoint visual_wall_point23(180, 180);
//    PyPoint visual_wall_point24(180, 150);
//    //point visual_wall_point25(459, 512);
//
//    std::vector<std::vector<PyPoint>> visual_wall;
//    visual_wall = {{visual_wall_point11, visual_wall_point12, visual_wall_point13, visual_wall_point14},
//                   {visual_wall_point21, visual_wall_point22, visual_wall_point23, visual_wall_point24}};
////    visual_wall = {{visual_wall_point21, visual_wall_point22, visual_wall_point23, visual_wall_point24}};
//    std::string map_path = "/home/tuqirui/workspace/global-path-planner-c/plan_map/coverage_map";
//    std::string map_path2 = "/home/tuqirui/workspace/global-path-planner-c/plan_map/diff_global_map";
//
//    //point path_start(150, 250);
//    //point path_end(524, 485);
//
//    PyPose path_start(0, 0, 0, 0, 0, 0, 0);
//    PyPose path_end(0, 0, 0, 0, 0, 0, 0);
//
//    cv::Point init_point_test = {2250, 700};
//    cv::Point goal_point_test = {2600, 1100};
//    std::vector<cv::Point> coverage_area_test = {{100, 100}, {100, 500}, {500, 500}, {500, 100}};
//    std::vector<cv::Point> path_test;
//    float grass_wide = 17;
//    float coincidence_rate = 0.4;
//
//    std::vector<std::vector<PyPose>> path_list;
//
//    cv::Mat virtual_walls_map_gray, virtual_walls_map, coverage_map;
//    virtual_walls_map = cv::imread(map_path+"/virtual_walls_map.png"); //virtual_walls_map.png
//    if(virtual_walls_map.empty())
//    {
//        std::cout << "map is empty" << std::endl;
//        return 0;
//    }
//    cvtColor(virtual_walls_map, virtual_walls_map_gray,cv::COLOR_BGR2GRAY);
//    threshold(virtual_walls_map_gray,virtual_walls_map_gray,130,255,cv::THRESH_BINARY);
//
//
//
//    std::vector<PyPose> coverage_path;
//    double coverage_ratio;
//    float origin_x = 0;
//    float origin_y = 0;
//    float resolution = 1;
//    int s1 = coverage_path_planner(init_point, coverage_area, map_path, origin_x, origin_y, resolution,
//             grass_wide, coincidence_rate,  path_start, path_end,
//            coverage_path, coverage_ratio);
//    if(s1 != 0)
//        return s1;
//    std::cout << "coverage path size = " << coverage_path.size() << std::endl;
//
////    set_visual_walls(map_path, visual_wall,resolution);
//    std::vector<std::vector<PyPose>> global_path_list;
////    int s2 = point_to_point_path_planner(init_point, goal_pose_list, map_path,origin_x,origin_y,resolution,
////             grass_wide, global_path_list);
////    if(s2 != 0)
////        return s2;
////    std::cout << "global_path_list = " << global_path_list.size() << std::endl;
//
//    return 0;
//}

PYBIND11_MODULE(planner_module, m) {
    m.def("point_to_point_path_planner", &point_to_point_path_planner, py::return_value_policy::reference);
    m.def("coverage_path_planner", &coverage_path_planner, py::return_value_policy::reference);
    m.def("set_visual_walls", &set_visual_walls);

    py::class_<PathVector>(m, "PathVector")
            .def(py::init<>())
            .def("pop_back", &PathVector::pop_back)
            .def("push_back", (void (PathVector::*)(const std::vector<PyPose> &)) &PathVector::push_back)
            .def("__len__", [](const PathVector &v) { return v.size();})
            .def("__iter__", [](PathVector &v) {
                return py::make_iterator(v.begin(), v.end());
            }, py::keep_alive<0, 1>());

    py::class_<PyPose>(m, "position")
            .def(py::init<float, float, float, float, float, float, float>())
            .def_readwrite("x", &PyPose::x)
            .def_readwrite("y", &PyPose::y)
            .def_readwrite("z", &PyPose::z)
            .def_readwrite("p1", &PyPose::p1)
            .def_readwrite("p2", &PyPose::p2)
            .def_readwrite("p3", &PyPose::p3)
            .def_readwrite("p4", &PyPose::p4);

    py::class_<PyPoint>(m, "point")
            .def(py::init<float, float>())
            .def_readwrite("x", &PyPoint::x)
            .def_readwrite("y", &PyPoint::y);
}