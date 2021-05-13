//
// Created by tuqirui on 21-1-12.
//

#include "CoveragePathPlanner/path_planner.h"

Mat find_subregion(Mat& map, int cost)
{
    int rows = map.rows;
    int cols = map.cols;
    Mat image(rows,cols,CV_8UC1);
    for(int i=0; i < rows; i++)
    {
        for(int j=0; j < cols; j++)
        {
            if (map.at<u_char>(i,j) == cost)
                image.at<u_char>(i,j) = 255;
            else
                image.at<u_char>(i,j) = 0;
        }
    }
    return image;
}

bool find_vertex(Mat subregion, std::vector<cv::Point>& contours_vertex, bool display)
{
    std::vector< std::vector< cv::Point> > contours;
//    cv::imshow("subregion", subregion);
//    cvWaitKey(0);
    cv::findContours(subregion,contours,cv::noArray(),cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
//    cv::drawContours(subregion, contours, -1, cv::Scalar::all(127));
    if(contours.size() == 0)
    {
        return false;
    }
    std::vector<int> counts_size;
    for(int i=0; i<contours.size(); i++)
    {
        counts_size.push_back(contours[i].size());
    }
    auto maxPosition = std::max_element(counts_size.begin(), counts_size.end());

    std::vector<std::vector<Point>> contours_poly(contours.size());
    cv::approxPolyDP(contours[maxPosition - counts_size.begin()], contours_poly[0], 5, true);

    if(display)
    {
        int rows = subregion.rows;
        int cols = subregion.cols;
        cv::Mat image(rows,cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::drawContours(image, contours_poly, 0, Scalar(0, 255, 255), 2, 8);  //绘制
        //std::cout << contours_poly[0].size() << std::endl;
        namedWindow("display_decompose");
        imshow("display_decompose", image);
        cvWaitKey(0);
    }
    contours_vertex = contours_poly[0];
    return true;
}

int max_x(std::vector<cv::Point> subregion_vertex)
{
    subregion_vertex.push_back(subregion_vertex[0]);
    int max_x = INT_MIN;
    for(int i=0; i<subregion_vertex.size()-1; i++)
    {
        int delta_x = std::abs(subregion_vertex[i].x - subregion_vertex[i+1].x);
        if(delta_x > max_x)
        {
            max_x = delta_x;
        }
    }
    return max_x;
}
int max_y(std::vector<cv::Point> subregion_vertex)
{
    subregion_vertex.push_back(subregion_vertex[0]);
    int max_y = INT_MIN;
    for(int i=0; i<subregion_vertex.size()-1; i++)
    {
        int delta_y = std::abs(subregion_vertex[i].y - subregion_vertex[i+1].y);
        if(delta_y > max_y)
        {
            max_y = delta_y;
        }
    }
    return max_y;
}

void visualize_decompose_area(cv::Mat subregion, std::vector<std::vector<cv::Point>> subregion_vertex_list, std::string map_path)
{
    int rows = subregion.rows;
    int cols = subregion.cols;
    cv::Mat image(rows,cols,CV_8UC3,cv::Scalar(0,0,0));
    std::vector<std::vector<cv::Point>> subregion_vertex_list_list;
    cv::RNG rng(time(0));
    for(int i=0; i<subregion_vertex_list.size(); i++)
    {

        subregion_vertex_list_list.push_back(subregion_vertex_list[i]);
        cv::drawContours(image, subregion_vertex_list_list, 0, Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)), 2, 8);  //绘制
        subregion_vertex_list_list.clear();
    }
//    namedWindow("display_decomposed");
//    imshow("display_decomposed", image);
//    cvWaitKey(0);
    cv::imwrite(map_path+"/display_decomposed.png", image);

    //std::cout << contours_poly[0].size() << std::endl;

}

void polygon_cv_to_verts(std::vector<cv::Point> polygon, std::vector<cxd::Vertex>& verts)
{
    cxd::Vertex vert;
    for(auto point1 : polygon)
    {
        vert.position.x = point1.x;
        vert.position.y = point1.y;
        verts.push_back(vert);
    }
}

void two_vector_verts_to_cv(std::vector<std::vector<cxd::Vertex>>& verts, std::vector<std::vector<cv::Point>>& polygon)
{
    cxd::Vertex vertex;
    std::vector<cxd::Vertex> vertex_list;
    cv::Point polygon_point;
    std::vector<cv::Point> subpolygon;
    for(int i=0; i<verts.size(); i++)
    {
        for(int j=0; j<verts[i].size(); j++)
        {
            polygon_point.x = verts[i][j].position.x;
            polygon_point.y = verts[i][j].position.y;
            subpolygon.push_back(polygon_point);
        }
        polygon.push_back(subpolygon);
        subpolygon.clear();
    }
}

bool BowPathPlanner(std::vector<cv::Point> polygon, float sweep_wide, float coincidence_rate,
                    cv::Mat dilate_image_coverage_prm, cv::Point start_point, std::vector<cv::Point>& path)
{
    if(polygon.size() < 3)
    {
        return false;
    }
    else if(max_x(polygon) < (sweep_wide/4) || max_y(polygon) < (sweep_wide/4))
    {
        return false;
    }


    PointVector candidatePath_first;
    bool isOptimal = computeConvexCoverage(polygon, sweep_wide, coincidence_rate, candidatePath_first, dilate_image_coverage_prm);//生成一条初始弓字形路径
    if (isOptimal == true && (candidatePath_first.size() != 0))
    {
        PointVector path_uncheck, path_check;
        path_uncheck = identifyOptimalAlternative(polygon, candidatePath_first, start_point); //根据起点与终点选择路径方向
        check_intersection(polygon, path_uncheck, path_check, dilate_image_coverage_prm); //检查所有路径与多边形是否有交点，并生成一条没有交点路径
        if (hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(path, false)) == true)
        {
//            std::cout << " false : path has Intersection with edge "<< std::endl;
            return false;
        }
        path = path_check;
        return true;
    }
    else
    {
        std::vector<PointVector> candidatePath_list;
        bool existsSecondOptimalPath = findSecondOptimalPath(polygon, sweep_wide, coincidence_rate, candidatePath_list, dilate_image_coverage_prm);
        if (existsSecondOptimalPath == true && (candidatePath_list.size() != 0)) //如果第二最优全覆盖遍历方向存在（不分割情况）
        {
            int path_size = INT_MAX;
            for(auto candidatePath : candidatePath_list)
            {
                // compute optimal alternative for second optimal path
                PointVector secondOptimalPath_uncheck, secondOptimalPath_check;
                secondOptimalPath_uncheck = identifyOptimalAlternative(polygon, candidatePath, start_point);
                check_intersection(polygon, secondOptimalPath_uncheck, secondOptimalPath_check, dilate_image_coverage_prm); //检查所有路径与多边形是否有交点，并生成一条没有交点路径
                if (!(hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(path, false)) == true) && secondOptimalPath_check.size() < path_size)
                {
                    path_size = secondOptimalPath_check.size();
                    path = secondOptimalPath_check;
                }
            }
            if(path.empty())
                return false;
            return true;
        }
        else
        {
            std::cout << "cannot plan coverage path" << std::endl;
            return false;
        }

    }
}

//bool coverage_path_planning_delaunay(cv::Mat img_b, cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv,
//                                     float grass_wide, float coincidence_rate, cv::Point start, std::string map_path)
//{
//    std::vector<std::vector<cv::Point>> sub_polygon_list;
//    TriangulationOptimizationDecompose triangulation_optimization_decompose(img_b, grass_wide);
//    triangulation_optimization_decompose.GetPolygonVertex();
//    triangulation_optimization_decompose.DelaunayDecompose();
//    sub_polygon_list = triangulation_optimization_decompose.sub_polygon_list_;
//    triangulation_optimization_decompose.VisualizeDecomposeArea();
//
//    for(int i=0; i<sub_polygon_list.size(); i++)
//    {
//        PointVector path;
//        if(BowPathPlanner(sub_polygon_list[i], grass_wide, coincidence_rate, PRM_map, start, path))
//        {
//            path_cv.push_back(path);
//            start = *(path.end()-1);
//        }
//
//    }
//}

int find_nearist_area(cv::Point start, std::vector<std::vector<cv::Point>> subregion_vertex_list)
{
    double l = INT_MAX;
    int idx_1, idx_2;
    for(int i=0; i<subregion_vertex_list.size(); i++)
    {
        for(int j=0; j<subregion_vertex_list[i].size(); j++)
        {
            double distance = std::pow(start.x - subregion_vertex_list[i][j].x, 2) + std::pow(start.y - subregion_vertex_list[i][j].y, 2);
            if(distance < l)
            {
                l = distance;
                idx_1 = i;
                idx_2 = j;
            }
        }
    }
    return idx_1;
}


bool coverage_path_planning_bcd(cv::Mat img_b, cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv, float grass_wide,
                                float coincidence_rate, cv::Point start, std::string map_path)
{
    cv::Mat img_find_angle;
    threshold(img_b,img_find_angle,0,255,THRESH_BINARY);

    float angle = cal_rat_angle(img_find_angle, map_path);
//    float angle = 0;
    cv::Mat img_rotate;
    rotate_arbitrarily_angle(img_b,img_rotate, -angle);

    //for debug
//    cv::Mat img_rotate_visual;
//    threshold(img_rotate,img_rotate_visual,0,255,THRESH_BINARY);
//    cv::imshow("img_rotate_visual", img_rotate_visual);
//    cv::waitKey(0);

    int cells=0;
    Mat separate_map=BCD::calc_bcd(img_rotate,cells);

    PointVector subpolygons;
    Mat subregion;
    std::vector<cv::Point> subregion_vertex;

    std::vector<PointVector> path_list;
    // start point of coverage path
    //cv::Point start;
    std::cout << "cells = " << cells-2 << std::endl;
    std::vector<std::vector<cv::Point>> subregion_vertex_list;
    for (int i=2; i<cells ;i++)
    {
        subregion = find_subregion(separate_map, i);
        if (!find_vertex(subregion, subregion_vertex, false)) {
            std::cout << "can not find contours " << std::endl;
            return false;
        }
        std::vector<cv::Point> subregion_vertex_rotate = path_transform(subregion_vertex, img_rotate, img_b, angle);
        subregion_vertex_list.push_back(subregion_vertex_rotate);
    }
    visualize_decompose_area(subregion, subregion_vertex_list, map_path);
    PointVector path;

    //for debug
//    if(BowPathPlanner(subregion_vertex_list[4], grass_wide, coincidence_rate, PRM_map, start, path))
//    {
//        path_cv.push_back(path);
//        start = *(path.end()-1);
//    }

    while(!subregion_vertex_list.empty())
    {
        int near_idx = find_nearist_area(start, subregion_vertex_list);
        PointVector path;
        if(BowPathPlanner(subregion_vertex_list[near_idx], grass_wide, coincidence_rate, PRM_map, start, path))
        {
            path_cv.push_back(path);
            start = *(path.end()-1);
        }
        subregion_vertex_list.erase(subregion_vertex_list.begin() + near_idx);
    }

    return true;
}

bool coverage_path_planning_concave(cv::Mat img_b, cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv, float grass_wide,
                                    float coincidence_rate, cv::Point start, std::string map_path)
{

//多边形分割
    std::vector<cv::Point> polygon;
    if(!find_vertex(img_b, polygon, false))
    {
        std::cout << "can not find contours " << std::endl;
        return false;
    }


    std::vector<cxd::Vertex > verts;
    polygon_cv_to_verts(polygon, verts);
    cxd::ConcavePolygon poly(verts);
    poly.convexDecomp();

    std::vector<cxd::ConcavePolygon > polygon_list;
    poly.returnLowestLevelPolys(polygon_list);

    std::vector<std::vector<cxd::Vertex>> ploygon_vector;
    std::vector<cxd::Vertex> sub;
    for(auto subpolygon : polygon_list)
    {
        sub = subpolygon.getVertices();
        ploygon_vector.push_back(sub);
        sub.clear();
    }

    std::vector<std::vector<cv::Point>> subpolygon_list;
    two_vector_verts_to_cv(ploygon_vector, subpolygon_list);
    visualize_decompose_area(img_b, subpolygon_list, map_path);

    for(int i=0; i<subpolygon_list.size(); i++)
    {
        PointVector path;
        if(BowPathPlanner(subpolygon_list[i], grass_wide, coincidence_rate, PRM_map, start, path))
        {
            path_cv.push_back(path);
            start = *(path.end()-1);
        }

    }

    return true;
}

void visualize_prm_path(std::vector<TGlobalOrd> path, Mat map)
{
    std::vector<cv::Point> cv_path;
    cv::Point cv_point;
    for(int i=0; i<path.size(); i++)
    {
        cv_point.x = path[i].x;
        cv_point.y = path[i].y;
        cv_path.push_back(cv_point);
    }
    for(int j=0; j<cv_path.size()-1; j++)
    {
        cv::line(map, cv_path[j], cv_path[j+1], cv::Scalar(125));
    }
//    cv::namedWindow("prm_path");
//    imshow("prm_path", map);
    //cvWaitKey(0);
}

bool prm_path_planning(cv::Mat PRM_map, std::vector<std::vector<cv::Point>>& path_cv, float grass_wide, std::vector<double> xy_area, std::vector<cv::Point> welt_path2)
{
    if(path_cv.size() == 0)
    {
        std::cout << "coverage path is null" << std::endl;
        return false;
    }
    std::vector<cv::Point> link_point;
    for(int i=0; i<welt_path2.size(); i++)
    {
        link_point.push_back(welt_path2[i]);
    }
    for(int i=0; i<path_cv.size(); i++)
    {
        for(int j=0; j<path_cv[i].size(); j++)
        {
            link_point.push_back(path_cv[i][j]);
        }
    }
//    PRMPlanner prm_planner(30, 0, xy_area[0], xy_area[1], xy_area[2], xy_area[3]);
//    prm_planner.initialize(PRM_map);
    std::vector<std::vector<TGlobalOrd>> prm_path_list;
    cv::Mat PRM_map_RGB;
    cv::cvtColor(PRM_map, PRM_map_RGB, cv::COLOR_GRAY2BGR);
    std::vector<std::vector<cv::Point>> path_cv_filter;
    path_cv_filter.push_back(path_cv[0]);
    int start_num = 0;
    for(int k=0; k<path_cv.size()-1; k++)
    {
        std::vector<cv::Point>::iterator start_iterator = path_cv[start_num].end()-1;
        std::vector<cv::Point>::iterator end_iterator = path_cv[k+1].begin();
        cv::Point start_prm = *start_iterator;
        cv::Point end_prm = *end_iterator;
        PRMPlanner prm_planner(30);
        prm_planner.initialize(PRM_map);
        if(!prm_planner.plan_with_astar(start_prm.x, start_prm.y, end_prm.x, end_prm.y, link_point))
            continue;
        std::vector<TGlobalOrd> prm_path = prm_planner.getpath();

        //visualize_prm_path(prm_path, PRM_map_RGB);

        std::vector<cv::Point> prm_path_cv;
        cv::Point prm_point_cv;
        for(int i=0; i<prm_path.size(); i++)
        {
            prm_point_cv.x = prm_path[i].x;
            prm_point_cv.y = prm_path[i].y;
//            path_cv[k].push_back(prm_point_cv);
            path_cv[k+1].insert(path_cv[k+1].begin()+i,prm_point_cv);
        }
        //path_cv.insert(path_cv.begin()+k, prm_path_cv.begin(), prm_path_cv.end());
        path_cv_filter.push_back(path_cv[k+1]);
        start_num = k+1;
    }
    path_cv = path_cv_filter;
    return true;
}



void path_to_vector_path(std::vector<std::vector<cv::Point>> path_cv, std::vector<cv::Point>& vector_path)
{
    for(int j=0;j<path_cv.size();j++)
    {
        for(int k=0;k<path_cv[j].size();k++)
        {
            vector_path.push_back(path_cv[j][k]);
        }
    }
}



bool to_coverage_start_path(cv::Mat virtual_walls_map, cv::Point init_point_cv, std::vector<std::vector<cv::Point>>& path_se,
                            std::vector<cv::Point>& prm_path_cv, std::string map_path)
{
    if(path_se.empty())
    {
        std::cout << "false :: coverage path in null" << std::endl;
        return false;
    }
    std::cout << "coverage path size = " << path_se.size() << std::endl;

    double path_start_point_x = path_se[0][0].x;
    double path_start_point_y = path_se[0][0].y;

    cv::Mat visual_map = virtual_walls_map.clone();
    cv::circle(visual_map, init_point_cv, 3, cv::Scalar(200));
    cv::circle(visual_map, path_se[0][0], 3, cv::Scalar(200));
    cv::imwrite(map_path + "/start_point_to_coverage_map.png", visual_map);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<double> xy_area;
    //astar path plan
    std::vector<cv::Point> astar_path;
    normal_astar_planner::NormalAstarPlannerCore astar;
    astar.setStart(init_point_cv);
    astar.setGoal(path_se[0][0]);
    astar.setMap(virtual_walls_map);
    if(!astar.makePlan(astar_path))
        return false;
    std::cout << "success astar plan" <<std::endl;


    contours.push_back(astar_path);
    PRMPlanner prm_planner1(30); //grass_wide
    prm_planner1.initialize(virtual_walls_map);
    if(!prm_planner1.plan_with_astar(init_point_cv.x, init_point_cv.y, path_start_point_x, path_start_point_y, astar_path))
        return false;
    std::vector<TGlobalOrd> prm_path = prm_planner1.getpath();
    cv::Point prm_point_cv;
    for(auto point1 : prm_path) {
        prm_point_cv.x = point1.x;
        prm_point_cv.y = point1.y;
        prm_path_cv.push_back(prm_point_cv);
    }

    path_se.insert(path_se.begin(), prm_path_cv);
    return true;
}

void visualize_path(std::vector<cv::Point> path, cv::Mat map, std::string map_path){
    std::vector<cv::Point> visual_path;
    cv::Point visual_point;
    for(int j=0; j<path.size(); j++)
    {
        visual_point.x = path[j].x;
        visual_point.y = path[j].y;
        visual_path.push_back(visual_point);
    }
    //int rows = map.rows;
    //int cols = map.cols;
    //cv::Mat image2(rows,cols,CV_8UC3,cv::Scalar(0,0,0));
    for (int k=0; k<visual_path.size()-1; k++)
    {
        cv::line(map, visual_path[k], visual_path[k+1], cv::Scalar(255, 0, 128));
    }
    //cv::drawContours(image2, visual_path, 0, Scalar(0, 255, 255), 2, 8);  //绘制
    //std::cout << contours_poly[0].size() << std::endl;
//    cv::namedWindow("BCD2_Window");
//    imshow("BCD2_Window", map);
//    cv::imwrite("/home/zhihui/coverage-path-planner-c/cpp.png", map);
//    cvWaitKey(0);
    cv::imwrite(map_path+"/cpp.png", map);

}

int cal_point_num(cv::Mat map)
{
    int k = 0;
    for(int i=0; i<map.cols; i++)
    {
        for(int j=0; j<map.rows; j++)
        {
            if(map.at<u_int8_t>(j,i) == 255)
            {
                k = k+1;
            }
        }
    }
    return k;
}

double  cal_coverage_ratio(std::vector<cv::Point> path, cv::Mat map, float robot_size, std::string map_path)
{
//    cv::bitwise_not(map, map);
    int coverage_map_point = cal_point_num(map);
    cv::Mat dilate_path_map =  Mat(Size(map.cols, map.rows), CV_8UC1, cv::Scalar(0));
    for (int k=0; k<path.size()-1; k++)
    {
        cv::line(dilate_path_map, path[k], path[k+1], cv::Scalar(255));
    }
    double dilate_window_size_ = std::ceil(robot_size);
    int size = static_cast<int>(dilate_window_size_ / 2.0);
    int type = cv::MORPH_RECT;
    cv::Mat element = getStructuringElement(type,cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
    dilate(dilate_path_map, dilate_path_map, element);

    cv::Mat dilate_path_map2 = cv::Mat::zeros(dilate_path_map.size(), CV_8UC1);
    dilate_path_map.copyTo(dilate_path_map2, map);
    int dilate_path_map_point = cal_point_num(dilate_path_map2);
//    cv::imshow("dilate_path_map", dilate_path_map);
//    cv::waitKey(0);
    cv::imwrite(map_path+"/dilate_path_map.png", dilate_path_map2);
    double p = double(dilate_path_map_point)/double(coverage_map_point);
    return p;
}