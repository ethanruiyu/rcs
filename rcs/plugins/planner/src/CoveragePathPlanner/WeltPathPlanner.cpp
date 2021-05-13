//
// Created by zhihui on 2020/11/30.
//

#include "CoveragePathPlanner/WeltPathPlanner.h"

WeltPathPlanner::WeltPathPlanner(cv::Mat dilate_map, cv::Point start_point)
{
    dilate_map_ = dilate_map;
    start_point_ = start_point;
}

double WeltPathPlanner::cal_distance(cv::Point point1, cv::Point point2)
{
    int delta_x = point2.x - point1.x;
    int delta_y = point2.y - point1.y;
    double l = std::sqrt(std::pow(delta_y,2) + std::pow(delta_x, 2));
    return l;
}

bool WeltPathPlanner::is_in_close_list(std::vector<std::vector<cv::Point>> close_list, std::vector<cv::Point> polygon)
{
    for(auto close_polygon : close_list)
    {
        if(close_polygon == polygon)
            return true;
    }
    return  false;
}

void WeltPathPlanner::AroundPolygonPath(std::vector<cv::Point> polygon, int polygon_start_idex, std::vector<cv::Point>& polygon_path)
{
    std::vector<cv::Point> path1 = polygon;
    for(auto point : polygon)
    {
        path1.push_back(point);
    }
    for(int i=polygon_start_idex; i < polygon_start_idex+polygon.size()+1; i++)
        polygon_path.push_back(path1[i]);
}

void WeltPathPlanner::PathSort(std::vector<std::vector<cv::Point>>  welt_path, cv::Point init_point, std::vector<std::vector<cv::Point>>& sort_path)
{
    std::vector<std::array<int , 2>>  idex_list;
    std::array<int , 2> idex = {0, 0};
    double l0_max = INT_MAX;
    for(int j=0; j < welt_path[0].size(); j++)
    {
        double l = cal_distance(init_point, welt_path[0][j]);
        if (l < l0_max)
        {
            l0_max = l;
            idex[0] = 0;
            idex[1] = j;
        }
    }
    idex_list.push_back(idex);
    std::vector<cv::Point> edge_path;
    AroundPolygonPath(welt_path[idex_list[0][0]], idex_list[0][1], edge_path);
    sort_path.push_back(edge_path);

    if(welt_path.size() > 1)
    {
        init_point = welt_path[idex[0]][idex[1]];
        std::vector<std::vector<cv::Point>> close_list;
        close_list.push_back(welt_path[0]);

        while (close_list.size() != welt_path.size())
        {
            double l_max = INT_MAX;
            for(int i=1; i < welt_path.size(); i++)
            {
                if(is_in_close_list(close_list, welt_path[i]))
                    continue;
                for(int j=0; j < welt_path[i].size(); j++)
                {
                    double l = cal_distance(init_point, welt_path[i][j]);
                    if (l < l_max)
                    {
                        l_max = l;
                        idex[0] = i;
                        idex[1] = j;
                    }
                }
            }
            idex_list.push_back(idex);
            close_list.push_back(welt_path[idex[0]]);
            init_point = welt_path[idex[0]][idex[1]];
        }

        for(int i=1; i<idex_list.size();i++)
        {
            std::vector<cv::Point> polygon_path;
            AroundPolygonPath(welt_path[idex_list[i][0]], idex_list[i][1], polygon_path);
            sort_path.push_back(polygon_path);
        }
    }




}

void WeltPathPlanner::find_xy_area(std::vector<cv::Point> polygon, std::vector<double>& xy_area)
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

void WeltPathPlanner::dilate_map(cv::Mat coverage_map, cv::Mat& dilate_image, float robot_size)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat coverage_map_rgb;
    cv::findContours(coverage_map.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    cv::drawContours(coverage_map_rgb, contours,-1,(255,0, 255), 8);
    for(const auto contour : contours)
    {
        for(const auto& point:contour)
        {
            cv::Point point1, point2;
            point1 = { point.x + int(robot_size)/2, point.y + int(robot_size)/2};
            point2 = { point.x - int(robot_size)/2, point.y - int(robot_size)/2};
            cv::rectangle(coverage_map, point1, point2, cv::Scalar(0), -1);
        }
    }
    dilate_image = coverage_map;

//    cv::imshow("countors1", coverage_map_rgb);
//    cv::imshow("countors", dilate_image);
//    cv::waitKey(0);

}

bool WeltPathPlanner::Plan(bool start_is_current_point)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilate_map_.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
//    int rows = dilate_map_.rows;
//    int cols = dilate_map_.cols;
//    cv::Mat image_rgb(rows,cols,CV_8UC3,cv::Scalar(0,0,0));
//    cv::drawContours(image_rgb, contours,-1,(255,0, 255), 8);
//    cv::imshow("countors", image_rgb);
//    cv::waitKey(0);
    std::vector<cv::Point> edge_contours;
    std::vector<std::vector<cv::Point>> visual_wall_contours;
    std::vector<double> contours_area_list;
    double area = 0;
    for(int i=0; i<contours.size(); i++)
    {
        double contours_area = contourArea(contours[i], false );
        if(contours_area > area)
        {
            edge_contours = contours[i];
            area = contours_area;
        }

    }
    for(int i=0; i<contours.size(); i++)
    {
        if(contours[i] != edge_contours)
            visual_wall_contours.push_back(contours[i]);
    }

    std::vector<cv::Point> edge_polygon;
    std::vector<std::vector<cv::Point>> visual_wall_polygon;
    std::vector<cv::Point> all_polygons;
    if(edge_contours.empty())
        return false;
    cv::approxPolyDP(edge_contours, edge_polygon, 5, true);
    for(int i=0; i<visual_wall_contours.size(); i++)
    {
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(visual_wall_contours[i], polygon, 5, true);
        if(polygon.size()>2)
            visual_wall_polygon.push_back(polygon);
    }

    std::vector<std::vector<cv::Point>> welt_path_unsort;
    welt_path_unsort.push_back(edge_polygon);
    for(auto wall : visual_wall_polygon)
    {
        welt_path_unsort.push_back(wall);
    }

//    std::vector<std::vector<cv::Point>> sort_path;
    PathSort(welt_path_unsort, start_point_, welt_path_);
    std::vector<double> xy_area;
    find_xy_area(welt_path_[0], xy_area);

    std::vector<cv::Point> start_vector;
    start_vector.push_back(start_point_);
    if(!start_is_current_point)
        welt_path_.insert(welt_path_.begin(),start_vector);

    std::vector<cv::Point> link_path;
    for(int i=0; i<welt_path_.size(); i++)
    {
        for(int j=0; j<welt_path_[i].size(); j++)
        {
            link_path.push_back(welt_path_[i][j]);
        }
    }

    cv::Mat prm_map;
    cv::bitwise_not(dilate_map_, prm_map);
    WeltPathPlanner::dilate_map(prm_map, prm_map, 3);
    welt_path_filte_.push_back(*(welt_path_.begin()));
    int start_num = 0;
    for(int k=0; k<welt_path_.size()-1; k++)
    {
        std::vector<cv::Point>::iterator start_iterator = welt_path_[start_num].end()-1;
        std::vector<cv::Point>::iterator end_iterator = welt_path_[k+1].begin();
        cv::Point start_prm = *start_iterator;
        cv::Point end_prm = *end_iterator;
        PRMPlanner prm_planner(30, 0, xy_area[0]-50, xy_area[1]+50,
                               xy_area[2]-50, xy_area[3]+50);
        prm_planner.initialize(prm_map);
        if(!prm_planner.plan_with_astar(start_prm.x, start_prm.y, end_prm.x, end_prm.y, link_path))
            continue;
        std::vector<TGlobalOrd> prm_path = prm_planner.getpath();

        std::vector<cv::Point> prm_path_cv;
        cv::Point prm_point_cv;
        for(int i=0; i<prm_path.size(); i++)
        {
            prm_point_cv.x = prm_path[i].x;
            prm_point_cv.y = prm_path[i].y;
            welt_path_[k+1].insert(welt_path_[k+1].begin()+i,prm_point_cv);
        }
        welt_path_filte_.push_back(welt_path_[k+1]);
        start_num = k+1;
    }
}

std::vector<cv::Point> WeltPathPlanner::GetPath()
{
    for(auto path : welt_path_filte_)
    {
        for(auto point : path)
        {
            welt_path_visualize_.push_back(point);
        }
    }
    return welt_path_visualize_;
}

std::vector<cv::Point> WeltPathPlanner::GetPoint()
{
    std::vector<cv::Point> welt_point;
    for(auto path : welt_path_)
    {
        for(auto point : path)
        {
            welt_point.push_back(point);
        }
    }
    return welt_point;
}

void WeltPathPlanner::VisualizePath(cv::Mat image_gray,std::string map_path)
{
    cv::Mat image_map_rgb;
    cv::cvtColor(image_gray, image_map_rgb, cv::COLOR_GRAY2BGR);
    if(welt_path_visualize_.empty())
        return;
    for (int k=0; k<welt_path_visualize_.size()-1; k++)
    {
        cv::line(image_map_rgb, welt_path_visualize_[k], welt_path_visualize_[k+1], cv::Scalar(255, 0, 128));
    }

    cv::imwrite(map_path+"/WeltPath.png", image_map_rgb);
}
