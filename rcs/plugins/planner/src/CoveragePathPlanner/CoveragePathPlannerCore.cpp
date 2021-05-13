//
// Created by tuqirui on 21-1-11.
//

#include "GlobalPathPlannerCore.h"

CoveragePathPlannerCore::CoveragePathPlannerCore(float coverage_wide, float coincidence_rate, std::string map_path)
{
    coverage_wide_ = coverage_wide;
    coincidence_rate_ = coincidence_rate;
    map_path_ = map_path;
    init();
}

void CoveragePathPlannerCore::init()
{
    is_get_map_ = false;
    is_get_init_point_ = false;
    is_get_goal_ = false;
}

void CoveragePathPlannerCore::setMap(cv::Mat& global_map)
{
    global_map_ = global_map;
    is_get_map_ = true;
    std::cout << "success set map" << std::endl;
}

void CoveragePathPlannerCore::setInitPose(cv::Point& init_point)
{
    init_point_ = init_point;
    is_get_init_point_ = true;
    std::cout << "success set init point" << std::endl;
}

void CoveragePathPlannerCore::setGoal(std::vector<cv::Point>& goal_area)
{
    initCoverageMap(goal_area, coverage_map_); //读取覆盖区域参数
//    coverage_map_ = global_map_; //直接读取覆盖地图
    is_get_goal_ = true;
    cv::imwrite(map_path_+"/coverage_map.png", coverage_map_);
    std::cout << "success set goal" << std::endl;
}

void CoveragePathPlannerCore::initCoverageMap(std::vector<cv::Point> polygon, cv::Mat& coverage_map)
{
    //coverage_map = cv::Mat(prm_map.rows, prm_map.cols, CV_8UC1, cv::Scalar(255));
    cv::Mat mask = cv::Mat::zeros(global_map_.size(), CV_8UC1);
    coverage_map = cv::Mat::zeros(global_map_.size(), CV_8UC1);
    int npt1;
    npt1 = polygon.size();
    cv::Point coverage_area[1][npt1];
    for(int j=0; j<polygon.size(); j++)
    {
        coverage_area[0][j] = polygon[j];
    }

    const cv::Point* ppt[1]={coverage_area[0]};
    int npt[]={npt1};
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255), 0);
    global_map_.copyTo(coverage_map, mask);
//    cv::imshow("coverage_map", coverage_map);
//    cv::waitKey(0);
}

bool CoveragePathPlannerCore::isExistVisualWall()
{
    cv::Mat img = coverage_map_.clone();
    cv::threshold(img,img,130,255,cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
//    for(int i=0; i<contours.size(); i++)
//    {
//        cv::drawContours(image,contours, i, cv::Scalar(255, 0, 0), 3, 8, hierarchy);
//    }
//    cv::drawContours(image,contours, 0, cv::Scalar(255, 0, 0), 3, 8, hierarchy);

    if(contours.size() > 2)
    {
        std::cout << "========= exit visual wall ==========" << std::endl;
        return true;
    }
    else
    {
        std::cout << "========= not exit visual wall ==========" << std::endl;
        return false;
    }
}

void CoveragePathPlannerCore::dilateMap(cv::Mat coverage_map, cv::Mat& dilate_image, float robot_size)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat coverage_map_rgb;
    cv::cvtColor(coverage_map, coverage_map_rgb, cv::COLOR_GRAY2BGR);
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

bool CoveragePathPlannerCore::filterPath()
{
    std::vector<cv::Point> vector_path;
    for(int j=0;j<coverage_path_.size();j++)
    {
        for(int k=0;k<coverage_path_[j].size();k++)
        {
            vector_path.push_back(coverage_path_[j][k]);
        }
    }
    coverage_path_filter_.push_back(vector_path[0]);
    for(int i=1; i<vector_path.size(); i++)
    {
        if(vector_path[i].x == vector_path[i-1].x && vector_path[i].y == vector_path[i-1].y)
            continue;
        coverage_path_filter_.push_back(vector_path[i]);
    }
    return true;
}


int CoveragePathPlannerCore::plan()
{
    std::cout << "start to coverage plan" << std::endl;
    if(!is_get_goal_ || !is_get_init_point_ || !is_get_map_)
        return COVERAGE_ERROR_INIT;

    //地圖處理
    cv::Mat dilate_image, dilate_image_prm_link, dilate_image_coverage_prm;
    dilateMap(coverage_map_.clone(), dilate_image, coverage_wide_);  //用于全覆盖路径规划地图

    cv::Mat prm_link_map;
    dilateMap(coverage_map_.clone(), dilate_image_prm_link, std::floor(coverage_wide_*0.9));//区域连接规划地图
    threshold(dilate_image_prm_link,prm_link_map,130,255,cv::THRESH_BINARY_INV);
    cv::imwrite(map_path_+"/prm_link_map.png", prm_link_map);

    cv::Mat start_point_to_coverage_map;
    dilateMap(global_map_.clone(), start_point_to_coverage_map, std::floor(coverage_wide_*0.7));//用于当前位置到覆盖区域起点 路径规划地图
    cv::bitwise_not(start_point_to_coverage_map, start_point_to_coverage_map);

    dilateMap(coverage_map_.clone(), dilate_image_coverage_prm, std::ceil(coverage_wide_*0.9));//用于全覆盖路径 路径与边界相交时重规划该路径地图
    threshold(dilate_image_coverage_prm,dilate_image_coverage_prm,130,255,cv::THRESH_BINARY_INV);


    //全覆盖弓字形路径
    if(isExistVisualWall())
    {
//        CPP_BCD_Planner::CPP_BCD cpp(dilate_image, grass_wide, coincidence_rate, map_path);
//            std::vector<cv::Point> cpp_path = cpp.Plan();
//            if(cpp_path.empty())
//            {
//               std::cout << "bcd cpp path is empty" << std::endl;
//                return 1;
//            }
//            std::vector<cv::Point> CPP_prm_path_cv = cpp.PrmLinkPlan(prm_link_map, polygon);
//            if(add_welt_path)
//            {
//                std::vector<cv::Point> welt_path1;
//                WeltPathPlanner WeltPathPlanner_1(dilate_image, CPP_prm_path_cv.back());
//                WeltPathPlanner_1.Plan();
//                welt_path1 = WeltPathPlanner_1.GetPath();
////                WeltPathPlanner1.VisualizePath(coverage_map);
//                for(auto point :welt_path1)
//                    CPP_prm_path_cv.push_back(point);
//            }
//            cpp.VisualizePath(CPP_prm_path_cv, coverage_map);
//            return 0;
        cv::Mat dilate_image_BCD = dilate_image.clone();
        threshold(dilate_image_BCD,dilate_image_BCD,130,1,cv::THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图（全覆盖路径规划地图） 可通行区域0,障碍物1
        if(!coverage_path_planning_bcd(dilate_image_BCD,dilate_image_coverage_prm, coverage_path_, coverage_wide_, coincidence_rate_, init_point_, map_path_)) //path_se
        {
            return COVERAGE_ERROR_COVERAGE_PATH_FAIL_TO_PLAN;
        }

    }
    else
    {
        cv::Mat dilate_image_concave = dilate_image.clone();
        threshold(dilate_image_concave,dilate_image_concave,130,1,cv::THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图（全覆盖路径规划地图） 可通行区域0,障碍物1
        if(!coverage_path_planning_concave(dilate_image_concave,dilate_image_coverage_prm, coverage_path_, coverage_wide_, coincidence_rate_, init_point_, map_path_)) //path_se
        {
            return COVERAGE_ERROR_COVERAGE_PATH_FAIL_TO_PLAN;
        }
    }


    //子区域连接路径规划
    if(!coverage_path_.empty() )
    {
        std::cout << "start link path plan" << std::endl;
        std::vector<cv::Point> welt_path_1;
        cv::Mat map_point = dilate_image.clone();
        dilateMap(map_point, map_point, 5);
        WeltPathPlanner WeltPathPlanner(map_point, init_point_);
        bool start_is_current_point = false;
        WeltPathPlanner.Plan(start_is_current_point);
        welt_path_1 = WeltPathPlanner.GetPoint();
        std::vector<double> xy_area;
        if(!prm_path_planning(prm_link_map, coverage_path_, coverage_wide_, xy_area, welt_path_1))
        {
            std::cout << "failed link path plan " << std::endl;
            return COVERAGE_ERROR_SUBREGION_CONNECT_PATH_FAIL_TO_PLAN;
        }
        std::cout << "success link path plan" << std::endl;
    }

    // 贴边路径
    std::cout << "start plan welt path" << std::endl;
    std::vector<cv::Point> welt_path2;
    cv::Point start_point1;
    bool start_is_current_point = false;
    if(coverage_path_.empty())
    {
        start_point1 = init_point_;
        start_is_current_point = true;
    }
    else
        start_point1 = coverage_path_.back().back();
    WeltPathPlanner WeltPathPlanner_2(dilate_image, start_point1);
    WeltPathPlanner_2.Plan(start_is_current_point);
    welt_path2 = WeltPathPlanner_2.GetPath(); //Getpath
    WeltPathPlanner_2.VisualizePath(coverage_map_, map_path_);
    if(welt_path2.empty())
    {
        std::cout << "no welt path "<< std::endl;
        return COVERAGE_ERROR_WELT_PATH_FAIL_TO_PLAN;
    }
    coverage_path_.push_back(welt_path2);
    std::cout << "success plan welt path" << std::endl;

    std::vector<cv::Point> vector_path;
    for(int j=0;j<coverage_path_.size();j++)
    {
        for(int k=0;k<coverage_path_[j].size();k++)
        {
            vector_path.push_back(coverage_path_[j][k]);
        }
    }
    coverage_divide_path_.push_back(vector_path);

    //初始點到全覆盖起点路徑
    std::cout << "start current point to coverage start point plan" << std::endl;
    if(!coverage_path_.empty() && (init_point_.x != coverage_path_[0][0].x && init_point_.y != coverage_path_[0][0].y))
    {
        if(start_point_to_coverage_map.at<uchar>(init_point_) != 0)
        {
            std::cout << "init point is not safe" << std::endl;
            return COVERAGE_ERROR_INIT_POINT_IS_NOT_SAFE;
        }
        if(start_point_to_coverage_map.at<uchar>(coverage_path_[0][0]) != 0)
        {
            std::cout << "goal point is not safe" << std::endl;
            return COVERAGE_ERROR_COVERAGE_START_POINT_IS_NOT_SAFE;
        }

        std::vector<cv::Point> prm_path_cv;
        if(!to_coverage_start_path(start_point_to_coverage_map, init_point_, coverage_path_, prm_path_cv, map_path_))
        {
            std::string s7("init start to coverage start path planning error");
            return COVERAGE_ERROR_INIT_POINT_TO_COVERAGE_START_PATH_FAIL_TO_PLAN;
        }
        std::cout << "finish current point to coverage start point plan" << std::endl;
        coverage_divide_path_.insert(coverage_divide_path_.begin(), prm_path_cv);
    }


    //路径格式转换（转成单向量路径）
    std::vector<cv::Point> visual_path;
    if(!coverage_path_.empty())
    {
        if(!filterPath())
        {
            std::string s8("two_vector_path convert vector_path error");
            return COVERAGE_ERROR_PATH_FAIL_TO_CONVERT_VECTOR;
        }

        //显示路径
        cv::Mat coverage_map_rgb;
        cv::Mat coverage_map_visualize;
        threshold(coverage_map_,coverage_map_visualize,0,255,cv::THRESH_BINARY);
        cv::cvtColor(coverage_map_visualize, coverage_map_rgb, COLOR_GRAY2BGR);
        visualize_path(coverage_path_filter_, coverage_map_rgb, map_path_);

        //计算覆盖率
        coverage_ratio_ = cal_coverage_ratio(coverage_path_filter_, coverage_map_, coverage_wide_, map_path_);
        std::cout << "coverage_ratio = " << coverage_ratio_ << std::endl;
    }
    return 0;
}

std::vector<cv::Point> CoveragePathPlannerCore::getPath()
{
    return coverage_path_filter_;
}

std::vector<std::vector<cv::Point>> CoveragePathPlannerCore::getCoverageDividePath()
{
    return coverage_divide_path_;
}

float CoveragePathPlannerCore::getCoverageRatio()
{
    return coverage_ratio_;
}