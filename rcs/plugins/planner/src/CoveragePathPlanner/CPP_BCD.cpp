//
// Created by zhihui on 2020/12/5.
//

#include "CoveragePathPlanner/CPP_BCD.h"

using namespace CPP_BCD_Planner;




CPP_BCD::CPP_BCD(cv::Mat map, float robot_size, float coincidence_rate, std::string map_path)
{
    map_ = map;
    robot_size_ = robot_size;
    coincidence_rate_ = coincidence_rate;
    map_path_ = map_path;
}

cv::Mat1b CPP_BCD::PreprocessMap(const cv::Mat1b& original_map)
{
    cv::Mat1b map = original_map.clone();
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY);
    return map;
}

int CPP_BCD::ComputeRobotRadius(const double& meters_per_pix, const double& robot_size_in_meters)
{
    int robot_radius = int(robot_size_in_meters / meters_per_pix);
    return robot_radius;
}

void CPP_BCD::rotate_arbitrarily_angle(cv::Mat &src,cv::Mat &dst,float angle)
{


    int maxBorder =(int) (cv::max(src.cols, src.rows)* 1.414 ); //即为sqrt(2)*max
    int dx = (maxBorder - src.cols)/2;
    int dy = (maxBorder - src.rows)/2;
    copyMakeBorder(src, dst, dy, dy, dx, dx, cv::BORDER_CONSTANT);
    cv::Point2f center( (float)(dst.cols/2) , (float) (dst.rows/2));
    cv::Mat affine_matrix = getRotationMatrix2D( center, angle, 1.0 );//求得旋转矩阵
    warpAffine(dst, dst, affine_matrix, dst.size());
}

// ========================== extract contours ===============================================
void CPP_BCD::ExtractRawContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& raw_wall_contours, std::vector<std::vector<cv::Point>>& raw_obstacle_contours)
{
    cv::Mat map = original_map.clone();
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY_INV);

    cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);


    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(original_map.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<int> wall_cnt_indices(contours.size());
    std::iota(wall_cnt_indices.begin(), wall_cnt_indices.end(), 0);

//    std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours](int lhs, int rhs){return contours[lhs].size() > contours[rhs].size();});
    std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours](int lhs, int rhs){return cv::contourArea(contours[lhs]) > cv::contourArea(contours[rhs]);});

    std::vector<cv::Point> raw_wall_contour = contours[wall_cnt_indices.front()];
    raw_wall_contours = {raw_wall_contour};

    cv::Mat mask = cv::Mat(original_map.size(), original_map.type(), 255);
    cv::fillPoly(mask, raw_wall_contours, 0);



    cv::Mat base = original_map.clone();
    base += mask;



    cv::threshold(base, base, 128, 255, cv::THRESH_BINARY_INV);



    cv::findContours(base, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    raw_obstacle_contours = contours;
}

void CPP_BCD::ExtractContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& wall_contours, std::vector<std::vector<cv::Point>>& obstacle_contours, int robot_radius=0)
{
    ExtractRawContours(original_map, wall_contours, obstacle_contours);

    if(robot_radius != 0)
    {
        cv::Mat3b canvas = cv::Mat3b(original_map.size(), CV_8U);
        canvas.setTo(cv::Scalar(255, 255, 255));


        cv::fillPoly(canvas, wall_contours, cv::Scalar(0, 0, 0));

        for(const auto& point:wall_contours.front())
        {
            cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
        }

        cv::fillPoly(canvas, obstacle_contours, cv::Scalar(255, 255, 255));


        for(const auto& obstacle_contour:obstacle_contours)
        {
            for(const auto& point:obstacle_contour)
            {
                cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
            }
        }

        cv::Mat canvas_;
        cv::cvtColor(canvas, canvas_, cv::COLOR_BGR2GRAY);
        cv::threshold(canvas_, canvas_, 200, 255, cv::THRESH_BINARY_INV);


        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(robot_radius,robot_radius), cv::Point(-1,-1));
        cv::morphologyEx(canvas_, canvas_, cv::MORPH_OPEN, kernel);



        ExtractRawContours(canvas_, wall_contours, obstacle_contours);
        std::vector<cv::Point> processed_wall_contour;
        cv::approxPolyDP(cv::Mat(wall_contours.front()), processed_wall_contour, 1, true);

        std::vector<std::vector<cv::Point>> processed_obstacle_contours(obstacle_contours.size());
        for(int i = 0; i < obstacle_contours.size(); i++)
        {
            cv::approxPolyDP(cv::Mat(obstacle_contours[i]), processed_obstacle_contours[i], 1, true);
        }

        wall_contours = {processed_wall_contour};
        obstacle_contours = processed_obstacle_contours;
    }
}

PolygonList CPP_BCD::ConstructObstacles(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& obstacle_contours)
{
    PolygonList obstacles;
    Polygon obstacle;

    for(const auto& obstacle_contour : obstacle_contours)
    {
        for(int j = 0; j < obstacle_contour.size()-1; j++)
        {
            cv::LineIterator line(original_map, obstacle_contour[j], obstacle_contour[j+1]);
            for(int k = 0; k < line.count-1; k++)
            {
                obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(original_map, obstacle_contour[obstacle_contour.size()-1], obstacle_contour[0]);
        for(int j = 0; j < line.count-1; j++)
        {
            obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        obstacles.emplace_back(obstacle);
        obstacle.clear();
    }

    return obstacles;
}

Polygon CPP_BCD::ConstructDefaultWall(const cv::Mat& original_map)
{
    std::vector<cv::Point> default_wall_contour = {cv::Point(0, 0), cv::Point(0, original_map.rows-1), cv::Point(original_map.cols-1, original_map.rows-1), cv::Point(original_map.cols-1, 0)};
    std::vector<std::vector<cv::Point>>default_wall_contours = {default_wall_contour};

    Polygon default_wall = ConstructObstacles(original_map, default_wall_contours).front();

    return default_wall;
}

Polygon CPP_BCD::ConstructWall(const cv::Mat& original_map, std::vector<cv::Point>& wall_contour)
{
    Polygon wall;

    if(!wall_contour.empty())
    {
        for(int i = 0; i < wall_contour.size()-1; i++)
        {
            cv::LineIterator line(original_map, wall_contour[i], wall_contour[i+1]);
            for(int j = 0; j < line.count-1; j++)
            {
                wall.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(original_map, wall_contour.back(), wall_contour.front());
        for(int i = 0; i < line.count-1; i++)
        {
            wall.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        return wall;
    }
    else
    {
        wall = ConstructDefaultWall(original_map);

        for(const auto& point : wall)
        {
            wall_contour.emplace_back(cv::Point(point.x, point.y));
        }

        return wall;
    }
}

 //=============================================  map decompose  ============================================

void CPP_BCD::AllocateObstacleEventType(const cv::Mat& map, std::vector<Event>& event_list)
{
    int index_offset;
    std::deque<int> in_out_index_list; // 只存放各种in和out的index

    int N = event_list.size();

    // determine in and out and middle
    for(int i = 0; i < N; i++)
    {
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = IN;
            in_out_index_list.emplace_back(i);
        }
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = MIDDLE;
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = OUT;
            in_out_index_list.emplace_back(i);
        }


        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }
    }

    // determine inner
    Point2D neighbor_point;
    int temp_index;

    for(auto in_out_index : in_out_index_list)
    {
        if(event_list[in_out_index].event_type == OUT)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_OUT;
            }
        }

        if(event_list[in_out_index].event_type == OUT_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_OUT_TOP;
            }
        }

        if(event_list[in_out_index].event_type == OUT_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_OUT_BOTTOM;
            }

        }

        if(event_list[in_out_index].event_type == IN)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_IN;
            }
        }


        if(event_list[in_out_index].event_type == IN_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_IN_TOP;
            }
        }

        if(event_list[in_out_index].event_type == IN_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_IN_BOTTOM;
            }
        }
    }

    // determine floor and ceiling
    std::deque<int> ceiling_floor_index_list;

    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(
                (event_list[in_out_index_list[0]].event_type==OUT
                 ||event_list[in_out_index_list[0]].event_type==OUT_TOP
                 ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM)
                &&
                (event_list[in_out_index_list[1]].event_type==IN
                 ||event_list[in_out_index_list[1]].event_type==IN_TOP
                 ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        if(
                (event_list[in_out_index_list[0]].event_type==IN
                 ||event_list[in_out_index_list[0]].event_type==IN_TOP
                 ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM)
                &&
                (event_list[in_out_index_list[1]].event_type==OUT
                 ||event_list[in_out_index_list[1]].event_type==OUT_TOP
                 ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }


    // filter ceiling and floor
    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
    {
        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
       && event_list[ceiling_floor_index_list.front()].event_type==CEILING
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
}

void CPP_BCD::AllocateWallEventType(const cv::Mat& map, std::vector<Event>& event_list)
{
    int index_offset;
    std::deque<int> in_out_index_list; // 只存放各种in和out的index

    int N = event_list.size();

    // determine in and out and middle
    for(int i = 0; i < N; i++)
    {
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = IN_EX;
            in_out_index_list.emplace_back(i);
        }
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = MIDDLE;
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = OUT_EX;
            in_out_index_list.emplace_back(i);
        }


        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }
    }

    // determine inner
    Point2D neighbor_point;
    int temp_index;
    for(auto in_out_index : in_out_index_list)
    {
        if(event_list[in_out_index].event_type == OUT_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_EX;
            }
        }

        if(event_list[in_out_index].event_type == OUT_TOP_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_TOP_EX;
            }
        }

        if(event_list[in_out_index].event_type == OUT_BOTTOM_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_BOTTOM_EX;
            }

        }

        if(event_list[in_out_index].event_type == IN_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_EX;
            }
        }


        if(event_list[in_out_index].event_type == IN_TOP_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_TOP_EX;
            }
        }

        if(event_list[in_out_index].event_type == IN_BOTTOM_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_BOTTOM_EX;
            }
        }
    }

    // determine floor and ceiling
    std::deque<int> ceiling_floor_index_list;

    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(
                (event_list[in_out_index_list[0]].event_type==OUT_EX
                 ||event_list[in_out_index_list[0]].event_type==OUT_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM_EX)
                &&
                (event_list[in_out_index_list[1]].event_type==IN_EX
                 ||event_list[in_out_index_list[1]].event_type==IN_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM_EX)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        if(
                (event_list[in_out_index_list[0]].event_type==IN_EX
                 ||event_list[in_out_index_list[0]].event_type==IN_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM_EX)
                &&
                (event_list[in_out_index_list[1]].event_type==OUT_EX
                 ||event_list[in_out_index_list[1]].event_type==OUT_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM_EX)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }


    // filter ceiling and floor
    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
    {
        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
       && event_list[ceiling_floor_index_list.front()].event_type==CEILING
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
}


std::vector<Event> CPP_BCD::InitializeEventList(const Polygon& polygon, int polygon_index)
{
    std::vector<Event> event_list;

    for(const auto& point : polygon)
    {
        event_list.emplace_back(Event(polygon_index, point.x, point.y));
    }

    return event_list;
}


std::vector<Event> CPP_BCD::GenerateObstacleEventList(const cv::Mat& map, const PolygonList& polygons)
{
    std::vector<Event> event_list;
    std::vector<Event> event_sublist;

    for(int i = 0; i < polygons.size(); i++)
    {
        event_sublist = InitializeEventList(polygons[i], i);
        AllocateObstacleEventType(map, event_sublist);
        event_list.insert(event_list.end(), event_sublist.begin(), event_sublist.end());
        event_sublist.clear();
    }

    std::sort(event_list.begin(), event_list.end());

    return event_list;
}

std::vector<Event> CPP_BCD::GenerateWallEventList(const cv::Mat& map, const Polygon& external_contour)
{
    std::vector<Event> event_list;

    event_list = InitializeEventList(external_contour, INT_MAX);
    AllocateWallEventType(map, event_list);
    std::sort(event_list.begin(), event_list.end());

    return event_list;
}


std::deque<std::deque<Event>> CPP_BCD::SliceListGenerator(const std::vector<Event>& wall_event_list, const std::vector<Event>& obstacle_event_list)
{
    std::vector<Event> event_list;
    event_list.insert(event_list.end(), obstacle_event_list.begin(), obstacle_event_list.end());
    event_list.insert(event_list.end(), wall_event_list.begin(), wall_event_list.end());
    std::sort(event_list.begin(), event_list.end());

    std::deque<std::deque<Event>> slice_list;
    std::deque<Event> slice;
    int x = event_list.front().x;

    for(auto event : event_list)
    {
        if(event.x != x)
        {
            slice_list.emplace_back(slice);

            x = event.x;
            slice.clear();
            slice.emplace_back(event);
        }
        else
        {
            slice.emplace_back(event);
        }
    }
    slice_list.emplace_back(slice);

    return slice_list;
}

void CPP_BCD::ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite = false)
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in);

    bottom_cell.ceiling.emplace_back(in);
    bottom_cell.floor.emplace_back(f);

    if(!rewrite)
    {
        int top_cell_index = cell_graph.size();
        int bottom_cell_index = cell_graph.size() + 1;

        top_cell.cellIndex = top_cell_index;
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(top_cell);
        cell_graph.emplace_back(bottom_cell);


        cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
    }
    else
    {
        cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
        cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

        int bottom_cell_index = cell_graph.size();
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(bottom_cell);

        cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());

    }
}

void CPP_BCD::ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite = false)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(c);
    new_cell.floor.emplace_back(f);

    if(!rewrite)
    {
        int new_cell_idx = cell_graph.size();
        new_cell.cellIndex = new_cell_idx;

        cell_graph.emplace_back(new_cell);


        cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
        cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

        cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
        cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);
    }
    else
    {
        cell_graph[top_cell_idx].ceiling.assign(new_cell.ceiling.begin(), new_cell.ceiling.end());
        cell_graph[top_cell_idx].floor.assign(new_cell.floor.begin(), new_cell.floor.end());

        cell_graph[top_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);
        cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    }

}

void CPP_BCD::ExecuteCeilOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& ceil_point)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void CPP_BCD::ExecuteFloorOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& floor_point)
{
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}

void CPP_BCD::ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite = false)
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in_top);

    bottom_cell.ceiling.emplace_back(in_bottom);
    bottom_cell.floor.emplace_back(f);


    if(!rewrite)
    {
        int top_cell_index = cell_graph.size();
        int bottom_cell_index = cell_graph.size() + 1;

        top_cell.cellIndex = top_cell_index;
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(top_cell);
        cell_graph.emplace_back(bottom_cell);


        cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
    }
    else
    {
        cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
        cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

        int bottom_cell_index = cell_graph.size();
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(bottom_cell);

        cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());
    }

}

void CPP_BCD::ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(inner_in);
    new_cell.floor.emplace_back(inner_in);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void CPP_BCD::ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in_top, Point2D inner_in_bottom)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(inner_in_top);
    new_cell.floor.emplace_back(inner_in_bottom);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void CPP_BCD::ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out);
}

void CPP_BCD::ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out_top);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out_bottom);
}

void CPP_BCD::DrawCells(cv::Mat& map, const CellNode& cell, cv::Scalar color=cv::Scalar(100, 100, 100))
{

    for(const auto& ceiling_point : cell.ceiling)
    {
        map.at<cv::Vec3b>(ceiling_point.y, ceiling_point.x) = cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
    }

    for(const auto& floor_point : cell.floor)
    {
        map.at<cv::Vec3b>(floor_point.y, floor_point.x) = cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), color);
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), color);
}

int CPP_BCD::CountCells(const std::deque<Event>& slice, int curr_idx)
{
    int cell_num = 0;
    for(int i = 0; i < curr_idx; i++)
    {
        if(
                (slice[i].event_type==IN)
                || (slice[i].event_type==IN_TOP)
                || (slice[i].event_type==INNER_IN)
                || (slice[i].event_type==INNER_IN_BOTTOM)
                || (slice[i].event_type==FLOOR)
                || (slice[i].event_type==IN_BOTTOM_EX)
                || (slice[i].event_type==INNER_IN_EX)
                || (slice[i].event_type==INNER_IN_TOP_EX)
                )
        {
            cell_num++;
        }
    }
    return cell_num;
}

std::deque<Event> CPP_BCD::FilterSlice(const std::deque<Event>& slice)
{
    std::deque<Event> filtered_slice;

    for(auto event : slice)
    {
        if(event.event_type!=MIDDLE && event.event_type!=UNALLOCATED)
        {
            filtered_slice.emplace_back(event);
        }
    }
    return filtered_slice;
}

void CPP_BCD::ExecuteCellDecomposition(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, const std::deque<std::deque<Event>>& slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    Point2D c, f;
    int c_index = INT_MAX, f_index = INT_MAX;
    int min_dist = INT_MAX;

    int event_y = INT_MAX;

    bool rewrite = false;

    std::vector<int> sub_cell_index_slices;
    std::deque<Event> curr_slice;

    int cell_counter = 0;

    for(const auto& raw_slice : slice_list)
    {
        curr_slice = FilterSlice(raw_slice);

        original_cell_index_slice.assign(cell_index_slice.begin(), cell_index_slice.end());

        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == INNER_IN_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end(); // 若为true，则覆盖

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j].x, curr_slice[j].y),
                                             c,
                                             f,
                                             rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }
            if(curr_slice[j].event_type == INNER_OUT_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];

                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }


                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_IN_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                             Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                             c,
                                             f,
                                             rewrite);


                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                            cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                    sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_OUT_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == IN_EX)
            {
                event_y = curr_slice[j].y;

                if(!cell_index_slice.empty())
                {
                    for(int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {
                            ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                            cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                    if(event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                    }
                    if(event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                    }

                }
                else
                {
                    ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                    cell_index_slice.emplace_back(int(cell_graph.size()-1));
                    curr_slice[j].isUsed = true;
                }

            }

            if(curr_slice[j].event_type == IN_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;

                if(!cell_index_slice.empty())
                {
                    for(int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {

                            ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                      Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                            cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));

                            curr_slice[j-1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                    if(event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                  Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                    if(event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                  Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                }
                else
                {
                    ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                              Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                    cell_index_slice.emplace_back(int(cell_graph.size()-1));

                    curr_slice[j-1].isUsed = true;
                    curr_slice[j].isUsed = true;
                }

            }


            if(curr_slice[j].event_type == OUT_EX)
            {
                event_y = curr_slice[j].y;

                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == OUT_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;

                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

        }


        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == IN)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end(); // 若为true，则覆盖

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j].x, curr_slice[j].y),
                                             c,
                                             f,
                                             rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }
            if(curr_slice[j].event_type == OUT)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];

                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }


                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == IN_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                             Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                             c,
                                             f,
                                             rewrite);


                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                            cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                    sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == OUT_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_IN)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_IN_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                  Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_OUT)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_OUT_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

        }

        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == CEILING)
            {
                cell_counter = CountCells(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteCeilOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
            if(curr_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteFloorOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
        }
    }
}





std::vector<CellNode> CPP_BCD::ConstructCellGraph(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& wall_contours, const std::vector<std::vector<cv::Point>>& obstacle_contours, const Polygon& wall, const PolygonList& obstacles)
{
    cv::Mat3b map = cv::Mat3b(original_map.size());
    map.setTo(cv::Scalar(0, 0, 0));

    cv::fillPoly(map, wall_contours, cv::Scalar(255, 255, 255));
    cv::fillPoly(map, obstacle_contours, cv::Scalar(0, 0, 0));

    std::vector<Event> wall_event_list = GenerateWallEventList(map, wall);
    std::vector<Event> obstacle_event_list = GenerateObstacleEventList(map, obstacles);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);

    return cell_graph;
}


// ================================= coverage path planner =======================================

std::vector<int> CPP_BCD::DetermineCellIndex(std::vector<CellNode>& cell_graph, const Point2D& point)
{
    std::vector<int> cell_index;

    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].ceiling.size(); j++)
        {
            if(point.x ==  cell_graph[i].ceiling[j].x && point.y >= cell_graph[i].ceiling[j].y && point.y <= cell_graph[i].floor[j].y)
            {
                cell_index.emplace_back(int(i));
            }
        }

    }
    return cell_index;
}

void CPP_BCD::WalkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path)
{
    if(!cell_graph[cell_index].isVisited)
    {
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(cell_graph[cell_index]);

//    for debugging
//    std::cout<< "cell: " <<cell_graph[cell_index].cellIndex<<std::endl;


    CellNode neighbor;
    int neighbor_idx = INT_MAX;

    for(int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++)
    {
        neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
        neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
        if(!neighbor.isVisited)
        {
            break;
        }
    }

    if(!neighbor.isVisited) // unvisited neighbor found
    {
        cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
        WalkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
    }
    else  // unvisited neighbor not found
    {

        if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
        {
            return;
        }
        else if(unvisited_counter == 0)
        {
            return;
        }
        else
        {
            WalkThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
        }
    }
}


std::deque<CellNode> CPP_BCD::GetVisittingPath(std::vector<CellNode>& cell_graph, int first_cell_index)
{
    std::deque<CellNode> visitting_path;

    if(cell_graph.size()==1)
    {
        visitting_path.emplace_back(cell_graph.front());
    }
    else
    {
        int unvisited_counter = cell_graph.size();
        WalkThroughGraph(cell_graph, first_cell_index, unvisited_counter, visitting_path);
        std::reverse(visitting_path.begin(), visitting_path.end());
    }

    return visitting_path;
}

void CPP_BCD::InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times)
{
    for(int i = 0; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(0, i, 255));
        }
    }

    for(int i = 254; i >= 0; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(0, 255, i));
        }
    }

    for(int i = 1; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(i, 255, 0));
        }
    }

    for(int i = 254; i >= 0; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(255, i, 0));
        }
    }

    for(int i = 1; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(255, 0, i));
        }
    }

    for(int i = 254; i >= 1; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(i, 0, 255));
        }
    }
}

std::vector<Point2D> CPP_BCD::ComputeCellCornerPoints(const CellNode& cell)
{

    Point2D topleft = cell.ceiling.front();
    Point2D bottomleft = cell.floor.front();
    Point2D bottomright = cell.floor.back();
    Point2D topright = cell.ceiling.back();

    // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner points（逆时针）
    std::vector<Point2D> corner_points = {topleft, bottomleft, bottomright, topright};

    return corner_points;
}

void CPP_BCD::UpdateColorMap(std::deque<cv::Scalar>& JetColorMap)
{
    cv::Scalar color = JetColorMap.front();
    JetColorMap.pop_front();
    JetColorMap.emplace_back(color);
}

std::deque<Point2D> CPP_BCD::GetBoustrophedonPath(std::vector<CellNode>& cell_graph, CellNode cell, int corner_indicator, int robot_radius)
{
    int delta, increment;

    std::deque<Point2D> path;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

    std::vector<Point2D> ceiling, floor;
    ceiling.assign(cell.ceiling.begin(), cell.ceiling.end());
    floor.assign(cell.floor.begin(), cell.floor.end());

    if(cell_graph[cell.cellIndex].isCleaned)
    {
        if(corner_indicator == TOPLEFT)
        {
            path.emplace_back(corner_points[TOPLEFT]);
        }
        if(corner_indicator == TOPRIGHT)
        {
            path.emplace_back(corner_points[TOPRIGHT]);
        }
        if(corner_indicator == BOTTOMLEFT)
        {
            path.emplace_back(corner_points[BOTTOMLEFT]);
        }
        if(corner_indicator == BOTTOMRIGHT)
        {
            path.emplace_back(corner_points[BOTTOMRIGHT]);
        }
    }
    else
    {
        if(corner_indicator == TOPLEFT)
        {
            int x, y, y_start, y_end;
            bool reverse = false;

            for(int i = 0; i < ceiling.size(); i = i + (robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));
//                    for(y = y_start; y <= y_end; y++)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }

//                    if((std::abs(floor[i+1].y-floor[i].y)>=2)&&(i+1<floor.size()))
//                    {
//                        delta = floor[i+1].y-floor[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));
//                        }
//                    }

//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着floor从左往右
//                            if( x+j >= floor.back().x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
//
//                            //提前转  前一个像素减后一个像素
//                            else if((floor[i+(j)].y-floor[i+(j+1)].y>=2)
//                               &&(j<=robot_radius+1)
//                               &&(j+1<=robot_radius+1))
//                            {
//                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y+increment*(k)));
//                                }
//                            }
//                            //滞后转 后一个像素减前一个像素
//                            else if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
//                                    &&(j+1<=robot_radius+1)
//                                    &&(j<=robot_radius+1))
//                            {
//                                path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y));
//
//                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i+(j+1)].x, cell.floor[i+(j+1)].y-abs(delta) +increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(floor[i+(j)]);
//                            }
//
//                        }
//                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

//                    for (y = y_start; y >= y_end; y--)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(ceiling[i+1].y-ceiling[i].y)>=2)&&(i+1<ceiling.size()))
//                    {
//                        delta = ceiling[i+1].y-ceiling[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
//                        }
//                    }

//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着ceiling从左往右
//                            if(x+j >= ceiling.back().x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
//
//                            // 提前转
//                            else if((ceiling[i+(j+1)].y-ceiling[i+(j)].y>=2)
//                               &&(j+1 <= robot_radius+1)
//                               &&(j <= robot_radius+1))
//                            {
//                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i+j].x, ceiling[i+j].y+increment*(k)));
//                                }
//                            }
//                            // 滞后转
//                            else if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
//                                    &&(j<=robot_radius+1)
//                                    &&(j+1<=robot_radius+1))
//                            {
//                                path.emplace_back(ceiling[i+(j)]);
//
//                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i+(j+1)].x, ceiling[i+(j+1)].y+abs(delta)+increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(ceiling[i+j]);
//                            }
//
//                        }
//                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == TOPRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1; i >= 0; i=i-(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

//                    for(y = y_start; y <= y_end; y++)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(floor[i-1].y-floor[i].y)>=2)&&(i-1>=0))
//                    {
//                        delta = floor[i-1].y-floor[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
//                        }
//                    }

//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着floor从右往左
//                            if(x-j <= floor.front().x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
//                            //提前转
//                            else if((floor[i-(j)].y-floor[i-(j+1)].y>=2)
//                               &&(j<=robot_radius+1)
//                               &&(j+1<=robot_radius+1))
//                            {
//                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y+increment*(k)));
//                                }
//                            }
//                            //滞后转
//                            else if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
//                                    &&(j+1<=robot_radius+1)
//                                    &&(j<=robot_radius+1))
//                            {
//                                path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y));
//
//                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i-(j+1)].x, cell.floor[i-(j+1)].y-abs(delta) +increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(floor[i-(j)]);
//                            }
//                        }
//                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

//                    for (y = y_start; y >= y_end; y--)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(ceiling[i-1].y-ceiling[i].y)>=2)&&(i-1>=0))
//                    {
//                        delta = ceiling[i-1].y-ceiling[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
//                        }
//                    }
//
//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着ceiling从右往左
//                            if( x-j <= ceiling.front().x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
//                            // 提前转
//                            else if((ceiling[i-(j+1)].y-ceiling[i-(j)].y>=2)
//                               &&(j+1 <= robot_radius+1)
//                               &&(j <= robot_radius+1))
//                            {
//                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i-j].x, ceiling[i-j].y+increment*(k)));
//                                }
//                            }
//                            // 滞后转
//                            else if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
//                                    &&(j<=robot_radius+1)
//                                    &&(j+1<=robot_radius+1))
//                            {
//                                path.emplace_back(ceiling[i-(j)]);
//
//                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i-(j+1)].x, ceiling[i-(j+1)].y+abs(delta)+increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(ceiling[i-j]);
//                            }
//                        }
//                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMLEFT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = 0; i < ceiling.size(); i=i+(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

//                    for(y = y_start; y >= y_end; y--)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(ceiling[i+1].y-ceiling[i].y)>=2)&&(i+1<ceiling.size()))
//                    {
//                        delta = ceiling[i+1].y-ceiling[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
//                        }
//                    }
//
//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着ceiling从左往右
//                            if(x+j >= ceiling.back().x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
//                            // 提前转
//                            else if((ceiling[i+(j+1)].y-ceiling[i+(j)].y>=2)
//                               &&(j+1 <= robot_radius+1)
//                               &&(j <= robot_radius+1))
//                            {
//                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i+j].x, ceiling[i+j].y+increment*(k)));
//                                }
//                            }
//                                // 滞后转
//                            else if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
//                                    &&(j<=robot_radius+1)
//                                    &&(j+1<=robot_radius+1))
//                            {
//                                path.emplace_back(ceiling[i+(j)]);
//
//                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i+(j+1)].x, ceiling[i+(j+1)].y+abs(delta)+increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(ceiling[i+j]);
//                            }
//                        }
//                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

//                    for (y = y_start; y <= y_end; y++)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(floor[i+1].y-floor[i].y)>=2)&&(i+1<floor.size()))
//                    {
//                        delta = floor[i+1].y-floor[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
//                        }
//                    }
//
//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着floor从左往右
//                            if(x+j >= floor.back().x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
//
//                            //提前转
//                            else if((floor[i+(j)].y-floor[i+(j+1)].y>=2)
//                               &&(j<=robot_radius+1)
//                               &&(j+1<=robot_radius+1))
//                            {
//                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y+increment*(k)));
//                                }
//                            }
//                                //滞后转
//                            else if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
//                                    &&(j+1<=robot_radius+1)
//                                    &&(j<=robot_radius+1))
//                            {
//                                path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y));
//
//                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i+(j+1)].x, cell.floor[i+(j+1)].y-abs(delta) +increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(floor[i+(j)]);
//                            }
//                        }
//                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1; i >= 0; i=i-(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

//                    for(y = y_start; y >= y_end; y--)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(ceiling[i-1].y-ceiling[i].y)>=2)&&(i-1>=0))
//                    {
//                        delta = ceiling[i-1].y-ceiling[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
//                        }
//                    }
//
//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着ceiling从右往左
//                            if(x-j <= ceiling.front().x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
//                            // 提前转
//                            else if((ceiling[i-(j+1)].y-ceiling[i-(j)].y>=2)
//                               &&(j+1 <= robot_radius+1)
//                               &&(j <= robot_radius+1))
//                            {
//                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i-j].x, ceiling[i-j].y+increment*(k)));
//                                }
//                            }
//                                // 滞后转
//                            else if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
//                                    &&(j<=robot_radius+1)
//                                    &&(j+1<=robot_radius+1))
//                            {
//                                path.emplace_back(ceiling[i-(j)]);
//
//                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(ceiling[i-(j+1)].x, ceiling[i-(j+1)].y+abs(delta)+increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(ceiling[i-j]);
//                            }
//
//                        }
//                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

//                    for (y = y_start; y <= y_end; y++)
//                    {
//                        path.emplace_back(Point2D(x, y));
//                    }
                    path.emplace_back(Point2D(x, y_start));
                    path.emplace_back(Point2D(x, y_end));

//                    if((std::abs(floor[i-1].y-floor[i].y)>=2)&&(i-1>=0))
//                    {
//                        delta = floor[i-1].y-floor[i].y;
//                        increment = delta/abs(delta);
//                        for(int k = 1; k <= abs(delta); k++)
//                        {
//                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
//                        }
//                    }
//
//                    if(robot_radius != 0)
//                    {
//                        for(int j = 1; j <= robot_radius+1; j++)
//                        {
//                            // 沿着floor从右往左
//                            if(x-j <= floor.front().x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
//                            //提前转
//                            else if((floor[i-(j)].y-floor[i-(j+1)].y>=2)
//                               &&(j<=robot_radius+1)
//                               &&(j+1<=robot_radius+1))
//                            {
//                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y+increment*(k)));
//                                }
//                            }
//                                //滞后转
//                            else if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
//                                    &&(j+1<=robot_radius+1)
//                                    &&(j<=robot_radius+1))
//                            {
//                                path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y));
//
//                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
//
//                                increment = delta/abs(delta);
//                                for(int k = 0; k <= abs(delta); k++)
//                                {
//                                    path.emplace_back(Point2D(floor[i-(j+1)].x, cell.floor[i-(j+1)].y-abs(delta) +increment*(k)));
//                                }
//                            }
//                            else
//                            {
//                                path.emplace_back(floor[i-(j)]);
//                            }
//
//                        }
//                    }

                    reverse = !reverse;
                }
            }
        }
    }

    return path;
}

Point2D CPP_BCD::FindNextEntrance(const Point2D& curr_point, const CellNode& next_cell, int& corner_indicator)
{
    Point2D next_entrance;

    int front_x = next_cell.ceiling.front().x;
    int back_x = next_cell.ceiling.back().x;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(next_cell);

    if(abs(curr_point.x - front_x) < abs(curr_point.x - back_x))
    {
        if(abs(curr_point.y - next_cell.ceiling.front().y)<abs(curr_point.y - next_cell.floor.front().y))
        {
            next_entrance = corner_points[TOPLEFT];
            corner_indicator = TOPLEFT;
        }
        else
        {
            next_entrance = corner_points[BOTTOMLEFT];
            corner_indicator = BOTTOMLEFT;
        }
    }
    else
    {
        if(abs(curr_point.y - next_cell.ceiling.back().y)<abs(curr_point.y - next_cell.floor.back().y))
        {
            next_entrance = corner_points[TOPRIGHT];
            corner_indicator = TOPRIGHT;
        }
        else
        {
            next_entrance = corner_points[BOTTOMRIGHT];
            corner_indicator = BOTTOMRIGHT;
        }
    }

    return next_entrance;
}

std::deque<Point2D> CPP_BCD::WalkInsideCell(CellNode cell, const Point2D& start, const Point2D& end)
{
    std::deque<Point2D> inner_path = {start};

    int start_ceiling_index_offset = start.x - cell.ceiling.front().x;
    int first_ceiling_delta_y = cell.ceiling[start_ceiling_index_offset].y - start.y;
    int end_ceiling_index_offset = end.x - cell.ceiling.front().x;
    int second_ceiling_delta_y = end.y - cell.ceiling[end_ceiling_index_offset].y;

    int start_floor_index_offset = start.x - cell.floor.front().x;
    int first_floor_delta_y = cell.floor[start_floor_index_offset].y - start.y;
    int end_floor_index_offset = end.x - cell.floor.front().x;
    int second_floor_delta_y = end.y - cell.floor[end_floor_index_offset].y;

    if((abs(first_ceiling_delta_y)+abs(second_ceiling_delta_y)) < (abs(first_floor_delta_y)+abs(second_floor_delta_y))) //to ceiling
    {
        int first_increment_y = 0;
        if(first_ceiling_delta_y != 0)
        {
            first_increment_y = first_ceiling_delta_y / abs(first_ceiling_delta_y);

            for(int i = 1; i <= abs(first_ceiling_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
            }
        }

        int delta_x = cell.ceiling[end_ceiling_index_offset].x - cell.ceiling[start_ceiling_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 0; i < abs(delta_x); i++)
        {
            // 提前转
            if((cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y>=2)
               &&(i+1 <= abs(delta_x))
               &&(i <= abs(delta_x)))
            {
                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*i].x, cell.ceiling[start_ceiling_index_offset+increment_x*i].y+increment*(j)));
                }
            }
                // 滞后转
            else if((cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y>=2)
                    &&(i<=abs(delta_x))
                    &&(i+1<=abs(delta_x)))
            {
                inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset+increment_x*(i)]);

                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;

                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y+abs(delta)+increment*(k)));
                }
            }
            else
            {
                inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset+(increment_x*i)]);
            }
        }

        int second_increment_y = 0;
        if(second_ceiling_delta_y!=0)
        {
            second_increment_y = second_ceiling_delta_y/abs(second_ceiling_delta_y);

            for(int i = 1; i <= abs(second_ceiling_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(cell.ceiling[end_ceiling_index_offset].x, cell.ceiling[end_ceiling_index_offset].y+(second_increment_y*i)));
            }
        }

    }
    else // to floor
    {
        int first_increment_y = 0;
        if(first_floor_delta_y != 0)
        {
            first_increment_y = first_floor_delta_y / abs(first_floor_delta_y);

            for(int i = 1; i <= abs(first_floor_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
            }
        }

        int delta_x = cell.floor[end_floor_index_offset].x - cell.floor[start_floor_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 0; i < abs(delta_x); i++)
        {
            //提前转
            if((cell.floor[start_floor_index_offset+increment_x*(i)].y-cell.floor[start_floor_index_offset+increment_x*(i+1)].y>=2)
               &&(i<=abs(delta_x))
               &&(i+1<=abs(delta_x)))
            {
                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y+increment*(j)));
                }
            }
                //滞后转
            else if((cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y>=2)
                    &&(i+1<=abs(delta_x))
                    &&(i<=abs(delta_x)))
            {
                inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y));

                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;

                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i+1)].x, cell.floor[start_floor_index_offset+increment_x*(i+1)].y-abs(delta) +increment*(k)));
                }
            }
            else
            {
                inner_path.emplace_back(cell.floor[start_floor_index_offset+(increment_x*i)]);
            }

        }

        int second_increment_y = 0;
        if(second_floor_delta_y!=0)
        {
            second_increment_y = second_floor_delta_y/abs(second_floor_delta_y);

            for(int i = 1; i <= abs(second_floor_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(cell.floor[end_floor_index_offset].x, cell.floor[end_floor_index_offset].y+(second_increment_y*i)));
            }
        }
    }
    return inner_path;
}

std::deque<std::deque<Point2D>> CPP_BCD::FindLinkingPath(const Point2D& curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, const CellNode& next_cell)
{
    std::deque<std::deque<Point2D>> path;
    std::deque<Point2D> path_in_curr_cell;
    std::deque<Point2D> path_in_next_cell;

    int exit_corner_indicator = INT_MAX;
    Point2D exit = FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator);
    path_in_curr_cell = WalkInsideCell(curr_cell, curr_exit, exit);

    next_entrance = FindNextEntrance(exit, next_cell, corner_indicator);

    int delta_x = next_entrance.x - exit.x;
    int delta_y = next_entrance.y - exit.y;

    int increment_x = 0;
    int increment_y = 0;

    if (delta_x != 0) {
        increment_x = delta_x / std::abs(delta_x);
    }
    if (delta_y != 0) {
        increment_y = delta_y / std::abs(delta_y);
    }

    int upper_bound = INT_MIN;
    int lower_bound = INT_MAX;

    if (exit.x >= curr_cell.ceiling.back().x)
    {
        upper_bound = curr_cell.ceiling.back().y;
        lower_bound = curr_cell.floor.back().y;
    }
    if (exit.x <= curr_cell.ceiling.front().x)
    {
        upper_bound = curr_cell.ceiling.front().y;
        lower_bound = curr_cell.floor.front().y;
    }

    if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound))
    {
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path_in_curr_cell.emplace_back(Point2D(exit.x, y));
        }
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path_in_curr_cell.emplace_back(Point2D(x, next_entrance.y));
        }
    }
    else
    {
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path_in_curr_cell.emplace_back(Point2D(x, exit.y));
        }
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path_in_next_cell.emplace_back(Point2D(next_entrance.x, y));
        }
    }

    path = {path_in_curr_cell, path_in_next_cell};

    return path;
}

std::deque<Point2D> CPP_BCD::WalkCrossCells(std::vector<CellNode>& cell_graph, std::deque<int> cell_path, const Point2D& start, const Point2D& end, int robot_radius)
{
    std::deque<Point2D> overall_path;
    std::deque<Point2D> sub_path;

    std::deque<std::deque<Point2D>> link_path;

    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());

    for(auto cell : cells)
    {
        cell.isCleaned = true;
    }

    Point2D curr_exit, next_entrance;
    int curr_corner_indicator, next_corner_indicator;

    next_entrance = FindNextEntrance(start, cells[cell_path[1]], next_corner_indicator);
    curr_exit = FindNextEntrance(next_entrance, cells[cell_path[0]], curr_corner_indicator);
    sub_path = WalkInsideCell(cells[cell_path[0]], start, curr_exit);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[0]], cells[cell_path[1]]);
    sub_path.insert(sub_path.end(), link_path.front().begin(), link_path.front().end());
    sub_path.insert(sub_path.end(), link_path.back().begin(), link_path.back().end());


    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    curr_corner_indicator = next_corner_indicator;


    for(int i = 1; i < cell_path.size()-1; i++)
    {
        sub_path = GetBoustrophedonPath(cell_graph, cells[cell_path[i]], curr_corner_indicator, robot_radius);
        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_exit = overall_path.back();
        next_entrance = FindNextEntrance(curr_exit, cells[cell_path[i+1]], next_corner_indicator);

        link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[i]], cells[cell_path[i+1]]);
        sub_path.insert(sub_path.end(), link_path.front().begin(), link_path.front().end());
        sub_path.insert(sub_path.end(), link_path.back().begin(), link_path.back().end());


        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_corner_indicator = next_corner_indicator;
    }

    sub_path = WalkInsideCell(cells[cell_path.back()], next_entrance, end);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    return overall_path;
}



std::deque<std::deque<Point2D>> CPP_BCD::StaticPathPlanning(const cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& start_point, int robot_radius, bool visualize_cells, bool visualize_path, int color_repeats=10)
{
    cv::Mat3b vis_map;
    cv::cvtColor(map, vis_map, cv::COLOR_GRAY2BGR);

    std::deque<std::deque<Point2D>> global_path;
    std::deque<Point2D> local_path;
    int corner_indicator = TOPLEFT;

    if(DetermineCellIndex(cell_graph, start_point).empty())
    {
        std::cout << "start is not safe" << std::endl;
        return global_path;
    }

    int start_cell_index = DetermineCellIndex(cell_graph, start_point).front();

    std::deque<Point2D> init_path = WalkInsideCell(cell_graph[start_cell_index], start_point, ComputeCellCornerPoints(cell_graph[start_cell_index])[TOPLEFT]);
    local_path.assign(init_path.begin(), init_path.end());

    std::deque<CellNode> cell_path = GetVisittingPath(cell_graph, start_cell_index);  //深度优先遍历


    if(visualize_cells)
    {
//        std::cout<<"cell graph has "<<cell_graph.size()<<" cells."<<std::endl;
//        for(int i = 0; i < cell_graph.size(); i++)
//        {
//            for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
//            {
//                std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
//            }
//        }

        for(const auto& cell : cell_graph)
        {
            DrawCells(vis_map, cell);
        }
        cv::imwrite(map_path_ + "/decompse_map.png", vis_map);
    }


    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, color_repeats);

    if(visualize_path)
    {
        cv::circle(vis_map, cv::Point(start_point.x, start_point.y), 1, cv::Scalar(0, 0, 255), -1);
        for(const auto& point : init_path)
        {
            vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
            UpdateColorMap(JetColorMap);

        }
    }

    std::deque<Point2D> inner_path;
    std::deque<std::deque<Point2D>> link_path;
    Point2D curr_exit;
    Point2D next_entrance;

    std::deque<int> return_cell_path;
    std::deque<Point2D> return_path;

    for(int i = 0; i < cell_path.size(); i++)
    {
        inner_path = GetBoustrophedonPath(cell_graph, cell_path[i], corner_indicator, robot_radius);
        local_path.insert(local_path.end(), inner_path.begin(), inner_path.end());
        if(visualize_path)
        {
            for(const auto& point : inner_path)
            {
                vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                UpdateColorMap(JetColorMap);
//                cv::imshow("map", vis_map);
//                cv::waitKey(1);
            }
        }

        cell_graph[cell_path[i].cellIndex].isCleaned = true;

        if(i < (cell_path.size()-1))
        {
            curr_exit = inner_path.back();
            next_entrance = FindNextEntrance(curr_exit, cell_path[i+1], corner_indicator);
//            link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator, cell_path[i], cell_path[i+1]);
            link_path = {{curr_exit}, {next_entrance}};
            // for debugging
//            std::cout<<std::endl;
//            for(int i = 0; i < link_path.front().size(); i++)
//            {
//                int idx = DetermineCellIndex(cell_graph, link_path.front()[i]).front();
//                std::cout<<"point lies in curr cell "<<idx<<std::endl;
//            }
//
//            for(int i = 0; i < link_path.back().size(); i++)
//            {
//                int idx = DetermineCellIndex(cell_graph, link_path.back()[i]).front();
//                std::cout<<"point lies in next cell "<<idx<<std::endl;
//            }
//            std::cout<<std::endl;


            local_path.insert(local_path.end(), link_path.front().begin(), link_path.front().end());
            global_path.emplace_back(local_path);
            local_path.clear();
            local_path.insert(local_path.end(), link_path.back().begin(), link_path.back().end());


            if(visualize_path)
            {
                for(const auto& point : link_path.front())
                {
//                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(255, 255, 255);
                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
//                    cv::imshow("map", vis_map);
//                    cv::waitKey(1);
                }

                for(const auto& point: link_path.back())
                {
//                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(255, 255, 255);
                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
//                    cv::imshow("map", vis_map);
//                    cv::waitKey(1);
                }

            }
        }
    }

    global_path.emplace_back(local_path);

//    if(visualize_cells||visualize_path)
//    {
//        cv::waitKey(0);
//    }

    return global_path;
}

std::deque<Point2D> CPP_BCD::path_transform(std::deque<Point2D> path, cv::Mat1b map_transform, cv::Mat1b map_original, double angle)
{
    Point2D point_rat;
    std::deque<Point2D> path_rat;
    cv::Point2f center_rat( (float)(map_transform.cols/2) , (float) (map_transform.rows/2));
    float a = (angle/CV_PI)*180;
    cv::Mat rat_matrix = getRotationMatrix2D( center_rat, a, 1.0 );
    for(Point2D point : path)
    {
        point_rat.x = rat_matrix.at<double>(0,0) * point.x + rat_matrix.at<double>(0,1) * point.y + rat_matrix.at<double>(0,2) - (map_transform.cols/2 - map_original.cols/2);
        point_rat.y = rat_matrix.at<double>(1,0) * point.x + rat_matrix.at<double>(1,1) * point.y + rat_matrix.at<double>(1,2) - (map_transform.rows/2 - map_original.rows/2);
        path_rat.push_back(point_rat);
    }
    return path_rat;
}

void CPP_BCD::filter_path(std::deque<std::deque<Point2D>>& path, std::deque<Point2D>& path_filter)
{
    std::deque<Point2D> path_deque;
    for(int i=0; i<path.size(); i++)
    {
        for(int j=0; j<path[i].size(); j++)
        {
            path_deque.push_back(path[i][j]);
        }
    }

    for(int i=0; i<path_deque.size(); i++)
    {
        if(i==0)
            path_filter.push_back(path_deque[i]);
        else {
            if (path_deque[i] != path_deque[i - 1])
                path_filter.push_back(path_deque[i]);
        }
    }

}

std::vector<cv::Point> CPP_BCD::Plan()
{
    // Parameter configuration
    float robot_radius = robot_size_;
//    double angle = std::atan2(61, 61);
//    float a = (angle/CV_PI)*180;
    map_ = PreprocessMap(map_);

    //transform map
    cv::Mat1b map_transform;
    map_transform = map_;
    rotate_arbitrarily_angle(map_,map_transform,0);  //-a

    //find contours
    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map_transform, wall_contours, obstacle_contours, std::floor((robot_size_)/3));
    Polygon wall = ConstructWall(map_transform, wall_contours.front());
    PolygonList obstacles = ConstructObstacles(map_transform, obstacle_contours);

    //decomposite map
    std::vector<CellNode> cell_graph = ConstructCellGraph(map_transform, wall_contours, obstacle_contours, wall, obstacles);

    //set start point
    Point2D start = cell_graph[0].ceiling.front(); //todo

    //coverage path planning
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map_transform, cell_graph, start,
                                                                                std::floor(robot_radius*(1-coincidence_rate_)), true, true);
    if(original_planning_path.empty())
        return path_cv_;
    std::deque<Point2D> path_filter;
    filter_path(original_planning_path, path_);
    if(path_.empty())
        return path_cv_;
    //transform path
    std::deque<Point2D> path_rat = path_transform(path_filter, map_transform, map_, 0); //

    for(int i=0; i<path_.size()-1; i++)
    {
        cv::Point p;
        p.x = path_[i].x;
        p.y = path_[i].y;
        path_cv_.push_back(p);
    }
    return  path_cv_;

}




bool CPP_BCD::isAccessible(cv::Mat &cspace, cv::Point p){

    if(cspace.at<uchar>(p) == 0)
    {
        return true;
    }
    return false;
}

bool CPP_BCD::canConnect(cv::Mat cspace, cv::Point start, cv::Point end)
{

    cv::LineIterator line(cspace, start, end);

    for(int i = 0; i < line.count; i++, line++){
        if(!isAccessible(cspace, line.pos())){
            return false;
        }
    }

    return true;
}

void CPP_BCD::find_xy_area(std::vector<cv::Point> polygon, std::vector<double>& xy_area)
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

bool CPP_BCD::find_sweeping_area(cv::Mat map, std::vector<cv::Point>& contours_vertex)
{
    std::vector< std::vector< cv::Point> > contours;
//    cv::imshow("subregion", subregion);
//    cvWaitKey(0);
    cv::findContours(map,contours,cv::noArray(),cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
//    cv::drawContours(subregion, contours, -1, cv::Scalar::all(127));
    if(contours.size() == 0)
    {
        return false;
    }

    std::vector<double> counts_area;
    for(int i=0; i<contours.size(); i++)
    {
        counts_area.push_back(contourArea(contours[i], false));
    }
    auto maxPosition = std::max_element(counts_area.begin(), counts_area.end());

    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    cv::approxPolyDP(contours[maxPosition - counts_area.begin()], contours_poly[0], 5, true);

    return true;
}

std::vector<cv::Point> CPP_BCD::PrmLinkPlan(cv::Mat prm_link_map, std::vector<cv::Point> coverage_area)
{

    std::vector<double> xy_area;
    find_xy_area(coverage_area, xy_area);

    std::vector<cv::Point> CPP_prm_path_cv;
    CPP_prm_path_cv.push_back(path_cv_[0]);
    int num=1;
    for(int i=0; i<path_cv_.size()-1; i++)
    {
        if(!canConnect(prm_link_map, path_cv_[i], path_cv_[i+1]))
        {
            num++;
            PRMPlanner prm_planner(30, 0, xy_area[0]-50, xy_area[1]+50,
                                   xy_area[2]-50, xy_area[3]+50);
            prm_planner.initialize(prm_link_map);
            prm_planner.plan(path_cv_[i].x, path_cv_[i].y, path_cv_[i+1].x, path_cv_[i+1].y);
            std::vector<TGlobalOrd> prm_path = prm_planner.getpath();
            cv::Point prm_point_cv;
            for(auto point : prm_path)
            {
                prm_point_cv.x = point.x;
                prm_point_cv.y = point.y;
                CPP_prm_path_cv.push_back(prm_point_cv);
            }
        }
        else
        {
            CPP_prm_path_cv.push_back(path_cv_[i+1]);
        }
    }
    std::cout << "can not connect path has : " << num << std::endl;
    CPP_prm_path_cv.push_back(path_cv_.back());
    return CPP_prm_path_cv;
}

void CPP_BCD::VisualizePath(std::vector<cv::Point> path_link, cv::Mat coverage_map)
{
    cv::Mat map_original_rgb1, map_original_rgb2;
    cv::cvtColor(coverage_map, map_original_rgb1, CV_GRAY2BGR);
    cv::cvtColor(coverage_map, map_original_rgb2, CV_GRAY2BGR);

    for(int i=0; i<path_cv_.size()-1; i++)
    {
        cv::line(map_original_rgb1, path_cv_[i], path_cv_[i+1], cv::Scalar(255, 0, 0));
    }
    cv::imwrite(map_path_ + "/cpp_path_ulink.png", map_original_rgb1);

    for(int i=0; i<path_link.size()-1; i++)
    {
        cv::line(map_original_rgb2, path_link[i], path_link[i+1], cv::Scalar(255, 0, 0));
    }
    cv::imwrite(map_path_ + "/cpp_path.png", map_original_rgb2);
//    cv::imshow("1111", map_original_rgb2);
//    cv::waitKey(0);
}