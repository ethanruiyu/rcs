//
// Created by tuqirui on 21-1-12.
//

#include "CoveragePathPlanner/set_start_end.h"

//判断点在线段上
bool IsPointOnLine(double px0, double py0, double px1, double py1, double px2, double py2)
{
    bool flag = false;
    double d1 = (px1 - px0) * (py2 - py0) - (px2 - px0) * (py1 - py0);
    if ((abs(d1) < EPSILON) && ((px0 - px1) * (px0 - px2) <= 0) && ((py0 - py1) * (py0 - py2) <= 0))
    {
        flag = true;
    }
    return flag;
}

//判断两线段相交
bool IsIntersect(double px1, double py1, double px2, double py2, double px3, double py3, double px4, double py4)
{
    bool flag = false;
    double d = (px2 - px1) * (py4 - py3) - (py2 - py1) * (px4 - px3);
    if (d != 0)
    {
        double r = ((py1 - py3) * (px4 - px3) - (px1 - px3) * (py4 - py3)) / d;
        double s = ((py1 - py3) * (px2 - px1) - (px1 - px3) * (py2 - py1)) / d;
        if ((r >= 0) && (r <= 1) && (s >= 0) && (s <= 1))
        {
            flag = true;
        }
    }
    return flag;
}

//判断点在多边形内
bool Point_In_Polygon_2D(double x, double y, std::vector<cv::Point> POL)
{
    POL.push_back(POL[0]);
    bool isInside = false;
    int count = 0;

    //
    int minX = INT_MAX;
    for (int i = 0; i < POL.size(); i++)
    {
        minX = std::min(minX, POL[i].x);
    }

    //
    double px = x;
    double py = y;
    double linePoint1x = x;
    double linePoint1y = y;
    double linePoint2x = minX -10;			//取最小的X值还小的值作为射线的终点
    double linePoint2y = y;

    //遍历每一条边
    for (int i = 0; i < POL.size() - 1; i++)
    {
        double cx1 = POL[i].x;
        double cy1 = POL[i].y;
        double cx2 = POL[i + 1].x;
        double cy2 = POL[i + 1].y;

        if (IsPointOnLine(px, py, cx1, cy1, cx2, cy2))
        {
            return true;
        }

        if (fabs(cy2 - cy1) < EPSILON)   //平行则不相交
        {
            continue;
        }

        if (IsPointOnLine(cx1, cy1, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
        {
            if (cy1 > cy2)			//只保证上端点+1
            {
                count++;
            }
        }
        else if (IsPointOnLine(cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
        {
            if (cy2 > cy1)			//只保证上端点+1
            {
                count++;
            }
        }
        else if (IsIntersect(cx1, cy1, cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))   //已经排除平行的情况
        {
            count++;
        }
    }

    if (count % 2 == 1)
    {
        isInside = true;
    }

    return isInside;
}

//点到直线的距离
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB)
{
    //求直线方程
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;
    // 代入点到直线距离公式
    float distance = 0;
    distance = ((float)abs(A*pointP.x + B*pointP.y + C)) / ((float)sqrtf(A*A + B*B)); return distance;
}

double cal_distance(cv::Point point1, cv::Point point2)
{
    int delta_x = point2.x - point1.x;
    int delta_y = point2.y - point1.y;
    double l = std::sqrt(std::pow(delta_y,2) + std::pow(delta_x, 2));
    return l;
}

float cal_point_to_line(cv::Point pointP, cv::Point pointA, cv::Point pointB)
{
    float dis = 0.f;

    float dx = pointB.x - pointA.x;
    float dy = pointB.y - pointA.y;

    // 两直线垂直，向量表示法，转换后公示
    float k = -((pointA.x - pointP.x)*dx + (pointA.y - pointP.y)*dy) / ( dx*dx + dy*dy);
    float footX = k*dx + pointA.x;
    float footY = k*dy + pointA.y;

    //if垂足是否落在线段上
    if( footY >= std::min(pointA.y, pointB.y) && footY <=std::max(pointA.y, pointB.y)
        && footX >= std::min(pointA.x, pointB.x) && footX <=std::max(pointA.x, pointB.x ) )
    {
        dis = sqrtf((footX-pointP.x)*(footX-pointP.x) + (footY-pointP.y)*(footY-pointP.y));
    }
    else
    {
        float dis1 = sqrtf((pointA.x-pointP.x)*(pointA.x-pointP.x) + (pointA.y-pointP.y)*(pointA.y-pointP.y));
        float dis2 = sqrtf((pointB.x-pointP.x)*(pointB.x-pointP.x) + (pointB.y-pointP.y)*(pointB.y-pointP.y));

        dis = ( dis1 < dis2 ? dis1 : dis2 );
    }

    return dis;
}

//点在多边形的区域位置
void find_point_position(std::vector<std::vector<cv::Point>> subregion_vertex_list, std::vector<std::vector<cv::Point>> path_cv, cv::Point point, int& subregion_idx, int& point_idx)
{

    for(int i=0; i<subregion_vertex_list.size(); i++)
    {
        if(Point_In_Polygon_2D(point.x, point.y, subregion_vertex_list[i]))
        {
            subregion_idx = i;
        }
    }
    double l_s = INT_MAX, l_l = INT_MAX;

    for(int j=0; j<path_cv[subregion_idx].size()-1; j++)
    {
        float distance_l = getDist_P2L(point, path_cv[subregion_idx][j], path_cv[subregion_idx][j+1]);

        if(distance_l < l_l)
        {
            l_l = distance_l;
            double distance_j = cal_distance(path_cv[subregion_idx][j], point);
            double distance_j1 = cal_distance(path_cv[subregion_idx][j+1], point);
            if(distance_j1 < distance_j)
                point_idx = j+1;
            else
                point_idx = j;
        }
    }
}

//获取起点到终点的路径
bool find_path_with_start_end(std::vector<std::vector<cv::Point>> path_cv, std::vector<std::vector<cv::Point>> subregion_vertex_list,
                              cv::Point path_start, cv::Point path_end, std::vector<std::vector<cv::Point>>& path_se)
{
    int start_subregion_idx = 0, end_subregion_idx = 0;
    int start_point_idx = 0, end_point_idx = 0;
    find_point_position(subregion_vertex_list, path_cv, path_start, start_subregion_idx, start_point_idx);
    find_point_position(subregion_vertex_list, path_cv, path_end, end_subregion_idx, end_point_idx);

    std::vector<cv::Point> path_se_start, path_se_end;
    path_se_start.push_back(path_start);
    path_se_end.push_back(path_end);
    path_se.push_back(path_se_start);

    std::vector<cv::Point> path_1, path_2;
    for (int k = 0; k < path_cv.size(); k++)
    {
        if (k == start_subregion_idx) {
            for (int i = start_point_idx; i < path_cv[k].size(); i++) {
                path_1.push_back(path_cv[k][i]);
            }
            path_se.push_back(path_1);
        }
        if (k > start_subregion_idx && k < end_subregion_idx) {
            path_se.push_back(path_cv[k]);
        }
        if (k == end_subregion_idx) {
            for (int i = 0; i < end_point_idx; i++) {
                path_2.push_back(path_cv[k][i]);
            }
            path_se.push_back(path_2);
        }

    }
    path_se.push_back(path_se_end);
    return true;
}