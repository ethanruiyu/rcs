//
// Created by zhihui on 2020/12/5.
//

#ifndef CPP_BCD_CPP_BCD_H
#define CPP_BCD_CPP_BCD_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <numeric>
#include "CoveragePathPlanner/prm_planner/prm_planner.h"

namespace CPP_BCD_Planner
{
    class Point2D
    {
    public:
        Point2D()
        {
            x = INT_MAX;
            y = INT_MAX;
        }
        Point2D(int x_pos, int y_pos)
        {
            x = x_pos;
            y = y_pos;
        }
        Point2D(const Point2D& point)
        {
            x = point.x;
            y = point.y;
        }
        int x;
        int y;
    };


    typedef std::vector<Point2D> Polygon;
    typedef std::vector<Polygon> PolygonList;
    typedef std::deque<Point2D> Edge;

    enum EventType
    {
        IN,
        IN_TOP,
        IN_BOTTOM,
        OUT,
        OUT_TOP,
        OUT_BOTTOM,
        INNER_IN,
        INNER_IN_TOP,
        INNER_IN_BOTTOM,
        INNER_OUT,
        INNER_OUT_TOP,
        INNER_OUT_BOTTOM,

        IN_EX,
        IN_TOP_EX,
        IN_BOTTOM_EX,
        OUT_EX,
        OUT_TOP_EX,
        OUT_BOTTOM_EX,
        INNER_IN_EX,
        INNER_IN_TOP_EX,
        INNER_IN_BOTTOM_EX,
        INNER_OUT_EX,
        INNER_OUT_TOP_EX,
        INNER_OUT_BOTTOM_EX,

        MIDDLE,
        CEILING,
        FLOOR,
        UNALLOCATED
    };

    enum VisualizationMode{PATH_MODE, ROBOT_MODE};

    const int TOPLEFT = 0;
    const int BOTTOMLEFT = 1;
    const int BOTTOMRIGHT = 2;
    const int TOPRIGHT = 3;

    const int palette_colors = 1530;

    class Event
    {
    public:
        Event(int obstacle_idx, int x_pos, int y_pos, EventType type=UNALLOCATED)
        {
            obstacle_index = obstacle_idx;
            x = x_pos;
            y = y_pos;
            event_type = type;
            original_index_in_slice = INT_MAX;
            isUsed = false;
        }

        int x;
        int y;
        int original_index_in_slice;
        int obstacle_index;
        EventType event_type;

        bool isUsed;
    };

    class CellNode
    {
    public:
        CellNode()
        {
            isVisited = false;
            isCleaned = false;
            parentIndex = INT_MAX;
            cellIndex = INT_MAX;
        }
        bool isVisited;
        bool isCleaned;
        Edge ceiling;
        Edge floor;

        int parentIndex;
        std::deque<int> neighbor_indices;

        int cellIndex;
    };

    inline bool operator<(const Point2D& p1, const Point2D& p2)
    {
        return (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y));
    }

    inline bool operator<(const Event& e1, const Event& e2)
    {
        return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y) || (e1.x == e2.x && e1.y == e2.y && e1.obstacle_index < e2.obstacle_index));
    }

    inline bool operator==(const Point2D& p1, const Point2D& p2)
    {
        return (p1.x==p2.x && p1.y==p2.y);
    }

    inline bool operator!=(const Point2D& p1, const Point2D& p2)
    {
        return !(p1==p2);
    }


    class CPP_BCD {
    public:
        CPP_BCD(cv::Mat map, float robot_size, float coincidence_rate, std::string map_path);

        //====================== map preprocess ==================
        cv::Mat1b PreprocessMap(const cv::Mat1b& original_map);
        int ComputeRobotRadius(const double& meters_per_pix, const double& robot_size_in_meters);
        void rotate_arbitrarily_angle(cv::Mat &src,cv::Mat &dst,float angle);

        //====================== extract contours  ==================
        void ExtractRawContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& raw_wall_contours, std::vector<std::vector<cv::Point>>& raw_obstacle_contours);
        void ExtractContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& wall_contours, std::vector<std::vector<cv::Point>>& obstacle_contours, int robot_radius);
        PolygonList ConstructObstacles(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& obstacle_contours);
        Polygon ConstructDefaultWall(const cv::Mat& original_map);
        Polygon ConstructWall(const cv::Mat& original_map, std::vector<cv::Point>& wall_contour);

        //=============================================  map decompose  ============================================
        void AllocateObstacleEventType(const cv::Mat& map, std::vector<Event>& event_list);
        void AllocateWallEventType(const cv::Mat& map, std::vector<Event>& event_list);
        std::vector<Event> InitializeEventList(const Polygon& polygon, int polygon_index);
        std::vector<Event> GenerateObstacleEventList(const cv::Mat& map, const PolygonList& polygons);
        std::vector<Event> GenerateWallEventList(const cv::Mat& map, const Polygon& external_contour);
        std::deque<std::deque<Event>> SliceListGenerator(const std::vector<Event>& wall_event_list, const std::vector<Event>& obstacle_event_list);
        void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite );
        void ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite );
        void ExecuteCeilOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& ceil_point);
        void ExecuteFloorOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& floor_point);
        void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite);
        void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in);
        void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in_top, Point2D inner_in_bottom);
        void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out);
        void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom);
        void DrawCells(cv::Mat& map, const CellNode& cell, cv::Scalar color);
        int CountCells(const std::deque<Event>& slice, int curr_idx);
        std::deque<Event> FilterSlice(const std::deque<Event>& slice);
        void ExecuteCellDecomposition(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, const std::deque<std::deque<Event>>& slice_list);
        std::vector<CellNode> ConstructCellGraph(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& wall_contours, const std::vector<std::vector<cv::Point>>& obstacle_contours, const Polygon& wall, const PolygonList& obstacles);

        // ================================= coverage path planner =======================================
        std::vector<int> DetermineCellIndex(std::vector<CellNode>& cell_graph, const Point2D& point);
        void WalkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path);
        std::deque<CellNode> GetVisittingPath(std::vector<CellNode>& cell_graph, int first_cell_index);
        void InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times);
        std::vector<Point2D> ComputeCellCornerPoints(const CellNode& cell);
        void UpdateColorMap(std::deque<cv::Scalar>& JetColorMap);
        std::deque<Point2D> GetBoustrophedonPath(std::vector<CellNode>& cell_graph, CellNode cell, int corner_indicator, int robot_radius);
        Point2D FindNextEntrance(const Point2D& curr_point, const CellNode& next_cell, int& corner_indicator);
        std::deque<Point2D> WalkInsideCell(CellNode cell, const Point2D& start, const Point2D& end);
        std::deque<std::deque<Point2D>> FindLinkingPath(const Point2D& curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, const CellNode& next_cell);
        std::deque<Point2D> WalkCrossCells(std::vector<CellNode>& cell_graph, std::deque<int> cell_path, const Point2D& start, const Point2D& end, int robot_radius);
        std::deque<std::deque<Point2D>> StaticPathPlanning(const cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& start_point, int robot_radius, bool visualize_cells, bool visualize_path, int color_repeats);
        std::deque<Point2D> path_transform(std::deque<Point2D> path, cv::Mat1b map_transform, cv::Mat1b map_original, double angle);
        void filter_path(std::deque<std::deque<Point2D>>& path, std::deque<Point2D>& path_filter);

        std::vector<cv::Point> Plan();
        bool isAccessible(cv::Mat &cspace, cv::Point p);
        bool canConnect(cv::Mat cspace, cv::Point start, cv::Point end);
        bool find_sweeping_area(cv::Mat map, std::vector<cv::Point>& contours_vertex);
        void find_xy_area(std::vector<cv::Point> polygon, std::vector<double>& xy_area);
        std::vector<cv::Point>  PrmLinkPlan(cv::Mat prm_link_map, std::vector<cv::Point> coverage_area);
        void VisualizePath(std::vector<cv::Point> path_link, cv::Mat coverage_map);

    private:
        cv::Mat map_;
        cv::Mat check_connect_map;
        float robot_size_;
        float coincidence_rate_;
        std::deque<Point2D> path_;
        std::vector<cv::Point> path_cv_;
        std::string map_path_;
    };


}

#endif //CPP_BCD_CPP_BCD_H
