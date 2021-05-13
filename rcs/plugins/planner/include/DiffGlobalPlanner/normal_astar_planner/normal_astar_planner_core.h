/*
 * Created by Dongxiao Wu on 12/22/18.
 * Contact with: wdxairforce@gmail.com
*/
#ifndef PROJECT_NORMAL_ASTAR_H
#define PROJECT_NORMAL_ASTAR_H

#include <iostream>
#include <queue>
#include <time.h>
#include <math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>


#include "normal_astar_planner/dubins/constants.h"
#include "normal_astar_planner/dubins/DubinsCurve.h"
#include "normal_astar_planner/dubins/helper.h"
#include "normal_astar_planner/dubins/node3d.h"


namespace normal_astar_planner
{
    enum STATUS
    {
        NONE,
        OPEN,
        CLOSED,
        OBS
    };

    struct AstarNode
    {
        int index_x, index_y;                   // Coordinate of each node
        STATUS status = STATUS::NONE;           // NONE, OPEN, CLOSED or OBS
        double gc = 0;                          // Actual cost
        double hc = 0;                          // heuristic cost
        double cost;
        AstarNode *parent = NULL;               // parent node
        bool operator>(const AstarNode &right) const
        {
            return cost > right.cost;
        }
    };

    struct CirclePoint
    {
        double x,y;
    };

    struct UpdateNode
    {
        int delta_x, delta_y;
        double step;
    };

    // cal heuristic estimated cost
    namespace astar
    {
        inline double calcDistance(double x1, double y1, double x2, double y2)
        {
            return std::hypot(x2 - x1, y2 - y1);
        }
    }

    class NormalAstarPlannerCore
    {
    public:
        NormalAstarPlannerCore();
        ~NormalAstarPlannerCore();

        void reset();
        bool validState() const;
//        void display_circle(nav_msgs::Path path);
        double Quaternion_to_euler(double x, double y, double z, double w);
        bool makePlan(std::vector<cv::Point>& path);

        // set data
        void setPruneSize(const double &prune_size);
        void setRobotSize(const double &robot_size);

        void setMap(cv::Mat map);
        void setStart(cv::Point initPose);
        void setGoal(cv::Point goalPose);

        void setDubins(bool is_conside_goal_pose, double turning_radius, int n);

        Node3D* dubinsShot(Node3D& start, const Node3D& goal, double robot_size, double turning_radius);
        void tracePath(const Node3D* node, std::vector<Node3D> &path);

    private:
        void resetCurrentMap();

        void setPath_current(const AstarNode &current_node);
        void resizeNode(int width, int height);
        bool isOutOfRange(int index_x, int index_y);
        bool detectCollision(int index_x, int index_y);

        bool isGoal(double x, double y, AstarNode goal_node);
        bool PtInPolygon (cv::Point p, std::vector<cv::Point> ptPolygon, int nCount);
        void setPath_goal(const AstarNode &goal);
        bool poseToIndex(cv::Point pose, int *index_x, int *index_y);
        bool searchPath(AstarNode &start_node, AstarNode &goal_node);


    private:
        double prune_size_;
        double robot_size_;
        bool b_map_set_;
        bool b_start_set_;
        bool b_goal_set_;
        bool node_initialized_;

        bool is_conside_goal_pose_ = false;
        double turning_radius_;
        int n_;

        std::vector<std::vector<AstarNode> > nodes_;
        std::vector<UpdateNode> update_nodes;
        std::priority_queue<AstarNode, std::vector<AstarNode>, std::greater<AstarNode>> openlist_;

        // map
        cv::Mat visualize_map_;
        // msg
        cv::Point start_pose_global_;
        cv::Point goal_pose_global_;
        // path
        std::vector<cv::Point> path_;
    };
}

#endif //PROJECT_NORMAL_ASTAR_H
