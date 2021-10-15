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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

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

class NormalAstarPlanner
{
public:
    NormalAstarPlanner();
    ~NormalAstarPlanner();

    bool validState() const;
    bool isAccessible(const cv::Point& p);
    bool canConnect(cv::Mat &cspace, cv::Point start, cv::Point end);
    std::vector<cv::Point> optimisePath(cv::Mat &cspace, std::vector<cv::Point> path);
    void initializeNode();
    bool firstplan(std::vector<cv::Point> &path);
    bool makePlan(std::vector<cv::Point>& path);


    void setMap(cv::Mat map);
    void setStart(cv::Point initPose);
    void setGoal(cv::Point goalPose);


private:
    void resetCurrentMap();

    void resizeNode(int width, int height);
    bool isOutOfRange(int index_x, int index_y);

    bool isGoal(double x, double y, AstarNode goal_node);
    void setPath_goal(const AstarNode &goal);
    bool searchPath(AstarNode &start_node, AstarNode &goal_node);


private:

    bool b_map_set_;
    bool b_start_set_;
    bool b_goal_set_;
    bool node_initialized_;


    std::vector<std::vector<AstarNode> > nodes_;
    std::vector<UpdateNode> update_nodes;
    std::priority_queue<AstarNode, std::vector<AstarNode>, std::greater<AstarNode>> openlist_;

    // map
    cv::Mat plan_map_;
    // msg
    cv::Point start_pose_global_;
    cv::Point goal_pose_global_;
    // path
    std::vector<cv::Point> path_;
};

#endif //PROJECT_NORMAL_ASTAR_H
