#ifndef PLANNER_H
#define PLANNER_H

#include <cmath>
#include <vector>
#include <queue>
#include <set>
#include "environment.h"


struct QuadTreePoint{
    double centerX;
    double centerY;
    double gridSize;
    double cost;
    QuadTreeNode* node;

    QuadTreePoint(){}
    QuadTreePoint(QuadTreeNode* node):
        centerX(node->centerX),
        centerY(node->centerY),
        gridSize(node->gridSize),
        cost(0),
        node(node){}
    ~QuadTreePoint(){}
};

inline bool operator< (const QuadTreePoint& point1,
        const QuadTreePoint& point2)
{
    return point1.cost > point2.cost;
}

inline bool operator> (const QuadTreePoint& point1,
        const QuadTreePoint& point2)
{
    return point1.cost < point2.cost;
}

inline bool operator== (const QuadTreePoint& point1,
        const QuadTreePoint& point2)
{
    return (point1.centerX == point2.centerX) 
        && (point1.centerY == point2.centerY);
}

class Planner
{
    public:
    int openSetSize;
    int numberOfNodeExpand;

    /* Quadtree related*/
    std::priority_queue<QuadTreePoint> qOpenSet;
    int AStarQuadTree(Environment &env);
    double reconstructPathQuadTree(Environment &env,
            QuadTreeNode* node);
};


#endif /* PLANNER_H */




