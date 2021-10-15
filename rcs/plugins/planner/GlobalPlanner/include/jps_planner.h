
#include<opencv2/opencv.hpp>
#include<iostream>



struct CalcJPSPt
{
    cv::Point pt; //OpenCV中点坐标
    int F, G, H; //F=G+H
    CalcJPSPt* parent; //parent的坐标，这里没有用指针，从而简化代码
    CalcJPSPt(cv::Point _pt) :pt(_pt), F(0), G(0), H(0), parent(NULL)  //变量初始化
    {
    }
    //获取行动方向
    //{1,0}=右  {-1,0}=左
    //{0,1}=下  {0,-1}=上
    //{1,-1}=右上  {1,1}=右下
    //{-1,-1}=左上  {-1,1}=左下
    cv::Point getDirection() {
        if (parent == NULL) return cv::Point(0, 0);
        //计算从当前点到父节点的行动方向
        int x = (pt.x - parent->pt.x) / std::max(abs(pt.x - parent->pt.x), 1);
        int y = (pt.y - parent->pt.y) / std::max(abs(pt.y - parent->pt.y), 1);
        return cv::Point(x, y);
    }
};

class JPSCalc
{
public:
    JPSCalc();
    ~JPSCalc();
    bool validState() const;
    void setMap(const cv::Mat& plan_map);
    void setStart(const cv::Point& start_point);
    void setGoal(const cv::Point& goal_point);
    //获取到路径
    bool isAccessible(const cv::Point& p);
    bool canConnect(cv::Mat &cspace, cv::Point start, cv::Point end);
    std::vector<cv::Point> optimisePath(cv::Mat &cspace, std::vector<cv::Point> path);
    bool firstplan(std::vector<cv::Point> &path);
    bool GetPath(std::vector<cv::Point>& path);

private:
    CalcJPSPt* findPath(CalcJPSPt& startPoint, CalcJPSPt& endPoint);
    //获取需要开始计算的起跳点
    std::vector<cv::Point> getNeighbourPoints(CalcJPSPt* point) const;
    //寻找跳跃点
    cv::Point checkJumpPoint(cv::Point targetpt, cv::Point prept);

    //判断开启/关闭列表中是否包含某点
    CalcJPSPt* isInList(const std::list<CalcJPSPt*>& list, const CalcJPSPt* point) const;
    //检测障碍点
    bool isInSites(const int x, int y) const;
    //从开启列表中返回最后节点值最小的节点
    CalcJPSPt* getLeastFpoint();

    bool b_start_set_, b_goal_set_, b_map_set_;
    cv::Mat plan_map_;
    cv::Point start_pose_;
    cv::Point goal_pose_;

    std::list<CalcJPSPt*> openList;  //开启列表
    std::list<CalcJPSPt*> closeList; //关闭列表

};

