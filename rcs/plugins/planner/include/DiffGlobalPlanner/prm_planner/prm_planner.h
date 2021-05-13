#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

//#include<ros/ros.h>
//#include<geometry_msgs/PoseStamped.h>
//#include<geometry_msgs/Point.h>
//#include<visualization_msgs/Marker.h>
//#include<sensor_msgs/image_encodings.h>
//#include<nav_msgs/Path.h>
#include<vector>
#include<chrono>
//#include<cv_bridge/cv_bridge.h>
//#include<image_transport/image_transport.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

//#include<planner.h>
#include<types.h>
#include<graph.h>

#include<random>

//: public Planner
//typedef nav_msgs::OccupancyGrid NavGridMap;

class PRMPlanner
{
 public:
  PRMPlanner()=default;
    explicit PRMPlanner(unsigned int density, float robot_size,double x_min, double x_max, double y_min, double y_max);
  ~PRMPlanner()=default;

  virtual bool initialize(cv::Mat &map);

  std::vector<TGlobalOrd> getpath(){return path_;};
  cv::Mat getmap(){return cspace_;};

  std::vector<TGlobalOrd> calc_circle_point(float x, float y, float r);

  bool plan_with_astar(double start_x, double start_y, double goal_x, double goal_y, std::vector<cv::Point>& astar_path);
  virtual bool plan(double start_x, double start_y, double goal_x, 
    double goal_y);

  /**virtual bool getPathROS(nav_msgs::Path &path, double resolution,
    geometry_msgs::Pose origin, 
    const geometry_msgs::PoseStamped &goal_pose);**/
  
  static const int FREE_SPACE_VALUE = 0;
  double robot_size ;

 
 private:
  bool ordinateAccessible(cv::Mat &cspace, TGlobalOrd ordinate);

  Vertex findOrAdd(TGlobalOrd ordinate);

  Vertex addOrdinate(TGlobalOrd ordinate);

  bool existsAsVertex(TGlobalOrd ord);

  Vertex nextVertexId();

  bool lookup(TGlobalOrd ord, Vertex &v);

  void embedNode(cv::Mat &cspace, Vertex node, unsigned int k, bool retry);

  std::vector<TGlobalOrd> getNeighbours(cv::Mat &cspace, Vertex node, 
    bool shouldConnect);

  cv::Point convertToCVPoint(TGlobalOrd ordinate);

  bool canConnect(cv::Mat &cspace, cv::Point start, cv::Point end);

  bool inMap(cv::Point p);

  bool isAccessible(cv::Mat &cspace, cv::Point p);

  bool violatingSpace(TGlobalOrd ord, double r);

  // define static function so that it can be used in lambda function
  static double distance(TGlobalOrd o1, TGlobalOrd o2);

  std::vector<TGlobalOrd> query(cv::Mat &cspace, TGlobalOrd start,
    TGlobalOrd goal);
  
  double freeConfigSpace(cv::Mat &cspace);

  std::vector<TGlobalOrd> optimisePath(cv::Mat &cspace,
    std::vector<TGlobalOrd> path);
  
  std::vector<TGlobalOrd> toOrdPath(std::vector<Vertex> path);

  void joinNetwork(cv::Mat &cspace, unsigned int k);

  std::vector<Vertex> prioritiseNodes();

  //bool initializeCSpace(cv::Mat &cspace, const NavGridMap &map);

  /**geometry_msgs::Quaternion getOrientationFromPoses(
    geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2);
  
  void indexToPose(double x, double y, geometry_msgs::PoseStamped &pose, 
    double resolution, geometry_msgs::Pose origin);**/

  bool sampling();

  int map_width_, map_height_;
  double resolution_;
  unsigned int density_;
  bool is_map_init_;
  std::vector<cv::Point> astar_path_;
  double x_min_ ; //= 0
  double x_max_ ; //= map_width_
  double y_min_ ; //= 0
  double y_max_ ; //= map_height_

  cv::Mat cspace_;
  std::map<Vertex, TGlobalOrd> network_;
  Vertex nextVertexId_;
  std::vector<TGlobalOrd> path_;
  Graph graph_;

//  image_transport::Publisher cspace_pub_;

  double ratio_;

  int num_pass_ = 0;
  int num_samples_ = 0;

};// class PRMPlanner



#endif// PRM_PLANNER_H
