#include<CoveragePathPlanner/prm_planner/prm_planner.h>



PRMPlanner::PRMPlanner(unsigned int density, float robot_size, double x_min, double x_max, double y_min, double y_max)
        : density_(density)
        , robot_size(robot_size)
        , is_map_init_(false)
        , graph_(Graph(density))
        , x_min_(x_min)
        , x_max_(x_max)
        , y_min_(y_min)
        , y_max_(y_max)
{
}
PRMPlanner::PRMPlanner(unsigned int density):graph_(Graph(density)){}

bool PRMPlanner::initialize(cv::Mat& map)
{
//  std::cout << "Initializing PRM planner" << std::endl;

  map_width_ = map.cols;
  map_height_ = map.rows;
  resolution_ = 0.05;

  if(map_width_ <= 0 || map_height_ <= 0)
  {
    std::cout << "The size of map can't be negative" << std::endl;
    return false;
  }

  cspace_ = map;
  /**if(!initializeCSpace(cspace_, map))
  {
    std::cout << "Failed to initialize map" << std::endl;
    return false;
  }**/
  
  graph_.clearContainer();

  is_map_init_ = true;

  // ros::NodeHandle private_nh("~/prm_planner");
  // image_transport::ImageTransport it(private_nh);
  // cspace_pub_ = it.advertise("cspace", 1);
  
  // sensor_msgs::ImagePtr cspace_msg = cv_bridge::CvImage(
  //   std_msgs::Header(), "bgr8", cspace_).toImageMsg();
  // cspace_pub_.publish(cspace_msg);

  //cv::namedWindow("cspace", CV_WINDOW_NORMAL);
  //cv::imshow("cspace", cspace_);
  //cv::waitKey(0);

  return true;
}

std::vector<TGlobalOrd> PRMPlanner::calc_circle_point(float x, float y, float r)
{
    TGlobalOrd point;
    std::vector<TGlobalOrd> point_list;
    for(float theta = 0; theta < 2*3.14 ;theta+=1.0)
    {
        point.x = x + r * cos(theta);
        point.y = y + r * sin(theta);
        point_list.push_back(point);
    }
    return point_list;
}


bool PRMPlanner:: plan_with_astar(double start_x, double start_y, double goal_x, double goal_y, std::vector<cv::Point>& astar_path)
{
//    std::cout << "PRM planner is planning..." << std::endl;

    if(!is_map_init_)
    {
        std::cout << "Planner is not initialized before planning" << std::endl;
        return false;
    }

    TGlobalOrd start = {start_x, start_y};
    TGlobalOrd goal = {goal_x, goal_y};

//    std::cout << "Get start and goal point" << std::endl;

    path_ = query(cspace_, start, goal);

    if(path_.size() > 0)
    {
//        std::cout << "Find a path in existing cspace" << std::endl;
        return true;
    }

    Vertex vStart, vGoal;


    density_ = 70;

    network_.clear();
    vStart = findOrAdd(start);
    vGoal = findOrAdd(goal);
    embedNode(cspace_, vStart, 1, true);
    embedNode(cspace_, vGoal, 1, true);

    for(int i=0; i<astar_path.size(); i++)
    {
        TGlobalOrd randomOrd;
        randomOrd.x = astar_path[i].x;
        randomOrd.y = astar_path[i].y;

        if(!isAccessible(cspace_, convertToCVPoint(randomOrd)))
        {
            continue; //Is not accessible in the ogmap, skip
        }

        addOrdinate(randomOrd);
    }


    joinNetwork(cspace_, density_);

    path_ = query(cspace_, start, goal);


    if(path_.size() <= 0)
    {
//        std::cout << "false : PRM planner failed to plan" << std::endl;
        return false;
    }
//    std::cout << "Find a new path in cspace" << std::endl;

    return true;
}


bool PRMPlanner:: plan(double start_x, double start_y, double goal_x, double goal_y) {
//    std::cout << "PRM planner is planning..." << std::endl;

    if (!is_map_init_) {
        std::cout << "Planner is not initialized before planning" << std::endl;
        return false;
    }

    TGlobalOrd start = {start_x, start_y};
    TGlobalOrd goal = {goal_x, goal_y};
    // check if the start or goal is already an existing point in graph
    if (!ordinateAccessible(cspace_, start)) {
        std::cout << "Start point is not safe" << std::endl;
        return false;
    }
    if (!ordinateAccessible(cspace_, goal)) {
        std::cout << "Goal point is not safe" << std::endl;
        return false;
    }
//    std::cout << "Get start and goal point" << std::endl;
    path_ = query(cspace_, start, goal);
    if (path_.size() > 0) {
//        std::cout << "Find a path in existing cspace" << std::endl;
        return true;
    }

    Vertex vStart, vGoal;
    unsigned int numNodes = 200;
    density_ = 20;
    while (path_.empty())
    {
        network_.clear();

        vStart = findOrAdd(start);
        vGoal = findOrAdd(goal);
        embedNode(cspace_, vStart, 1, true);
        embedNode(cspace_, vGoal, 1, true);

        double freeSpace = freeConfigSpace(cspace_);
        double r = (1.0 / (double) numNodes) * std::sqrt(
                (freeSpace * (numNodes - std::pow(numNodes, 0.5))) / M_PI);


        while (network_.size() < numNodes) {
            TGlobalOrd randomOrd;
            std::default_random_engine generator(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count());

            std::uniform_real_distribution<double> xDist(x_min_, x_max_); //map_width_
            std::uniform_real_distribution<double> yDist(y_min_, y_max_); //map_height_

            randomOrd.x = std::round((xDist(generator) * 10.0)) / 10.0;
            randomOrd.y = std::round((yDist(generator) * 10.0)) / 10.0;

            if(existsAsVertex(randomOrd))
            {
                continue; //Already exists in graph, skip
            }

            if(!inMap(convertToCVPoint(randomOrd)))
            {
                continue;
            }

            if(!isAccessible(cspace_, convertToCVPoint(randomOrd)))
            {
                continue; //Is not accessible in the ogmap, skip
            }


            if(violatingSpace(randomOrd, r))
            {
                continue; //We want uniform distribution, skip
            }

            addOrdinate(randomOrd);
        }

        joinNetwork(cspace_, density_);
        path_ = query(cspace_, start, goal);
        numNodes += 100;
        density_ += 10;
        std::cout <<"numNodes = "<< numNodes - 100 << "  density_ = " <<  density_ - 10 << std::endl;
        if(numNodes >=500 || density_ >= 50)
        {
            break;
        }
    }


    if (path_.size() <= 0) {
//        std::cout << "PRM planner failed to plan" << std::endl;
        return false;
    }
//    std::cout << "success prm path plan" << std::endl;

    return true;
}

bool PRMPlanner::ordinateAccessible(cv::Mat &cspace, TGlobalOrd ordinate)
{
  if(existsAsVertex(ordinate) 
    || isAccessible(cspace, convertToCVPoint(ordinate)))
  {
    return true;
  }
  return false;
}

Vertex PRMPlanner::findOrAdd(TGlobalOrd ordinate)
{
  Vertex v;
  if(existsAsVertex(ordinate)){
    lookup(ordinate, v);
  } else {
    v = addOrdinate(ordinate);
  }

  return v;
}

Vertex PRMPlanner::addOrdinate(TGlobalOrd ordinate)
{
  //Generate a new vertex and add to graph, also adding
  //to the internal lookup table.
  Vertex v = nextVertexId();
  graph_.addVertex(v);
  network_.insert(std::make_pair(v, ordinate));

  return v;
}

bool PRMPlanner::existsAsVertex(TGlobalOrd ord)
{
  for(auto &v: network_){
    if(v.second == ord){
      return true;
    }
  }

  return false;
}

Vertex PRMPlanner::nextVertexId()
{
  Vertex temp = nextVertexId_;
  nextVertexId_++;

  return temp;
}

bool PRMPlanner::lookup(TGlobalOrd ord, Vertex &v)
{
  for(auto &vert: network_){
    if(vert.second == ord){
      v = vert.first;
      return true;
    }
  }

  return false;
}

void PRMPlanner::embedNode(cv::Mat &cspace, Vertex node, unsigned int k, 
  bool retry)
{
  std::vector<TGlobalOrd> neighbours;
  TGlobalOrd nodeOrd = network_[node];

  //Get all nodes in the network ordered by distance to this node
  neighbours = getNeighbours(cspace, node, false);

  int timesConnected(0);
  for(auto const &neighbour: neighbours){
    bool connected = false;

    if(timesConnected == k || !graph_.canConnect(node)){
      //We've reached the max tries or the node has maxed out its connections
      break;
    }

    Vertex vNeighbour;
    if(!lookup(neighbour, vNeighbour)){
      //something went wrong adding this neighbour, continue to next
      continue;
    }

    //Attempt to connect to neighbour
    cv::Point pCurrent = convertToCVPoint(nodeOrd);
    cv::Point pN = convertToCVPoint(neighbour);
    if(canConnect(cspace, pCurrent, pN))
    {
      connected = graph_.addEdge(node, vNeighbour, 
        distance(nodeOrd, neighbour));
    }

    if(connected){
      //If we successfully connected to a neighbour
      timesConnected++;
    } else if(!retry) {
      //If we are not retrying to find connections, increment count anyway
      timesConnected++;
    }
  }
}

std::vector<TGlobalOrd> PRMPlanner::getNeighbours(cv::Mat &cspace, 
  Vertex node, bool shouldConnect)
{
  std::vector<TGlobalOrd> neighbours;
  TGlobalOrd nodeOrd = network_[node];

  //Attempt to connect each node in network to k closest neighbours
  for(auto const &neighbour: network_){
    if(neighbour.first == node){
      continue; //don't want to connect to ourselves
    }

    //If we care about our ability to connect, then we must check
    if(shouldConnect){
      cv::Point pCurrent = convertToCVPoint(nodeOrd);
      cv::Point pN = convertToCVPoint(neighbour.second);

      if(!canConnect(cspace, pCurrent, pN)){
        continue; //Will skip this neighbour
      }

      if(!graph_.canConnect(neighbour.first)){
        continue;
      }
    }

    //Add neighbour to list
    neighbours.push_back(neighbour.second);
  }

  //Sort neighbours by distance.
  std::sort(neighbours.begin(), neighbours.end(), [nodeOrd](
    const TGlobalOrd &lhs, const TGlobalOrd &rhs){
      return distance(lhs, nodeOrd) < distance(rhs, nodeOrd);});

  return neighbours;
}

cv::Point PRMPlanner::convertToCVPoint(TGlobalOrd ordinate)
{
  // todo:
  // int convertedX = map_height_ - ordinate.y - 1;
  // int convertedY = ordinate.x;
  int convertedX = ordinate.x;
  int convertedY = ordinate.y;

  return cv::Point(convertedX, convertedY);
}

bool PRMPlanner::canConnect(cv::Mat &cspace, cv::Point start, cv::Point end)
{
  //Do a bounds check
  if(!inMap(start) || !inMap(end)){
    return false;
  }

  //Iterate through each pixel between both points, checking that
  //each pixel is white = free space
  cv::LineIterator line(cspace, start, end);
//  cv::Mat cspace_rgb;
//    cv::cvtColor(cspace, cspace_rgb, cv::COLOR_GRAY2BGR);
//  cv::line(cspace_rgb,start,end,cv::Scalar(0,255,0));
//  cv::Mat resize_cspace;
//  cv::resize(cspace_rgb,resize_cspace,cv::Size(cspace_rgb.rows / 2,cspace_rgb.cols /2));
//  cv::namedWindow("connet mat",0);
//  cv::imshow("connet mat",resize_cspace);
//  cv::waitKey(0);
//  static int count = 0;
//  std::string strOut = "/home/zhihui/tmp_png/" + std::to_string(count) + ".png";
//  cv::imwrite(strOut,cspace_rgb);
//  count++;
//  cv::imwrite("/home/zhihui/111.png",cspace_rgb);
//  cv::Point p1, p2, p3, p4;
//  float r2 = robot_size * 0.4;
  for(int i = 0; i < line.count; i++, line++){

    if(!inMap(line.pos()))
    {
//        printf("inMap false\n");
        return false;
    }
    if(!isAccessible(cspace, line.pos())){
//        printf("isAccesssible false\n");
      return false;
    }
  }

//  printf("canConnect success\n");
  return true;
}

bool PRMPlanner::inMap(cv::Point p)
{
  // todo:
  if(p.x <= map_width_ && p.x >= 0 && p.y <= map_height_ && p.y >=0)
  {
    //std::cout << "In map" << std::endl;
    return true;
  }
  // std::cout << "Not in map" << std::endl;
  return false;
}

bool PRMPlanner::isAccessible(cv::Mat &cspace, cv::Point p){
  if(!inMap(p)){
    return false;
  }

  if(cspace.at<uchar>(p) == FREE_SPACE_VALUE)
  {
    return true;
  }
  return false;
}

bool PRMPlanner::violatingSpace(TGlobalOrd ord, double r){
  for(auto const & n : network_){
    if(distance(ord, n.second) < 2 * r){
      return true;
    }
  }

  return false;
}

double PRMPlanner::distance(TGlobalOrd o1, TGlobalOrd o2){
  double a = std::abs(o2.x - o1.x);
  double b = std::abs(o2.y - o1.y);

  return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
}

std::vector<TGlobalOrd> PRMPlanner::query(cv::Mat &cspace, TGlobalOrd start,
  TGlobalOrd goal)
{
  Vertex vStart, vGoal;

  if(!lookup(start, vStart) || !lookup(goal, vGoal)){
    return std::vector<TGlobalOrd>();
  }

  //Assumes the path has already been found
  std::vector<Vertex> vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return optimisePath(cspace_, toOrdPath(vPath));
//      return toOrdPath(vPath);
  }

  return std::vector<TGlobalOrd>();
}

double PRMPlanner::freeConfigSpace(cv::Mat &cspace)
{
  unsigned int freePixels(0);

//  for(int i = 0; i < cspace.rows; i++){
//    for(int j = 0; j < cspace.cols; j++){
//      if(cspace.at<uchar>(j,i) == FREE_SPACE_VALUE){
//        freePixels++;
//      }
//    }
//  }
    for(int i = 0; i < cspace.cols; i++){
        for(int j = 0; j < cspace.rows; j++){
            if(cspace.at<uchar>(j,i) == FREE_SPACE_VALUE){
                freePixels++;
            }
        }
    }

  //After finding the amount of free pixels, multiply by the resolution^3
  //to get the effective volume in metres
  // todo: resolution^3 ?
  return freePixels * resolution_ * resolution_ * resolution_;
}

std::vector<TGlobalOrd> PRMPlanner::optimisePath(cv::Mat &cspace,
  std::vector<TGlobalOrd> path)
{
  std::vector<TGlobalOrd> optPath;

  if(path.size() == 0){
    return optPath; //No path to optimise return empty path
  }

  //Start with the first node
  optPath.push_back(path.at(0));

  //While the goal is not in the optimised path
  while(std::find(optPath.begin(), optPath.end(), path.back()) == optPath.end())
  {
    TGlobalOrd ordCurr = optPath.back();
    cv::Point pCurrent = convertToCVPoint(ordCurr);

    //Starting at the end of the path and moving backwards, determine
    //if we can directly connect to the current ordinate
    for(unsigned i = path.size(); i-- > 0;){
      if(path[i] == ordCurr){
        optPath.push_back(path[i+1]);
        break; //We have reached the current ordinate
      }

      cv::Point pTest = convertToCVPoint(path[i]);
      if(canConnect(cspace, pCurrent, pTest)){
        optPath.push_back(path[i]);
        break; //We have found the earliest node to directly connect to
      }
    }
  }

  return optPath;
}

std::vector<TGlobalOrd> PRMPlanner::toOrdPath(std::vector<Vertex> path){
  std::vector<TGlobalOrd> ordPath;

  for(auto const &v: path){
    ordPath.push_back(network_[v]);
  }

  return ordPath;
}

void PRMPlanner::joinNetwork(cv::Mat &cspace, unsigned int k){
  //Attempt to connect each node in the network to its k closest neighbours
  //Nodes that have the least amount of connections are embedded first
  for(auto const &node: prioritiseNodes()){
    if(!graph_.canConnect(node)){
      continue; //This node has already maxed out its connections
    }

    embedNode(cspace, node, k, false);
  }
}

std::vector<Vertex> PRMPlanner::prioritiseNodes(){
  //a list of <vertex, edgeCount>
  std::vector<std::pair<Vertex, unsigned int>> nodeConnections;

  for(auto const &entry: network_){
    nodeConnections.push_back(std::make_pair(
      entry.first, graph_.getEdgeCount(entry.first)));
  }

  //Prioritise connection order by nodes who have the least amount of edges
  std::sort(nodeConnections.begin(), nodeConnections.end(),
    [](const std::pair<Vertex, unsigned int> &lhs, 
       std::pair<Vertex, unsigned int> &rhs)
    {
      return lhs.second < rhs.second;
    }
  );

  //Strip the edge count
  std::vector<Vertex> nodes;
  for(auto const &node: nodeConnections){
    nodes.push_back(node.first);
  }

  return nodes;
}

/**bool PRMPlanner::getPathROS(nav_msgs::Path &path, double resolution,
  geometry_msgs::Pose origin, const geometry_msgs::PoseStamped &goal_pose)
{
  std::cout << "Get path from PRM planner" << std::endl;

  if(path_.size() <= 0)
  {
    std::cout << "Path is empty" << std::endl;
    return false;
  }

  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  ratio_ = 1.0;// sampling ratio

  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped next_pose;

  indexToPose(path_[0].x, path_[0].y, pose, resolution, origin);

  for(std::size_t i = 1; i < path_.size(); i++)
  {
    if(!sampling())
      continue;
    
    indexToPose(path_[i].x, path_[i].y, next_pose, resolution, origin);

    geometry_msgs::Quaternion q = getOrientationFromPoses(pose, next_pose);
    pose.pose.orientation = q;
    //next_pose.pose.orientation = q;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    path.poses.push_back(pose);
    //path.poses.push_back(next_pose);
    pose = next_pose;
  }

  // path.poses[path.poses.size() - 1].pose.orientation = 
  //   goal_pose.pose.orientation;
  path.poses.push_back(goal_pose);
  return true;
}**/

// todo
/**bool PRMPlanner::initializeCSpace(cv::Mat &cspace, const NavGridMap &map)
{
  std::cout << "Initialize c space of prm planner" << std::endl;
  if(map_width_ <= 0 || map_height_ <= 0)
  {
    return false;
  }

  cspace = cv::Mat(map_height_, map_width_, CV_8U);
  for(int r = 0; r < map_height_; r++)
  {
    for(int c = 0; c < map_width_; c++)
    {
      int map_id = (map_height_ - r - 1) * map_width_ + c;
      // todo
      // cspace.at<uchar>(c, r) = map.data[map_id];
      cspace.at<uchar>(r, c) = map.data[map_id];
    }
  }
  //cv::namedWindow("cspace", CV_WINDOW_NORMAL);
  //cv::imshow("cspace", cspace_);
  //cv::waitKey(0);
  return true;
}**/

/**geometry_msgs::Quaternion PRMPlanner::getOrientationFromPoses(
  geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2)
{
  double delta_x = p2.pose.position.x - p1.pose.position.x;
  double delta_y = p2.pose.position.y - p1.pose.position.y;
  double yaw = std::atan2(delta_y, delta_x);

  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

void PRMPlanner::indexToPose(double x, double y,
  geometry_msgs::PoseStamped &pose, double resolution, 
  geometry_msgs::Pose origin)
{
  pose.pose.position.x = x * resolution + origin.position.x;
  pose.pose.position.y = y * resolution + origin.position.y;
}**/

bool PRMPlanner::sampling()
{
  ++num_pass_;
  if(static_cast<double>(num_samples_) / num_pass_ < ratio_)
  {
      ++num_samples_;
      return true;
  }
  return false;
}


