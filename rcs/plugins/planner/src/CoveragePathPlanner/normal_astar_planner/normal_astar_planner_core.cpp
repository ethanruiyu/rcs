/*
 * Created by Dongxiao Wu on 12/22/18.
 * Contact with: wdxairforce@gmail.com
*/

#include "CoveragePathPlanner/normal_astar_planner/normal_astar_planner_core.h"

using namespace normal_astar_planner;

NormalAstarPlannerCore::NormalAstarPlannerCore()
{
    node_initialized_ = false;
    b_map_set_ = false;
    b_start_set_ = false;
    b_goal_set_ = false;
    prune_size_ = 0;
}

NormalAstarPlannerCore::~NormalAstarPlannerCore()
{

}

//制定计划
bool NormalAstarPlannerCore::makePlan(std::vector<cv::Point> &path) {

    if (!node_initialized_) {
        resizeNode(visualize_map_.cols, visualize_map_.rows);
        // initial the update action (for the surrounding judge!)
        update_nodes.resize(8);
        {
            update_nodes[0].delta_x = 1;
            update_nodes[0].delta_y = 0;
            update_nodes[0].step = 1;

            update_nodes[1].delta_x = 1;
            update_nodes[1].delta_y = -1;
            update_nodes[1].step = std::sqrt(2);

            update_nodes[2].delta_x = 0;
            update_nodes[2].delta_y = -1;
            update_nodes[2].step = 1;

            update_nodes[3].delta_x = -1;
            update_nodes[3].delta_y = -1;
            update_nodes[3].step = std::sqrt(2);

            update_nodes[4].delta_x = -1;
            update_nodes[4].delta_y = 0;
            update_nodes[4].step = 1;

            update_nodes[5].delta_x = -1;
            update_nodes[5].delta_y = 1;
            update_nodes[5].step = std::sqrt(2);

            update_nodes[6].delta_x = 0;
            update_nodes[6].delta_y = 1;
            update_nodes[6].step = 1;

            update_nodes[7].delta_x = 1;
            update_nodes[7].delta_y = 1;
            update_nodes[7].step = std::sqrt(2);

        }
        node_initialized_ = true;
    }
    // initial the Map
    resetCurrentMap();
    // set start and goal cell
    int start_index_x = 0,  start_index_y = 0;
    int goal_index_x = 0, goal_index_y = 0;
    if((poseToIndex(start_pose_global_, &start_index_x, &start_index_y) &&
        poseToIndex(goal_pose_global_, &goal_index_x, &goal_index_y)))
    {
        AstarNode start_node, goal_node;
        start_node.index_x = start_index_x;
        start_node.index_y = start_index_y;
        goal_node.index_x = goal_index_x;
        goal_node.index_y = goal_index_y;
        start_node.gc = 0;
        start_node.hc = astar::calcDistance(start_index_x, start_index_y, goal_index_x, goal_index_y);
        start_node.cost = start_node.gc + start_node.hc;

        // begin to search path
        if(searchPath(start_node, goal_node)){
            //LOG(INFO) << "success to search the path" << std::endl;
           // ROS_INFO("success to search the path");
            // process path
//            pruenPath(path_);
//            smoothPath(path_, 0.49, 0.25, 0.05);
//            interpolateOrientationOfPath(path_);
            path = path_;
        }
        else
        {
            std::cout << "fail to search the path" << std::endl;
            return false;
        }
    }
    else {
        std::cout << "error start or goal index" << std::endl;
        return false;
    }
    return true;
}

//调整节点大小
void NormalAstarPlannerCore::resizeNode(int width, int height) {

    std::cout << "resize: width" << width << " height : " << height << std::endl;

    nodes_.resize(height);

    for (int i = 0; i < height; i++)
        nodes_[i].resize(width);
}


//路径数据类型转换
void NormalAstarPlannerCore::tracePath(const Node3D* node, std::vector<Node3D> &path) {
    if (node == nullptr) {
        path = path;
        return;
    }
    path.push_back(*node);
    tracePath(node->getPred(), path);
}


void NormalAstarPlannerCore::setPath_goal(const AstarNode &goal) {


    // From the goal node to the start node
    AstarNode *node = &nodes_[goal.index_y][goal.index_x];

    while (node != NULL) {
        // Set path as ros message
        cv::Point path_point;
        path_point.x = node->index_x;
        path_point.y = node->index_y ;
        path_.push_back(path_point);

        // To the next node
        node = node->parent;
    }

    // Reverse the vector to be start to goal order
    std::reverse(path_.begin(), path_.end());
}



double NormalAstarPlannerCore::Quaternion_to_euler(double x, double y, double z, double w)
{
    double roll, pich, yaw;
    roll = atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    pich = asin(2*(w*y-z*z));
    yaw = atan2(2*(w*z+x*y),1-2*(z*z+y*y));
    return yaw;
}


//搜索路径
bool NormalAstarPlannerCore::searchPath(AstarNode& start_node, AstarNode& goal_node){

    // Set start node
    AstarNode &temp_start_node = nodes_[start_node.index_y][start_node.index_x];
    temp_start_node.index_x      = start_node.index_x;
    temp_start_node.index_y      = start_node.index_y;
    temp_start_node.gc     = 0;
    temp_start_node.status = STATUS::OPEN;
    openlist_.push(temp_start_node);

    while (!openlist_.empty()) {
        // Pop minimum cost node from openlist
        AstarNode temp_current_astar_node = openlist_.top();   // openlist_按升序排列

        openlist_.pop();
        nodes_[temp_current_astar_node.index_y][temp_current_astar_node.index_x].status = STATUS::CLOSED;
        AstarNode &current_astar_node = nodes_[temp_current_astar_node.index_y][temp_current_astar_node.index_x];
        // for each update
        std::vector<CirclePoint> circle_point_list;
        bool flag = false;
        for(const auto &update : update_nodes){
            double move_cost  = update.step;
            // Calculate index of the next state
            int next_index_x = current_astar_node.index_x + update.delta_x;
            int next_index_y = current_astar_node.index_y + update.delta_y;

//         LOG(INFO) << "B next_index_x " << next_index_x << std::endl;
//         LOG(INFO) << "B next_index_y " << next_index_y << std::endl;
            // Check if the index is valid
            if (isOutOfRange(next_index_x, next_index_y) || detectCollision(next_index_x, next_index_y))
                continue;


            AstarNode *next_astar_node = &nodes_[next_index_y][next_index_x];
            double next_hc = astar::calcDistance(next_index_x, next_index_y, goal_node.index_x, goal_node.index_y);

//         LOG(INFO) << "A next_index_x " << next_index_x << std::endl;
//         LOG(INFO) << "A next_index_y " << next_index_y << std::endl;
//         LOG(INFO) << "next_astar_node->status " << next_astar_node->status << std::endl;


            // goal check
            if(isGoal(next_index_x, next_index_y, goal_node)){
                next_astar_node->status  = STATUS::OPEN;
                next_astar_node->index_x = next_index_x;
                next_astar_node->index_y = next_index_y;
                next_astar_node->gc      = current_astar_node.gc + move_cost;
                next_astar_node->hc      = next_hc;
                next_astar_node->parent  = &current_astar_node;

                setPath_goal(*next_astar_node);
                return true;
            }


            // NONE
            if (next_astar_node->status == STATUS::NONE) {
                next_astar_node->status  = STATUS::OPEN;
                next_astar_node->index_x = next_index_x;
                next_astar_node->index_y = next_index_y;
                next_astar_node->gc      = current_astar_node.gc + move_cost;
                next_astar_node->hc      = next_hc;
                next_astar_node->parent  = &current_astar_node;

                next_astar_node->cost = next_astar_node->gc + next_astar_node->hc;
                openlist_.push(*next_astar_node);
                continue;
            }

            // OPEN or CLOSED
            if (next_astar_node->status == STATUS::OPEN || next_astar_node->status == STATUS::CLOSED) {
                //if (current_astar_node.gc + move_cost + next_hc < next_astar_node->gc + next_hc) {
                if(current_astar_node.gc + move_cost < next_astar_node->gc){
                    next_astar_node->status  = STATUS::OPEN;
                    next_astar_node->index_x = next_index_x;
                    next_astar_node->index_y = next_index_y;
                    next_astar_node->gc      = current_astar_node.gc + move_cost;
                    next_astar_node->hc      = next_hc;
                    next_astar_node->parent  = &current_astar_node;

                    next_astar_node->cost = next_astar_node->gc + next_astar_node->hc;
                    openlist_.push(*next_astar_node);
                    continue;
                }
            }
        }

    }
    std::cout << "Openlist is Empty!" << std::endl;
    return false;
}

//简化路径，减少路径点个数
//void NormalAstarPlannerCore::pruenPath(basics::Path &path){
//    basics::Path out_path;
//    // resolution( m/cell )
//    int prune_pixel_size = static_cast<int>(prune_size_ / map_.info.resolution);   // 最小单位格
//    for(std::size_t id = 0; id < path.poses.size() - 1; ++id){
//        if(id % prune_pixel_size == 0){
//            basics::PoseStamped pose = path.poses[id];
//            out_path.poses.push_back(pose);
//        }
//    }
//    basics::PoseStamped pose = path.poses[path.poses.size() - 1];
//    out_path.poses.push_back(pose);
//
//    path.poses.clear();
//    path = out_path;
//    time_t time1;
//    time(&time1);
//    path.header.stamp = time1;
//    path.header.frame_id = "/map";
//}

//平滑路径
//void NormalAstarPlannerCore::smoothPath(basics::Path &path, const double& weight_data, const double& weight_smooth, const double& tolerance) {
//
//    if (path.poses.size() <= 2) {
//        //cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
//        return;
//    }
//
//    const basics::Path &path_in = path;
//    basics::Path smoothPath_out = path_in;
//
//    double change = tolerance;
//    double xtemp, ytemp;
//    int nIterations = 0;
//
//    int size = path_in.poses.size();
//
//    while (change >= tolerance) {
//        change = 0.0;
//        for (std::size_t i = 1; i < size - 1; i++) {
//
//            xtemp = smoothPath_out.poses[i].pose.position.x;
//            ytemp = smoothPath_out.poses[i].pose.position.y;
//
//            smoothPath_out.poses[i].pose.position.x += weight_data
//                                                       * (path_in.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
//            smoothPath_out.poses[i].pose.position.y += weight_data
//                                                       * (path_in.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);
//
//            smoothPath_out.poses[i].pose.position.x += weight_smooth
//                                                       * (smoothPath_out.poses[i - 1].pose.position.x + smoothPath_out.poses[i + 1].pose.position.x
//                                                          - (2.0 * smoothPath_out.poses[i].pose.position.x));
//            smoothPath_out.poses[i].pose.position.y += weight_smooth
//                                                       * (smoothPath_out.poses[i - 1].pose.position.y + smoothPath_out.poses[i + 1].pose.position.y
//                                                          - (2.0 * smoothPath_out.poses[i].pose.position.y));
//
//            change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
//            change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
//
//        }
//        nIterations++;
//    }
//
//    path = smoothPath_out;
//    time_t time1;
//    time(&time1);
//    path.header.stamp = time1;
//    path.header.frame_id = "/map";
//}

//插值路径方向
//void NormalAstarPlannerCore::interpolateOrientationOfPath(basics::Path &path){
//    for(std::size_t id_current = 0; id_current < path.poses.size()-1; ++id_current){
//
//        std::size_t id_next = id_current + 1;
//        basics::Pose pose_current = path.poses[id_current].pose;
//        basics::Pose pose_next = path.poses[id_next].pose;
//
//        double delta_x = pose_next.position.x - pose_current.position.x;
//        double delta_y = pose_next.position.y - pose_current.position.y;
//        double yaw = std::atan2(delta_y, delta_x);
//
//        Eigen::Quaterniond quaternion;
//        quaternion = basics::createQuaternionFromYaw(yaw);
//        path.poses[id_current].pose.orientation.x = quaternion.x();
//        path.poses[id_current].pose.orientation.y = quaternion.y();
//        path.poses[id_current].pose.orientation.z = quaternion.z();
//        path.poses[id_current].pose.orientation.w = quaternion.w();
//
//    }
//
//    //handle the end of path
//    path.poses[path.poses.size()-1].pose.orientation = goal_pose_global_.pose.orientation;
//}

//是否超出范围
bool NormalAstarPlannerCore::isOutOfRange(int index_x, int index_y)
{
    if (index_x < 0 || index_x >= static_cast<int>(visualize_map_.cols) || index_y < 0 || index_y >= static_cast<int>(visualize_map_.rows))
        return true;

    return false;
}

//检测碰撞
bool NormalAstarPlannerCore::detectCollision(int index_x, int index_y) {

    if (isOutOfRange(index_x, index_y))
        return true;
    if (nodes_[index_y][index_x].status == STATUS::OBS)
        return true;
    if (isOutOfRange(index_x+2, index_y+2))
        return true;
    if (nodes_[index_y+2][index_x+2].status == STATUS::OBS)
        return true;
    if (nodes_[index_y][index_x+2].status == STATUS::OBS)
        return true;
    if (nodes_[index_y+2][index_x].status == STATUS::OBS)
        return true;
    if (isOutOfRange(index_x-2, index_y-2))
        return true;
    if (nodes_[index_y-2][index_x-2].status == STATUS::OBS)
        return true;
    if (nodes_[index_y][index_x-2].status == STATUS::OBS)
        return true;
    if (nodes_[index_y-2][index_x].status == STATUS::OBS)
        return true;
    return false;
}

bool NormalAstarPlannerCore::poseToIndex(cv::Point pose, int *index_x, int *index_y){
    // normal A star planner not consider orientation
    *index_x = pose.x;
    *index_y = pose.y;
    if(*index_x < 0 || *index_x > visualize_map_.cols || *index_y < 0 || *index_y > visualize_map_.rows){
        *index_x = -1;
        *index_y = -1;
        std::cout << "invalid pose" << std::endl;
        return false;
    }

    std::cout << "poseToIndex: " << *index_x << " " << *index_y << std::endl;

    return true;
}

void NormalAstarPlannerCore::resetCurrentMap(){
    if (!node_initialized_) {
        resizeNode(visualize_map_.cols, visualize_map_.rows);
        node_initialized_ = true;
    }
    cv::Mat astar_map(visualize_map_.rows, visualize_map_.cols, CV_8UC1, cv::Scalar(0));

    for (size_t i = 0; i < visualize_map_.rows; i++) {
        for (size_t j = 0; j < visualize_map_.cols; j++) {
            // Index of subscribing OccupancyGrid message
//            int cost = visualize_map_.at<int>(i,j);
            int cost = *(visualize_map_.data + visualize_map_.step[0] * i + visualize_map_.step[1] * j) ;

            // more than threshold or unknown area
            if (cost > 100/* || cost < 0 */) {
                nodes_[i][j].status = STATUS::OBS;
                *(astar_map.data + astar_map.step[0] * i + astar_map.step[1] * j) = 255;
            }
            else{
                nodes_[i][j].hc     = 0;
                nodes_[i][j].status = STATUS::NONE;
                nodes_[i][j].parent = NULL;
                *(astar_map.data + astar_map.step[0] * i + astar_map.step[1] * j) = 0;
            }
        }
    }
}

//是否目标
bool NormalAstarPlannerCore::isGoal(double x, double y, AstarNode goal_node)
{
    // Check the pose of goal
    if (goal_node.index_x == x && goal_node.index_y == y) {
        return true;
    }

    return false;
}



void NormalAstarPlannerCore::setRobotSize(const double &robot_size)
{
    robot_size_ = robot_size;
}


//设置地图
void NormalAstarPlannerCore::setMap(cv::Mat map)
{
    visualize_map_ = map;
    b_map_set_ = true;
}


//设置起点
void NormalAstarPlannerCore::setStart(cv::Point initPose)
{
//    std::cout << "set start pose (x, y) " << initPose.pose.position.x << ", " << initPose.pose.position.y << std::endl;
    start_pose_global_ = initPose;
    b_start_set_ = true;
}

//设置目标点
void NormalAstarPlannerCore::setGoal(cv::Point goalPose)
{
//    std::cout << "set goal pose (x, y) " << goalPose.pose.position.x << ", " << goalPose.pose.position.y << std::endl;
    goal_pose_global_ = goalPose;
    b_goal_set_ = true;
}


//是否有效状态
bool NormalAstarPlannerCore::validState() const{
    return (b_map_set_ && b_start_set_ && b_goal_set_);
}



bool NormalAstarPlannerCore::PtInPolygon (cv::Point p, std::vector<cv::Point> ptPolygon, int nCount) {
    int nCross = 0;
    for (int i = 0; i < nCount; i++) {
        cv::Point p1 = ptPolygon[i];
        cv::Point p2 = ptPolygon[(i + 1) % nCount];

        if ( p1.y == p2.y ) // p1p2 与 y=p0.y平行
            continue;

        if ( p.y < std::min(p1.y, p2.y) ) // 交点在p1p2延长线上
            continue;

        if ( p.y >= std::max(p1.y, p2.y) ) // 交点在p1p2延长线上
            continue;

        double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;
        if ( x > p.x )
            nCross++;
    }
    return (nCross % 2 == 1);
}

void NormalAstarPlannerCore::setDubins(bool is_conside_goal_pose, double turning_radius, int n)
{
    is_conside_goal_pose_ = is_conside_goal_pose;
    turning_radius_ = turning_radius;
    n_ = n;
}
