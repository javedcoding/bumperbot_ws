#include <queue>
#include "bumperbot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace bumperbot_planning
{
DijkstraPlanner:: DijkstraPlanner() : Node("dijkstra_node")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", map_qos, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1)); // In Humble, it will be "/costmap/costmap"

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&DijkstraPlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/dijkstra/path", 10);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/dijkstra/visited_map", 10);
}

void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    map_ = map;
    visited_map_.header.frame_id = map->header.frame_id;
    visited_map_.info = map->info;
    visited_map_.data = std::vector<int8_t>(visited_map_.info.height*visited_map_.info.width, -1);
}

void DijkstraPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    if(!map_){
        RCLCPP_ERROR(get_logger(), "No map received!");
        return;
    }
    
    visited_map_.data = std::vector<int8_t>(visited_map_.info.height*visited_map_.info.width, -1);
    geometry_msgs::msg::TransformStamped map_to_base_tf;
    
    try{
        map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
    } catch(tf2::TransformException & ex){
        RCLCPP_ERROR(get_logger(), "Could not transform from map to base_footprint");
        return;
    }
    
    geometry_msgs::msg::Pose map_to_base_pose;
    map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
    map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
    map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

    auto path = plan(map_to_base_pose, pose->pose);
    if(!path.poses.empty()){
        RCLCPP_INFO(get_logger(), "Shortest path found");
        path_pub_->publish(path);
    } else{
        RCLCPP_WARN(get_logger(), "No path found to the goal!");
    }
}

nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose & start, geometry_msgs::msg::Pose & goal)
{
    std::vector<std::pair<int, int>> explore_directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };
    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
    std::vector<GraphNode> visited_nodes;

    pending_nodes.push(worldToGrid(start));
    GraphNode active_node;
    while(!pending_nodes.empty() && rclcpp::ok()){
        active_node = pending_nodes.top(); //take the first node into active node
        pending_nodes.pop(); //As taken remove the currently active node

        if(worldToGrid(goal) == active_node){ //if the current node is the goal node then break the loop
            break;
        }

        for (const auto & dir:explore_directions){ //selecting each 4 neighboring directions
            GraphNode new_node = active_node + dir; //operator+ or the sum operator is going to add the exploredirection to current graphnode making it a new node
            if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && 
                poseOnMap(new_node) && map_->data.at(poseToCell(new_node)) < 99 && map_->data.at(poseToCell(new_node)) >=0){ //checking from start to end of visited_nodes if the current neighbour is visited
                    new_node.cost = active_node.cost + 1 + map_->data.at(poseToCell(new_node)); //add the cost and the costmap values
                    new_node.prev = std::make_shared<GraphNode>(active_node); //put the active node to previously selected node
                    pending_nodes.push(new_node); 
                    visited_nodes.push_back(new_node); //put the active node to previously visited node
            }

        }

        visited_map_.data.at(poseToCell(active_node)) = 10; //put the occupancy grid to make it visited as green in rviz map
        map_pub_->publish(visited_map_);
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;

    while(active_node.prev && rclcpp::ok()){
        geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
        geometry_msgs::msg::PoseStamped last_pose_stamped;
        last_pose_stamped.header.frame_id = map_->header.frame_id;
        last_pose_stamped.pose = last_pose;
        path.poses.push_back(last_pose_stamped);
        active_node = *active_node.prev;
    }

    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
    int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
    int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);
    return GraphNode(grid_x,grid_y);
}

bool DijkstraPlanner::poseOnMap(const GraphNode & node)
{ //Checking if the node is actually inside the map
    return node.x >= 0 && node.x < static_cast<int>(map_->info.width) && 
        node.y >= 0 && node.y < static_cast<int>(map_->info.height); 
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
{
    return node.y * map_->info.width + node.x;
}

geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node){
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
    pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
    return pose;
}
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}