#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>

namespace bumperbot_motion
{
PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"), 
kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0),
prev_linear_error_(0.0), prev_angular_error_(0.0)
{
    //declaration of parameters and binding them with constructor variables
    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("step_size", step_size_);
    declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

    // Get parameters from the declared parameters
    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    step_size_ = get_parameter("step_size").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd_next_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));
    last_cycle_time_ = get_clock()->now();
}

void PDMotionPlanner::controlLoop()
{
    if (global_plan_.poses.empty()) {
         RCLCPP_WARN(get_logger(), "No global plan received yet.");
         return;
        }
        
    geometry_msgs::msg::TransformStamped robot_pose;
    try {
        robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
        return;
    }

    if(!transformPlan(robot_pose.header.frame_id)) {
        RCLCPP_ERROR(get_logger(), "Unable to transform plan to robot's frame");
        return;
    }

    geometry_msgs::msg::PoseStamped robot_pose_stamped; 
    robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
    robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
    robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
    robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;
    
    auto next_pose = getNextPose(robot_pose_stamped);
    double dx = next_pose.pose.position.x - robot_pose_stamped.pose.position.x;
    double dy = next_pose.pose.position.y - robot_pose_stamped.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if(distance <= 0.1) {
        RCLCPP_INFO(get_logger(), "Reached Goal, stopping!");
        global_plan_.poses.clear(); // Clear the plan
        return;
    }

    next_pose_pub_->publish(next_pose);
    tf2::Transform robot_tf, next_pose_tf, next_pose_robot_tf;
    tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
    tf2::fromMsg(next_pose.pose, next_pose_tf);

    next_pose_robot_tf = robot_tf.inverse() * next_pose_tf; // Transform next pose to robot's frame with error
    double linear_error = next_pose_robot_tf.getOrigin().getX(); //calculated pid error in linear x direction
    double angular_error = next_pose_robot_tf.getOrigin().getY(); //calculated pid error in angular y direction
    double dt = (get_clock()->now() - last_cycle_time_).seconds(); // Time since last cycle
    double linear_error_derivative = (linear_error - prev_linear_error_) / dt; // Derivative of linear error
    double angular_error_derivative = (angular_error - prev_angular_error_) / dt; // Derivative of angular error

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative, -max_linear_velocity_, max_linear_velocity_);
    cmd_vel.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative, -max_angular_velocity_, max_angular_velocity_);
    // already Limited the velocities now to publish the command
    cmd_pub_->publish(cmd_vel);

    // Update previous errors and time
    last_cycle_time_ = get_clock()->now();
    prev_linear_error_ = linear_error;
    prev_angular_error_ = angular_error;
}


bool PDMotionPlanner::transformPlan(const std::string & frame)
{
    if (global_plan_.header.frame_id == frame) {
        return true; // No transformation needed
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Get the transformation from the target frame to the global plan's frame
        transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
    } catch (tf2::LookupException & ex) {
        RCLCPP_ERROR_STREAM(get_logger(), 
        "Could not transform plan from frame " << global_plan_.header.frame_id << " to " << frame << ": " << ex.what());
        return false;
    }
    
    for (auto & pose : global_plan_.poses) {
        tf2::doTransform(pose, pose, transform);
    }
    
    global_plan_.header.frame_id = frame;
    return true;
}

void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    global_plan_ = *path;
}

geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
    geometry_msgs::msg::PoseStamped next_pose = global_plan_.poses.back();
    for(auto post_it = global_plan_.poses.rbegin(); post_it != global_plan_.poses.rend(); ++post_it) {
        double dx = post_it->pose.position.x - robot_pose.pose.position.x;
        double dy = post_it->pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance > step_size_) {
            next_pose = *post_it;
        } else {
            break; // Found the next pose that is at least step_size away
        }
    }
    return next_pose;
}
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}