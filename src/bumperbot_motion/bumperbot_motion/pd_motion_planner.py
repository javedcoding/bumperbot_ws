#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener

class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner")
        self.declare_parameter("kp", 2.0) #this is a proportional gain parameter and default is 2
        self.declare_parameter("kd", 0.1) #this is a derivative gain parameter and default is 0.1
        self.declare_parameter("step_size", 0.2) #this is the minimum distance the robot wants to go before checking and default is 0.2
        self.declare_parameter("max_linear_velocity", 0.3) #this is a max linear velocity robot wants to gain and default is 0.3
        self.declare_parameter("max_angular_velocity", 1.0) #this is a max angular velocity robot wants to gain and default is 1.0

        self.kp = self.get_parameter("kp").value #class variable kp
        self.kd = self.get_parameter("kd").value #class variable kd
        self.step_size = self.get_parameter("step_size").value #class variable step_size
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value #class variable max_linear_velocity
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value #class variable max_angular_velocity

        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.next_pose_pub = self.create_publisher(PoseStamped, "/pd/next_pose", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop) #10hz frequency to check and fix listener
        self.global_plan = None

    def path_callback(self, path: Path):
        self.global_plan = path

    def control_loop(self):
       if not self.global_plan or not self.global_plan.poses:
           return
       
       try:
           robot_pose_transform = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        
       except Exception as ex:
           self.get_logger().warn(f"Could not transform : {ex}")
           return
       
       self.get_logger().info(f"frame_id Robot Pose: {robot_pose_transform.header.frame_id}")
       self.get_logger().info(f"frame_id Global Plan: {self.global_plan.header.frame_id}")

def main():
    rclpy.init()
    pd_motion_planner = PDMotionPlanner()
    rclpy.spin(pd_motion_planner)
    pd_motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()