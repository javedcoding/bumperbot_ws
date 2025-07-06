#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, inverse_matrix

class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner")
        self.declare_parameter("kp", 2.0) #this is a proportional gain parameter and default is 2
        self.declare_parameter("kd", 0.1) #this is a derivative gain parameter and default is 0.1
        self.declare_parameter("step_size", 0.2) #this is the minimum distance the robot wants to go before checking and default is 0.2
        self.declare_parameter("max_linear_velocity", 0.3) #this is a max linear velocity robot wants to gain and default is 0.3
        self.declare_parameter("max_angular_velocity", 1.0) #this is a max angular velocity robot wants to gain and default is 1.0

        self.kp = self.get_parameter("kp").value #class variable kp is proportional gain
        self.kd = self.get_parameter("kd").value #class variable kd is derivative gain
        self.step_size = self.get_parameter("step_size").value #class variable step_size
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value #class variable max_linear_velocity
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value #class variable max_angular_velocity

        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10) #subscribe to the path topic of the a_star path planner
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #publishes messages to the cmd_vel topic to control the robot's velocity
        self.next_pose_pub = self.create_publisher(PoseStamped, "/pd/next_pose", 10) #publishes the next pose to the pd/next_pose topic for visualization

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop) #10hz frequency to check and fix listener
        self.global_plan = None
        self.prev_angular_error = 0.0 #this is used to calculate the derivative error
        self.prev_linear_error = 0.0 #this is used to calculate the derivative error
        self.last_cycle_time = self.get_clock().now() #this is used to calculate the time difference between the last cycle and current cycle

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
       
       if not self.tranfsorm_plan(robot_pose_transform.header.frame_id):
           self.get_logger().error("Unable to transform plan to robot's frame")
           return
       
       robot_pose = PoseStamped()
       robot_pose.header.frame_id = robot_pose_transform.header.frame_id
       robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
       robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
       robot_pose.pose.orientation = robot_pose_transform.transform.rotation

       next_pose: PoseStamped = self.get_next_pose(robot_pose)
       dx = next_pose.pose.position.x - robot_pose.pose.position.x
       dy = next_pose.pose.position.y - robot_pose.pose.position.y
       distance = math.sqrt(dx**2 + dy**2)

       if distance <= 0.1:
            self.get_logger().info("Robot reached the goal!")
            self.global_plan.poses.clear()
            return
       
       self.next_pose_pub.publish(next_pose)
       #here we used PID control to calculate the linear and angular velocities error but can be implementing kalman filter as well
       robot_tf = quaternion_matrix([robot_pose.pose.orientation.x,
                                     robot_pose.pose.orientation.y,
                                     robot_pose.pose.orientation.z,
                                     robot_pose.pose.orientation.w,])
       robot_tf[0][3] = robot_pose.pose.position.x
       robot_tf[1][3] = robot_pose.pose.position.y

       next_pose_tf = quaternion_matrix([next_pose.pose.orientation.x,
                                         next_pose.pose.orientation.y,
                                         next_pose.pose.orientation.z,
                                         next_pose.pose.orientation.w,])
       next_pose_tf[0][3] = next_pose.pose.position.x
       next_pose_tf[1][3] = next_pose.pose.position.y

       #Let's inverse the next pose transformation matrix to get the relative position of the next pose with respect to the robot
       next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)
       #Now get linear and angular errors
       linear_error = next_pose_robot_tf[0][3] #x position error of the next pose with respect to the robot
       angular_error = next_pose_robot_tf[1][3] #y position error of the next pose with respect to the robot
       #get the drivative errors
       dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9 #get the time difference in seconds
       linear_error_derivative = (linear_error - self.prev_linear_error) / dt if dt > 0 else 0.0
       angular_error_derivative = (angular_error - self.prev_angular_error) / dt if dt > 0 else 0.0

       #Now send the cmd_vel to the robot
       cmd_vel = Twist()
       cmd_vel.linear.x = max(-self.max_linear_velocity, min(self.kp * linear_error + self.kd * linear_error_derivative, self.max_linear_velocity))
       cmd_vel.angular.z = max(-self.max_angular_velocity, min(self.kp * angular_error + self.kd * angular_error_derivative, self.max_angular_velocity))
       self.cmd_pub.publish(cmd_vel)

       self.prev_linear_error = linear_error
       self.prev_angular_error = angular_error
       self.last_cycle_time = self.get_clock().now()


    def tranfsorm_plan(self, frame):
        if self.global_plan.header.frame_id == frame:
            return True
        try:
            transform = self.tf_buffer.lookup_transform(frame, self.global_plan.header.frame_id, rclpy.time.Time())
        except Exception as ex:
            self.get_logger().error(f"Could not transform plan from fram {self.global_plan.header.frame_id} to {frame}: {ex}")
            return False
        
        tranform_matrix = quaternion_matrix([transform.transform.rotation.x,
                                             transform.transform.rotation.y,
                                             transform.transform.rotation.z,
                                             transform.transform.rotation.w,])
        tranform_matrix[0][3] = transform.transform.translation.x
        tranform_matrix[1][3] = transform.transform.translation.y

        for pose in self.global_plan.poses:
            pose_matrix = quaternion_matrix([pose.pose.orientation.x,
                                             pose.pose.orientation.y,
                                             pose.pose.orientation.z,
                                             pose.pose.orientation.w,])
            tranform_matrix[0][3] = pose.pose.position.x
            tranform_matrix[1][3] = pose.pose.position.y
            transformed_pose = concatenate_matrices(pose_matrix, tranform_matrix)
            [pose.pose.orientation.x, pose.pose.orientation.y, 
             pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
            [pose.pose.position.x, pose.pose.position.y,
             pose.pose.position.z] = translation_from_matrix(transformed_pose)
            pose.header.frame_id = frame

        self.global_plan.header.frame_id = frame
        return True
    
    def get_next_pose(self, robot_pose: PoseStamped):
        next_pose = self.global_plan.poses[-1]
        for pose in reversed(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance > self.step_size:
                next_pose = pose
            else:
                break
        return next_pose

def main():
    rclpy.init()
    pd_motion_planner = PDMotionPlanner()
    rclpy.spin(pd_motion_planner)
    pd_motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()