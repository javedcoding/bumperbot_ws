#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose 
import rclpy.time
from tf2_ros import Buffer, TransformListener, LookupException 
from queue import PriorityQueue


class GraphNode:
    def __init__(self, x, y, cost=0, prev_node=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev_node = prev_node

    def __lt__(self, other):
        return self.cost < other.cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])
    



class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_node")
        
        ##Subscriber Nodes

        #below part is going to receive occupancy grid from the nav message and quality of service is required durability of transient local
        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)

        #below part is going to receive destination position from geometry messages
        self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        ##Publisher Nodes

        #below part is going to publish the calculated path for the odometry to receive
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)

        #below part is going to publish visualization simulation for occupancy grid where the robot has already visited
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", map_qos)

        self.map_ = None #used for planning and be changed accordingly initially none
        self.visited_map_ = OccupancyGrid() #this will hold all the map positions or grids the robot has already visited

        #Below is to retrieve the robot's position in the simulation map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def map_callback(self, map_msg:OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id #this gives the map frame id of the simulation map
        self.visited_map_.info = map_msg.info #this gives only resolution and infor about simulation map

    
    def goal_callback(self, pose:PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No Map received!")
            return
        
        self.visited_map_.data = [-1]*(self.map_.info.height * self.map_.info.width) #this will be used to make the map empty (reset) or all values will be -1 of grids
        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(self.map_.header.frame_id, "base_footprint", rclpy.time.Time()) #gives the current position
        except LookupException:
            self.get_logger().error("Could not transform from map to base_footprint")
            return
        
        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose) #calling the Dijkstra path planning using intial position and destination position

        if path.poses:
            self.get_logger().info("Shortest path found")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal")

    
    def plan(self, start, goal):
        explore_direction = [(-1, 0), (1, 0), (0, 1), (0, -1)] #left, right, top, bottom 1 positions
        pending_nodes = PriorityQueue()
        visited_nodes = set()
        start_node = self.world_to_grid(start)
        pending_nodes.put(start_node)

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get() #extracts the first node of the pending nodes from the queue

            if active_node == self.world_to_grid(goal):
                break

            for dir_x, dir_y in explore_direction:
                new_node: GraphNode = active_node + (dir_x, dir_y)

                if new_node not in visited_nodes and self.pose_on_map(new_node) and self.map_.data[self.pose_to_cell(new_node) == 0]:
                    new_node.cost = active_node.cost + 1 #as the edge nodes are given 1 as costs it is currently in breath first search
                    new_node.prev_node = active_node
                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)

            self.visited_map_data[self.pose_to_cell(active_node)] = 10 #this makes the visited path value on map as green
            self.map_pub.publish(self.visited_map_)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        while active_node and active_node.prev and rclpy.ok():
            last_pose: Pose = self.grif_to_world(active_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            active_node = active_node.prev

        path.poses.reverse()
        return path



    def grid_to_world(self, node:GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose


    def world_to_grid(self, pose:Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)
    

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.width
    
    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x


def main():
    rclpy.init()
    node = DijkstraPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()