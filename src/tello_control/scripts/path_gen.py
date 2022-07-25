#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
from time import sleep
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose, PoseStamped # Handles TransformStamped message
from nav_msgs.msg import Path
import numpy as np


class PathGenNode(Node):
    """
    Create an ArucoNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('path_gen_node')
        self.path_pub = self.create_publisher(Path,'path', 10)


        self.subscription = self.create_subscription(
        Pose, 
        'pose', 
        self.listener_callback, 
        10)
        self.subscription # prevent unused variable warning

    
    def listener_callback(self, msg):
        """
        Callback function.
        This function gets called every time a message is received.
        """

        #print(msg)
        x_i = msg.position.x
        y_i = msg.position.y
        z_i = msg.position.z
        P_cam = np.array([x_i,y_i,z_i]).T
        P_marker = np.array([0,0,0]).T
        t = 0
        seq = 0
        self.path = Path()
        self.path.header.frame_id = 'camera_depth_frame'
        self.path.header.stamp = self.get_clock().now().to_msg()
        for i in range(5):
            X_t = P_cam + t*(P_marker - P_cam)
            self.posestamp = PoseStamped()
            self.posestamp.header.stamp = self.get_clock().now().to_msg()
            self.posestamp.header.frame_id = 'camera_depth_frame'
            self.posestamp.pose.position.x = X_t[0]
            self.posestamp.pose.position.y = X_t[1]
            self.posestamp.pose.position.z = X_t[2]
            self.path.poses.append(self.posestamp)
            path_fin = self.path 
            t += 0.25
            i += 1
            

        self.path_pub.publish(path_fin)
        #sleep(0.02)
        

def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  path_gen_node = PathGenNode()
   
  # Spin the node so the callback function is called.
  rclpy.spin(path_gen_node)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  path_gen_node.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
    main()