#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
# Publishes a coordinate transformation between an ArUco marker and a camera
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
   
# Import the necessary ROS 2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped, Pose # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster
import os
import math
import time
import sys 
#sys.path.append("/home/mohamed/dev_ws/install/opencv_tools/lib/python3.8/site-packages/opencv_tools")
from tello_control.kalman import Kalman
from tello_control.msg import TelloPose 

 
# Import Python libraries
import cv2 # OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
 
class ArucoNode(Node):
  """
  Create an ArucoNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_node')
    
    # Declare parameters
    self.declare_parameter("aruco_dictionary_name", "DICT_ARUCO_ORIGINAL")
    self.declare_parameter("aruco_marker_side_length", 0.1815)
    self.declare_parameter("camera_calibration_parameters_filename", "/home/mohamed/dev_ws/src/opencv_tools/data/calibration_chessboard.yaml")
    self.declare_parameter("image_topic", "/drone_video_frames")
    self.declare_parameter("aruco_marker_name", "aruco_marker")
     
    # Read parameters
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    self.camera_calibration_parameters_filename = self.get_parameter(
      "camera_calibration_parameters_filename").get_parameter_value().string_value
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value
 
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))

  
    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
      self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
      
    #TELLO DRONE CALIBRATION
    self.mtx = np.array([
      [1.41751417e+03, 0.00000000e+00, 5.73407595e+02],
      [0.00000000e+00, 1.42339298e+03, 3.92504178e+02],
      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    self.dst = np.array([[ 1.00585204e+00, -3.01089540e+01,  9.82743988e-03, -1.41835250e-02,
                2.87673404e+02]])

    cv_file.release()
     
    self.t_prev = float(time.time())

    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(
      aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
    self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.


    # self.H = np.array([
    #             [1,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #             [0,1,0,0,0,0,0,0,0,0,0,0,0,0],
    #             [0,0,1,0,0,0,0,0,0,0,0,0,0,0],
    #             [0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    #             [0,0,0,0,1,0,0,0,0,0,0,0,0,0],
    #             [0,0,0,0,0,1,0,0,0,0,0,0,0,0],
    #             [0,0,0,0,0,0,1,0,0,0,0,0,0,0]
    #             ])
    self.H = np.identity(6)

    #self.Q = 0.002*np.eye(14) #process noise covariance
    self.Q = 0.002*np.eye(6) #process noise covariance
    #self.R = np.identity(7) #0.3 #measurement noise covariance
    self.R = np.identity(6) #0.3 #measurement noise covariance

    #self.x_hat_k = np.array([[-1.8,0,2,0,0,0,1,0,0,0,0,0,0,1]]).T # x_t = [T,q,T_dot,q_dot]
    self.x_hat_k = np.array([[-1.8,0.,2.,0.,0.,0.]]).T # x_t = [T,T_dot]
    #self.P_k = np.eye(14) #initialize covariance matrix
    self.P_k = np.eye(6) #initialize covariance matrix
    self.kalman = Kalman(self.H, self.R, self.Q)

    self.subscription = self.create_subscription(
      TelloPose, 
      image_topic, 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.pose_pub = self.create_publisher(Pose, 'pose', 1)
    self.pos = Pose()
    # Initialize the transform broadcaster
    self.tfbroadcaster = TransformBroadcaster(self)
       
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()


    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    #self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data.img)

    #Velocity of the drone from data
    vel_x = data.velocity.x
    vel_y = data.velocity.y
    vel_z = data.velocity.z
     
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters,
      cameraMatrix=self.mtx, distCoeff=self.dst)
    
    t_now = float(time.time())
    dT = (t_now - self.t_prev)
    
    self.x_hat_k, self.P_k = self.kalman.predict(self.x_hat_k, self.P_k, dT)
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
     
      # Draw a square around detected markers in the video frame
      cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
 
      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        self.aruco_marker_side_length,
        self.mtx,
        self.dst)
         
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      #https://github.com/opencv/opencv/issues/8813
      for i, marker_id in enumerate(marker_ids):  
 
       
        # Store the translation (i.e. position) information
        # t.transform.translation.x = tvecs[i][0][0]
        # t.transform.translation.y = tvecs[i][0][1]
        # t.transform.translation.z = tvecs[i][0][2]
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        # print("pos Rotation Matrix:",rotation_matrix)
        # if 0 < rotation_matrix[0][2] < 1:
        #   rotation_matrix[0:3, 0:3] *= np.array([
        #                 [ 1,   1,  -1],
        #                 [ 1,   1,  -1],
        #                 [ 1,  1,  1],
        #               ])
        #   print("negative Rotation Matrix:", rotation_matrix)
        #   forward = np.array([0, 0, 1])
        #   T = tvecs[i]
        #   tnorm = T / np.linalg.norm(T)
        #   axis = np.cross(tnorm, forward)
        #   angle = -2*math.acos(tnorm @ forward)
          #rotation_matrix[0:3, 0:3] = cv2.Rodrigues(angle * axis)[0] @ rotation_matrix[0:3, 0:3]

        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
         
        # Quaternion format     
        # t.transform.rotation.x = quat[0] 
        # t.transform.rotation.y = quat[1] 
        # t.transform.rotation.z = quat[2] 
        # t.transform.rotation.w = quat[3] 

        #z_k_1 = np.array([[tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2], quat[0], quat[1], quat[2], quat[3]]]).T
        z_k_1 = np.array([[tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2], vel_x, vel_y, vel_z]]).T
        self.x_hat_k, self.P_k = self.kalman.update(self.x_hat_k, self.P_k, z_k_1)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_depth_frame'
        t.child_frame_id = self.aruco_marker_name
        t.transform.translation.x = self.x_hat_k[0][0]
        t.transform.translation.y = self.x_hat_k[1][0]
        t.transform.translation.z = self.x_hat_k[2][0]
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 
        self.tfbroadcaster.sendTransform(t)
        self.pos.position.x = self.x_hat_k[0][0]
        self.pos.position.y = self.x_hat_k[1][0]
        self.pos.position.z = self.x_hat_k[2][0]
        self.pos.orientation.x = quat[0]
        self.pos.orientation.y = quat[1]
        self.pos.orientation.z = quat[2]
        self.pos.orientation.w = quat[3]
        self.pose_pub.publish(self.pos)    
          
        # Draw the axes on the marker
        cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)        

    self.t_prev = t_now   
    # Display image


    # Create the coordinate transform

    # Send the transform
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  aruco_node = ArucoNode()
   
  # Spin the node so the callback function is called.
  rclpy.spin(aruco_node)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  aruco_node.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
    main()