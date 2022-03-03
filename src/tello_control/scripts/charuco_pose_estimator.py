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
from geometry_msgs.msg import TransformStamped, Accel # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster
import os
import time
import sys 
from tello_control.kalman import Kalman
from tello_control.msg import TelloPose 
 
# Import Python libraries
import cv2 # OpenCV library
import cv2.aruco as aruco
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

    topic_name = "/video_frames"
    self.dictionary=aruco.Dictionary_get(aruco.DICT_5X5_1000)
    self.board = aruco.CharucoBoard_create(
        squaresX=5, 
        squaresY=7, 
        squareLength=0.04, 
        markerLength=0.02, 
        dictionary=self.dictionary)
    self.arucoParams = aruco.DetectorParameters_create()

    
    #CHARUCO BOARD CALIBRATION (COMPUTER)
    self.mtx = np.array([
    [743.41116567, 0.          , 479.16745299],
     [0.          , 742.16273303, 269.83681487],
     [0.          , 0.          , 1.          ]])
    self.dst = np.array([[ 0.24915784, -0.60878258,  0.00273825,  0.0115768,   0.52518434]])

    #TELLO DRONE CALIBRATION
    # self.mtx = np.array([
    #   [1.41751417e+03, 0.00000000e+00, 5.73407595e+02],
    #   [0.00000000e+00, 1.42339298e+03, 3.92504178e+02],
    #   [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    # self.dst = np.array([[ 1.00585204e+00, -3.01089540e+01,  9.82743988e-03, -1.41835250e-02,
    #             2.87673404e+02]])
     
    self.t_prev = float(time.time())

      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      TelloPose, 
      topic_name, 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.pose_pub = self.create_publisher(Accel, 'pose', 1)
    self.pos = Accel()
    # Initialize the transform broadcaster
    self.tfbroadcaster = TransformBroadcaster(self)
       
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()


    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data.img)
     
    # Detect ArUco markers in the video frame
    (corners, ids, rejected) = aruco.detectMarkers(
      current_frame, self.dictionary, parameters=self.arucoParams,
      cameraMatrix=self.mtx, distCoeff=self.dst)
    aruco.refineDetectedMarkers(current_frame, self.board, corners, ids, rejected)

    # Check that at least one ArUco marker was detected
    if ids is not None:
      try:
        charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, current_frame, self.board)
        im_with_charuco_board = aruco.drawDetectedCornersCharuco(current_frame, charucoCorners, charucoIds, (0,255,0))
        retval, rvecs, tvecs = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self.board, self.mtx, self.dst,np.empty(1),np.empty(1))  # posture estimation from a charuco board
        if retval == True:
          pass
            #im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, self.mtx, self.dst, rvecs, tvecs, 100)  # axis length 100 can be changed according to your requirement    
          

        # Store the translation (i.e. position) information
        # t.transform.translation.x = tvecs[i][0][0]
        # t.transform.translation.y = tvecs[i][0][1]
        # t.transform.translation.z = tvecs[i][0][2]
        # Store the rotation information
        #rotation_matrix = np.eye(4)
        #rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        #r = R.from_matrix(rotation_matrix[0:3, 0:3])
        #quat = r.as_quat()   
          
        # Quaternion format     
        # t.transform.rotation.x = quat[0] 
        # t.transform.rotation.y = quat[1] 
        # t.transform.rotation.z = quat[2] 
        # t.transform.rotation.w = quat[3] 


      except Exception as e:
          print(e)

    # Create the coordinate transform
    # t.header.stamp = self.get_clock().now().to_msg()
    # t.header.frame_id = 'camera_depth_frame'
    # t.child_frame_id = self.aruco_marker_name
    # Send the transform
    #self.tfbroadcaster.sendTransform(t) 
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