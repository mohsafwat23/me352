from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    image_publisher = Node(
        package="tello_control",
        executable="tello_image_publisher.py",
    )
    listener_node = Node(
        package="tello_control",
        executable="aruco_marker_pose_tello.py",
    )
    path_gen_node = Node(
        package="tello_control",
        executable="path_gen.py",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
    )
    ld.add_action(image_publisher)
    ld.add_action(listener_node)
    ld.add_action(path_gen_node)
    ld.add_action(rviz_node)
    return ld