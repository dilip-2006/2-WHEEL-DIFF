from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





def generate_launch_description():
    # Path to the URDF file
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("diff_robot"), "urdf", "four_wheel_diff_robot.urdf"]
    )
    namePackage = 'diff_robot'
    modelFileRelativePath = 'urdf/four_wheel_diff_robot.urdf'
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robotDescription,
        'use_sim_time': True}]
    )
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Set to True if using Gazebo          ,mku9897.8
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'resolution': 0.05,  # Map resolution
            'max_laser_range': 10.0,  # Maximum range of the LiDAR
        }]
    )


   

    # Joint State Publisher GUI Node (optional, for testing joints)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

   

    # RViz2 Node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("diff_robot"), "rviz", "robot.rviz"])],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
      
        
        slam_toolbox_node,
    ])