import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  # urdf_file_name = 'r2d2.urdf.xml'

  # print("urdf_file_name : {}".format(urdf_file_name))

#   urdf = os.path.join(
#       get_package_share_directory('urdf_tutorial'),
#       urdf_file_name)

  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='move_group_python_interface',
          executable='move_group_python_interface_tutorial',
          name='move_group',
          output='screen'),
  ])
