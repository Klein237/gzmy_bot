# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution , Command

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    
    pkg_project_gazebo = get_package_share_directory('gzmy_bot')
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('gzmy_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
   
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])

    controller_params_file = os.path.join(get_package_share_directory('gzmy_bot'),'config','my_controllers.yaml')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty_gz.world'
        ]
        )}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[params],
        remappings=remappings,
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params]

    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params,controller_params_file]
        # parameters=[{'robot_description': robot_description_config},
        #             controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    twist_mux_params = os.path.join(get_package_share_directory('gzmy_bot'),'config','twrist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

     # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'my_bot',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            
        ])

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       #arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_gazebo, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control', default_value='false',
                              description='ROS2 control enabled if true'),
        bridge,
        ros_gz_image_bridge,
        robot_state_publisher,
        joint_state_publisher_node,

        twist_mux,

        joint_broad_spawner,
        diff_drive_spawner,


        #delayed_controller_manager,
        #delayed_diff_drive_spawner,
        #delayed_joint_broad_spawner,

        start_gazebo_ros_spawner_cmd,
        rviz
    ])
