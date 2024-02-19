# Copyright (c) 2023 PickNik, Inc.
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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# needed for including other launch files
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")

    launch_arguments = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Kinova arm camera TF
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_camera_publisher",
        output="log",
        # parameters=[{"--x":"0.", "--y":"0.05", "--z":"0.0", "--roll":"0.0", "--pitch":"0.0", "--yaw":"3.14159", 
        #              "--frame-id":"end_effector_link", "--child-frame-id":"camera_depth_frame"}],
        # for some reason the eef_link->camera_link static pub seems to take yaw-pitch-roll order
        # instead of roll-pitch-yaw 
        arguments=["0.0", "0.05", "0.0", "3.14159", "0.0", "0", "end_effector_link", "camera_link"],
        # eef_link->camera_depth_frame static pub takes roll-pitch-yaw order
        # arguments=["0.0", "0.05", "0.0", "0.0", "0.0", "3.14159", "end_effector_link", "camera_depth_frame"],
    )

    # Map offset from World/base_link frame TF
    static_tf_map_offset = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_offset_publisher",
        output="log",
        # parameters=[{"--x":"-1.0", "--y":"-1.0", "--z":"-0.5", "--roll":"0.0", "--pitch":"0.0", "--yaw":"0.0", 
        #              "--frame-id":"world", "--child-frame-id":"map_offset"}],
        arguments=["-1.0", "-1.0", "-0.5", "0.0", "0.0", "0.0", "world", "map_offset"],
    )

    # ros2_vision_node = Node(
    #     package="kinova_vision",
    #     executable="kinova_vision.launch.py",
    #     parameters=[],
    #     output="both",
    # )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        # condition=IfCondition(use_internal_bus_gripper_comm),
    )

    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "/controller_manager"],
    )

    # rviz with moveit configuration and camera params
    rviz_config_file = (
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config")+"/config/nbv.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        fault_controller_spawner,
        move_group_node,
        static_tf,
        static_tf_camera,
        static_tf_map_offset,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use arm's internal gripper connection",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_external_cable",
            default_value="false",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    

    # Get the path to the package containing the camera launch file
    camera_pkg_share_dir = FindPackageShare('kinova_vision')
    camera_launch_file_path = PythonExpression([
        "'", camera_pkg_share_dir, "/launch/kinova_vision.launch.py'"
    ])

    # declare the necessary arguments for the camera launch file, i.e. 'device' arg should be the robot ip
    # default robot ip is 192.168.1.10
    camera_ip_launch_arg = DeclareLaunchArgument('device', 
                                                 default_value='192.168.1.10', 
                                                 description="robot camera IP",
                                                 )

    # create camera launch description
    camera_launch_file_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file_path),
        launch_arguments={'device': LaunchConfiguration('device')}.items(),
    )

    return LaunchDescription(declared_arguments + 
                             [camera_ip_launch_arg, camera_launch_file_include] + 
                             [OpaqueFunction(function=launch_setup)])
