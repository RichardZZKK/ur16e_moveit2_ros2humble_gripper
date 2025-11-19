#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro

def generate_launch_description():
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": "192.168.1.100",
            "use_fake_hardware": "true",
            "initial_joint_controller": "joint_trajectory_controller",
            "description_file": PathJoinSubstitution([
                FindPackageShare('ur16e_with_gripper_description'), 'urdf', 'ur16e_with_suction_gripper.xacro'
            ]),
        }.items()
    )

    moveit = TimerAction(period=2.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
            ]),
            launch_arguments={
                "ur_type": "ur16e",
                "launch_rviz": "true",
                "description_file": PathJoinSubstitution([
                    FindPackageShare('ur16e_with_gripper_description'), 'urdf', 'ur16e_with_suction_gripper.xacro'
                ]),
            }.items()
        )
    ])

    ur_type = "ur16e"

    ur_description_pkg = get_package_share_directory("ur_description")
    urdf_xacro_path = os.path.join(ur_description_pkg, "urdf", "ur.urdf.xacro")

    robot_description_raw = xacro.process_file(
        urdf_xacro_path,
        mappings={
            "ur_type": ur_type,
            "name": "ur",
            "prefix": "",
        },
    ).toxml()

    robot_description = {"robot_description": robot_description_raw}

    ur_moveit_pkg = get_package_share_directory("ur_moveit_config")
    srdf_xacro_path = os.path.join(ur_moveit_pkg, "srdf", "ur.srdf.xacro")

    robot_description_semantic_raw = xacro.process_file(
        srdf_xacro_path,
        mappings={"name": "ur"},
    ).toxml()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_raw
    }

    kinematics_yaml_path = os.path.join(ur_moveit_pkg, "config", "kinematics.yaml")

    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "ur_manipulator": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
            }
        }
    }

    action_server = TimerAction(period=2.0, actions=[
        Node(
            package="ur16e_move_server",
            executable="move_to_pose_server",
            output="screen",
            parameters=[
                {
                    "planning_group": "ur_manipulator",
                    "end_effector_link": "rws_gripper_tcp",
                    "approach_height": 0.10,
                    "eef_step": 0.005,
                },
                # robot_description,
                # robot_description_semantic,
                robot_description_kinematics,
                ],
        )
    ])

    return LaunchDescription([ur_control, moveit, action_server])
