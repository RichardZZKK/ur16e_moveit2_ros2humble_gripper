#!/usr/bin/env python3
"""
Suction Gripper MTP Bringup Launch File for Simulation
用于在RViz中进行模拟测试的启动文件
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    use_suction_gripper_arg = DeclareLaunchArgument(
        'use_suction_gripper',
        default_value='true',
        description='Use suction gripper MTP server instead of basic move_to_pose_server'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='Robot IP address'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware for simulation'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_suction_gripper = LaunchConfiguration('use_suction_gripper')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
    # ========== UR Control ==========
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "description_file": PathJoinSubstitution([
                FindPackageShare('ur16e_with_gripper_description'), 'urdf', 'ur16e_with_suction_gripper.xacro'
            ]),
        }.items()
    )
    
    # ========== MoveIt ==========
    moveit = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
                ]),
                launch_arguments={
                    "ur_type": "ur16e",
                    "launch_rviz": launch_rviz,
                    "use_sim_time": use_sim_time,
                    "description_file": PathJoinSubstitution([
                        FindPackageShare('ur16e_with_gripper_description'), 'urdf', 'ur16e_with_suction_gripper.xacro'
                    ]),
                }.items()
            )
        ]
    )
    
    # ========== Robot Description for Action Server ==========
    ur_type = "ur16e"
    
    # Load URDF
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
    
    # Load SRDF
    ur_moveit_pkg = get_package_share_directory("ur_moveit_config")
    srdf_xacro_path = os.path.join(ur_moveit_pkg, "srdf", "ur.srdf.xacro")
    
    robot_description_semantic_raw = xacro.process_file(
        srdf_xacro_path,
        mappings={"name": "ur"},
    ).toxml()
    
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_raw
    }
    
    # Load kinematics
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
    
    # ========== Suction Gripper MTP Server ==========
    suction_gripper_server = TimerAction(
        period=5.0,  # Wait for MoveIt to be ready
        actions=[
            Node(
                package="ur16e_move_server",
                executable="suction_gripper_mtp_server",
                name="suction_gripper_mtp_server",
                output="screen",
                parameters=[
                    {
                        # Basic configuration
                        "robot_prefix": "",  # Empty for single arm setup
                        "planning_group": "ur_manipulator",
                        "end_effector_link": "rws_gripper_tcp",  # 修改为实际的link名称
                        
                        # Motion parameters
                        "pick_place_height": 0.30,
                        "look_height": 0.23,
                        "object_clearance": 0.05,
                        "eef_step": 0.005,
                        "cartesian_min_fraction": 0.90,
                        
                        # Force/Torque parameters (for simulation, these won't be used)
                        "ft_setpoint": 64.0,
                        "ft_error_allowance": 1.0,
                        "velocity_z": -0.015,
                        "ft_control_rate": 30.0,
                        
                        # Gripper IO parameters (for simulation, these won't be used)
                        "gripper_io_fun": 1,
                        "gripper_io_pin": 12,
                        "gripper_activate_state": 1,
                        "gripper_deactivate_state": 0,
                        "gripper_activation_delay": 1.0,
                        
                        # Controller names (adjust based on your setup)
                        "scaled_joint_trajectory_controller": "scaled_joint_trajectory_controller",
                        "forward_position_controller": "forward_position_controller",
                        
                        # Pre-action pose
                        "use_pre_action_pose": False,
                        
                        # Perception correction offsets
                        "perception_offset_x": -0.01,
                        "perception_offset_y": -0.005,
                        
                        # Simulation mode flag (you can add this to skip FT/IO operations)
                        "use_sim_time": use_sim_time,
                    },
                    robot_description_kinematics,
                ],
                condition=IfCondition(use_suction_gripper),
            )
        ]
    )
    
    # ========== Basic Move To Pose Server (Alternative) ==========
    basic_move_server = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ur16e_move_server",
                executable="move_to_pose_server",
                name="move_to_pose_server",
                output="screen",
                parameters=[
                    {
                        "planning_group": "ur_manipulator",
                        "end_effector_link": "suction_gripper_tcp",
                        "approach_height": 0.10,
                        "eef_step": 0.005,
                    },
                    robot_description_kinematics,
                ],
                condition=IfCondition(
                    # Use NOT condition - launch basic server only if not using suction gripper
                    # This is a workaround since launch doesn't have NotCondition
                    LaunchConfiguration('use_suction_gripper', default='false')
                ),
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        launch_rviz_arg,
        use_suction_gripper_arg,
        robot_ip_arg,
        use_fake_hardware_arg,
        
        # Nodes
        ur_control,
        moveit,
        suction_gripper_server,
        # basic_move_server,  # Uncomment if you want the option to switch
    ])
