import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur16e",
            description="Type/series of used UR robot.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")

    # Get paths
    ur_description_package = FindPackageShare("ur_description")
    ur_moveit_config_package = FindPackageShare("ur_moveit_config")

    # Joint limits and kinematics config
    joint_limit_params = PathJoinSubstitution(
        [ur_description_package, "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [ur_description_package, "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [ur_description_package, "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [ur_description_package, "config", ur_type, "visual_parameters.yaml"]
    )

    # Robot description content
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([ur_description_package, "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            'prefix:=""',
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Kinematics config for MoveIt
    kinematics_yaml_file = PathJoinSubstitution(
        [ur_moveit_config_package, "config", "kinematics.yaml"]
    )

    # MTC node
    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            robot_description,
            kinematics_yaml_file,
        ],
    )

    return LaunchDescription(declared_arguments + [mtc_node])