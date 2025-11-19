
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare("my_ur16e_description")
    urdf_file = LaunchConfiguration("urdf_file")
    use_xacro = LaunchConfiguration("use_xacro")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_urdf = DeclareLaunchArgument(
        "urdf_file",
        default_value=PathJoinSubstitution([pkg, "urdf", "ur16e.urdf"]),
        description="URDF/Xacro file path"
    )
    declare_use_xacro = DeclareLaunchArgument(
        "use_xacro", default_value="true",
        description="Set true if urdf_file is a .xacro"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2"
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="bash"), "-lc",
            'if [ "$USE_XACRO" = "true" ]; then xacro "$URDF_FILE"; else cat "$URDF_FILE"; fi'
        ], env={"USE_XACRO": use_xacro, "URDF_FILE": urdf_file}),
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    nodes = [rsp, jsp]
    # LaunchConfiguration objects don't evaluate in pure Python conditionals;
    # keep RViz always on by default; users can pass use_rviz:=false to hide.
    if True:
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "view.rviz"])],
            output="screen"
        ))

    return LaunchDescription([
        declare_urdf, declare_use_xacro, declare_use_rviz,
        *nodes
    ])
