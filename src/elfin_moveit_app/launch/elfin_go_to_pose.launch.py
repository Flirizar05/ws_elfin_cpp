from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro

def generate_launch_description():
    # --- Argumentos CLI ---
    x_arg         = LaunchConfiguration("x")
    y_arg         = LaunchConfiguration("y")
    z_arg         = LaunchConfiguration("z")
    roll_deg_arg  = LaunchConfiguration("roll_deg")
    pitch_deg_arg = LaunchConfiguration("pitch_deg")
    yaw_deg_arg   = LaunchConfiguration("yaw_deg")

    # --- Archivos URDF y SRDF ---
    elfin_desc = get_package_share_directory("elfin_description")
    urdf = os.path.join(elfin_desc, "urdf", "elfin3.urdf.xacro")

    elfin_moveit = get_package_share_directory("elfin3_ros2_moveit2")
    srdf = os.path.join(elfin_moveit, "config", "elfin3.srdf")

    robot_description = xacro.process_file(urdf).toxml()
    with open(srdf, "r") as f:
        robot_description_semantic = f.read()

    # --- Nodo principal ---
    node = Node(
        package="elfin_moveit_app",
        executable="elfin_moveit_app",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
             "robot_description_semantic": robot_description_semantic},

            # Forzar tipo float
            {"x":         ParameterValue(x_arg,         value_type=float),
             "y":         ParameterValue(y_arg,         value_type=float),
             "z":         ParameterValue(z_arg,         value_type=float),
             "roll_deg":  ParameterValue(roll_deg_arg,  value_type=float),
             "pitch_deg": ParameterValue(pitch_deg_arg, value_type=float),
             "yaw_deg":   ParameterValue(yaw_deg_arg,   value_type=float)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("x",         default_value="-0.3"),
        DeclareLaunchArgument("y",         default_value="0.0"),
        DeclareLaunchArgument("z",         default_value="0.7"),
        DeclareLaunchArgument("roll_deg",  default_value="0.0"),
        DeclareLaunchArgument("pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("yaw_deg",   default_value="0.0"),
        node,
    ])
