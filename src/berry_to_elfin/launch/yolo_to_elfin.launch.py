from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    args = [
        DeclareLaunchArgument('detections_topic', default_value='/blackberry/detections/xyz'),
        DeclareLaunchArgument('camera_frame',     default_value='camera_link'),
        DeclareLaunchArgument('target_frame',     default_value='elfin_base'),
        DeclareLaunchArgument('workspace_root',   default_value='/home/francisco/workspace/ws_elfin_cpp'),

        DeclareLaunchArgument('class_filter',     default_value='-1'),
        DeclareLaunchArgument('min_z',            default_value='0.05'),
        DeclareLaunchArgument('max_z',            default_value='2.0'),
        DeclareLaunchArgument('offset_x_base',    default_value='0.20'),

        DeclareLaunchArgument('roll_deg',         default_value='0.0'),
        DeclareLaunchArgument('pitch_deg',        default_value='-90.0'),
        DeclareLaunchArgument('yaw_deg',          default_value='0.0'),

        DeclareLaunchArgument('wait_sec',         default_value='5.0'),
        DeclareLaunchArgument('auto_execute',     default_value='false'),
        DeclareLaunchArgument('use_gripper',      default_value='true'),

        DeclareLaunchArgument('close_force',      default_value='40'),
        DeclareLaunchArgument('close_width',      default_value='0'),
        DeclareLaunchArgument('open_force',       default_value='40'),
        DeclareLaunchArgument('open_width',       default_value='900'),
        DeclareLaunchArgument('grip_delay',       default_value='1.5'),
    ]

    params = {
        'detections_topic': LaunchConfiguration('detections_topic'),
        'camera_frame':     LaunchConfiguration('camera_frame'),
        'target_frame':     LaunchConfiguration('target_frame'),
        'workspace_root':   LaunchConfiguration('workspace_root'),

        'class_filter':   ParameterValue(LaunchConfiguration('class_filter'),   value_type=int),
        'min_z':          ParameterValue(LaunchConfiguration('min_z'),          value_type=float),
        'max_z':          ParameterValue(LaunchConfiguration('max_z'),          value_type=float),
        'offset_x_base':  ParameterValue(LaunchConfiguration('offset_x_base'),  value_type=float),

        'roll_deg':       ParameterValue(LaunchConfiguration('roll_deg'),       value_type=float),
        'pitch_deg':      ParameterValue(LaunchConfiguration('pitch_deg'),      value_type=float),
        'yaw_deg':        ParameterValue(LaunchConfiguration('yaw_deg'),        value_type=float),

        'wait_sec':       ParameterValue(LaunchConfiguration('wait_sec'),       value_type=float),
        'auto_execute':   ParameterValue(LaunchConfiguration('auto_execute'),   value_type=bool),
        'use_gripper':    ParameterValue(LaunchConfiguration('use_gripper'),    value_type=bool),

        'close_force':    ParameterValue(LaunchConfiguration('close_force'),    value_type=int),
        'close_width':    ParameterValue(LaunchConfiguration('close_width'),    value_type=int),
        'open_force':     ParameterValue(LaunchConfiguration('open_force'),     value_type=int),
        'open_width':     ParameterValue(LaunchConfiguration('open_width'),     value_type=int),
        'grip_delay':     ParameterValue(LaunchConfiguration('grip_delay'),     value_type=float),
    }

    return LaunchDescription(args + [
        Node(
            package='berry_to_elfin',
            executable='yolo_to_elfin',
            output='screen',
            parameters=[params],
        )
    ])
