""" Launch File which takes care of spawning the ros2 controllers for the panda robot. 
    This can be used for multi-robot simulation as NAMESPACES can be defined.
"""

from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import  IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    FindExecutable
    )
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):

    #Launch Arguments
    prefix = LaunchConfiguration('prefix')
    robot_name = LaunchConfiguration('name')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_description_config_rviz = LaunchConfiguration('use_rviz')
    use_joint_pub = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui = LaunchConfiguration('use_joint_pub_gui')


    # Needed packages
    panda_moveit2_config = get_package_share_directory('panda_moveit2_config')
    panda_urdf = get_package_share_directory('panda_urdf')

    # Included Launch File
    panda_description = PathJoinSubstitution([panda_urdf, 'launch', 'panda_description.launch.py'])

    # ROS2 Controllers  
    ros2_controller_parameters = PathJoinSubstitution(
        [panda_moveit2_config, 'ros2_controllers', 'ros2_position_controller_namespaced.yaml']
        )
    

    # URDF from description file
    panda_description_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            ' ',
            PathJoinSubstitution([panda_urdf, 'urdf', 'panda.urdf.xacro']), 
            ' ',
            'name:=', robot_name,
            ' ',
            'prefix:=', prefix,
            ' ',
            'ros2_control_plugin:=', ros2_control_plugin,
            ' '
        ]
        ) 
    robot_description = {"robot_description": panda_description_urdf}

    # Use the Panda description launch file
    panda_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(panda_description),
        launch_arguments={'name': robot_name,
                          'prefix': prefix,
                          'ros2_control_plugin': ros2_control_plugin,
                          'use_sim_time' : use_sim_time,
                          'log_level' : log_level,
                          'use_description_config_rviz' : use_description_config_rviz,
                          'use_joint_pub' : use_joint_pub,
                          'use_joint_pub_gui' : use_joint_pub_gui,
                          }.items()
        )
    
    # Ros2_control_node (only for fake controller)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="log",
        arguments=["--ros-args", "--log-level", log_level],
        namespace= robot_name,
            parameters=[
                # {
                #     # 'robot_description' : ParameterValue(robot_description_launch_arg, value_type=str)
                #     # LaunchConfiguration('robot_description')
                # },
                {"use_sim_time": use_sim_time},
                ros2_controller_parameters,
                robot_description
            ],
            condition=(
                IfCondition(
                    PythonExpression(["'",ros2_control_plugin,"'"," == ","'fake'",]))
                ),
        )

    panda_arm_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=['-c',  f'{robot_name.perform(context)}/controller_manager', 'panda_arm_controller', 
                       "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        )

    panda_hand_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=['-c', f'{robot_name.perform(context)}/controller_manager', 'panda_hand_controller', 
                       "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],

        )
    
    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=['-c', f'{robot_name.perform(context)}/controller_manager', 'joint_state_broadcaster', 
                       "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    
    return [
        panda_description_launch,
        controller_manager_node,
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[panda_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=panda_arm_controller,
                on_exit=[panda_hand_controller],
            )
        )
        # panda_arm_controller,
        # panda_hand_controller,
        # joint_state_broadcaster
    ]



def generate_launch_description():
    
    declared_arguments = generate_declared_arguments()

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            'name',
            default_value='panda',
            description="Name of the robot",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed then also joint names in the controllers' configuration \
            have to be updated.",
        ),
        DeclareLaunchArgument(
            'ros2_control_plugin',
            default_value='fake',
            description="The ros2_control plugin that should be loaded for the manipulator ('fake' or 'ign').",
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Level of logging for the ros2_nodes. Possible args ("debug", "info", "warn", "error", "fatal").'
        ),
        DeclareLaunchArgument(
            'use_description_config_rviz',
            default_value='false',
            choices=('true', 'false'),
            description='launches RViz description config if set to `true`.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        ),
        DeclareLaunchArgument(
            'use_joint_pub',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, launches the joint_state_publisher node.'
        ),
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, launches the joint_state_publisher GUI.'
        ),
    ]

