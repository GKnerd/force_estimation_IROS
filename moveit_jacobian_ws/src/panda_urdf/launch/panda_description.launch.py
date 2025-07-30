""" Launch File to launch the description of the Panda Robot. 

    Later used inside simulations as a sub-launch file. 
"""
# TODO: Still monolithic as in the way the urdf is loaded in, make it more flexible by using 
# maybe a function, which loads the robot_description in dynamically, so it does not have to 
# be re-used for every launch file where the robot description is needed.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node 
from launch.substitutions import FindExecutable, LaunchConfiguration

from typing import List

def generate_launch_description():

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all launch arguments
    robot_name = LaunchConfiguration('name')
    prefix = LaunchConfiguration('prefix')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_description_config_rviz = LaunchConfiguration('use_description_config_rviz')
    use_joint_pub = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui = LaunchConfiguration('use_joint_pub_gui')

    # Necessary packages
    panda_urdf = get_package_share_directory(package_name='panda_urdf')
    
    # Panda rviz2 config
    rviz_config = PathJoinSubstitution([panda_urdf, 'rviz', 'panda_view.rviz'])
    
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
     
    # Default robot description
    robot_description = {'robot_description': panda_description_urdf}

    nodes = [

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=robot_name,
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                robot_description,
                ],
            ),

        # Joint State Publisher Node
        Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_name,
        output={'both': 'log'},
        condition=IfCondition(use_joint_pub),
        ),

        # Joint_state_publisher_gui Node
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="both",
            namespace=robot_name,
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],  
            condition=IfCondition(use_joint_pub_gui),
        ),

        #Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output="screen",
            namespace=robot_name,
            arguments=[
                '--display-config', rviz_config,
                '--ros-args', '--log-level', log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_description_config_rviz)
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


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
            default_value='true',
            choices=('true', 'false'),
            description='launches the description RViz config if set to `true`.',
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
