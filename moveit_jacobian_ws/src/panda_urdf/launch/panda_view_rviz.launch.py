""" Launch File to view the Panda model (URDF) in rviz2.

    In this launch file the panda_urdf is used loaded directly without substitutions
    and launch arguments being used. A monolithic but less confusing approach. Also the 
    namespace argument is hard coded, which kind of defeats the purpose, but it serves as
    a good starting point.

"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    # Paths
    panda_urdf = get_package_share_directory(package_name='panda_urdf')

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
    panda_description = {"robot_description": panda_description_urdf}
    namespace = 'panda'
    
    # Panda rviz2 config
    rviz_config = PathJoinSubstitution([panda_urdf, 'rviz', 'panda_view.rviz'])
    
    #List of nodes to be launched
    nodes = [

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=namespace,
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                # {'frame_prefix': 'panda/'},
                {'use_sim_time': use_sim_time},
                panda_description,
                ],
            # remappings=[
            #     ("/tf", remapped_tf),
            #     ("/tf_static", remapped_tf_static)
            #     ]
            ),
        
        #Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output="screen",
            namespace=namespace,
            arguments=[
                '--display-config', rviz_config,
                '--ros-args', '--log-level', log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
                #         remappings=[
                # ("/tf", remapped_tf),
                # ("/tf_static", remapped_tf_static)
                # ]
            ),

        # Joint_state_publisher_gui Node
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="both",
            namespace=namespace,
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],   
                #             remappings=[
                # ("/tf", remapped_tf),
                # ("/tf_static", remapped_tf_static)
                # ]  
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
        )
    ]
