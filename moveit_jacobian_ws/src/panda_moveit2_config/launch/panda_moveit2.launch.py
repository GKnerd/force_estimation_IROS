""" Configure and setup MOVE GROUP for planning with MoveIt 2 for robot simulation of Panda.
"""

import os
import yaml
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    )
from launch_ros.actions import Node


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

    #Paths to needed packages
    panda_moveit2_config = get_package_share_directory('panda_moveit2_config')
    panda_urdf = get_package_share_directory('panda_urdf')

    # Included Launch File
    panda_ros2_controllers_launch_file = PathJoinSubstitution([panda_moveit2_config, 'launch', 'panda_ros2_controllers.launch.py'])

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

    # SRDF
    _robot_description_semantic_xml = os.path.join(panda_moveit2_config, 'srdf' ,'Panda.srdf')
    with open(_robot_description_semantic_xml, 'r') as f:
        _robot_semantic_description = f.read()

    robot_description_semantic = {"robot_description_semantic": _robot_semantic_description}

    ### PARAMS SPECIFIC TO MOVE GROUP ####

    # Panda Kinematics
    _robot_description_kinematics_yaml = load_yaml(
        panda_moveit2_config, os.path.join("config", "kinematics.yaml")
    )
    
    robot_description_kinematics = {
        "robot_description_kinematics": _robot_description_kinematics_yaml
    }

    # Panda Joint limits
    joint_limits = {
        "robot_description_planning": load_yaml(
            panda_moveit2_config, os.path.join("config", "joint_limits.yaml")
        )
    }

    # Planning pipeline
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.31416,
        },
    }
    _ompl_yaml = load_yaml(
        panda_moveit2_config, os.path.join("config", "ompl_planning.yaml")
    )
    planning_pipeline["ompl"].update(_ompl_yaml)

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # MoveIt controller manager
    moveit_controller_manager_yaml = load_yaml(
        panda_moveit2_config, os.path.join("config", "moveit_controller_manager_namespaced.yaml")
    )
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controller_manager_yaml,
    }

    # Trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    rviz_config= os.path.join(
                get_package_share_directory("panda_moveit2_config"),
                "rviz",
                "moveit_panda_sim.rviz",
            )
    
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    remappings = [
        (
            f'{robot_name.perform(context)}/get_planning_scene',
            f'/{robot_name.perform(context)}/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name.perform(context)}/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name.perform(context)}/gripper_controller/follow_joint_trajectory'
        ),
    ]
    
    panda_ros2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(panda_ros2_controllers_launch_file),
        launch_arguments={  'name': robot_name,
                            'prefix': prefix,
                            'ros2_control_plugin': ros2_control_plugin,
                            'use_sim_time' : use_sim_time,
                            'log_level' : log_level,
                            'use_description_config_rviz' : use_description_config_rviz,
                            'use_joint_pub' : use_joint_pub,
                            'use_joint_pub_gui' : use_joint_pub_gui
                        }.items()
    )

           # move_group (with execution)
    move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
            {
                'planning_scene_monitor_options': {'robot_description':'robot_description',
                                                   'joint_state_topic': f'{robot_name.perform(context)}/joint_states',
                },
            },

                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                moveit_controller_manager,
                move_group_capabilities,
                {"use_sim_time": use_sim_time},
            ],
            remappings= remappings
        )
            # rviz2
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                planning_pipeline,
                joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            remappings= remappings
        )



    return [
        panda_ros2_controllers_launch,
        move_group_node,
        rviz_node
    ]

def generate_launch_description():
    
    declared_arguments = generate_declared_arguments()

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def load_yaml(package_share_dir: str, file_path: str):
    """
    Load yaml configuration based on package share directory and file path relative to its share.
    """
    absolute_file_path = os.path.join(package_share_dir, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
    
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
