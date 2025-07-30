"""Configure and setup move group for planning with MoveIt 2 for single robot simulation"""

import os
import yaml
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    )
from launch_ros.actions import Node



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

def generate_launch_description():
    
    declared_arguments = generate_declared_arguments()

    #Launch Arguments
    prefix= LaunchConfiguration('prefix')
    name = LaunchConfiguration('name')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')


    #Paths
    panda_urdf = get_package_share_directory('panda_urdf')
    panda_moveit2_config = get_package_share_directory('panda_moveit2_config')

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            ' ',
            PathJoinSubstitution([panda_urdf, 'urdf', 'panda.urdf.xacro']),
            ' ',
            'name:=', name,
            ' ',
            'prefix:=', prefix,
            ' ',
            'ros2_control_plugin:=', ros2_control_plugin,
            ' '
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = os.path.join(panda_moveit2_config, 'srdf/Panda.srdf')
    with open(_robot_description_semantic_xml, 'r') as f:
        _robot_semantic_description = f.read()

    robot_description_semantic = {"robot_description_semantic": _robot_semantic_description}

    # Kinematics
    _robot_description_kinematics_yaml = load_yaml(
        panda_moveit2_config, os.path.join("config", "kinematics.yaml")
    )
    robot_description_kinematics = {
        "robot_description_kinematics": _robot_description_kinematics_yaml
    }

    # Joint limits
    joint_limits = {
        "robot_description_planning": load_yaml(
            panda_moveit2_config, os.path.join("config", "joint_limits.yaml")
        )
    }

    # Servo
    servo_params = {
        "moveit_servo": load_yaml(
            panda_moveit2_config, os.path.join("config", "servo.yaml")
        )
    }

    servo_params["moveit_servo"].update({"use_gazebo": use_sim_time})
 
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
        panda_moveit2_config, os.path.join("config", "moveit_controller_manager.yaml")
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

    # controller_parameters = PathJoinSubstitution(
    #     [panda_moveit2_config, 'config', 'ros2_position_controller_w_mimic_joints.yaml']
    #     )
    
    controller_parameters = PathJoinSubstitution(
        [panda_moveit2_config, 'config', 'ros2_position_controller.yaml']
        )

    rviz_config= os.path.join(
                get_package_share_directory("panda_moveit2_config"),
                "rviz",
                "moveit_panda_sim.rviz",
            )
    

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 50.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # ros2_control_node (only for fake controller)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],

            parameters=[
                robot_description,
                controller_parameters,
                {"use_sim_time": use_sim_time},
            ],
            condition=(
                IfCondition(
                    PythonExpression(
                        [
                            "'",
                            ros2_control_plugin,
                            "'",
                            " == ",
                            "'fake'",
                        ]
                    )
                )
            ),
        ),

        # move_group (with execution)
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
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
        ),
        # move_servo
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                servo_params,
                {"use_sim_time": use_sim_time},
            ],
        ),

        # rviz2
        Node(
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
        ),
    ]

    # Add nodes for loading controllers
    for controller in moveit_controller_manager_yaml["controller_names"] + ["joint_state_broadcaster"]:
        nodes.append(
            # controller_manager_spawner
            Node(
                package="controller_manager",
                executable="spawner",
                output="log",
                arguments=[controller, "--ros-args", "--log-level", log_level],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        )
    
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

