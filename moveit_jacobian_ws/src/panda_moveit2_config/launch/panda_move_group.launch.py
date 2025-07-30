""" Configure and setup MOVE GROUP for planning with MoveIt 2 for robot simulation of Panda.
"""

import os
import yaml
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
    )
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):

   #Launch Arguments
    prefix= LaunchConfiguration('prefix')
    robot_name = LaunchConfiguration('name')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    robot_description_launch_arg = LaunchConfiguration('robot_description_launch_arg')
    robot_description_semantic_launch_arg = LaunchConfiguration('semantic_robot_description_launch_arg')
    use_local_robot_description= LaunchConfiguration('use_local_robot_description')
    use_local_semantic_robot_description= LaunchConfiguration('use_local_semantic_robot_description')


    # Package Paths
    panda_urdf = get_package_share_directory('panda_urdf')
    panda_moveit2_config = get_package_share_directory('panda_moveit2_config')
    hrc_space_bringup = get_package_share_directory('hrc_space_bringup')

    # ---------------------------------------------------------------------------------------------- #    
    # URDF
    _robot_description_xml = Command(
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
    local_robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = os.path.join(panda_moveit2_config, 'srdf' '/Panda.srdf')
    with open(_robot_description_semantic_xml, 'r') as f:
        _robot_semantic_description = f.read()

    local_robot_description_semantic = {"robot_description_semantic": _robot_semantic_description}


   # Conditions
    use_local_robot_description_condition = IfCondition(PythonExpression(["'", use_local_robot_description, "' == 'true'"]))
    use_local_semantic_robot_description_condition = IfCondition(PythonExpression(["'", use_local_semantic_robot_description, "' == 'true'"]))

    # Use local or provided robot descriptions
    robot_description = local_robot_description if use_local_robot_description_condition.evaluate(context) else {"robot_description": robot_description_launch_arg.perform(context)}
    robot_description_semantic = local_robot_description_semantic if use_local_semantic_robot_description_condition.evaluate(context) else {"robot_description_semantic": robot_description_semantic_launch_arg.perform(context)}
    
    print(robot_description)
    # ---------------------------------------------------------------------------------------------- #    
    
    ### PARAMS SPECIFIC TO MOVE GROUP ####

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
    #### END OF PARAMS SPECIFIC TO MOVE GROUP ####


    # Robot State Publisher Node
    robot_state_publisher =  Node(
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
            condition=[
                IfCondition(PythonExpression(["'", use_local_robot_description, "' == 'true'"]))
            ]
            )

    #### PARAMS FOR ROS2 CONTROLLERS ####

    # ROS2 Controllers  
    ros2_controller_parameters = PathJoinSubstitution(
        [hrc_space_bringup, 'config', 'joint_ros2_position_controllers.yaml']
        )

    #### END OF PARAMS FOR ROS2 CONTROLLERS ####


    #### ROS2 CONTROLLER NODES #####

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
    
    ###### MOVE GROUP NODE

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
            robot_state_publisher,
            controller_manager_node,
            rviz_node,
            controller_manager_node,
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[panda_arm_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=panda_arm_controller,
                    on_exit=[panda_hand_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=panda_hand_controller,
                    on_exit=[move_group_node],
                )
            ),

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
            'robot_description_launch_arg',
            default_value='',
            description='Robot description included from another launch file, a URDF file.'
        ),
        DeclareLaunchArgument(
            'use_local_robot_description',
            default_value='true',
            description='Use the robot description (URDF) specified in this file and not from the launch argument.'
        ),
        DeclareLaunchArgument(
            'semantic_robot_description_launch_arg',
            default_value='',
            description='Semantic robot description included from another launch file, a SRDF file.'
        ),
        DeclareLaunchArgument(
            'use_local_semantic_robot_description',
            default_value='true',
            description='Use the semantic robot description (SRDF) specified in this file and not from the launch argument.'
        )
    ]