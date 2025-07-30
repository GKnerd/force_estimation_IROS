import os
import yaml
from typing import List

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from ament_index_python.packages import get_package_share_directory



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

    # Package Share Directory
    panda_urdf = get_package_share_directory("panda_urdf")
    panda_moveit2_config = get_package_share_directory("panda_moveit2_config")
    
    declared_arguments = generate_declared_arguments()
    
    # Launch Arguments
    robot_joint_states_file = LaunchConfiguration('robot_joint_states_file')


    # ---------------------------- Robot Descriptions ---------------------------- # 
    # Panda URDF
    panda_description_urdf = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                ' ',
                PathJoinSubstitution([panda_urdf, 'urdf', 'panda.urdf.xacro']), 
                ' ',
                'name:=', "panda",
                ' ',
                'prefix:=', "",
                ' ',
                'ros2_control_plugin:=', "fake",
                ' '
            ]
        )
     
    panda_description = {"robot_description": panda_description_urdf}

    # SRDF
    _robot_description_semantic_xml = os.path.join(panda_moveit2_config, 'srdf/Panda.srdf')
    with open(_robot_description_semantic_xml, 'r') as f:
        _robot_semantic_description = f.read()

    panda_description_semantic = {"robot_description_semantic": _robot_semantic_description}


    # ---------------------------- MoveIT Controller Manager ---------------------------- # 

    moveit_controller_manager = load_yaml(panda_moveit2_config, os.path.join('config' '/moveit_controllers.yaml'))

    # Panda State Publisher
    panda_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[
                panda_description
                ],
    )
    # ---------------------------- MoveIT Kinematics ---------------------------- # 
    _robot_description_kinematics_yaml = load_yaml(
        panda_moveit2_config, os.path.join("config", "kinematics.yaml")
    )
    robot_description_kinematics = {
        "robot_description_kinematics": _robot_description_kinematics_yaml
    }

    #  SRDF Publisher
    panda_semantic_state_pub = Node(
                    package='moveit_ros_move_group', 
                    executable='move_group',
                    output='both',
                    parameters=[
                        panda_description_semantic,
                        panda_description,
                        moveit_controller_manager,
                        robot_description_kinematics,
                        {"publish_robot_description_semantic": True}

                        ],
    )

    # Cartesian Velocity Computation Node
    jacobian = Node(
        package='external_force_computation',
        executable='jacobian_computation',
        output='both',
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {"robot_joint_states_file": robot_joint_states_file}
        ]
    )


    return  LaunchDescription(declared_arguments + [panda_state_pub, panda_semantic_state_pub, jacobian])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            'robot_joint_states_file',
            default_value='/home/docker/robot_data/coexistence/external_force_estimates/joint_states/aggregated_joint_states.json',
            description='File containing the joint states of the robot for a given scenario.',
        )
    ]   
