""" Launch File to view the Panda model (URDF) in rviz2.
    
    This launch file uses the panda_description launch file found in the same
    directory, a more modular approach making the individual launch files reusable. 
    Still relatively simple.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:

    # Packages
    panda_urdf = get_package_share_directory(package_name='panda_urdf')

    # Paths
    panda_description = PathJoinSubstitution([panda_urdf, 'launch', 'panda_description.launch.py'])

    # Launch Arguments
    name = 'panda'
    ros2_control_plugin = 'fake'
    use_sim_time = 'false'
    log_level = 'info'
    use_description_config_rviz= 'true'
    use_joint_pub = 'false'
    use_joint_pub_gui = 'true'

    launch_descriptions = [

        IncludeLaunchDescription(PythonLaunchDescriptionSource(panda_description),
        launch_arguments=[('name', name),
                          ('ros2_control_plugin', ros2_control_plugin),
                          ('use_sim_time', use_sim_time),
                          ('log_level', log_level),
                          ('use_rviz', use_description_config_rviz),
                          ('use_joint_pub', use_joint_pub),
                          ('use_joint_pub_gui', use_joint_pub_gui),
                          ]),
    ]

    return LaunchDescription(launch_descriptions)