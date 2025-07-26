from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to individual launch files
    gazebo_launch = os.path.join(
        get_package_share_directory('satellite_gazebo'),
        'launch',
        'simulation.launch.py'
    )
    sensors_launch = os.path.join(
        get_package_share_directory('satellite_sensors'),
        'launch',
        'sensors.launch.py'
    )
    custom_ekf_launch = os.path.join(
        get_package_share_directory('satellite_custom_ekf'),
        'launch',
        'custom_ekf.launch.py'
    )
    state_estimation_launch = os.path.join(
        get_package_share_directory('satellite_state_estimation'),
        'launch',
        'state_estimation.launch.py'
    )

    # Include launch descriptions
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': ['true']}.items()
    )
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch),
        launch_arguments={'use_sim_time': ['true']}.items()
    )
    custom_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_ekf_launch),
        launch_arguments={'use_sim_time': ['true']}.items()
    )
    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_estimation_launch),
        launch_arguments={'use_sim_time': ['true']}.items()
    )

    # Return the full launch description
    return LaunchDescription([
        gazebo,
        sensors,
        custom_ekf,
        state_estimation
    ])