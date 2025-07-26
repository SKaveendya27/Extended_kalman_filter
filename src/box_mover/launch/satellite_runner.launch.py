import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ros_gz_sim = get_package_share_directory('ros_gz_sim')
pkg_dir = get_package_share_directory('box_mover')
world_path = os.path.join(pkg_dir,"worlds", "space.world")
satellite_path = os.path.join(pkg_dir,"models", "box.sdf")
bridge_params = os.path.join(pkg_dir,'config','param.yaml')
use_sim_time = LaunchConfiguration('use_sim_time', default='true')


def generate_launch_description():

    # Node to control the satellite
    move_box_node = Node(
        package='box_mover',
        executable='move_box',
        name='move_box_node',
        output='screen',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    satellite_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'satellite',
            '-file', satellite_path,
            '-x', '0',
            '-y', '0',
            '-z', '2'
        ],
        output='screen',
    )

    return LaunchDescription([
        move_box_node,
        gzserver_cmd,
        start_gazebo_ros_bridge_cmd,
        satellite_spawner_cmd
    ])
