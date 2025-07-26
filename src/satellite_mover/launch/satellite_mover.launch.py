import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression

robot_name_in_model = "camera_model"
ros_gz_sim = get_package_share_directory('ros_gz_sim')
pkg_dir = get_package_share_directory('satellite_mover')
satellite_description_dir = get_package_share_directory('satellite_description')
camera_model_dir = get_package_share_directory('camera_description')
world_path = os.path.join(pkg_dir,"world", "microgravity_world.xml")
satellite_path = os.path.join(satellite_description_dir,'urdf','sat.sdf')
camera_path = os.path.join(camera_model_dir,'urdf', 'camera_model.sdf')
bridge_params = os.path.join(pkg_dir,'config','param.yaml')
ekf_config = os.path.join(pkg_dir,'config','ekf_config_copy.yaml')
use_sim_time = LaunchConfiguration('use_sim_time', default='true')


def generate_launch_description():

    # Node to control the satellite
    move_satellite_node = Node(
        package='satellite_mover',
        executable='move_satellite',
        name='move_satellite_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    VisualOdometry_publisher_node = Node(
        package='satellite_mover',
        executable='VisualOdometry_publisher',
        name='VisualOdometry_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    DummyPublisher_node= Node(
        package='satellite_mover',
        executable='DummyPublisher',
        name='DummyPublisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        remappings=[(f"/world/default/model/{robot_name_in_model}/joint_state", "/joint_states")]
        + [
            (f"/model/{robot_name_in_model}/cmd_vel", f"/set_target_velocity")
        ],
        parameters=[
            {f"qos_overrides./model/{robot_name_in_model}/cmd_vel.subscriber.reliability": "reliable"},
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    declare_gui_param = DeclareLaunchArgument(
        name="gui", default_value="True", description="Whether to start the GUI"
    )

    declare_world_param = DeclareLaunchArgument(
        name="world", default_value=world_path, description="Full path to the world model file to load"
    )

    declare_update_rate_param = DeclareLaunchArgument(
        name="update_rate",
        default_value="0",
        description="Update rate in Hertz. \n"
        "It affect the simulation depending on the max_step_size defined in the world: \n"
        "If max_step_size=0.001, to have simulation time 2x real time an update_rate of 2000 \n"
        "is required: `max_step_size*update_rate = 0.001*2000=2` \n"
        "NOTE:update_rate is the maximum update rate but Gazebo doesn't guarantee to reach it \n"
        "the complexity of the simulation and the hw used will affect it.",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                 # Set the update rate
                " -z ",
                LaunchConfiguration("update_rate"),
                # Enable/Disable the GUI
                # PythonExpression(['"" if ', LaunchConfiguration("gui"), ' else " -s "']),
                # Start the simulation right away
                " -r ",
                # World to load
                LaunchConfiguration("world"),
                ]
            }.items()
    )

    satellite_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'satellite',
            '-file', satellite_path,
            '-x', '0.01',
            '-y', '1.5',
            '-z', '8.0',
            '-R', '-1.5708',   
            '-P', '0.0',  
            '-Y', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # camera_spawner_cmd = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', robot_name_in_model,
    #         '-file', camera_path,
    #         '-x', '0',
    #         '-y', '0',
    #         '-z', '2',
    #         '-R', '0.0',   
    #         '-P', '0.0',  
    #         '-Y', '0.0'
    #     ],
    #     output='both',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )


    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config,{'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        declare_gui_param,
        declare_world_param,
        declare_update_rate_param,
        move_satellite_node,
        gzserver_cmd,
        DummyPublisher_node,
        VisualOdometry_publisher_node,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,
        satellite_spawner_cmd,
        # camera_spawner_cmd,
        ekf_node,
        # odometry_noise_addr_node
        DeclareLaunchArgument('rviz', default_value='true',
                               description='Open RViz.'),
        # joint_state_publisher_gui,
        # robot_state_publisher,
        # # rviz
    ])
