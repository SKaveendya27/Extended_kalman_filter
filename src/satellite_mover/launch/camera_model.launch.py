""" Documentation on how to use the launch package:
    https://docs.ros.org/en/ros2_packages/rolling/api/launch/index.html
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Constants for paths to different files and folders
    pkg_dir = get_package_share_directory('satellite_mover')
    camera_model_dir = get_package_share_directory('camera_description')
    package_name = "camera_description"
    robot_name_in_model = "camera_model"
    urdf_file_path = os.path.join(camera_model_dir,'urdf', 'rs_l515_only_hr.urdf.xacro')
    world_file_path = os.path.join(pkg_dir,"world", "microgravity_world.xml")

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    default_gui = "True"
    default_world_path = os.path.join(pkg_share, world_file_path)
    # Update rate of 0 means that Gazebo try to run the simulation as fast as possible
    default_update_rate = "0"

    declare_gui_param = DeclareLaunchArgument(
        name="gui", default_value=default_gui, description="Whether to start the GUI"
    )

    declare_world_param = DeclareLaunchArgument(
        name="world", default_value=default_world_path, description="Full path to the world model file to load"
    )

    declare_update_rate_param = DeclareLaunchArgument(
        name="update_rate",
        default_value=default_update_rate,
        description="Update rate in Hertz. \n"
        "It affect the simulation depending on the max_step_size defined in the world: \n"
        "If max_step_size=0.001, to have simulation time 2x real time an update_rate of 2000 \n"
        "is required: `max_step_size*update_rate = 0.001*2000=2` \n"
        "NOTE:update_rate is the maximum update rate but Gazebo doesn't guarantee to reach it \n"
        "the complexity of the simulation and the hw used will affect it.",
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_description = {"robot_description": xacro.process_file(urdf_model_path).toxml()}
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Start Gazebo server
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [
                # Set the update rate
                " -z ",
                LaunchConfiguration("update_rate"),
                # Enable/Disable the GUI
                PythonExpression(['"" if ', LaunchConfiguration("gui"), ' else " -s "']),
                # Start the simulation right away
                " -r ",
                # World to load
                LaunchConfiguration("world"),
            ]
        }.items(),
    )

    #Initial camera pose for floating camera with tool link added
    start_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-name", robot_name_in_model,
        "-topic", "robot_description",
        "-x", "-0.455",     # X position
        "-y", "-0.095",     # Y position
        "-z", "1.487",     # Z position
        "-R", "-3.14",     # Roll
        "-P", "-1.57",    # Pitch
        "-Y", "3.14"      # Yaw
    ],
    output="both",
    )

    # Gz - ROS Bridge
    start_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/realsense_l515_/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/realsense_l515_/depth_image@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/realsense_l515_/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Joint states (IGN -> ROS2)
            "/world/default/model/" + robot_name_in_model + "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ]
        + [
            # Joint target (IGN <-> ROS2)
            f"/model/{robot_name_in_model}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"
        ],
        remappings=[("/world/default/model/" + robot_name_in_model + "/joint_state", "/joint_states")]
        + [
            (f"/model/{robot_name_in_model}/cmd_vel", f"/set_target_velocity")
        ],
        parameters=[
            {f"qos_overrides./model/{robot_name_in_model}/cmd_vel.subscriber.reliability": "reliable"}
        ],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options(variable/configurations)
    ld.add_action(declare_gui_param)
    ld.add_action(declare_world_param)
    ld.add_action(declare_update_rate_param)

    # Add any actions
    ld.add_action(start_gazebo)
    ld.add_action(start_bridge)
    ld.add_action(start_spawn_entity)
    ld.add_action(start_robot_state_publisher)

    return ld
