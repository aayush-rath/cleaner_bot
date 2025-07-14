import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('cleaner_bot_description')
    world_path = os.path.join(pkg_share, 'world', 'kitchen_world.sdf')
    model_path = os.path.join(pkg_share, 'models')

    gz_sim_launch_path = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    env_set = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value= model_path + ':' +
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    declare_world_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": ['-r ', world_path],
            "on_exit_shutdown": "True"
        }.items()
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    return LaunchDescription([
        env_set,
        declare_world_path,
        ros_gz_bridge_node
    ])