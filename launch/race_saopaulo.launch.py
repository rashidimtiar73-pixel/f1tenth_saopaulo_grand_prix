import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    sim_config = os.path.join(pkg_dir, 'config', 'sim_saopaulo.yaml')
    controller_config = os.path.join(pkg_dir, 'config', 'controller_params.yaml')

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config],
        output='screen'
    )

    controller_node = Node(
        package='f1tenth_saopaulo_grand_prix',
        executable='dynamic_apex_gap_node',
        name='dynamic_apex_gap_node',
        parameters=[controller_config],
        output='screen'
    )

    return LaunchDescription([
        bridge_node,
        controller_node
    ])
