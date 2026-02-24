from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Force sensor node
        Node(
            package='netft_reader',
            executable='netft_raw_node',  #
            name='netft_raw_node',
            output='screen'
        ),

        # NDI Aurora sensor node
        Node(
            package='ndi_aurora_node',
            executable='ndi_aurora_raw_node',
            name='ndi_aurora_raw_node',
            output='screen'
        )
    ])
