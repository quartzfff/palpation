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



# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([

#         # -------- Force sensor --------
#         Node(
#             package='netft_reader',
#             executable='netft_raw_node',
#             name='netft_raw_node',
#             output='screen'
#         ),

#         # -------- NDI sensor 1 --------
#         Node(
#             package='ndi_aurora_node',
#             executable='ndi_aurora_raw_node',
#             name='ndi_sensor_1',   # must be unique
#             output='screen',
#             parameters=[{
#                 "tool_index": 0,
#                 "topic": "/ndi_1",
#                 "frame_id": "ndi_1"
#             }]
#         ),

#         # -------- NDI sensor 2 --------
#         Node(
#             package='ndi_aurora_node',
#             executable='ndi_aurora_raw_node',
#             name='ndi_sensor_2',   # must be unique
#             output='screen',
#             parameters=[{
#                 "tool_index": 1,
#                 "topic": "/ndi_2",
#                 "frame_id": "ndi_2"
#             }]
#         ),
#     ])