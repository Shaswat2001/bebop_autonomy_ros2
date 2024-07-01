# bebop_node_launch.py
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='bebop_pipeline',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='bebop_driver',
                    plugin='bebop_driver::BebopDriverComponent',
                    name='bebop_driver_component')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])