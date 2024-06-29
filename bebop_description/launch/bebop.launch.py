from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_param_list = []

    declared_param_list.append(DeclareLaunchArgument(
        "description_package",
        default_value="bebop_description",
        description="ROS2 package name"))

    declared_param_list.append(DeclareLaunchArgument(
            "description_file",
            default_value="bebop_base.urdf.xacro",
            description="URDF/XACRO description file with tshe robot."))
    
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content,value_type=str)}

    robot_state_publisher_node = Node(
        name='robot_state_publisher',
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_param_list + nodes_to_start)