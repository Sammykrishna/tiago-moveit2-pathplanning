from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_param_path = PathJoinSubstitution(
        [FindPackageShare("moveit_wrapper"), "params", "tiago.yaml"]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("moveit_wrapper"), "params", "default.rviz"]
    )

    # Arguments
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_path,
        description="Path to the YAML parameter file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node and container",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", default_value="false", description="Whether to start RViz"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config_path,
        description="Path to the RViz config file",
    )

    # Node container and (optionally) RViz
    container = ComposableNodeContainer(
        name="moveit_wrapper_container",
        namespace=LaunchConfiguration("namespace"),
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_wrapper",
                plugin="moveit_wrapper::MoveitWrapper",
                name="moveit_wrapper",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    LaunchConfiguration("param_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            )
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_wrapper_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        namespace=LaunchConfiguration("namespace"),
    )

    return LaunchDescription(
        [
            param_file_arg,
            use_sim_time_arg,
            namespace_arg,
            start_rviz_arg,
            rviz_config_arg,
            container,
            rviz_node,
        ]
    )
