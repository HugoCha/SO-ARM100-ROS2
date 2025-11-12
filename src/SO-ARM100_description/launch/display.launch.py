from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Define launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Whether to use simulated hardware (true) or real hardware (false)"
        ),
        DeclareLaunchArgument(
            "gripper_type",
            default_value="jaw",
            description="Type of gripper: 'jaw' or 'none'"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 with the robot model"
        ),
    ]

    # Get launch configuration variables
    use_sim = LaunchConfiguration("use_sim")
    gripper_type = LaunchConfiguration("gripper_type")
    use_rviz = LaunchConfiguration("use_rviz")

    # Paths
    pkg_share = FindPackageShare("so_arm100_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "so_arm100.urdf.xacro"])

    # Process Xacro to URDF dynamically
    robot_description = Command([
        "xacro ", xacro_file,
        " use_sim:=", use_sim,
        " gripper_type:=", gripper_type
    ])

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_base",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(use_sim),
        output="screen"
    )

    # RViz2 node (optional)
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "display_config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        # use nvidia GPU for rendering
        additional_env={
            "__NV_PRIME_RENDER_OFFLOAD": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "nvidia"}
    )

    return LaunchDescription(declared_arguments + 
                             [joint_state_publisher_node, 
                              robot_state_publisher, static_tf, 
                              rviz_node])
