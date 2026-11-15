from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "m0609",
            package_name="dsr_moveit_config_m0609"
        )
        .robot_description(
            file_path="config/m0609.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/dsr.srdf"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .to_moveit_configs()
    )

    moveit_pose_test = Node(
        package="move_action_cpp",
        executable="moveit_pose_test",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ]
    )

    return LaunchDescription([
        moveit_pose_test
    ])
