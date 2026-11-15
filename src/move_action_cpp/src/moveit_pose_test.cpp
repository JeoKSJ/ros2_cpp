#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_pose_test",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
    );

    // ROS2 callback을 처리할 executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    using MoveGroupInterface =
        moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface move_group(
        node,
        "manipulator"
    );

    auto current_pose = move_group.getCurrentPose().pose;

    RCLCPP_INFO(
        node->get_logger(),
        "Current TCP Pose\n"
        "x: %.3f\n"
        "y: %.3f\n"
        "z: %.3f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z
    );

    RCLCPP_INFO(
        node->get_logger(),
        "End Effector Link: %s",
        move_group.getEndEffectorLink().c_str()
    );

    // ============================================
    auto joint_values = move_group.getCurrentJointValues();

    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG2RAD = PI / 180.0;

    RCLCPP_INFO(
        node->get_logger(),
        "Current joints: %.3f %.3f %.3f %.3f %.3f %.3f",
        joint_values[0],
        joint_values[1],
        joint_values[2],
        joint_values[3],
        joint_values[4],
        joint_values[5]
    );

    // 현재 0 자세에서 로봇을 조금 접어준다.
    // 단위는 radian.
    std::vector<double> target_joints = {
        0.0, 0.0, 90.0 * DEG2RAD, 0.0, 90.0 * DEG2RAD, 0.0
    };

    move_group.setJointValueTarget(target_joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        (move_group.plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Home pose planning success"
        );

        auto result = move_group.execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Moved to home pose"
            );
        }
        else
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Failed to execute home pose"
            );
        }
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Home pose planning failed"
        );
    }
    rclcpp::shutdown();

    spin_thread.join();

    return 0;
}