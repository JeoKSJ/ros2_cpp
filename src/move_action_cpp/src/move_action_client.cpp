#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include "custom_interfaces/action/move_to.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker.hpp"

int main(int argc, char ** argv)
{
    const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    using MoveTo = custom_interfaces::action::MoveTo;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveTo>;
    using Marker = visualization_msgs::msg::Marker;
    const auto node = std::make_shared<rclcpp::Node>("move_to_action_client");

    MoveTo::Goal goal;
    try {
        if (args.size() != 4) {
            throw std::invalid_argument("expected three coordinates");
        }
        const auto coordinate = [](const std::string & value) {
            std::size_t end = 0;
            const double number = std::stod(value, &end);
            if (end != value.size() || !std::isfinite(number)) {
                throw std::invalid_argument("expected a finite number");
            }
            return number;
        };
        goal.x = coordinate(args[1]);
        goal.y = coordinate(args[2]);
        goal.z = coordinate(args[3]);
    } catch (const std::exception & error) {
        RCLCPP_ERROR(node->get_logger(), "%s. Usage: move_action_client x y z", error.what());
        rclcpp::shutdown();
        return 2;
    }

    const auto client = rclcpp_action::create_client<MoveTo>(node, "/move_to");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/move_to is unavailable");
        rclcpp::shutdown();
        return 1;
    }

    const auto publisher = node->create_publisher<Marker>("/move_to_marker", 10);
    Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "move_to";
    marker.id = 0;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.7;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    bool finished = false;
    int exit_code = 1;
    rclcpp_action::Client<MoveTo>::SendGoalOptions options;
    options.goal_response_callback = [&](GoalHandle::SharedPtr handle) {
        if (!handle) {
            RCLCPP_ERROR(node->get_logger(), "Goal rejected");
            finished = true;
        }
    };
    options.feedback_callback =
        [&](GoalHandle::SharedPtr, const std::shared_ptr<const MoveTo::Feedback> feedback) {
            marker.header.stamp = node->now();
            marker.pose.position.x = feedback->current_x;
            marker.pose.position.y = feedback->current_y;
            marker.pose.position.z = feedback->current_z;
            publisher->publish(marker);
        };
    options.result_callback = [&](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success)
        {
            RCLCPP_INFO(node->get_logger(), "%s", result.result->message.c_str());
            exit_code = 0;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Goal ended with code %d: %s",
                static_cast<int>(result.code),
                result.result ? result.result->message.c_str() : "");
        }
        finished = true;
    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    client->async_send_goal(goal, options);
    while (rclcpp::ok() && !finished) {
        executor.spin_once(std::chrono::milliseconds(100));
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
