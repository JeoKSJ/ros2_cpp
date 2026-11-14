#include <memory>       // 메모리: make_shared, shared_ptr

#include "rclcpp/rclcpp.hpp"    // ros2 cpp기능
#include "std_srvs/srv/set_bool.hpp"    // 서비스 타입(SetBool)

class GripperServer : public rclcpp::Node   // rclcpp를 가져옴으로써 create_service나 get_logger를 사용 가능.
{
public:
    GripperServer() : Node("gripper_server")
    {
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_gripper",
            std::bind(
                &GripperServer::gripper_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
    }

private:
    void gripper_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if(request->data)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "OPEN request received"
            );

            response->success = true;
            response->message = "Gripper opened";
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "CLOSE request received"
            );

            response->success = true;
            response->message = "Gripper closed";
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;  
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GripperServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}