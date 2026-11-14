#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_nums.hpp"

class AddServer : public rclcpp::Node
{
public:
    AddServer() : Node("add_server")
    {
        service_ =
            this->create_service<custom_interfaces::srv::AddNums>(
                "add_nums",
                std::bind(
                    &AddServer::add_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );
        
        RCLCPP_INFO(
            this->get_logger(),
            "Add service ready"
        );
    }

private:
    void add_callback(
        const std::shared_ptr<custom_interfaces::srv::AddNums::Request> request,
        std::shared_ptr<custom_interfaces::srv::AddNums::Response> response)
        {
            response->num3 = request->num1 + request->num2;
            
            RCLCPP_INFO(
                this->get_logger(),
                "%ld + %ld = %ld",
                request->num1,
                request->num2,
                response->num3
            );
        }

    rclcpp::Service<custom_interfaces::srv::AddNums>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AddServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}