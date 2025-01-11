#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

using namespace std::chrono_literals;

class ParameterModifier : public rclcpp::Node
{
public:
    ParameterModifier() : Node("parameter_modifier")
    {
        // Create parameter event subscription
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Create service client for the parameter reader node
        param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/" + remote_node_name + "/set_parameters");

        // Create timer to periodically toggle reader status
        toggle_timer_ = this->create_wall_timer(
            5s, // Toggle every 5 seconds
            std::bind(&ParameterModifier::toggle_reader_status, this));

        RCLCPP_INFO(this->get_logger(), "Parameter Modifier Node initialized");
    }

private:
    void toggle_reader_status()
    {
        if (!param_client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter service not available");
            return;
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        // Toggle between "active" and "inactive"
        static bool is_active = true;
        is_active = !is_active;

        // Create parameter message
        rcl_interfaces::msg::Parameter status_param;
        status_param.name = "/reader_status";
        status_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        status_param.value.string_value = is_active ? "active" : "inactive";
        request->parameters.push_back(status_param);

        // Send the request asynchronously
        auto result_future = param_client_->async_send_request(
            request,
            [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
            {
                auto result = future.get();
                if (result->results[0].successful)
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully toggled reader status to: %s",
                                is_active ? "active" : "inactive");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to toggle reader status: %s",
                                 result->results[0].reason.c_str());
                }
            });
    }

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    rclcpp::TimerBase::SharedPtr toggle_timer_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client_;
    std::string remote_node_name = "parameter_reader";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterModifier>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}