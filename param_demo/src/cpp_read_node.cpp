#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ParameterReader : public rclcpp::Node
{
public:
    ParameterReader() : Node("parameter_reader")
    {
        // Declare parameters with initial values
        this->declare_parameter("/reader_status", "inactive");

        // Create parameter event subscription
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Callback function that handles parameter changes
        auto parameters_callback = [this](const rclcpp::Parameter &parameter)
        {
            bool current_reader_status = this->get_parameter("/reader_status").as_string() == "active";
            if (!current_reader_status)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Reader is inactive. Cannot update parameters.");
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Parameter '%s' changed to: %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str());
        };

        // Subscribe to parameters
        for (const auto &param_name : remote_param_names)
        {
            std::string full_param_name = "my_robot." + param_name;
            auto handle = param_subscriber_->add_parameter_callback(
                full_param_name,
                parameters_callback,
                remote_node_name);
            cb_handles_.push_back(handle);
        }

        RCLCPP_INFO(this->get_logger(), "Parameter Reader Node initialized");
    }

private:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles_;

    std::string remote_node_name = "parameter_writer";
    std::vector<std::string> remote_param_names = {
        "name",
        "debug_mode",
        "max_speed",
        "sensor_list"};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}