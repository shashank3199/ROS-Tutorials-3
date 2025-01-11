#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ParameterWriter : public rclcpp::Node
{
public:
    ParameterWriter() : Node("parameter_writer")
    {
        // Initialize counters for parameter updates
        update_counter_ = 0;
        speed_value_ = 0.5;

        // Declare parameters with initial values
        this->declare_parameter("my_robot.name", "RoboX");
        this->declare_parameter("my_robot.debug_mode", true);
        this->declare_parameter("my_robot.max_speed", speed_value_);

        std::vector<std::string> initial_sensors = {"lidar"};
        this->declare_parameter("my_robot.sensor_list", initial_sensors);

        // Create a timer for periodic parameter updates
        timer_ = this->create_wall_timer(
            2s, // Update every 2 seconds
            std::bind(&ParameterWriter::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Parameter Writer Node initialized");

        // Declare sampled parameters
        this->declare_parameter("param_sample_robot.name", "");
        this->declare_parameter("param_sample_robot.debug_mode", false);
        this->declare_parameter("param_sample_robot.max_speed", 0.0);
        this->declare_parameter("param_sample_robot.sensor_list", std::vector<std::string>());

        // Get the sampled parameters
        std::string name;
        bool debug_mode;
        double max_speed;
        std::vector<std::string> sensor_list;

        this->get_parameter("param_sample_robot.name", name);
        this->get_parameter("param_sample_robot.debug_mode", debug_mode);
        this->get_parameter("param_sample_robot.max_speed", max_speed);
        this->get_parameter("param_sample_robot.sensor_list", sensor_list);

        RCLCPP_INFO(this->get_logger(), "Sampled parameters:");
        RCLCPP_INFO(this->get_logger(), "Name: %s", name.c_str());
        RCLCPP_INFO(this->get_logger(), "Debug mode: %s", (debug_mode ? "true" : "false"));
        RCLCPP_INFO(this->get_logger(), "Max speed: %.2f", max_speed);

        std::string sensors_str = "Sensor list: ";
        for (const auto &sensor : sensor_list)
        {
            sensors_str += sensor + ", ";
        }
        sensors_str = sensors_str.substr(0, sensors_str.length() - 2);
        RCLCPP_INFO(this->get_logger(), "Sensor list: %s", sensors_str.c_str());
    }

private:
    void timer_callback()
    {
        try
        {
            update_counter_++;

            // 1. Toggle debug mode using set_parameter
            bool current_debug = this->get_parameter("my_robot.debug_mode").as_bool();
            auto debug_param = rclcpp::Parameter("my_robot.debug_mode", !current_debug);
            this->set_parameter(debug_param);
            RCLCPP_INFO(this->get_logger(), "Debug mode toggled to: %s", (!current_debug ? "true" : "false"));

            // 2. Update max_speed using set_parameter
            speed_value_ += 0.5;
            if (speed_value_ > 5.0)
                speed_value_ = 0.5;

            auto speed_param = rclcpp::Parameter("my_robot.max_speed", speed_value_);
            this->set_parameter(speed_param);
            RCLCPP_INFO(this->get_logger(), "Max speed updated to: %.2f", speed_value_);

            // 3. Update multiple parameters at once using set_parameters
            std::vector<rclcpp::Parameter> params;

            // Update robot name with a counter
            std::string new_name = "RoboX_" + std::to_string(update_counter_);
            params.emplace_back("my_robot.name", new_name);

            // Update sensor list by adding sensors based on counter
            std::vector<std::string> new_sensors;
            new_sensors.push_back("lidar"); // Always have lidar
            if (update_counter_ % 2 == 0)
            {
                new_sensors.push_back("camera");
            }
            if (update_counter_ % 3 == 0)
            {
                new_sensors.push_back("ultrasonic");
            }
            params.emplace_back("my_robot.sensor_list", new_sensors);

            // Set multiple parameters at once
            auto results = this->set_parameters(params);

            // Check if all parameter updates were successful
            bool all_success = true;
            for (const auto &result : results)
            {
                if (!result.successful)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to update parameter: %s", result.reason.c_str());
                    all_success = false;
                }
            }

            if (all_success)
            {
                RCLCPP_INFO(this->get_logger(), "Robot name updated to: %s", new_name.c_str());

                std::string sensors_str = "Updated sensor list: ";
                for (const auto &sensor : new_sensors)
                {
                    sensors_str += sensor + ", ";
                }
                sensors_str = sensors_str.substr(0, sensors_str.length() - 2);
                RCLCPP_INFO(this->get_logger(), "%s", sensors_str.c_str());
            }

            // Print a separator for better log readability
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        }
        catch (const rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter not declared: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error updating parameters: %s", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int update_counter_;
    double speed_value_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterWriter>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error during node execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}