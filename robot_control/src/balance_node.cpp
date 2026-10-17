/*
 * Copyright 2026 Mezael Docoy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "robot_control/balance_controller.hpp"

#include <chrono>

class BalanceNode : public rclcpp::Node
{
public:
    BalanceNode() : Node("balance_node")
    {
        observation_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/state/observation", 10, std::bind(&BalanceNode::observationCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 10);

        controller_ = std::make_shared<BalanceController>();

        declare_parameter("kp", 10.0);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 0.2);
        declare_parameter("position_kp", 0.002);
        declare_parameter("velocity_kd", 0.002);
        declare_parameter("max_pitch_reference", 0.02);
        declare_parameter("integral_limit", 0.5);
        declare_parameter("output_limit", 0.5);

        controller_->setParameters(get_parameter("kp").as_double(),
                                   get_parameter("ki").as_double(),
                                   get_parameter("kd").as_double(),
                                   get_parameter("position_kp").as_double(),
                                   get_parameter("velocity_kd").as_double(),
                                   get_parameter("max_pitch_reference").as_double(),
                                   get_parameter("integral_limit").as_double(),
                                   get_parameter("output_limit").as_double());

        controller_->reset();

        watchdog_timer_ =
            create_wall_timer(std::chrono::milliseconds(20), std::bind(&BalanceNode::watchdogCallback, this));

        RCLCPP_INFO(get_logger(), "IRIS PID Controller Started");
    }

private:
    void observationCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() != 9)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Expected nine-element IRIS observation");

            return;
        }

        const auto current_time = now();

        last_observation_time_ = std::chrono::steady_clock::now();
        observation_received_ = true;

        if(!initialized_)
        {
            last_time_ = current_time;
            initialized_ = true;
            return;
        }

        const double dt = (current_time - last_time_).seconds();

        last_time_ = current_time;

        if(dt <= 0.0 || dt > 0.1)
        {
            controller_->reset();
            publishZeroCommand();
            return;
        }

        const float torque = controller_->update(
            msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[7], msg->data[8], static_cast<float>(dt));

        std_msgs::msg::Float64MultiArray command;
        command.data = { torque, torque };

        cmd_pub_->publish(command);
    }

    void watchdogCallback()
    {
        if(!observation_received_)
        {
            return;
        }

        const auto elapsed = std::chrono::steady_clock::now() - last_observation_time_;

        if(elapsed <= std::chrono::milliseconds(100))
        {
            return;
        }

        controller_->reset();
        initialized_ = false;
        observation_received_ = false;
        publishZeroCommand();

        RCLCPP_WARN(get_logger(), "Observation timeout; publishing zero wheel effort");
    }

    void publishZeroCommand()
    {
        std_msgs::msg::Float64MultiArray command;
        command.data = { 0.0, 0.0 };

        cmd_pub_->publish(command);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr observation_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

    std::shared_ptr<BalanceController> controller_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    rclcpp::Time last_time_;

    std::chrono::steady_clock::time_point last_observation_time_;

    bool initialized_ = false;
    bool observation_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceNode>());
    rclcpp::shutdown();
    return 0;
}
