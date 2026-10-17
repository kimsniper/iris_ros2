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
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "robot_control/balance_controller.hpp"
#include <cmath>

class BalanceNode : public rclcpp::Node
{
public:
    BalanceNode() : Node("balance_node")
    {
        pitch_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/state/pitch", 10, std::bind(&BalanceNode::pitchCallback, this, std::placeholders::_1));

        pitch_rate_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/state/pitch_rate", 10, std::bind(&BalanceNode::pitchRateCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 10);

        gain_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/current_pid_gains", 10);

        gain_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pid_gains", 10, std::bind(&BalanceNode::gainCallback, this, std::placeholders::_1));

        controller_ = std::make_shared<BalanceController>();

        declare_parameter("kp", 30.0);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 1.2);

        double p = get_parameter("kp").as_double();
        double i = get_parameter("ki").as_double();
        double d = get_parameter("kd").as_double();

        current_kp_ = p;
        current_ki_ = i;
        current_kd_ = d;

        RCLCPP_INFO(get_logger(), "PID updated: kp=%.3f ki=%.3f kd=%.3f", p, i, d);

        controller_->setGains(p, i, d);

        parameter_callback_handle_ =
            this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                double p_val = get_parameter("kp").as_double();
                double i_val = get_parameter("ki").as_double();
                double d_val = get_parameter("kd").as_double();
                for(const auto& param : parameters)
                {
                    if(param.get_name() == "kp")
                        p_val = param.as_double();
                    if(param.get_name() == "ki")
                        i_val = param.as_double();
                    if(param.get_name() == "kd")
                        d_val = param.as_double();
                }

                current_kp_ = p_val;
                current_ki_ = i_val;
                current_kd_ = d_val;

                RCLCPP_INFO(get_logger(), "PID updated: kp=%.3f ki=%.3f kd=%.3f", p_val, i_val, d_val);
                controller_->setGains(p_val, i_val, d_val);

                std_msgs::msg::Float64MultiArray gain_msg;
                gain_msg.data = { current_kp_, current_ki_, current_kd_ };
                gain_pub_->publish(gain_msg);

                std::string cmd = "ign service -s /world/empty/set_pose "
                                  "--reqtype ignition.msgs.Pose "
                                  "--reptype ignition.msgs.Boolean "
                                  "--timeout 2000 "
                                  "--req 'name: \"two_wheel_robot\" position { x: 0 y: 0 z: 0.3 }'";

                std::system(cmd.c_str());

                return result;
            });

        last_time_ = now();

        timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&BalanceNode::updateLoop, this));

        RCLCPP_INFO(get_logger(), "Balance Controller Started");
    }

private:
    void pitchCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_ = msg->data;
    }

    void pitchRateCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_rate_ = msg->data;
    }

    void updateLoop()
    {
        auto now_time = now();
        double dt = (now_time - last_time_).seconds();
        if(dt <= 0.0)
            return;
        last_time_ = now_time;

        double control = controller_->update(pitch_, pitch_rate_, dt);

        if(control > 40.0)
            control = 40.0;

        if(control < -40.0)
            control = -40.0;

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.resize(2);
        cmd.data[0] = control;
        cmd.data[1] = control;

        cmd_pub_->publish(cmd);

        std_msgs::msg::Float64MultiArray gain_msg;
        gain_msg.data = { current_kp_, current_ki_, current_kd_ };
        gain_pub_->publish(gain_msg);
    }

    void gainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if(msg->data.size() < 3)
            return;

        double kp = msg->data[0];
        double ki = msg->data[1];
        double kd = msg->data[2];

        current_kp_ = kp;
        current_ki_ = ki;
        current_kd_ = kd;

        RCLCPP_INFO(get_logger(), "PID updated: kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);

        controller_->setGains(kp, ki, kd);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_rate_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gain_sub_;

    std_msgs::msg::Float64MultiArray cmd;

    std::shared_ptr<BalanceController> controller_;

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    rclcpp::Time last_time_;

    rclcpp::TimerBase::SharedPtr timer_;

    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;

    double current_kp_ = 0.0;
    double current_ki_ = 0.0;
    double current_kd_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceNode>());
    rclcpp::shutdown();
    return 0;
}
