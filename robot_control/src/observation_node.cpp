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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

class ObservationNode : public rclcpp::Node
{
public:
    ObservationNode() : Node("observation_node")
    {
        pitch_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/state/pitch", 10, std::bind(&ObservationNode::pitchCallback, this, std::placeholders::_1));

        pitch_rate_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/state/pitch_rate", 10, std::bind(&ObservationNode::pitchRateCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&ObservationNode::imuCallback, this, std::placeholders::_1));

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ObservationNode::jointStateCallback, this, std::placeholders::_1));

        observation_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/state/observation", 10);

        timer_ =
            create_wall_timer(std::chrono::milliseconds(20), std::bind(&ObservationNode::publishObservation, this));

        RCLCPP_INFO(get_logger(), "IRIS Observation Node Started");
    }

private:
    void pitchCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_ = msg->data;
        pitch_received_ = true;
        pitch_time_ = now();
    }

    void pitchRateCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_rate_ = msg->data;
        pitch_rate_received_ = true;
        pitch_rate_time_ = now();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        accel_x_ = msg->linear_acceleration.x;
        accel_z_ = msg->linear_acceleration.z;
        imu_received_ = true;
        imu_time_ = now();
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const auto left_iterator = std::find(msg->name.begin(), msg->name.end(), "left_wheel_joint");

        const auto right_iterator = std::find(msg->name.begin(), msg->name.end(), "right_wheel_joint");

        if(left_iterator == msg->name.end() || right_iterator == msg->name.end())
        {
            return;
        }

        const auto left_index = std::distance(msg->name.begin(), left_iterator);

        const auto right_index = std::distance(msg->name.begin(), right_iterator);

        if(left_index >= static_cast<long>(msg->position.size()) ||
           right_index >= static_cast<long>(msg->position.size()) ||
           left_index >= static_cast<long>(msg->velocity.size()) ||
           right_index >= static_cast<long>(msg->velocity.size()))
        {
            return;
        }

        left_wheel_position_ = msg->position[left_index];
        right_wheel_position_ = msg->position[right_index];
        left_wheel_velocity_ = msg->velocity[left_index];
        right_wheel_velocity_ = msg->velocity[right_index];

        joint_state_received_ = true;
        joint_state_time_ = now();
    }

    bool inputIsFresh(const rclcpp::Time& input_time, const rclcpp::Time& current_time) const
    {
        return (current_time - input_time).seconds() <= 0.1;
    }

    void publishObservation()
    {
        if(!pitch_received_ || !pitch_rate_received_ || !imu_received_ || !joint_state_received_)
        {
            return;
        }

        const auto current_time = now();

        if(!inputIsFresh(pitch_time_, current_time) || !inputIsFresh(pitch_rate_time_, current_time) ||
           !inputIsFresh(imu_time_, current_time) || !inputIsFresh(joint_state_time_, current_time))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IRIS observation input is stale");

            return;
        }

        std_msgs::msg::Float32MultiArray observation;
        observation.data = { static_cast<float>(pitch_),
                             static_cast<float>(pitch_rate_),
                             static_cast<float>(left_wheel_velocity_),
                             static_cast<float>(right_wheel_velocity_),
                             static_cast<float>(accel_x_),
                             static_cast<float>(accel_z_),
                             0.0f,
                             static_cast<float>(left_wheel_position_),
                             static_cast<float>(right_wheel_position_) };

        observation_pub_->publish(observation);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_rate_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr observation_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time pitch_time_;
    rclcpp::Time pitch_rate_time_;
    rclcpp::Time imu_time_;
    rclcpp::Time joint_state_time_;

    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    double accel_x_ = 0.0;
    double accel_z_ = 0.0;
    double left_wheel_velocity_ = 0.0;
    double right_wheel_velocity_ = 0.0;
    double left_wheel_position_ = 0.0;
    double right_wheel_position_ = 0.0;

    bool pitch_received_ = false;
    bool pitch_rate_received_ = false;
    bool imu_received_ = false;
    bool joint_state_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObservationNode>());
    rclcpp::shutdown();
    return 0;
}
