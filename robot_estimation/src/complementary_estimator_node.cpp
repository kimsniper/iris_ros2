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
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float64.hpp"

#include "robot_estimation/complementary_filter.hpp"
#include <cmath>

class ComplementaryEstimatorNode : public rclcpp::Node
{
public:
    ComplementaryEstimatorNode() : Node("robot_estimator"), initialized_(false)
    {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&ComplementaryEstimatorNode::imuCallback, this, std::placeholders::_1));

        quat_pub_ = create_publisher<geometry_msgs::msg::Quaternion>("/state/imu_quaternion", 10);

        pitch_pub_ = create_publisher<std_msgs::msg::Float64>("/state/pitch", 10);

        pitch_rate_pub_ = create_publisher<std_msgs::msg::Float64>("/state/pitch_rate", 10);

        filter_ = std::make_shared<ComplementaryFilter>();

        RCLCPP_INFO(get_logger(), "IRIS Complementary Estimator Started");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const rclcpp::Time current_time = msg->header.stamp;

        if(!initialized_)
        {
            const double initial_pitch = std::atan2(-msg->linear_acceleration.x, msg->linear_acceleration.z);

            filter_->reset(initial_pitch);
            last_time_ = current_time;
            initialized_ = true;

            publishState(initial_pitch, msg->angular_velocity.y);

            return;
        }

        const double dt = (current_time - last_time_).seconds();

        if(dt <= 0.0 || dt > 0.1)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring IMU sample with invalid dt: %.6f", dt);

            last_time_ = current_time;
            return;
        }

        last_time_ = current_time;

        const double pitch =
            filter_->update(msg->angular_velocity.y, msg->linear_acceleration.x, msg->linear_acceleration.z, dt);

        publishState(pitch, msg->angular_velocity.y);
    }

    void publishState(double pitch, double pitch_rate)
    {
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = 0.0;
        quaternion.y = std::sin(0.5 * pitch);
        quaternion.z = 0.0;
        quaternion.w = std::cos(0.5 * pitch);

        quat_pub_->publish(quaternion);

        std_msgs::msg::Float64 pitch_msg;
        pitch_msg.data = pitch;
        pitch_pub_->publish(pitch_msg);

        std_msgs::msg::Float64 pitch_rate_msg;
        pitch_rate_msg.data = pitch_rate;
        pitch_rate_pub_->publish(pitch_rate_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_rate_pub_;

    std::shared_ptr<ComplementaryFilter> filter_;
    rclcpp::Time last_time_;
    bool initialized_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComplementaryEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
