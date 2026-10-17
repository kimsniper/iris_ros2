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

#include <algorithm>

#include "robot_control/balance_controller.hpp"

BalanceController::BalanceController()
{
    kp = 0.0f;
    ki = 0.0f;
    kd = 0.0f;
    position_kp = 0.0f;
    velocity_kd = 0.0f;
    max_pitch_reference = 0.0f;
    integral_limit = 0.0f;
    output_limit = 0.0f;

    reset();
}

void BalanceController::reset()
{
    integral = 0.0f;
    target_wheel_position = 0.0f;
    target_wheel_position_initialized = false;
}

void BalanceController::setParameters(float p,
                                      float i,
                                      float d,
                                      float position_p,
                                      float velocity_d,
                                      float maximum_pitch_reference,
                                      float maximum_integral,
                                      float maximum_output)
{
    kp = p;
    ki = i;
    kd = d;
    position_kp = position_p;
    velocity_kd = velocity_d;
    max_pitch_reference = maximum_pitch_reference;
    integral_limit = maximum_integral;
    output_limit = maximum_output;
}

float BalanceController::update(float pitch,
                                float pitch_rate,
                                float left_wheel_velocity,
                                float right_wheel_velocity,
                                float left_wheel_position,
                                float right_wheel_position,
                                float dt)
{
    const float average_wheel_velocity = 0.5f * (left_wheel_velocity + right_wheel_velocity);

    const float average_wheel_position = 0.5f * (left_wheel_position + right_wheel_position);

    if(!target_wheel_position_initialized)
    {
        target_wheel_position = average_wheel_position;
        target_wheel_position_initialized = true;
    }

    const float position_error = average_wheel_position - target_wheel_position;

    float pitch_reference = -(position_kp * position_error + velocity_kd * average_wheel_velocity);

    pitch_reference = std::max(-max_pitch_reference, std::min(max_pitch_reference, pitch_reference));

    const float balance_error = pitch - pitch_reference;

    integral += balance_error * dt;

    integral = std::max(-integral_limit, std::min(integral_limit, integral));

    float torque = kp * balance_error + ki * integral + kd * pitch_rate;

    torque = std::max(-output_limit, std::min(output_limit, torque));

    return torque;
}
