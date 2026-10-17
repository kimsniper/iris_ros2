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

    integral = 0.0f;
}

void BalanceController::setGains(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

float BalanceController::update(float pitch, float pitch_rate, float dt)
{
    float error = 0.0f - pitch;

    integral += error * dt;

    integral = std::max(-1.0f, std::min(1.0f, integral));

    static float filtered_pitch_rate = 0.0f;
    filtered_pitch_rate = 0.8f * filtered_pitch_rate + 0.2f * (-pitch_rate);
    float derivative = filtered_pitch_rate;

    float output = kp * error + ki * integral + kd * derivative;

    return -output;
}