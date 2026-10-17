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

#pragma once

class BalanceController
{
public:
    BalanceController();

    void reset();

    void setParameters(float kp,
                       float ki,
                       float kd,
                       float position_kp,
                       float velocity_kd,
                       float max_pitch_reference,
                       float integral_limit,
                       float output_limit);

    float update(float pitch,
                 float pitch_rate,
                 float left_wheel_velocity,
                 float right_wheel_velocity,
                 float left_wheel_position,
                 float right_wheel_position,
                 float dt);

private:
    float kp;
    float ki;
    float kd;
    float position_kp;
    float velocity_kd;
    float max_pitch_reference;
    float integral_limit;
    float output_limit;

    float integral;
    float target_wheel_position;
    bool target_wheel_position_initialized;
};
