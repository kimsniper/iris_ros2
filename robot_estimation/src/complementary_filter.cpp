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

#include "robot_estimation/complementary_filter.hpp"

#include <cmath>

ComplementaryFilter::ComplementaryFilter(double alpha,
                                         double max_accel_correction,
                                         double gravity,
                                         double accel_norm_tolerance)
  : alpha(alpha)
  , max_accel_correction(max_accel_correction)
  , gravity(gravity)
  , accel_norm_tolerance(accel_norm_tolerance)
  , pitch(0.0)
  , previous_gyro_y(0.0)
  , has_previous_gyro_y(false)
{}

void ComplementaryFilter::reset(double initial_pitch)
{
    pitch = initial_pitch;
    previous_gyro_y = 0.0;
    has_previous_gyro_y = false;
}

double ComplementaryFilter::wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double ComplementaryFilter::update(double gyro_y, double accel_x, double accel_z, double dt)
{
    double average_gyro_y = gyro_y;

    if(has_previous_gyro_y)
    {
        average_gyro_y = 0.5 * (previous_gyro_y + gyro_y);
    }

    previous_gyro_y = gyro_y;
    has_previous_gyro_y = true;

    const double predicted_pitch = wrapAngle(pitch + average_gyro_y * dt);

    const double accel_norm = std::hypot(accel_x, accel_z);

    if(std::abs(accel_norm - gravity) > accel_norm_tolerance)
    {
        pitch = predicted_pitch;
        return pitch;
    }

    const double accel_pitch = std::atan2(-accel_x, accel_z);

    const double correction = wrapAngle(accel_pitch - predicted_pitch);

    if(std::abs(correction) > max_accel_correction)
    {
        pitch = predicted_pitch;
        return pitch;
    }

    const double effective_alpha = std::pow(alpha, dt / 0.02);

    pitch = wrapAngle(predicted_pitch + (1.0 - effective_alpha) * correction);

    return pitch;
}
