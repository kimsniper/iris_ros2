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

#include "robot_estimation/imu_ekf.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace
{
float quaternionNorm(const std::array<float, 4>& quaternion)
{
    return std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] +
                     quaternion[3] * quaternion[3]);
}

float quaternionPitch(const std::array<float, 4>& quaternion)
{
    const float value = 2.0f * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]);

    return std::asin(std::max(-1.0f, std::min(1.0f, value)));
}
}  // namespace

TEST(ImuEKFTest, ResetRestoresIdentity)
{
    ImuEKF estimator;

    estimator.setDt(0.01f);
    estimator.predict({ 0.0f, 1.0f, 0.0f });
    estimator.reset();

    const auto quaternion = estimator.getQuaternion();
    const auto bias = estimator.getGyroBias();

    EXPECT_NEAR(quaternion[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(quaternion[1], 0.0f, 1.0e-6f);
    EXPECT_NEAR(quaternion[2], 0.0f, 1.0e-6f);
    EXPECT_NEAR(quaternion[3], 1.0f, 1.0e-6f);

    for(float value : bias)
    {
        EXPECT_NEAR(value, 0.0f, 1.0e-6f);
    }
}

TEST(ImuEKFTest, RemainsStableAtIdentity)
{
    ImuEKF estimator;
    estimator.setDt(0.002f);

    for(int step = 0; step < 5000; step++)
    {
        estimator.predict({ 0.0f, 0.0f, 0.0f });

        estimator.update({ 0.0f, 0.0f, 9.81f });
    }

    const auto quaternion = estimator.getQuaternion();

    EXPECT_NEAR(quaternionNorm(quaternion), 1.0f, 1.0e-5f);

    EXPECT_NEAR(quaternionPitch(quaternion), 0.0f, 1.0e-4f);
}

TEST(ImuEKFTest, IntegratesPitchRate)
{
    ImuEKF estimator;
    estimator.setDt(0.002f);

    constexpr float pitch_rate = 0.2f;
    constexpr int steps = 500;

    for(int step = 0; step < steps; step++)
    {
        estimator.predict({ 0.0f, pitch_rate, 0.0f });
    }

    const float expected_pitch = pitch_rate * 0.002f * steps;

    const auto quaternion = estimator.getQuaternion();

    EXPECT_NEAR(quaternionNorm(quaternion), 1.0f, 1.0e-5f);

    EXPECT_NEAR(quaternionPitch(quaternion), expected_pitch, 2.0e-3f);
}

TEST(ImuEKFTest, ConvergesToKnownPitch)
{
    ImuEKF estimator;
    estimator.setDt(0.002f);

    constexpr float target_pitch = 0.20f;

    const std::array<float, 3> acceleration = { -9.81f * std::sin(target_pitch), 0.0f, 9.81f * std::cos(target_pitch) };

    for(int step = 0; step < 5000; step++)
    {
        estimator.predict({ 0.0f, 0.0f, 0.0f });

        estimator.update(acceleration);
    }

    const auto quaternion = estimator.getQuaternion();

    EXPECT_NEAR(quaternionNorm(quaternion), 1.0f, 1.0e-5f);

    EXPECT_NEAR(quaternionPitch(quaternion), target_pitch, 1.0e-2f);
}

TEST(ImuEKFTest, RejectsHighAcceleration)
{
    ImuEKF estimator;
    estimator.setDt(0.002f);

    for(int step = 0; step < 100; step++)
    {
        estimator.predict({ 0.0f, 0.0f, 0.0f });

        estimator.update({ 20.0f, 0.0f, 20.0f });
    }

    const auto quaternion = estimator.getQuaternion();

    EXPECT_NEAR(quaternionPitch(quaternion), 0.0f, 1.0e-4f);
}

TEST(ImuEKFTest, IgnoresInvalidDt)
{
    ImuEKF estimator;

    estimator.setDt(-1.0f);
    estimator.predict({ 0.0f, 1.0f, 0.0f });

    const auto quaternion = estimator.getQuaternion();

    EXPECT_NEAR(quaternionPitch(quaternion), 0.002f, 1.0e-4f);
}
