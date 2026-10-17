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

#include "robot_control/balance_controller.hpp"

#include "gtest/gtest.h"

namespace
{
BalanceController createController()
{
    BalanceController controller;

    controller.setParameters(10.0f, 0.0f, 0.2f, 0.002f, 0.002f, 0.02f, 0.5f, 0.5f);

    return controller;
}
}  // namespace

TEST(BalanceControllerTest, ProducesZeroAtInitialTarget)
{
    auto controller = createController();

    const float torque = controller.update(0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.02f);

    EXPECT_NEAR(torque, 0.0f, 1.0e-6f);
}

TEST(BalanceControllerTest, ProducesEqualPositivePitchCorrection)
{
    auto controller = createController();

    const float torque = controller.update(0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NEAR(torque, 0.1f, 1.0e-6f);
}

TEST(BalanceControllerTest, UsesPitchRate)
{
    auto controller = createController();

    const float torque = controller.update(0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NEAR(torque, 0.1f, 1.0e-6f);
}

TEST(BalanceControllerTest, LimitsOutputTorque)
{
    auto controller = createController();

    const float positive_torque = controller.update(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NEAR(positive_torque, 0.5f, 1.0e-6f);

    controller.reset();

    const float negative_torque = controller.update(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NEAR(negative_torque, -0.5f, 1.0e-6f);
}

TEST(BalanceControllerTest, AppliesPositionCorrection)
{
    auto controller = createController();

    controller.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);

    const float torque = controller.update(0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.02f);

    EXPECT_NEAR(torque, 0.02f, 1.0e-6f);
}

TEST(BalanceControllerTest, AppliesVelocityCorrection)
{
    auto controller = createController();

    const float torque = controller.update(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NEAR(torque, 0.02f, 1.0e-6f);
}

TEST(BalanceControllerTest, ResetCapturesNewPositionTarget)
{
    auto controller = createController();

    controller.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);

    controller.reset();

    const float torque = controller.update(0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 2.0f, 0.02f);

    EXPECT_NEAR(torque, 0.0f, 1.0e-6f);
}
