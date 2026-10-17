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

#include <array>

class ImuEKF
{
public:
    ImuEKF();

    void reset();

    void setDt(float dt);

    void predict(const std::array<float, 3>& gyro);
    void update(const std::array<float, 3>& accel);

    std::array<float, 4> getQuaternion() const;
    std::array<float, 3> getGyroBias() const;

private:
    static std::array<float, 4> normalizeQuaternion(const std::array<float, 4>& quaternion);

    static std::array<float, 4> multiplyQuaternions(const std::array<float, 4>& left,
                                                    const std::array<float, 4>& right);

    static std::array<float, 3> rotateGravityToBody(const std::array<float, 4>& quaternion);

    static void invert3x3(const std::array<float, 9>& matrix, std::array<float, 9>& inverse, bool& valid);

    void enforceCovarianceSymmetry();

    std::array<float, 4> quaternion;
    std::array<float, 3> gyro_bias;
    std::array<float, 36> covariance;

    float dt;
    float gravity;
    float accel_norm_tolerance;
    float innovation_gate;

    float gyro_noise_variance;
    float gyro_bias_noise_variance;
    float accel_noise_variance;
};
