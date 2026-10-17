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
#include <cmath>
#include <limits>

namespace
{
constexpr int kErrorStateSize = 6;
constexpr int kMeasurementSize = 3;

int covarianceIndex(int row, int column)
{
    return row * kErrorStateSize + column;
}

int measurementIndex(int row, int column)
{
    return row * kMeasurementSize + column;
}

std::array<float, 9> skewSymmetric(const std::array<float, 3>& vector)
{
    return { 0.0f, -vector[2], vector[1], vector[2], 0.0f, -vector[0], -vector[1], vector[0], 0.0f };
}
}  // namespace

ImuEKF::ImuEKF()
{
    reset();

    dt = 0.002f;
    gravity = 9.81f;
    accel_norm_tolerance = 2.0f;
    innovation_gate = 11.345f;

    gyro_noise_variance = 1.0e-6f;
    gyro_bias_noise_variance = 1.0e-8f;
    accel_noise_variance = 2.5e-3f;
}

void ImuEKF::reset()
{
    quaternion = { 0.0f, 0.0f, 0.0f, 1.0f };
    gyro_bias = { 0.0f, 0.0f, 0.0f };
    covariance.fill(0.0f);

    for(int i = 0; i < 3; i++)
    {
        covariance[covarianceIndex(i, i)] = 1.0e-2f;
        covariance[covarianceIndex(i + 3, i + 3)] = 1.0e-3f;
    }
}

void ImuEKF::setDt(float new_dt)
{
    if(std::isfinite(new_dt) && new_dt > 0.0f)
    {
        dt = new_dt;
    }
}

std::array<float, 4> ImuEKF::getQuaternion() const
{
    return quaternion;
}

std::array<float, 3> ImuEKF::getGyroBias() const
{
    return gyro_bias;
}

std::array<float, 4> ImuEKF::normalizeQuaternion(const std::array<float, 4>& input)
{
    const float norm = std::sqrt(input[0] * input[0] + input[1] * input[1] + input[2] * input[2] + input[3] * input[3]);

    if(!std::isfinite(norm) || norm <= std::numeric_limits<float>::epsilon())
    {
        return { 0.0f, 0.0f, 0.0f, 1.0f };
    }

    return { input[0] / norm, input[1] / norm, input[2] / norm, input[3] / norm };
}

std::array<float, 4> ImuEKF::multiplyQuaternions(const std::array<float, 4>& left, const std::array<float, 4>& right)
{
    return { left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1],

             left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0],

             left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3],

             left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2] };
}

std::array<float, 3> ImuEKF::rotateGravityToBody(const std::array<float, 4>& input)
{
    const float qx = input[0];
    const float qy = input[1];
    const float qz = input[2];
    const float qw = input[3];

    return { 2.0f * (qx * qz - qw * qy), 2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz };
}

void ImuEKF::invert3x3(const std::array<float, 9>& matrix, std::array<float, 9>& inverse, bool& valid)
{
    const float determinant = matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
                              matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
                              matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);

    if(!std::isfinite(determinant) || std::abs(determinant) < 1.0e-9f)
    {
        inverse.fill(0.0f);
        valid = false;
        return;
    }

    inverse[0] = (matrix[4] * matrix[8] - matrix[5] * matrix[7]) / determinant;

    inverse[1] = (matrix[2] * matrix[7] - matrix[1] * matrix[8]) / determinant;

    inverse[2] = (matrix[1] * matrix[5] - matrix[2] * matrix[4]) / determinant;

    inverse[3] = (matrix[5] * matrix[6] - matrix[3] * matrix[8]) / determinant;

    inverse[4] = (matrix[0] * matrix[8] - matrix[2] * matrix[6]) / determinant;

    inverse[5] = (matrix[2] * matrix[3] - matrix[0] * matrix[5]) / determinant;

    inverse[6] = (matrix[3] * matrix[7] - matrix[4] * matrix[6]) / determinant;

    inverse[7] = (matrix[1] * matrix[6] - matrix[0] * matrix[7]) / determinant;

    inverse[8] = (matrix[0] * matrix[4] - matrix[1] * matrix[3]) / determinant;

    valid = true;
}

void ImuEKF::enforceCovarianceSymmetry()
{
    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = row + 1; column < kErrorStateSize; column++)
        {
            const float symmetric_value =
                0.5f * (covariance[covarianceIndex(row, column)] + covariance[covarianceIndex(column, row)]);

            covariance[covarianceIndex(row, column)] = symmetric_value;

            covariance[covarianceIndex(column, row)] = symmetric_value;
        }

        const int diagonal_index = covarianceIndex(row, row);

        if(!std::isfinite(covariance[diagonal_index]) || covariance[diagonal_index] < 1.0e-12f)
        {
            covariance[diagonal_index] = 1.0e-12f;
        }
    }
}

void ImuEKF::predict(const std::array<float, 3>& gyro)
{
    if(!std::isfinite(dt) || dt <= 0.0f)
    {
        return;
    }

    const std::array<float, 3> angular_velocity = { gyro[0] - gyro_bias[0],
                                                    gyro[1] - gyro_bias[1],
                                                    gyro[2] - gyro_bias[2] };

    const float angular_speed =
        std::sqrt(angular_velocity[0] * angular_velocity[0] + angular_velocity[1] * angular_velocity[1] +
                  angular_velocity[2] * angular_velocity[2]);

    std::array<float, 4> delta_quaternion;

    if(angular_speed > 1.0e-8f)
    {
        const float half_angle = 0.5f * angular_speed * dt;

        const float scale = std::sin(half_angle) / angular_speed;

        delta_quaternion = {
            angular_velocity[0] * scale, angular_velocity[1] * scale, angular_velocity[2] * scale, std::cos(half_angle)
        };
    }
    else
    {
        delta_quaternion = {
            0.5f * angular_velocity[0] * dt, 0.5f * angular_velocity[1] * dt, 0.5f * angular_velocity[2] * dt, 1.0f
        };
    }

    quaternion = normalizeQuaternion(multiplyQuaternions(quaternion, delta_quaternion));

    std::array<float, 36> transition = {};
    std::array<float, 36> process_noise = {};

    for(int i = 0; i < kErrorStateSize; i++)
    {
        transition[covarianceIndex(i, i)] = 1.0f;
    }

    const std::array<float, 9> angular_velocity_skew = skewSymmetric(angular_velocity);

    for(int row = 0; row < 3; row++)
    {
        for(int column = 0; column < 3; column++)
        {
            transition[covarianceIndex(row, column)] -= angular_velocity_skew[measurementIndex(row, column)] * dt;
        }

        transition[covarianceIndex(row, row + 3)] = -dt;

        process_noise[covarianceIndex(row, row)] = gyro_noise_variance * dt;

        process_noise[covarianceIndex(row + 3, row + 3)] = gyro_bias_noise_variance * dt;
    }

    std::array<float, 36> transition_covariance = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                transition_covariance[covarianceIndex(row, column)] +=
                    transition[covarianceIndex(row, inner)] * covariance[covarianceIndex(inner, column)];
            }
        }
    }

    std::array<float, 36> predicted_covariance = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                predicted_covariance[covarianceIndex(row, column)] +=
                    transition_covariance[covarianceIndex(row, inner)] * transition[covarianceIndex(column, inner)];
            }

            predicted_covariance[covarianceIndex(row, column)] += process_noise[covarianceIndex(row, column)];
        }
    }

    covariance = predicted_covariance;
    enforceCovarianceSymmetry();
}

void ImuEKF::update(const std::array<float, 3>& accel)
{
    const float accel_norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    if(!std::isfinite(accel_norm) || accel_norm <= 1.0e-6f)
    {
        return;
    }

    if(std::abs(accel_norm - gravity) > accel_norm_tolerance)
    {
        return;
    }

    const std::array<float, 3> measurement = { accel[0] / accel_norm, accel[1] / accel_norm, accel[2] / accel_norm };

    const std::array<float, 3> expected = rotateGravityToBody(quaternion);

    const std::array<float, 3> innovation = { measurement[0] - expected[0],
                                              measurement[1] - expected[1],
                                              measurement[2] - expected[2] };

    const std::array<float, 9> expected_skew = skewSymmetric(expected);

    std::array<float, 18> measurement_jacobian = {};

    for(int row = 0; row < kMeasurementSize; row++)
    {
        for(int column = 0; column < kMeasurementSize; column++)
        {
            measurement_jacobian[row * kErrorStateSize + column] = expected_skew[measurementIndex(row, column)];
        }
    }

    std::array<float, 18> jacobian_covariance = {};

    for(int row = 0; row < kMeasurementSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                jacobian_covariance[row * kErrorStateSize + column] +=
                    measurement_jacobian[row * kErrorStateSize + inner] * covariance[covarianceIndex(inner, column)];
            }
        }
    }

    std::array<float, 9> innovation_covariance = {};

    for(int row = 0; row < kMeasurementSize; row++)
    {
        for(int column = 0; column < kMeasurementSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                innovation_covariance[measurementIndex(row, column)] +=
                    jacobian_covariance[row * kErrorStateSize + inner] *
                    measurement_jacobian[column * kErrorStateSize + inner];
            }

            if(row == column)
            {
                innovation_covariance[measurementIndex(row, column)] += accel_noise_variance;
            }
        }
    }

    std::array<float, 9> inverse_innovation_covariance;
    bool inverse_valid = false;

    invert3x3(innovation_covariance, inverse_innovation_covariance, inverse_valid);

    if(!inverse_valid)
    {
        return;
    }

    float normalized_innovation = 0.0f;

    for(int row = 0; row < kMeasurementSize; row++)
    {
        for(int column = 0; column < kMeasurementSize; column++)
        {
            normalized_innovation +=
                innovation[row] * inverse_innovation_covariance[measurementIndex(row, column)] * innovation[column];
        }
    }

    if(!std::isfinite(normalized_innovation) || normalized_innovation > innovation_gate)
    {
        return;
    }

    std::array<float, 18> covariance_jacobian_transpose = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kMeasurementSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                covariance_jacobian_transpose[row * kMeasurementSize + column] +=
                    covariance[covarianceIndex(row, inner)] * measurement_jacobian[column * kErrorStateSize + inner];
            }
        }
    }

    std::array<float, 18> kalman_gain = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kMeasurementSize; column++)
        {
            for(int inner = 0; inner < kMeasurementSize; inner++)
            {
                kalman_gain[row * kMeasurementSize + column] +=
                    covariance_jacobian_transpose[row * kMeasurementSize + inner] *
                    inverse_innovation_covariance[measurementIndex(inner, column)];
            }
        }
    }

    std::array<float, 6> correction = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kMeasurementSize; column++)
        {
            correction[row] += kalman_gain[row * kMeasurementSize + column] * innovation[column];
        }
    }

    const float attitude_error_norm =
        std::sqrt(correction[0] * correction[0] + correction[1] * correction[1] + correction[2] * correction[2]);

    std::array<float, 4> correction_quaternion;

    if(attitude_error_norm > 1.0e-8f)
    {
        const float half_angle = 0.5f * attitude_error_norm;

        const float scale = std::sin(half_angle) / attitude_error_norm;

        correction_quaternion = {
            correction[0] * scale, correction[1] * scale, correction[2] * scale, std::cos(half_angle)
        };
    }
    else
    {
        correction_quaternion = { 0.5f * correction[0], 0.5f * correction[1], 0.5f * correction[2], 1.0f };
    }

    quaternion = normalizeQuaternion(multiplyQuaternions(quaternion, correction_quaternion));

    for(int index = 0; index < 3; index++)
    {
        gyro_bias[index] += correction[index + 3];
    }

    std::array<float, 36> identity_minus_gain_jacobian = {};

    for(int i = 0; i < kErrorStateSize; i++)
    {
        identity_minus_gain_jacobian[covarianceIndex(i, i)] = 1.0f;
    }

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            for(int inner = 0; inner < kMeasurementSize; inner++)
            {
                identity_minus_gain_jacobian[covarianceIndex(row, column)] -=
                    kalman_gain[row * kMeasurementSize + inner] *
                    measurement_jacobian[inner * kErrorStateSize + column];
            }
        }
    }

    std::array<float, 36> left_covariance = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                left_covariance[covarianceIndex(row, column)] +=
                    identity_minus_gain_jacobian[covarianceIndex(row, inner)] *
                    covariance[covarianceIndex(inner, column)];
            }
        }
    }

    std::array<float, 36> joseph_covariance = {};

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            for(int inner = 0; inner < kErrorStateSize; inner++)
            {
                joseph_covariance[covarianceIndex(row, column)] +=
                    left_covariance[covarianceIndex(row, inner)] *
                    identity_minus_gain_jacobian[covarianceIndex(column, inner)];
            }
        }
    }

    for(int row = 0; row < kErrorStateSize; row++)
    {
        for(int column = 0; column < kErrorStateSize; column++)
        {
            float measurement_noise_term = 0.0f;

            for(int inner = 0; inner < kMeasurementSize; inner++)
            {
                measurement_noise_term += kalman_gain[row * kMeasurementSize + inner] * accel_noise_variance *
                                          kalman_gain[column * kMeasurementSize + inner];
            }

            joseph_covariance[covarianceIndex(row, column)] += measurement_noise_term;
        }
    }

    covariance = joseph_covariance;
    enforceCovarianceSymmetry();
}
