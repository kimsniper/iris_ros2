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

class ComplementaryFilter
{
public:
    ComplementaryFilter(double alpha = 0.98,
                        double max_accel_correction = 0.5,
                        double gravity = 9.81,
                        double accel_norm_tolerance = 2.0);

    void reset(double pitch = 0.0);

    double update(double gyro_y, double accel_x, double accel_z, double dt);

private:
    static double wrapAngle(double angle);

    double alpha;
    double max_accel_correction;
    double gravity;
    double accel_norm_tolerance;

    double pitch;
    double previous_gyro_y;
    bool has_previous_gyro_y;
};
