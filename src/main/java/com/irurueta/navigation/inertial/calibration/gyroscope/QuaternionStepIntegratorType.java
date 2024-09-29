/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.inertial.calibration.gyroscope;

/**
 * Indicates type of quaternion integrator step.
 * Different types exist with different levels of accuracy and computation complexity.
 */
public enum QuaternionStepIntegratorType {
    /**
     * Performs quaternion integration using Euler's method, which is the less accurate and has the
     * smallest computational complexity.
     */
    EULER_METHOD,

    /**
     * Performs quaternion integration using mid-point algorithm, which offers a medium accuracy and
     * computational complexity.
     */
    MID_POINT,

    /**
     * Performs quaternion integration using Runge-Kutta of 4th order (aka RK4) algorithm, which
     * offers high accuracy at the expense of higher computational complexity.
     */
    RUNGE_KUTTA,

    /**
     * Performs quaternion integration Based on Young Soo Suh. "Orientation estimation using a quaternion-based
     * indirect Kalman filter with adaptive estimation of external acceleration". 2010.
     * This method can achieve higher accuracy than Runge-Kutta
     */
    SUH,

    /**
     * Performs quaternion integration based on Trawny, N. "Indirect Kalman Filter for 3D Attitude Estimation". 2005,
     * which offers a medium accuracy and computational complexity.
     */
    TRAWNY,

    /**
     * Performs quaternion integration based on Yuan, S. "Quaternion-based Unscented Kalman Filter for Real-time". 2015,
     * which offers a medium accuracy and computational complexity.
     */
    YUAN
}
