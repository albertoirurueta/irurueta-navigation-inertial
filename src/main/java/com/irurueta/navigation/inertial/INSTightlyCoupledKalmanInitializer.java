/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

/**
 * Initializes the tightly coupled INS/GNSS extended Kalman filter error covariance
 * matrix.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_TC_P_matrix.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_TC_P_matrix.m
 * </a>
 */
public class INSTightlyCoupledKalmanInitializer {

    /**
     * Number of parameters of the Kalman filter.
     */
    public static final int NUM_PARAMS = 17;

    /**
     * Constructor.
     * Prevents instantiation of helper class.
     */
    private INSTightlyCoupledKalmanInitializer() {
    }

    /**
     * Initializes INS/GNS tightly coupled Kalman filter error covariance matrix.
     *
     * @param config Kalman filter configuration.
     * @param result instance where resulting initialized error covariance matrix
     *               will be stored. Matrix must be 17x17, otherwise it will be resized.
     */
    @SuppressWarnings("DuplicatedCode")
    public static void initialize(final INSTightlyCoupledKalmanInitializerConfig config, final Matrix result) {
        if (result.getRows() != NUM_PARAMS || result.getColumns() != NUM_PARAMS) {
            try {
                result.resize(NUM_PARAMS, NUM_PARAMS);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        final var initAttUnc = config.getInitialAttitudeUncertainty();
        final var initVelUnc = config.getInitialVelocityUncertainty();
        final var initPosUnc = config.getInitialPositionUncertainty();
        final var initBaUnc = config.getInitialAccelerationBiasUncertainty();
        final var initBgUnc = config.getInitialGyroscopeBiasUncertainty();
        final var initClockOffset = config.getInitialClockOffsetUncertainty();
        final var initClockDrift = config.getInitialClockDriftUncertainty();

        final var initAttUnc2 = initAttUnc * initAttUnc;
        final var initVelUnc2 = initVelUnc * initVelUnc;
        final var initPosUnc2 = initPosUnc * initPosUnc;
        final var initBaUnc2 = initBaUnc * initBaUnc;
        final var initBgUnc2 = initBgUnc * initBgUnc;
        final var initClockOffset2 = initClockOffset * initClockOffset;
        final var initClockDrift2 = initClockDrift * initClockDrift;

        result.initialize(0.0);

        for (var i = 0; i < 3; i++) {
            result.setElementAt(i, i, initAttUnc2);
        }
        for (var i = 3; i < 6; i++) {
            result.setElementAt(i, i, initVelUnc2);
        }
        for (var i = 6; i < 9; i++) {
            result.setElementAt(i, i, initPosUnc2);
        }
        for (var i = 9; i < 12; i++) {
            result.setElementAt(i, i, initBaUnc2);
        }
        for (var i = 12; i < 15; i++) {
            result.setElementAt(i, i, initBgUnc2);
        }
        result.setElementAt(15, 15, initClockOffset2);
        result.setElementAt(16, 16, initClockDrift2);
    }

    /**
     * Initializes INS/GNS tightly coupled Kalman filter error covariance matrix.
     *
     * @param config Kalman filter configuration.
     * @return initialized error covariance matrix.
     */
    public static Matrix initialize(final INSTightlyCoupledKalmanInitializerConfig config) {
        Matrix result = null;
        try {
            result = new Matrix(NUM_PARAMS, NUM_PARAMS);
            initialize(config, result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return result;
    }
}
