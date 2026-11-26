/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.units.AngularSpeed;

/**
 * Interface for non-linear gyroscope calibrator where bias is unknown
 * and needs to be estimated.
 */
public interface UnknownBiasNonLinearGyroscopeCalibrator extends UnknownBiasGyroscopeCalibrator {

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    double getInitialBiasX();

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasX(double initialBiasX) throws LockedException;

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    double getInitialBiasY();

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasY(double initialBiasY) throws LockedException;

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    double getInitialBiasZ();

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasZ(double initialBiasZ) throws LockedException;

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    AngularSpeed getInitialBiasAngularSpeedX();

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    void getInitialBiasAngularSpeedX(AngularSpeed result);

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasX(AngularSpeed initialBiasX) throws LockedException;

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    AngularSpeed getInitialBiasAngularSpeedY();

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    void getInitialBiasAngularSpeedY(AngularSpeed result);

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasY(AngularSpeed initialBiasY) throws LockedException;

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    AngularSpeed getInitialBiasAngularSpeedZ();

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    void getInitialBiasAngularSpeedZ(AngularSpeed result);

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasZ(AngularSpeed initialBiasZ) throws LockedException;

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution
     * expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBias(double initialBiasX, double initialBiasY, double initialBiasZ)
            throws LockedException;

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBias(AngularSpeed initialBiasX, AngularSpeed initialBiasY,
                        AngularSpeed initialBiasZ) throws LockedException;

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of initial bias.
     */
    double[] getInitialBias();

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void getInitialBias(double[] result);

    /**
     * Sets initial bias to be used to find a solution as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void setInitialBias(double[] initialBias) throws LockedException;

    /**
     * Gets initial bias to be used to find a solution as a column matrix.
     *
     * @return initial bias to be used to find a solution as a column matrix.
     */
    Matrix getInitialBiasAsMatrix();

    /**
     * Gets initial bias to be used to find a solution as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getInitialBiasAsMatrix(Matrix result);

    /**
     * Sets initial bias to be used to find a solution as an array.
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setInitialBias(Matrix initialBias) throws LockedException;

    /**
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @return initial bias coordinates.
     */
    AngularSpeedTriad getInitialBiasAsTriad();

    /**
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    void getInitialBiasAsTriad(AngularSpeedTriad result);

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBias initial bias coordinates to be set.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBias(AngularSpeedTriad initialBias) throws LockedException;
}
