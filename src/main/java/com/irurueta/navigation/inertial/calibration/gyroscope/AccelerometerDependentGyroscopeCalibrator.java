/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.units.Acceleration;

/**
 * Defines a gyroscope calibrator that depends on previously known accelerometer
 * parameters (bias, scaling and cross coupling errors).
 */
public interface AccelerometerDependentGyroscopeCalibrator extends GyroscopeCalibrator {

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known x-coordinate of accelerometer bias.
     */
    double getAccelerometerBiasX();

    /**
     * Sets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasX known x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBiasX(double accelerometerBiasX) throws LockedException;

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known y-coordinate of accelerometer bias.
     */
    double getAccelerometerBiasY();

    /**
     * Sets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasY known y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBiasY(double accelerometerBiasY) throws LockedException;

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known z-coordinate of accelerometer bias.
     */
    double getAccelerometerBiasZ();

    /**
     * Sets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasZ known z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBiasZ(double accelerometerBiasZ) throws LockedException;

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known x-coordinate of accelerometer bias.
     */
    Acceleration getAccelerometerBiasXAsAcceleration();

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    void getAccelerometerBiasXAsAcceleration(Acceleration result);

    /**
     * Sets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBiasX(Acceleration accelerometerBiasX) throws LockedException;

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known y-coordinate of accelerometer bias.
     */
    Acceleration getAccelerometerBiasYAsAcceleration();

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    void getAccelerometerBiasYAsAcceleration(Acceleration result);

    /**
     * Sets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBiasY(Acceleration accelerometerBiasY) throws LockedException;

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known z-coordinate of accelerometer bias.
     */
    Acceleration getAccelerometerBiasZAsAcceleration();

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    void getAccelerometerBiasZAsAcceleration(Acceleration result);

    /**
     * Sets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBiasZ(Acceleration accelerometerBiasZ) throws LockedException;

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBias(
            double accelerometerBiasX, double accelerometerBiasY, double accelerometerBiasZ)
            throws LockedException;

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerBias(
            Acceleration accelerometerBiasX,
            Acceleration accelerometerBiasY,
            Acceleration accelerometerBiasZ) throws LockedException;

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known accelerometer bias.
     */
    double[] getAccelerometerBias();

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    void getAccelerometerBias(double[] result);

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBias known accelerometer bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    void setAccelerometerBias(double[] accelerometerBias) throws LockedException;

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known accelerometer bias.
     */
    Matrix getAccelerometerBiasAsMatrix();

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getAccelerometerBiasAsMatrix(Matrix result);

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBias known accelerometer bias. Must be 3x1.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setAccelerometerBias(Matrix accelerometerBias) throws LockedException;

    /**
     * Gets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer x scaling factor.
     */
    double getAccelerometerSx();

    /**
     * Sets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSx known accelerometer x scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerSx(double accelerometerSx) throws LockedException;

    /**
     * Gets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer y scaling factor.
     */
    double getAccelerometerSy();

    /**
     * Sets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSy known accelerometer y scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerSy(double accelerometerSy) throws LockedException;

    /**
     * Gets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer z scaling factor.
     */
    double getAccelerometerSz();

    /**
     * Sets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSz known accelerometer z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerSz(double accelerometerSz) throws LockedException;

    /**
     * Gets known accelerometer x-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer x-y cross coupling error.
     */
    double getAccelerometerMxy();

    /**
     * Sets known accelerometer x-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxy known accelerometer x-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerMxy(double accelerometerMxy) throws LockedException;

    /**
     * Gets known accelerometer x-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer x-z cross coupling error.
     */
    double getAccelerometerMxz();

    /**
     * Sets known accelerometer x-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxz known accelerometer x-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerMxz(double accelerometerMxz) throws LockedException;

    /**
     * Gets known accelerometer y-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer y-x cross coupling error.
     */
    double getAccelerometerMyx();

    /**
     * Sets known accelerometer y-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerMyx(double accelerometerMyx) throws LockedException;

    /**
     * Gets known accelerometer y-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer y-z cross coupling error.
     */
    double getAccelerometerMyz();

    /**
     * Sets known accelerometer y-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerMyz(double accelerometerMyz) throws LockedException;

    /**
     * Gets known accelerometer z-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer z-x cross coupling error.
     */
    double getAccelerometerMzx();

    /**
     * Sets known accelerometer z-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerMzx(double accelerometerMzx) throws LockedException;

    /**
     * Gets known accelerometer z-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer z-y cross coupling error.
     */
    double getAccelerometerMzy();

    /**
     * Sets known accelerometer z-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerMzy(double accelerometerMzy) throws LockedException;

    /**
     * Sets known accelerometer scaling factors to be used to fix measured
     * specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerSx known accelerometer x scaling factor.
     * @param accelerometerSy known accelerometer y scaling factor.
     * @param accelerometerSz known accelerometer z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerScalingFactors(
            double accelerometerSx, double accelerometerSy, double accelerometerSz)
            throws LockedException;

    /**
     * Sets known accelerometer cross coupling errors to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxy known accelerometer x-y cross coupling
     *                         error.
     * @param accelerometerMxz known accelerometer x-z cross coupling
     *                         error.
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerCrossCouplingErrors(
            double accelerometerMxy, double accelerometerMxz,
            double accelerometerMyx, double accelerometerMyz,
            double accelerometerMzx, double accelerometerMzy) throws LockedException;

    /**
     * Sets known accelerometer scaling factors and cross coupling errors
     * to be used to fix measured specific force and find cross biases
     * introduced by the accelerometer.
     *
     * @param accelerometerSx  known accelerometer x scaling factor.
     * @param accelerometerSy  known accelerometer y scaling factor.
     * @param accelerometerSz  known accelerometer z scaling factor.
     * @param accelerometerMxy known accelerometer x-y cross coupling
     *                         error.
     * @param accelerometerMxz known accelerometer x-z cross coupling
     *                         error.
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    void setAccelerometerScalingFactorsAndCrossCouplingErrors(
            double accelerometerSx, double accelerometerSy,
            double accelerometerSz, double accelerometerMxy,
            double accelerometerMxz, double accelerometerMyx,
            double accelerometerMyz, double accelerometerMzx,
            double accelerometerMzy) throws LockedException;

    /**
     * Gets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @return known accelerometer scale factors and cross coupling
     * errors matrix.
     */
    Matrix getAccelerometerMa();

    /**
     * Gets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    void getAccelerometerMa(Matrix result);

    /**
     * Sets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @param accelerometerMa known accelerometer scale factors and
     *                        cross coupling errors matrix. Must be 3x3.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    void setAccelerometerMa(Matrix accelerometerMa) throws LockedException;
}
