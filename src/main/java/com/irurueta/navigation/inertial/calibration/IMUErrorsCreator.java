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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.NotReadyException;

/**
 * Utility class to create {@link IMUErrors} by combining different sources of estimated data.
 * Sources of data can be any accelerometer calibrator implementing {@link AccelerometerCalibrationSource},
 * any gyroscope calibrator implementing {@link GyroscopeCalibrationSource}, or any measurement
 * generator, static interval detector or noise estimator implementing {@link AccelerometerNoiseRootPsdSource}
 * or {@link GyroscopeNoiseRootPsdSource}.
 */
public class IMUErrorsCreator {

    /**
     * A source of estimated accelerometer calibration parameters.
     */
    private AccelerometerCalibrationSource accelerometerCalibrationSource;

    /**
     * A source of estimated gyroscope calibration parameters.
     */
    private GyroscopeCalibrationSource gyroscopeCalibrationSource;

    /**
     * A source of estimated accelerometer noise root PSD.
     */
    private AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource;

    /**
     * A source of estimated gyroscope noise root PSD.
     */
    private GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource;

    /**
     * Accelerometer quantization level expressed in meters per squared second (m/s^2).
     * By default, it is zero when no quantization is assumed.
     */
    private double accelerometerQuantizationLevel;

    /**
     * Gyro quantization level expressed in radians per second (rad/s).
     * By default, it is zero when no quantization is assumed.
     */
    private double gyroQuantizationLevel;

    /**
     * Constructor.
     */
    public IMUErrorsCreator() {
    }

    /**
     * Constructor.
     *
     * @param accelerometerCalibrationSource  A source of estimated accelerometer calibration parameters.
     * @param gyroscopeCalibrationSource      A source of estimated gyroscope calibration parameters.
     * @param accelerometerNoiseRootPsdSource A source of estimated accelerometer noise root PSD.
     * @param gyroscopeNoiseRootPsdSource     A source of estimated gyroscope noise root PSD.
     */
    public IMUErrorsCreator(
            final AccelerometerCalibrationSource accelerometerCalibrationSource,
            final GyroscopeCalibrationSource gyroscopeCalibrationSource,
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource,
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource) {
        this.accelerometerCalibrationSource = accelerometerCalibrationSource;
        this.gyroscopeCalibrationSource = gyroscopeCalibrationSource;
        this.accelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
        this.gyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
    }

    /**
     * Constructor.
     *
     * @param accelerometerCalibrationSource  A source of estimated accelerometer calibration parameters.
     * @param gyroscopeCalibrationSource      A source of estimated gyroscope calibration parameters.
     * @param accelerometerNoiseRootPsdSource A source of estimated accelerometer noise root PSD.
     * @param gyroscopeNoiseRootPsdSource     A source of estimated gyroscope noise root PSD.
     * @param accelerometerQuantizationLevel  Accelerometer quantization level expressed in meters per squared
     *                                        second (m/s^2).
     * @param gyroQuantizationLevel           Gyro quantization level expressed in radians per second (rad/s).
     */
    public IMUErrorsCreator(
            final AccelerometerCalibrationSource accelerometerCalibrationSource,
            final GyroscopeCalibrationSource gyroscopeCalibrationSource,
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource,
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource,
            final double accelerometerQuantizationLevel,
            final double gyroQuantizationLevel) {
        this(accelerometerCalibrationSource, gyroscopeCalibrationSource, accelerometerNoiseRootPsdSource,
                gyroscopeNoiseRootPsdSource);
        this.accelerometerQuantizationLevel = accelerometerQuantizationLevel;
        this.gyroQuantizationLevel = gyroQuantizationLevel;
    }

    /**
     * Gets source of estimated accelerometer calibration parameters.
     *
     * @return source of estimated accelerometer calibration parameters.
     */
    public AccelerometerCalibrationSource getAccelerometerCalibrationSource() {
        return accelerometerCalibrationSource;
    }

    /**
     * Sets source of estimated accelerometer calibration parameters.
     *
     * @param accelerometerCalibrationSource source of estimated accelerometer
     *                                       parameters.
     */
    public void setAccelerometerCalibrationSource(final AccelerometerCalibrationSource accelerometerCalibrationSource) {
        this.accelerometerCalibrationSource = accelerometerCalibrationSource;
    }

    /**
     * Gets source of estimated gyroscope calibration parameters.
     *
     * @return source of estimated gyroscope calibration parameters.
     */
    public GyroscopeCalibrationSource getGyroscopeCalibrationSource() {
        return gyroscopeCalibrationSource;
    }

    /**
     * Sets source of estimated gyroscope calibration parameters.
     *
     * @param gyroscopeCalibrationSource source of estimated gyroscope calibration
     *                                   parameters.
     */
    public void setGyroscopeCalibrationSource(final GyroscopeCalibrationSource gyroscopeCalibrationSource) {
        this.gyroscopeCalibrationSource = gyroscopeCalibrationSource;
    }

    /**
     * Gets source of estimated accelerometer noise root PSD.
     *
     * @return source of estimated accelerometer noise root PSD.
     */
    public AccelerometerNoiseRootPsdSource getAccelerometerNoiseRootPsdSource() {
        return accelerometerNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated accelerometer noise root PSD.
     *
     * @param accelerometerNoiseRootPsdSource source of estimated accelerometer
     *                                        noise root PSD.
     */
    public void setAccelerometerNoiseRootPsdSource(
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource) {
        this.accelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
    }

    /**
     * Gets source of estimated gyroscope noise root PSD.
     *
     * @return source of estimated gyroscope noise root PSD.
     */
    public GyroscopeNoiseRootPsdSource getGyroscopeNoiseRootPsdSource() {
        return gyroscopeNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated gyroscope noise root PSD.
     *
     * @param gyroscopeNoiseRootPsdSource source of estimated gyroscope noise
     *                                    root PSD.
     */
    public void sstGyroscopeNoiseRootPsdSource(final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource) {
        this.gyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
    }

    /**
     * Gets accelerometer quantization level expressed in meters per squared second
     * (m/s^2).
     * By default it is zero when no quantization is assumed.
     *
     * @return accelerometer quantization level.
     */
    public double getAccelerometerQuantizationLevel() {
        return accelerometerQuantizationLevel;
    }

    /**
     * Sets accelerometer quantization level expressed in meters per squared second
     * (m/s^2).
     *
     * @param accelerometerQuantizationLevel accelerometer quantization level.
     */
    public void setAccelerometerQuantizationLevel(final double accelerometerQuantizationLevel) {
        this.accelerometerQuantizationLevel = accelerometerQuantizationLevel;
    }

    /**
     * Gets gyroscope quantization level expressed in radians per second (rad/s).
     * By default it is zero when no quantization is assumed.
     *
     * @return gyroscope quantization level.
     */
    public double getGyroQuantizationLevel() {
        return gyroQuantizationLevel;
    }

    /**
     * Sets gyroscope quantization level expressed in radians per second (rad/s).
     *
     * @param gyroQuantizationLevel gyroscope quantization level.
     */
    public void setGyroQuantizationLevel(final double gyroQuantizationLevel) {
        this.gyroQuantizationLevel = gyroQuantizationLevel;
    }

    /**
     * Indicates whether all sources have been provided in order to be able
     * to create an {@link IMUErrors} instance.
     *
     * @return true if creator is ready, false otherwise.
     */
    public boolean isReady() {
        return accelerometerNoiseRootPsdSource != null && gyroscopeNoiseRootPsdSource != null
                && accelerometerCalibrationSource != null
                && accelerometerCalibrationSource.getEstimatedBiases() != null
                && accelerometerCalibrationSource.getEstimatedMa() != null
                && gyroscopeCalibrationSource != null
                && gyroscopeCalibrationSource.getEstimatedBiases() != null
                && gyroscopeCalibrationSource.getEstimatedMg() != null
                && gyroscopeCalibrationSource.getEstimatedGg() != null;
    }

    /**
     * Creates an {@link IMUErrors} instance containing estimated IMU
     * (accelerometer + gyroscope) parameters.
     *
     * @return instance containing estimated IMU parameters.
     * @throws NotReadyException if creator is not ready.
     */
    public IMUErrors create() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var ba = accelerometerCalibrationSource.getEstimatedBiases();
        final var ma = accelerometerCalibrationSource.getEstimatedMa();

        final var bg = gyroscopeCalibrationSource.getEstimatedBiases();
        final var mg = gyroscopeCalibrationSource.getEstimatedMg();
        final var gg = gyroscopeCalibrationSource.getEstimatedGg();

        final var accelerometerNoiseRootPsd = accelerometerNoiseRootPsdSource.getAccelerometerBaseNoiseLevelRootPsd();
        final var gyroscopeNoiseRootPsd = gyroscopeNoiseRootPsdSource.getGyroscopeBaseNoiseLevelRootPsd();

        return new IMUErrors(ba, bg, ma, mg, gg, accelerometerNoiseRootPsd, gyroscopeNoiseRootPsd,
                accelerometerQuantizationLevel, gyroQuantizationLevel);
    }
}
