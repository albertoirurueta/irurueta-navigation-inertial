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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.Arrays;
import java.util.Objects;

/**
 * Contains Inertial Measurement Unit (IMU) errors statistics obtained from
 * calibration.
 * This data can also be used to generate synthetic IMU data.
 * IMU errors are related to accelerometer and gyroscope calibration parameters.
 */
public class IMUErrors implements Serializable, Cloneable {

    /**
     * Number of components of accelerometer measures.
     */
    public static final int ACCELEROMETER_COMPONENTS = 3;

    /**
     * Number of components og gyro measures.
     */
    public static final int GYRO_COMPONENTS = 3;

    /**
     * Number of components minus one.
     */
    private static final int COMPONENTS_MINUS_ONE = 2;

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2).
     * By default, it is assumed to be all zeros.
     */
    private double[] accelerometerBiases = new double[ACCELEROMETER_COMPONENTS];

    /**
     * Gyro biases for each IMU axis expressed in radians per second (rad/s).
     * By default, it is assumed to be all zeros.
     */
    private double[] gyroBiases = new double[GYRO_COMPONENTS];

    /**
     * Contains accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So that:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     * By default, it is the 3x3 zero matrix.
     */
    private Matrix accelerometerScaleFactorAndCrossCouplingErrors;

    /**
     * Contains gyro scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -gammaXy    gammaXz ]
     *          [gammaYx    1           -gammaYz]
     *          [-gammaZx   gammaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * gammaXy   sz * gammaXz ]
     *          [myx   sy   myz]            [sx * gammaYx   sy              -sz * gammaYz]
     *          [mzx   mzy  sz ]            [-sx * gammaZx  sy * gammaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically gammaYx, gammaZx and gammaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     * By default, it is the 3x3 zero matrix.
     */
    private Matrix gyroScaleFactorAndCrossCouplingErrors;

    /**
     * 3x3 matrix containing cross biases introduced by the specific forces sensed
     * by the accelerometer.
     * Values of this matrix are expressed in (rad-sec/m).
     * By default, it is all zeros.
     */
    private Matrix gyroGDependentBiases;

    /**
     * Accelerometer noise root PSD expressed in (m * s^-1.5).
     * By default it is zero.
     */
    private double accelerometerNoiseRootPSD;

    /**
     * Gyro noise root PSD expressed in (rad * s^-0.5).
     * By default, it is zero.
     */
    private double gyroNoiseRootPSD;

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
    public IMUErrors() {
        try {
            accelerometerScaleFactorAndCrossCouplingErrors = Matrix.identity(
                    ACCELEROMETER_COMPONENTS, ACCELEROMETER_COMPONENTS);
            gyroScaleFactorAndCrossCouplingErrors = Matrix.identity(GYRO_COMPONENTS, GYRO_COMPONENTS);
            gyroGDependentBiases = new Matrix(ACCELEROMETER_COMPONENTS, ACCELEROMETER_COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiases                            accelerometer biases for each IMU axis expressed in meters
     *                                                       per squared second (m/s^2). Must have length 3.
     * @param gyroBiases                                     gyro biases for each IMU axis expressed in radians per
     *                                                       second (rad/s). Must have length 3.
     * @param accelerometerScaleFactorAndCrossCouplingErrors accelerometer scale factors and cross coupling errors.
     *                                                       Must be 3x3.
     * @param gyroScaleFactorAndCrossCouplingErrors          gyro scale factors and cross coupling errors. Must be 3x3.
     * @param accelerometerNoiseRootPSD                      accelerometer noise root PSD expressed in (m * s^-1.5).
     * @param gyroNoiseRootPSD                               gyro noise root PSD expressed in (rad * s^-0.5).
     * @throws IllegalArgumentException if any value is invalid.
     */
    public IMUErrors(final double[] accelerometerBiases, final double[] gyroBiases,
                     final Matrix accelerometerScaleFactorAndCrossCouplingErrors,
                     final Matrix gyroScaleFactorAndCrossCouplingErrors, final double accelerometerNoiseRootPSD,
                     final double gyroNoiseRootPSD) {
        this();
        setAccelerometerBiases(accelerometerBiases);
        setGyroBiases(gyroBiases);
        setAccelerometerScaleFactorAndCrossCouplingErrors(accelerometerScaleFactorAndCrossCouplingErrors);
        setGyroScaleFactorAndCrossCouplingErrors(gyroScaleFactorAndCrossCouplingErrors);
        setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);
        setGyroNoiseRootPSD(gyroNoiseRootPSD);
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiases                            accelerometer biases for each IMU axis expressed in meters
     *                                                       per squared second (m/s^2). Must be 3x1.
     * @param gyroBiases                                     gyro biases for each IMU axis expressed in radians per
     *                                                       second (rad/s). Must be 3x1.
     * @param accelerometerScaleFactorAndCrossCouplingErrors accelerometer scale factors and cross coupling errors.
     *                                                       Must be 3x3.
     * @param gyroScaleFactorAndCrossCouplingErrors          gyro scale factors and cross coupling errors. Must be 3x3.
     * @param accelerometerNoiseRootPSD                      accelerometer noise root PSD expressed in (m * s^-1.5).
     * @param gyroNoiseRootPSD                               gyro noise root PSD expressed in (rad * s^-0.5).
     * @throws IllegalArgumentException if any value is invalid.
     */
    public IMUErrors(final Matrix accelerometerBiases, final Matrix gyroBiases,
                     final Matrix accelerometerScaleFactorAndCrossCouplingErrors,
                     final Matrix gyroScaleFactorAndCrossCouplingErrors, final double accelerometerNoiseRootPSD,
                     final double gyroNoiseRootPSD) {
        this();
        setAccelerometerBiases(accelerometerBiases);
        setGyroBiases(gyroBiases);
        setAccelerometerScaleFactorAndCrossCouplingErrors(accelerometerScaleFactorAndCrossCouplingErrors);
        setGyroScaleFactorAndCrossCouplingErrors(gyroScaleFactorAndCrossCouplingErrors);
        setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);
        setGyroNoiseRootPSD(gyroNoiseRootPSD);
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiases                            accelerometer biases for each IMU axis. Must have length 3.
     * @param gyroBiases                                     gyro biases for each IMU axis. Must have length 3.
     * @param accelerometerScaleFactorAndCrossCouplingErrors accelerometer scale factors and cross coupling errors.
     *                                                       Must be 3x3.
     * @param gyroScaleFactorAndCrossCouplingErrors          gyro scale factors and cross coupling errors. Must be 3x3.
     * @param accelerometerNoiseRootPSD                      accelerometer noise root PSD expressed in (m * s^-1.5).
     * @param gyroNoiseRootPSD                               gyro noise root PSD expressed in (rad * s^-0.5).
     * @throws IllegalArgumentException if any value is invalid.
     */
    public IMUErrors(final Acceleration[] accelerometerBiases, final AngularSpeed[] gyroBiases,
                     final Matrix accelerometerScaleFactorAndCrossCouplingErrors,
                     final Matrix gyroScaleFactorAndCrossCouplingErrors, final double accelerometerNoiseRootPSD,
                     final double gyroNoiseRootPSD) {
        this();
        setAccelerometerBiases(accelerometerBiases);
        setGyroBiases(gyroBiases);
        setAccelerometerScaleFactorAndCrossCouplingErrors(accelerometerScaleFactorAndCrossCouplingErrors);
        setGyroScaleFactorAndCrossCouplingErrors(gyroScaleFactorAndCrossCouplingErrors);
        setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);
        setGyroNoiseRootPSD(gyroNoiseRootPSD);
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiases                            accelerometer biases for each IMU axis expressed in meters
     *                                                       per squared second (m/s^2). Must have length 3.
     * @param gyroBiases                                     gyro biases for each IMU axis expressed in radians per
     *                                                       second (rad/s). Must have length 3.
     * @param accelerometerScaleFactorAndCrossCouplingErrors accelerometer scale factors and cross coupling errors.
     *                                                       Must be 3x3.
     * @param gyroScaleFactorAndCrossCouplingErrors          gyro scale factors and cross coupling errors. Must be 3x3.
     * @param gyroGDependentBiases                           Cross biases introduced by the specific forces sensed by
     *                                                       the accelerometer expressed in (rad-sec/m). Must be 3x3.
     * @param accelerometerNoiseRootPSD                      accelerometer noise root PSD expressed in (m * s^-1.5).
     * @param gyroNoiseRootPSD                               gyro noise root PSD expressed in (rad * s^-0.5).
     * @param accelerometerQuantizationLevel                 accelerometer quantization level expressed in meters per
     *                                                       squared second (m/s^2).
     * @param gyroQuantizationLevel                          gyro quantization level expressed in radians per second
     *                                                       (rad/s).
     * @throws IllegalArgumentException if any value is invalid.
     */
    public IMUErrors(final double[] accelerometerBiases, final double[] gyroBiases,
                     final Matrix accelerometerScaleFactorAndCrossCouplingErrors,
                     final Matrix gyroScaleFactorAndCrossCouplingErrors, final Matrix gyroGDependentBiases,
                     final double accelerometerNoiseRootPSD, final double gyroNoiseRootPSD,
                     final double accelerometerQuantizationLevel, final double gyroQuantizationLevel) {
        this();
        setAccelerometerBiases(accelerometerBiases);
        setGyroBiases(gyroBiases);
        setAccelerometerScaleFactorAndCrossCouplingErrors(accelerometerScaleFactorAndCrossCouplingErrors);
        setGyroScaleFactorAndCrossCouplingErrors(gyroScaleFactorAndCrossCouplingErrors);
        setGyroGDependentBiases(gyroGDependentBiases);
        setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);
        setGyroNoiseRootPSD(gyroNoiseRootPSD);
        setAccelerometerQuantizationLevel(accelerometerQuantizationLevel);
        setGyroQuantizationLevel(gyroQuantizationLevel);
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiases                            accelerometer biases for each IMU axis expressed in meters
     *                                                       per squared second (m/s^2). Must be 3x1.
     * @param gyroBiases                                     gyro biases for each IMU axis expressed in radians per
     *                                                       second (rad/s). Must be 3x1.
     * @param accelerometerScaleFactorAndCrossCouplingErrors accelerometer scale factors and cross coupling errors.
     *                                                       Must be 3x3.
     * @param gyroScaleFactorAndCrossCouplingErrors          gyro scale factors and cross coupling errors. Must be 3x3.
     * @param gyroGDependentBiases                           Cross biases introduced by the specific forces sensed by
     *                                                       the accelerometer expressed in (rad-sec/m). Must be 3x3.
     * @param accelerometerNoiseRootPSD                      accelerometer noise root PSD expressed in (m * s^-1.5).
     * @param gyroNoiseRootPSD                               gyro noise root PSD expressed in (rad * s^-0.5).
     * @param accelerometerQuantizationLevel                 accelerometer quantization level expressed in meters per
     *                                                       squared second (m/s^2).
     * @param gyroQuantizationLevel                          gyro quantization level expressed in radians per second
     *                                                       (rad/s).
     * @throws IllegalArgumentException if any value is invalid.
     */
    public IMUErrors(final Matrix accelerometerBiases, final Matrix gyroBiases,
                     final Matrix accelerometerScaleFactorAndCrossCouplingErrors,
                     final Matrix gyroScaleFactorAndCrossCouplingErrors, final Matrix gyroGDependentBiases,
                     final double accelerometerNoiseRootPSD, final double gyroNoiseRootPSD,
                     final double accelerometerQuantizationLevel, final double gyroQuantizationLevel) {
        this();
        setAccelerometerBiases(accelerometerBiases);
        setGyroBiases(gyroBiases);
        setAccelerometerScaleFactorAndCrossCouplingErrors(accelerometerScaleFactorAndCrossCouplingErrors);
        setGyroScaleFactorAndCrossCouplingErrors(gyroScaleFactorAndCrossCouplingErrors);
        setGyroGDependentBiases(gyroGDependentBiases);
        setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);
        setGyroNoiseRootPSD(gyroNoiseRootPSD);
        setAccelerometerQuantizationLevel(accelerometerQuantizationLevel);
        setGyroQuantizationLevel(gyroQuantizationLevel);
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiases                            accelerometer biases for each IMU axis. Must have length 3.
     * @param gyroBiases                                     gyro biases for each IMU axis. Must have length 3.
     * @param accelerometerScaleFactorAndCrossCouplingErrors accelerometer scale factors and cross coupling errors.
     *                                                       Must be 3x3.
     * @param gyroScaleFactorAndCrossCouplingErrors          gyro scale factors and cross coupling errors. Must be 3x3.
     * @param gyroGDependentBiases                           Cross biases introduced by the specific forces sensed by
     *                                                       the accelerometer expressed in (rad-sec/m). Must be 3x3.
     * @param accelerometerNoiseRootPSD                      accelerometer noise root PSD expressed in (m * s^-1.5).
     * @param gyroNoiseRootPSD                               gyro noise root PSD expressed in (rad * s^-0.5).
     * @param accelerometerQuantizationLevel                 accelerometer quantization level.
     * @param gyroQuantizationLevel                          gyro quantization level.
     * @throws IllegalArgumentException if any value is invalid.
     */
    public IMUErrors(final Acceleration[] accelerometerBiases, final AngularSpeed[] gyroBiases,
                     final Matrix accelerometerScaleFactorAndCrossCouplingErrors,
                     final Matrix gyroScaleFactorAndCrossCouplingErrors, final Matrix gyroGDependentBiases,
                     final double accelerometerNoiseRootPSD, final double gyroNoiseRootPSD,
                     final Acceleration accelerometerQuantizationLevel,
                     final AngularSpeed gyroQuantizationLevel) {
        this();
        setAccelerometerBiases(accelerometerBiases);
        setGyroBiases(gyroBiases);
        setAccelerometerScaleFactorAndCrossCouplingErrors(accelerometerScaleFactorAndCrossCouplingErrors);
        setGyroScaleFactorAndCrossCouplingErrors(gyroScaleFactorAndCrossCouplingErrors);
        setGyroGDependentBiases(gyroGDependentBiases);
        setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);
        setGyroNoiseRootPSD(gyroNoiseRootPSD);
        setAccelerometerQuantizationLevel(accelerometerQuantizationLevel);
        setGyroQuantizationLevel(gyroQuantizationLevel);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    @SuppressWarnings("CopyConstructorMissesField")
    public IMUErrors(final IMUErrors input) {
        this();
        copyFrom(input);
    }

    /**
     * Gets accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2).
     * By default, it is assumed to be all zeros.
     *
     * @return accelerometer biases for each IMU axis.
     */
    public double[] getAccelerometerBiases() {
        final var result = new double[ACCELEROMETER_COMPONENTS];
        getAccelerometerBiases(result);
        return result;
    }

    /**
     * Gets accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2).
     * By default, it is assumed to be all zeros.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAccelerometerBiases(final double[] result) {
        if (result.length != ACCELEROMETER_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        System.arraycopy(accelerometerBiases, 0, result, 0, ACCELEROMETER_COMPONENTS);
    }

    /**
     * Sets accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2).
     *
     * @param accelerometerBiases accelerometer biases for each IMU axis.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setAccelerometerBiases(final double[] accelerometerBiases) {
        if (accelerometerBiases.length != ACCELEROMETER_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        this.accelerometerBiases = accelerometerBiases;
    }

    /**
     * Gets accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2) as a column matrix.
     * By default, it is assumed to be all zeros.
     *
     * @return 3x1 column matrix containing accelerometer biases for each IMU axis.
     */
    public Matrix getAccelerometerBiasesAsMatrix() {
        return Matrix.newFromArray(accelerometerBiases);
    }

    /**
     * Gets accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2) as a column matrix.
     * By default, it is assumed to be all zeros.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void getAccelerometerBiasesAsMatrix(final Matrix result) {
        result.setSubmatrix(0, 0, COMPONENTS_MINUS_ONE, 0,
                accelerometerBiases);
    }

    /**
     * Sets accelerometer biases for each IMU axis expressed in meters per squared
     * second (m/s^2) from a 3x1 column matrix.
     *
     * @param accelerometerBiases 3x1 column matrix containing values to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setAccelerometerBiases(final Matrix accelerometerBiases) {
        if (accelerometerBiases.getRows() != ACCELEROMETER_COMPONENTS || accelerometerBiases.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        try {
            accelerometerBiases.getSubmatrixAsArray(0, 0,
                    COMPONENTS_MINUS_ONE, 0, this.accelerometerBiases);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets accelerometer biases for each IMU axis.
     * By default, it is assumed to be all zeros.
     *
     * @return accelerometer biases for each IMU axis.
     */
    public Acceleration[] getAccelerometerBiasesAsAcceleration() {
        final var result = new Acceleration[ACCELEROMETER_COMPONENTS];
        getAccelerometerBiasesAsAcceleration(result);
        return result;
    }

    /**
     * Gets accelerometer biases for each IMU axis.
     * By default, it is assumed to be all zeros.
     *
     * @param result instance where data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAccelerometerBiasesAsAcceleration(final Acceleration[] result) {
        if (result.length != ACCELEROMETER_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        for (var i = 0; i < ACCELEROMETER_COMPONENTS; i++) {
            final var a = result[i];
            if (a == null) {
                result[i] = new Acceleration(accelerometerBiases[i], AccelerationUnit.METERS_PER_SQUARED_SECOND);
            } else {
                a.setValue(accelerometerBiases[i]);
                a.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            }
        }
    }

    /**
     * Sets accelerometer biases for each IMU axis.
     *
     * @param accelerometerBiases accelerometer biases to be set.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setAccelerometerBiases(final Acceleration[] accelerometerBiases) {
        if (accelerometerBiases.length != ACCELEROMETER_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        for (var i = 0; i < ACCELEROMETER_COMPONENTS; i++) {
            this.accelerometerBiases[i] = convertAcceleration(accelerometerBiases[i]);
        }
    }

    /**
     * Gets gyro biases for each IMU axis expressed in radians per second (rad/s).
     * By default, it is assumed to be all zeros.
     *
     * @return gyro biases for each IMU axis.
     */
    public double[] getGyroBiases() {
        final var result = new double[GYRO_COMPONENTS];
        getGyroBiases(result);
        return result;
    }

    /**
     * Gets gyro biases for each IMU axis expressed in radians per second (rad/s).
     * By default, it is assumed to be all zeros.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getGyroBiases(final double[] result) {
        if (result.length != GYRO_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        System.arraycopy(gyroBiases, 0, result, 0, GYRO_COMPONENTS);
    }

    /**
     * Sets gyro biases for each IMU axis expressed in radians per second (rad/s).
     *
     * @param gyroBiases gyro biases for each IMU axis.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setGyroBiases(final double[] gyroBiases) {
        if (gyroBiases.length != GYRO_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        this.gyroBiases = gyroBiases;
    }

    /**
     * Gets gyro biases for each IMU axis expressed in radians per second (rad/s)
     * as a column matrix.
     * By default, it is assumed to be all zeros.
     *
     * @return 3x1 column matrix containing gyro biases for each IMU axis.
     */
    public Matrix getGyroBiasesAsMatrix() {
        return Matrix.newFromArray(gyroBiases);
    }

    /**
     * Gets gyro biases for each IMU axis expressed in radians per second (rad/s)
     * as a column matrix.
     * By default, it is assumed to be all zeros.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void getGyroBiasesAsMatrix(final Matrix result) {
        result.setSubmatrix(0, 0, COMPONENTS_MINUS_ONE, 0, gyroBiases);
    }

    /**
     * Sets gyro biases for each IMU axis expressed in radians per second (rad/s)
     * from a 3x1 column matrix.
     *
     * @param gyroBiases 3x1 column matrix containing values to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setGyroBiases(final Matrix gyroBiases) {
        if (gyroBiases.getRows() != GYRO_COMPONENTS || gyroBiases.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        try {
            gyroBiases.getSubmatrixAsArray(0, 0, COMPONENTS_MINUS_ONE, 0,
                    this.gyroBiases);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets gyro biases for each IMU axis.
     * By default, it is assumed to be all zeros.
     *
     * @return gyro biases for each IMU axis.
     */
    public AngularSpeed[] getGyroBiasesAsAngularSpeed() {
        final var result = new AngularSpeed[GYRO_COMPONENTS];
        getGyroBiasesAsAngularSpeed(result);
        return result;
    }

    /**
     * Gets gyro biases for each IMU axis.
     * By default, it is assumed to be all zeros.
     *
     * @param result instance where data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getGyroBiasesAsAngularSpeed(final AngularSpeed[] result) {
        if (result.length != GYRO_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        for (var i = 0; i < GYRO_COMPONENTS; i++) {
            final var as = result[i];
            if (as == null) {
                result[i] = new AngularSpeed(gyroBiases[i], AngularSpeedUnit.RADIANS_PER_SECOND);
            } else {
                as.setValue(gyroBiases[i]);
                as.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            }
        }
    }

    /**
     * Sets gyro biases for each IMU axis.
     *
     * @param gyroBiases gyro biases to be set.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setGyroBiases(final AngularSpeed[] gyroBiases) {
        if (gyroBiases.length != GYRO_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        for (int i = 0; i < GYRO_COMPONENTS; i++) {
            this.gyroBiases[i] = convertAngularSpeed(gyroBiases[i]);
        }
    }

    /**
     * Gets accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So that:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     * By default, it is the 3x3 identity matrix.
     *
     * @return accelerometer scale factors and cross coupling errors.
     */
    public Matrix getAccelerometerScaleFactorAndCrossCouplingErrors() {
        return new Matrix(accelerometerScaleFactorAndCrossCouplingErrors);
    }

    /**
     * Gets accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So that:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     * By default, it is the 3x3 identity matrix.
     *
     * @param result instance where data of scale factor and cross coupling matrix will
     *               be copied to. If needed, result instance will be resized.
     */
    public void getAccelerometerScaleFactorAndCrossCouplingErrors(final Matrix result) {
        result.copyFrom(accelerometerScaleFactorAndCrossCouplingErrors);
    }

    /**
     * Sets accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So that:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @param accelerometerScaleFactorAndCrossCouplingErrors scale factors and cross coupling
     *                                                       matrix to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setAccelerometerScaleFactorAndCrossCouplingErrors(
            final Matrix accelerometerScaleFactorAndCrossCouplingErrors) {
        if (accelerometerScaleFactorAndCrossCouplingErrors.getRows() != ACCELEROMETER_COMPONENTS
                || accelerometerScaleFactorAndCrossCouplingErrors.getColumns() != ACCELEROMETER_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        this.accelerometerScaleFactorAndCrossCouplingErrors.copyFrom(accelerometerScaleFactorAndCrossCouplingErrors);
    }

    /**
     * Gets gyro scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -gammaXy    gammaXz ]
     *          [gammaYx    1           -gammaYz]
     *          [-gammaZx   gammaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * gammaXy   sz * gammaXz ]
     *          [myx   sy   myz]            [sx * gammaYx   sy              -sz * gammaYz]
     *          [mzx   mzy  sz ]            [-sx * gammaZx  sy * gammaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically gammaYx, gammaZx and gammaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     * By default, it is the 3x3 identity matrix.
     *
     * @return gyro scale factors and cross coupling errors.
     */
    public Matrix getGyroScaleFactorAndCrossCouplingErrors() {
        return new Matrix(gyroScaleFactorAndCrossCouplingErrors);
    }

    /**
     * Gets gyro scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -gammaXy    gammaXz ]
     *          [gammaYx    1           -gammaYz]
     *          [-gammaZx   gammaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * gammaXy   sz * gammaXz ]
     *          [myx   sy   myz]            [sx * gammaYx   sy              -sz * gammaYz]
     *          [mzx   mzy  sz ]            [-sx * gammaZx  sy * gammaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically gammaYx, gammaZx and gammaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     * By default, it is the 3x3 identity matrix.
     *
     * @param result instance where data of scale factor and cross coupling matrix will
     *               be copied to. If needed, result instance will be resized.
     */
    public void getGyroScaleFactorAndCrossCouplingErrors(final Matrix result) {
        result.copyFrom(gyroScaleFactorAndCrossCouplingErrors);
    }

    /**
     * Sets gyro scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -gammaXy    gammaXz ]
     *          [gammaYx    1           -gammaYz]
     *          [-gammaZx   gammaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * gammaXy   sz * gammaXz ]
     *          [myx   sy   myz]            [sx * gammaYx   sy              -sz * gammaYz]
     *          [mzx   mzy  sz ]            [-sx * gammaZx  sy * gammaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically gammaYx, gammaZx and gammaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @param gyroScaleFactorAndCrossCouplingErrors scale factors and cross coupling
     *                                              matrix to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setGyroScaleFactorAndCrossCouplingErrors(final Matrix gyroScaleFactorAndCrossCouplingErrors) {
        if (gyroScaleFactorAndCrossCouplingErrors.getRows() != GYRO_COMPONENTS
                || gyroScaleFactorAndCrossCouplingErrors.getColumns() != GYRO_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        this.gyroScaleFactorAndCrossCouplingErrors.copyFrom(gyroScaleFactorAndCrossCouplingErrors);
    }

    /**
     * Gets 3x3 matrix containing cross biases introduced by the specific forces
     * sensed by the accelerometer.
     * Values of this matrix are expressed in (rad-sec/m).
     * By default, it is all zeros.
     *
     * @return cross biases introduced by the specific forces sensed by the
     * accelerometer.
     */
    public Matrix getGyroGDependentBiases() {
        return new Matrix(gyroGDependentBiases);
    }

    /**
     * Gets 3x3 matrix containing cross biases introduced by the specific forces
     * sensed by the accelerometer.
     * Values of this matrix are expressed in (rad-sec/m).
     * By default, it is all zeros.
     *
     * @param result instance where data will be stored. If needed, result instance
     *               will be resized.
     */
    public void getGyroGDependentBiases(final Matrix result) {
        result.copyFrom(gyroGDependentBiases);
    }

    /**
     * Sets 3x3 matrix containing cross biases introduced by the specific forces
     * sensed by the accelerometer.
     * Values of this matrix are expressed in (rad-sec/m).
     *
     * @param gyroGDependentBiases cross biases introduced by the specific forces
     *                             sensed by the accelerometer to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setGyroGDependentBiases(final Matrix gyroGDependentBiases) {
        if (gyroGDependentBiases.getRows() != ACCELEROMETER_COMPONENTS
                || gyroGDependentBiases.getColumns() != ACCELEROMETER_COMPONENTS) {
            throw new IllegalArgumentException();
        }

        this.gyroGDependentBiases.copyFrom(gyroGDependentBiases);
    }

    /**
     * Gets accelerometer noise root PSD expressed in (m * s^-1.5).
     * By default it is zero.
     *
     * @return accelerometer noise root PSD.
     */
    public double getAccelerometerNoiseRootPSD() {
        return accelerometerNoiseRootPSD;
    }

    /**
     * Sets accelerometer noise root PSD expressed in (m * s^-1.5)
     *
     * @param accelerometerNoiseRootPSD accelerometer noise root PSD to be set.
     */
    public void setAccelerometerNoiseRootPSD(final double accelerometerNoiseRootPSD) {
        this.accelerometerNoiseRootPSD = accelerometerNoiseRootPSD;
    }

    /**
     * Gets accelerometer noise PSD expressed in (m^2 * s^-3).
     * By default, it is zero.
     *
     * @return accelerometer noise PSD.
     */
    public double getAccelerometerNoisePSD() {
        return accelerometerNoiseRootPSD * accelerometerNoiseRootPSD;
    }

    /**
     * Sets accelerometer noise PSD expressed in (m^2 * s^-3).
     *
     * @param accelerometerNoisePSD accelerometer noise PSD to be set.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setAccelerometerNoisePSD(final double accelerometerNoisePSD) {
        if (accelerometerNoisePSD < 0.0) {
            throw new IllegalArgumentException();
        }

        accelerometerNoiseRootPSD = Math.sqrt(accelerometerNoisePSD);
    }

    /**
     * Gets gyro noise root PSD expressed in (rad * s^-0.5).
     * By default, it is zero.
     *
     * @return gyro noise root PSD.
     */
    public double getGyroNoiseRootPSD() {
        return gyroNoiseRootPSD;
    }

    /**
     * Sets gyro noise root PSD expressed in (rad * s^-0.5).
     *
     * @param gyroNoiseRootPSD gyro noise root PSD to be set.
     */
    public void setGyroNoiseRootPSD(final double gyroNoiseRootPSD) {
        this.gyroNoiseRootPSD = gyroNoiseRootPSD;
    }

    /**
     * Gets gyro noise PSD expressed in (rad^2/s).
     * By default, it is zero.
     *
     * @return gyro noise PSD.
     */
    public double getGyroNoisePSD() {
        return gyroNoiseRootPSD * gyroNoiseRootPSD;
    }

    /**
     * Sets gyro noise PSD expressed in (rad^2/s).
     *
     * @param gyroNoisePSD gyro noise PSD.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setGyroNoisePSD(final double gyroNoisePSD) {
        if (gyroNoisePSD < 0.0) {
            throw new IllegalArgumentException();
        }

        gyroNoiseRootPSD = Math.sqrt(gyroNoisePSD);
    }

    /**
     * Gets accelerometer quantization level expressed in meters per squared second
     * (m/s^2).
     * By default, it is zero when no quantization is assumed.
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
     * @param accelerometerQuantizationLevel accelerometer quantization level to be
     *                                       set.
     */
    public void setAccelerometerQuantizationLevel(final double accelerometerQuantizationLevel) {
        this.accelerometerQuantizationLevel = accelerometerQuantizationLevel;
    }

    /**
     * Gets accelerometer quantization level.
     * By default, it is zero when no quantization is assumed.
     *
     * @return accelerometer quantization level.
     */
    public Acceleration getAccelerometerQuantizationLevelAsAcceleration() {
        return new Acceleration(accelerometerQuantizationLevel, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets accelerometer quantization level.
     * By default, it is zero when no quantization is assumed.
     *
     * @param result instance where value will be stored.
     */
    public void getAccelerometerQuantizationLevelAsAcceleration(final Acceleration result) {
        result.setValue(accelerometerQuantizationLevel);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets accelerometer quantization level.
     *
     * @param accelerometerQuantizationLevel accelerometer quantization level to be set.
     */
    public void setAccelerometerQuantizationLevel(final Acceleration accelerometerQuantizationLevel) {
        this.accelerometerQuantizationLevel = convertAcceleration(accelerometerQuantizationLevel);
    }

    /**
     * Gets gyro quantization level expressed in radians per second (rad/s).
     * By default, it is zero when no quantization is assumed.
     *
     * @return gyro quantization level expressed in radians per second.
     */
    public double getGyroQuantizationLevel() {
        return gyroQuantizationLevel;
    }

    /**
     * Sets gyro quantization level expressed in radians per second (rad/s).
     *
     * @param gyroQuantizationLevel gyro quantization level to be set.
     */
    public void setGyroQuantizationLevel(final double gyroQuantizationLevel) {
        this.gyroQuantizationLevel = gyroQuantizationLevel;
    }

    /**
     * Gets gyro quantization level.
     * By default, it is zero when no quantization is assumed.
     *
     * @return gyro quantization level.
     */
    public AngularSpeed getGyroQuantizationLevelAsAngularSpeed() {
        return new AngularSpeed(gyroQuantizationLevel, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets gyro quantization level.
     * By default, it is zero when no quantization is assumed.
     *
     * @param result instance where value will be stored.
     */
    public void getGyroQuantizationLevelAsAngularSpeed(final AngularSpeed result) {
        result.setValue(gyroQuantizationLevel);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets gyro quantization level.
     *
     * @param gyroQuantizationLevel gyro quantization level.
     */
    public void setGyroQuantizationLevel(final AngularSpeed gyroQuantizationLevel) {
        this.gyroQuantizationLevel = convertAngularSpeed(gyroQuantizationLevel);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final IMUErrors output) {
        output.copyFrom(this);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final IMUErrors input) {
        input.getAccelerometerBiases(accelerometerBiases);
        input.getGyroBiases(gyroBiases);
        accelerometerScaleFactorAndCrossCouplingErrors.copyFrom(input.accelerometerScaleFactorAndCrossCouplingErrors);
        gyroScaleFactorAndCrossCouplingErrors.copyFrom(input.gyroScaleFactorAndCrossCouplingErrors);
        gyroGDependentBiases.copyFrom(input.gyroGDependentBiases);
        accelerometerNoiseRootPSD = input.accelerometerNoiseRootPSD;
        gyroNoiseRootPSD = input.gyroNoiseRootPSD;
        accelerometerQuantizationLevel = input.accelerometerQuantizationLevel;
        gyroQuantizationLevel = input.gyroQuantizationLevel;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in
     * collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependentBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel, Arrays.hashCode(accelerometerBiases), Arrays.hashCode(gyroBiases));
    }

    /**
     * Checks if provided object is an IMUErrors instance having exactly the same
     * contents as this instance.
     *
     * @param o Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }

        final var imuErrors = (IMUErrors) o;
        return Double.compare(imuErrors.accelerometerNoiseRootPSD, accelerometerNoiseRootPSD) == 0 &&
                Double.compare(imuErrors.gyroNoiseRootPSD, gyroNoiseRootPSD) == 0 &&
                Double.compare(imuErrors.accelerometerQuantizationLevel, accelerometerQuantizationLevel) == 0 &&
                Double.compare(imuErrors.gyroQuantizationLevel, gyroQuantizationLevel) == 0 &&
                Arrays.equals(accelerometerBiases, imuErrors.accelerometerBiases) &&
                Arrays.equals(gyroBiases, imuErrors.gyroBiases) &&
                accelerometerScaleFactorAndCrossCouplingErrors.equals(
                        imuErrors.accelerometerScaleFactorAndCrossCouplingErrors) &&
                gyroScaleFactorAndCrossCouplingErrors.equals(imuErrors.gyroScaleFactorAndCrossCouplingErrors) &&
                gyroGDependentBiases.equals(imuErrors.gyroGDependentBiases);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (IMUErrors) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Converts acceleration instance to meters per squared second (m/s^2).
     *
     * @param acceleration instance to be converted.
     * @return converted value.
     */
    private double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(), acceleration.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param angularSpeed instance ot be converted.
     * @return converted value.
     */
    private double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return AngularSpeedConverter.convert(angularSpeed.getValue().doubleValue(), angularSpeed.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }
}
